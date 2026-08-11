"""
mujoco_preview.py — run_se3_control 的 MuJoCo 闭环预览与自动碰撞判定
====================================================================

定位
----
实机反复"撞"（肘/前臂向基部折叠撞基座）的根因: 静态 IK 可达性预检
只能看到**参考轨迹**（可达/限位/奇异），看不见 **GIC 闭环动态行为**
——肘部向基部折叠是闭环任务空间误差驱动的结果，静态检查拦不住；
而 MuJoCo 纯视觉仿真里连杆碰撞体又是禁用的（contype=0），只能靠人眼盯。

本模块给 run_se3_control 提供 ``--preview`` 的引擎:

  1. **闭环仿真**: 复用 verify_gic_mujoco 的 URDF→MuJoCo XML 转换, 与实机
     Phase2 主阶段**同一套参数** (轨迹/起步混合/带宽/阻尼/力矩限幅) 跑闭环;
     ``directTorque`` 直接发力矩, ``servoJ`` 复用 ServoJTorqueBridge +
     内层**计算力矩位置伺服**模拟 UR 内层 servoJ (重力补偿 + 速度/加速度前馈 +
     临界阻尼, 带宽 = ``servo_bandwidth``).
  2. **实时可视化**: MuJoCo viewer + 末端红色轨迹 trail + 半透明基座柱,
     真机一根手指都不用动.
  3. **自动碰撞判定** (本模块核心价值): 沿仿真得到的关节轨迹逐点 FK,
     检查连杆到**基座柱** / **地面**的最小净距, 打印确定性的
     ✓/✗ 结论 —— 不用盯着屏幕.

用法 (由 run_se3_control.py --preview 驱动)::

  python se3_control/scripts/run_se3_control.py --robot ur3 \
      --control-mode servoJ --task circle --duration 16 --bandwidth 10 --preview

碰撞判定几何约定 (UR 系列)
---------------------------
基座柱: 世界 z 轴附近的垂直圆柱, 半径约 ``col_radius`` (UR3 ≈ 0.09 m),
柱顶高度约 ``col_top_z`` (UR3 ≈ 0.152 m, 即 shoulder 转轴高度).
碰撞判定 **带高度条件**: 连杆水平净距 d < col_radius + link_half 且
连杆 z < col_top_z + link_half 才算撞 —— 否则 home 位姿下肘部距基座轴
仅 3.7 cm 但 z=0.39 高于柱顶, 会被误报.
"""

import logging
import os
import sys
import tempfile
import time

import numpy as np

# 必须先于任何 `from core.*` 导入把 se3_control/ 放进 sys.path:
# 本模块既可被 se3_control/scripts/*.py 以顶层 `core.mujoco_preview`
# 方式加载 (此时 se3_control/ 已在路径上), 也可被根目录脚本/tests 以
# `se3_control.core.mujoco_preview` 包方式加载 (此时 `core` 不在顶层).
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_DIR = os.path.dirname(_SCRIPT_DIR)   # se3_control/
if _PROJECT_DIR not in sys.path:
    sys.path.insert(0, _PROJECT_DIR)

from core.arm_log import ArmCsvLogger, arm_log_row


# ====================================================================
# 1. 碰撞自动判定
# ====================================================================

def check_simulated_collisions(robot_model, q_traj,
                               chain_links=None, ee_frame=None,
                               col_radius=0.09, col_top_z=0.152,
                               link_half=0.04, floor_thresh=0.03):
    """在关节轨迹上检查基座柱 / 地面碰撞风险 (逐点 FK).

    对每个时刻: 求每个移动连杆 frame 原点 + 相邻连杆间**中点** (杆体扫掠,
    例如肩→肘中点捕捉上臂贴柱), 检查:
      - 基座柱: 水平净距 d < col_radius+link_half 且 z < col_top_z+link_half
      - 地面:   z - link_half < floor_thresh
    排除 ``shoulder_link`` (柱顶转轴, 原点恒在轴上) 与第一个移动连杆的
    原点 (upper_arm 原点即肩部转轴, 恒在柱顶) —— 检查中点代替.

    :param robot_model: RobotModel 实例 (需 get_frame_pose)
    :param q_traj: (T, nv) 关节轨迹
    :param chain_links: 移动连杆 frame 名 (按运动链顺序, 不含肩部转轴)
    :param col_radius: 基座柱半径 (m)
    :param col_top_z:  基座柱顶高度 (m)
    :param link_half:  连杆等效半宽/半高 (m), 计入净距余量
    :param floor_thresh: 地面安全厚度 (m), 连杆底部低于该值判风险
    :returns: verdict dict — ok/min_base_d/min_base_t/min_base_name/
              min_z/min_z_t/min_z_name/first_violation/n_violations/T
    """
    if chain_links is None:
        chain_links = ['upper_arm_link', 'forearm_link',
                       'wrist_1_link', 'wrist_2_link', 'wrist_3_link']
    if ee_frame is None:
        ee_frame = robot_model.ee_frame_name

    d_thresh = col_radius + link_half
    lo_z = col_top_z + link_half
    f_thresh = floor_thresh + link_half

    q_traj = np.atleast_2d(np.asarray(q_traj, dtype=float))
    n_t = q_traj.shape[0]

    violations = []            # (t_idx, name, kind, value)
    min_d, min_d_t, min_d_name = float('inf'), 0.0, ''
    min_z, min_z_t, min_z_name = float('inf'), 0.0, ''
    chain = chain_links + [ee_frame]

    for i in range(n_t):
        robot_model.update(q_traj[i])
        ps = [robot_model.get_frame_pose(n)[0] for n in chain]
        # 检查点: 非首个移动连杆的原点 + 相邻连杆间中点; 末尾补 EE
        pts = []
        for j in range(1, len(ps) - 1):
            pts.append((f"mid({chain[j-1]}→{chain[j]})", 0.5 * (ps[j-1] + ps[j])))
            pts.append((chain[j], ps[j]))
        pts.append((ee_frame, ps[-1]))

        for name, p in pts:
            d = float(np.sqrt(p[0]**2 + p[1]**2))
            z = float(p[2])
            if d < min_d:
                min_d, min_d_t, min_d_name = d, i, name
            if z < min_z:
                min_z, min_z_t, min_z_name = z, i, name
            if d < d_thresh and z < lo_z:
                violations.append((i, name, 'base', d))
            if z - link_half < f_thresh:
                violations.append((i, name, 'floor', z))

    first = min(violations, key=lambda v: (v[0], 0 if v[2] == 'base' else 1)) \
        if violations else None

    return {
        'ok': not violations,
        'min_base_d': min_d, 'min_base_t': min_d_t, 'min_base_name': min_d_name,
        'min_z': min_z, 'min_z_t': min_z_t, 'min_z_name': min_z_name,
        'first_violation': first,
        'n_violations': len(violations),
        'T': n_t,
    }


def print_collision_report(verdict, dt, logger=None):
    """把碰撞判定打印成可读的 ✓/✗ 结论."""
    logger = logger or logging.getLogger("mujoco_preview")
    v = verdict
    t_end = v['T'] * dt if v['T'] else 0.0
    logger.info(f"\n{'='*50}")
    logger.info(f"碰撞判定 ({t_end:.1f}s, {v['T']} 步):")
    logger.info(f"  全程最小基座净距: {v['min_base_d']*100:6.1f} cm "
                f"@ t={v['min_base_t']*dt:5.1f}s (连杆 {v['min_base_name']})")
    logger.info(f"  全程最低连杆高度: {v['min_z']*100:6.1f} cm "
                f"@ t={v['min_z_t']*dt:5.1f}s (连杆 {v['min_z_name']})")
    if v['ok']:
        logger.info(f"  ✓ 全程无碰撞风险 — 可上真机")
    else:
        t, name, kind, val = v['first_violation']
        what = f"{name} 距基座柱仅 {val*100:.1f} cm" if kind == 'base' \
            else f"{name} 低至 {val*100:.1f} cm"
        logger.warning(f"  ✗ 存在碰撞风险: t={t*dt:.1f}s {what} (共 {v['n_violations']} 次越界)")
        logger.warning(f"    请调整 --center/--radius/任务参数后重跑 --preview; 或先让臂远离基部")
    logger.info(f"{'='*50}")


# ====================================================================
# 2. MuJoCo 闭环预览
# ====================================================================

def run_preview(robot_name, urdf_path, ee_frame, home_q, traj,
                task_cfg=None, bandwidth=20.0, damping=1.0,
                torque_limits=None, duration=15.0, ctrl_dt=0.004,
                blend_time=0.5, control_mode='directTorque',
                servo_bandwidth=30.0, show_viewer=True, speed=1.0,
                link_to_mesh=None, mesh_subdir='', start_q=None,
                logger=None, log_dir=None):
    """MuJoCo 闭环预览: 跑与实机 Phase2 相同的任务, 返回含碰撞判定的结果.

    :param traj: TrajectoryFuncs — 期望轨迹 (regulation 传 make_static_traj;
                 circle/line 传 build_trajectory 结果, 覆盖已应用)
    :param task_cfg: task_config.get_task_config(robot) — 提供 trail 可视化参数
    :param control_mode: 'directTorque' → data.ctrl=GIC 力矩;
                         'servoJ' → ServoJTorqueBridge 折算关节目标位,
                        内层计算力矩位置伺服模拟 UR 内层 servoJ.
    :param servo_bandwidth: servoJ 内层伺服带宽 (rad/s, 默认 30).
                        内层采用临界阻尼 + 重力补偿 + 速度/加速度前馈的
                        计算力矩伺服 τ = M·(ddq + 2ω(dq_des−dq) + ω²(q_des−q)) + bias,
                        带宽须明显高于外环 GIC 带宽 (CLI servoJ 上限 10) 才能稳定;
                        之前用裸 PD (无重力补偿/前馈) 在 circle 任务上发散成混乱轨迹.
    :param show_viewer: False → headless, 只出碰撞结论 (测试/冒烟用)
    :param speed: 实时倍速 (>1 加速). 默认 1.0 = 与实机同步节奏.
    :param log_dir: 若给定, 每控制周期写**与实机相同格式**的全分辨率 CSV 到该目录
                    (用于与真机 run_se3_control.py --log-dir 的记录逐列对照).
    :returns: dict — verdict / q / t / p / tau / pos_err / rot_err
    """
    logger = logger or logging.getLogger("mujoco_preview")
    import mujoco
    # 延迟 import: verify_gic_mujoco 是脚本模块 (有 if __name__ 保护),
    # 仅预览路径用到, 避免实机运行路径引入其 import 链.
    from scripts.verify_gic_mujoco import urdf_joints_to_mujoco_xml

    # ── 1. 构建 MuJoCo 模型 (URDF→XML, 含 motor actuator) ──
    xml_str = urdf_joints_to_mujoco_xml(urdf_path, ee_frame,
                                        link_to_mesh=link_to_mesh,
                                        mesh_subdir=mesh_subdir,
                                        debug=False)
    tmpf = tempfile.NamedTemporaryFile(suffix='.xml', mode='w', delete=False)
    tmpf.write(xml_str)
    tmpf.close()
    try:
        model = mujoco.MjModel.from_xml_path(tmpf.name)
        data = mujoco.MjData(model)
    finally:
        os.unlink(tmpf.name)
    nv = model.nv
    physics_dt = float(model.opt.timestep)
    n_sub = max(1, int(round(ctrl_dt / physics_dt)))

    # ── 2. RobotModel (Pinocchio, 同一 URDF) ──
    from robot_model.robot_model import RobotModel
    robot = RobotModel(urdf_path, ee_frame_name=ee_frame,
                       robot_name=robot_name, verbose=False)

    # ── 3. 起始位姿 (默认 home; 可 --preview-start-q 传入真机当前位形) ──
    if start_q is not None:
        q0 = np.asarray(start_q, dtype=float).ravel()
        if len(q0) != nv:
            raise ValueError(f"start_q 维度 {len(q0)} != nv {nv}")
    else:
        q0 = np.asarray(home_q, dtype=float).ravel()
    data.qpos[:nv] = q0.copy()
    data.qvel[:nv] = np.zeros(nv)
    mujoco.mj_forward(model, data)
    robot.update(q0)
    p_start, R_start = robot.get_pose()

    # ── 4. 控制器 (+ servoJ 桥) ──
    torque_limits = np.asarray(torque_limits, dtype=float).ravel() \
        if torque_limits is not None else None
    from core.gic_controller import GICController
    from core.servo_bridge import ServoJTorqueBridge
    from core.se3_math import rotmat_slerp
    from core.trajectory import eval_body_twist
    controller = GICController(robot, bandwidth=bandwidth, damping=damping,
                               torque_limits=torque_limits)
    bridge = ServoJTorqueBridge(robot, controller, ctrl_dt, ref_damp=15.0) \
        if control_mode == 'servoJ' else None
    if bridge is not None:
        bridge.reset(q0)

    # ── 5. 记录 + trail 参数 ──
    trail_cfg = getattr(task_cfg, 'trail', {}) if task_cfg is not None else {}
    TRAIL_INTERVAL = trail_cfg.get('interval', 8)
    TRAIL_MAX = trail_cfg.get('max_points', 1200)
    TRAIL_SIZE = trail_cfg.get('sphere_size', 0.006)
    TRAIL_COLOR = np.array(trail_cfg.get('color', [1.0, 0.2, 0.2, 0.85]),
                           dtype=float)

    n_steps = max(1, int(duration / ctrl_dt))
    q_log = np.zeros((n_steps, nv))
    t_log = np.zeros(n_steps)
    p_log = np.zeros((n_steps, 3))
    tau_log = np.zeros((n_steps, nv))
    err_log = np.zeros(n_steps)
    rerr_log = np.zeros(n_steps)

    # 全分辨率 CSV (与实机 run_tracking 相同列序, 供对照分析)
    csv_log = None
    if log_dir:
        os.makedirs(log_dir, exist_ok=True)
        fname = os.path.join(log_dir, f"sim_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        csv_log = ArmCsvLogger(fname, nv)
        logger.info(f"[Preview] 记录每控制周期数据 → {fname}")

    # ── 6. Viewer ──
    viewer = None
    trail_actual = []
    if show_viewer:
        try:
            from mujoco.viewer import launch_passive
            viewer = launch_passive(model, data)
            time.sleep(0.5)
        except Exception as e:
            logger.warning(f"[Viewer] 启动失败: {e} (headless 继续)")
            show_viewer = False

    logger.info(f"[Preview] 仿真: task 闭环 {duration:.1f}s @ {ctrl_dt*1000:.0f}ms "
                f"({n_sub}×{physics_dt*1000:.1f}ms 物理子步) "
                f"control={control_mode} bw={bandwidth:.0f} ζ={damping:.1f} "
                f"blend={blend_time:.1f}s")

    t_phys = 0.0
    started_wall = time.time()
    stopped = False
    for i in range(n_steps):
        t = i * ctrl_dt

        if viewer is not None and not viewer.is_running():
            logger.info(f"[Viewer] 窗口已关闭 — 提前结束仿真 (t={t:.1f}s)")
            n_steps = i
            stopped = True
            break

        # ── 期望轨迹 + 起步混合 (与实机 run_tracking 相同) ──
        bf = 1.0 if blend_time <= 0 else min(1.0, t / blend_time)
        pd_ref = traj.pd_t(t).ravel()
        Rd_ref = traj.Rd_t(t).reshape(3, 3)
        pd = (1.0 - bf) * p_start + bf * pd_ref
        Rd = rotmat_slerp(R_start, Rd_ref, bf)
        vd, wd, dvd, dwd = eval_body_twist(traj, t, Rd, bf)

        # ── 读取当前状态 ──
        q = data.qpos[:nv].copy()
        dq = data.qvel[:nv].copy()

        # ── 控制 ──
        if bridge is not None:
            q_servo, tau = bridge.compute(q, dq, pd, Rd, vd, wd, dvd, dwd)
            # 内层计算力矩位置伺服 (模拟 UR 内层 servoJ 跟踪 q_servo):
            #   τ = M·(ddq_des + 2ω·(dq_des−dq) + ω²·(q_des−q)) + bias
            # 含重力补偿 (bias) + 速度/加速度前馈 (dq_des/ddq_des) + 临界阻尼,
            # 惯性/偏置量取当前位形 Pinocchio 值 (与 MuJoCo 同 URDF → 精确).
            # 裸 PD (无重力补偿/前馈) 会因腕部力矩限幅饱和 + 积分漂移而发散.
            robot.update(q, dq)
            dq_des = bridge.dq_target
            ddq_des = bridge.ddq_target
            M = robot.get_full_inertia()
            bias = robot.get_bias_torque()
            tau_servo = (M @ (ddq_des
                              + 2.0 * servo_bandwidth * (dq_des - dq)
                              + servo_bandwidth**2 * (q_servo - q))
                         + bias)
            if torque_limits is not None:
                tau_servo = np.clip(tau_servo, -torque_limits, torque_limits)
            data.ctrl[:model.nu] = tau_servo[:model.nu]
        else:
            tau = controller.compute(q, dq, pd, Rd, vd, wd, dvd, dwd)
            data.ctrl[:model.nu] = tau[:model.nu]

        # ── 物理子步 ──
        for _ in range(n_sub):
            mujoco.mj_step(model, data)
        t_phys += ctrl_dt

        # ── 记录 ──
        q_log[i] = q
        t_log[i] = t
        tau_log[i] = tau.ravel()
        site_p = data.site_xpos[0].copy() if model.nsite > 0 \
            else robot.get_pose()[0]
        p_log[i] = site_p
        ep = site_p - pd_ref
        err_log[i] = float(np.linalg.norm(ep))
        R_cur = data.site_xmat[0].copy().reshape(3, 3) if model.nsite > 0 \
            else robot.get_pose()[1]
        R_rel = R_cur.T @ Rd_ref
        c = np.clip(0.5 * (np.trace(R_rel) - 1.0), -1.0, 1.0)
        rerr_log[i] = float(np.arccos(c))

        # ── 全分辨率 CSV (与实机 run_tracking 同列序) ──
        if csv_log is not None:
            q_s = q_servo if bridge is not None else q
            dq_d = bridge.dq_target if bridge is not None else [np.nan] * nv
            tl_row = list(torque_limits) if torque_limits is not None \
                else [np.nan] * nv
            csv_log.write(arm_log_row(
                nv, t, bf, err_log[i], rerr_log[i],
                pd, pd_ref, site_p, q, dq, q_s, dq_d, tau, tl_row))

        # ── Viewer trail ──
        if viewer is not None:
            if i % TRAIL_INTERVAL == 0:
                trail_actual.append(site_p.copy())
                if len(trail_actual) > TRAIL_MAX:
                    trail_actual.pop(0)
            if i % 5 == 0:
                viewer.user_scn.ngeom = 0
                # 基座柱半透明圆柱 (辅助目测净距)
                mujoco.mjv_initGeom(
                    viewer.user_scn.geoms[0],
                    mujoco.mjtGeom.mjGEOM_CYLINDER,
                    np.array([0.09, 0.15, 0]),
                    np.array([0.0, 0.0, 0.15]),
                    np.eye(3).flatten(),
                    np.array([0.35, 0.35, 0.4, 0.30]),
                )
                viewer.user_scn.ngeom = 1
                ngeom = min(len(trail_actual), viewer.user_scn.maxgeom - 1)
                for j in range(ngeom):
                    pos = trail_actual[j]
                    mujoco.mjv_initGeom(
                        viewer.user_scn.geoms[j + 1],
                        mujoco.mjtGeom.mjGEOM_SPHERE,
                        np.array([TRAIL_SIZE, 0, 0]),
                        pos, np.eye(3).flatten(), TRAIL_COLOR,
                    )
                viewer.user_scn.ngeom = 1 + ngeom
                viewer.sync()

        # ── 实时节流 ──
        if show_viewer and speed > 0:
            time.sleep(ctrl_dt / speed)

    wall = time.time() - started_wall
    if viewer is not None:
        viewer.close()

    if csv_log is not None:
        csv_log.close()

    q_log = q_log[:n_steps]
    t_log = t_log[:n_steps]
    p_log = p_log[:n_steps]
    tau_log = tau_log[:n_steps]
    err_log = err_log[:n_steps]
    rerr_log = rerr_log[:n_steps]

    logger.info(f"[Preview] 仿真完成: {t_log[-1]:.1f}s 仿真 / {wall:.1f}s 墙钟 "
                f"({n_steps/(wall if wall>0 else 1):.0f} 控制步/s)")

    # ── 7. 碰撞判定 ──
    verdict = check_simulated_collisions(robot, q_log)
    print_collision_report(verdict, ctrl_dt, logger=logger)

    return {
        'verdict': verdict,
        'q': q_log, 't': t_log, 'p': p_log,
        'tau': tau_log, 'pos_err': err_log, 'rot_err': rerr_log,
        'stopped': stopped, 'wall_sec': wall,
    }
