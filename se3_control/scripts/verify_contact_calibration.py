#!/usr/bin/env python
"""阶段 0: 接触模型标定 — 纯运动学压入, 标定 MuJoCo 接触的 K_env 与回跳特性.

计划 docs/plan/force_interaction_experiments_plan.md 附录 A.9 阶段 0.
**不接入任何控制器** (无 GAC/GIC), 目标是标定"接触模型本身"的数值特性,
避免后续实验三的失稳被接触求解器行为污染.

Part A — 静态压深扫描:
    直接设置 qpos (IK 求压入位形) + mj_forward, 逐压深读接触力,
    拟合线性刚度  K_env = dF/d(penetration), 报告线性区间与 R².
    同时按 solref 时间常数扫描, 看 K_env 随环境刚度标尺的变化.

Part B — 动态冲击 (回跳特性):
    MuJoCo 内置 position actuator (隐式积分, 对近零惯量腕部稳定; 腕关节另加
    dof_armature 电机转子惯量, 修正 URDF 缺失腕部电机惯量) 以恒定
    approach_speed 逼近并压入刚体球, 记录冲击瞬态, 测:
      峰值力 / 稳态力 / 接触力超调 / make-break 断开次数 /
      回弹速度 (v_rebound) / 恢复系数 e ≈ |v_rebound|/|v_impact| / 调节时间.
    扫 approach_speed 看回跳是否随冲击能量变化.

注意: Part A 静态接触力 = K_env·pen (零速度, 无阻尼项);
      Part B 动态接触力还含 solref 阻尼项 B·peṅ (damping=1 临界阻尼),
      故冲击峰值力 >> 同压深之静态力. 两者分开解读.

接触力来源: mujoco.mj_contactForce 逐接触累加 (只统计工具尖参与的接触),
与 ee_force 传感器交叉验证 (见探针, 两者一致).

用法示例:
  python se3_control/scripts/verify_contact_calibration.py
  python se3_control/scripts/verify_contact_calibration.py --robot ur3
  python se3_control/scripts/verify_contact_calibration.py --approach-speeds 0.05 0.1 0.2
  python se3_control/scripts/verify_contact_calibration.py --solref-times 0.02 0.005 0.002

产物:
  控制台标定报告 + se3_control/figures/contact/calibration_partA.png (F vs 压深)
  + calibration_partB.png (冲击时间序列).
"""

import argparse
import math
import os
import re
import sys
import tempfile

import numpy as np

# ── 路径注入 (与 verify_gac_mujoco.py 相同的约定) ──
_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
for _p in (_PROJECT_ROOT,
           os.path.join(_PROJECT_ROOT, 'se3_control'),
           os.path.join(_PROJECT_ROOT, 'se3_control', 'scripts')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import mujoco

from verify_gac_mujoco import urdf_joints_to_mujoco_xml
from config.robot_configs import get_robot_config, get_urdf_path
from robot_model import RobotModel

# 接触判定阈值 (F 低于此视为"断开")
F_OFF = 2.0        # N
# 线性拟合的最小压深 (过滤激活预载区)
PEN_FIT_MIN = 2e-4 # m


def _is_position_actuator(model):
    """检测模型是否使用 MuJoCo 内置 <position> 驱动器.

    position 驱动器经 biasprm[1] = -kp 施加 kp·(ctrl - q) - kv·q̇;
    motor 驱动器 biasprm 全 0, ctrl 单位为力矩. 静态扫描需知道 ctrl
    语义才能把驱动器置零 (position 型要 ctrl=q, motor 型要 ctrl=0).
    """
    return bool(np.any(np.abs(model.actuator_biasprm[:, 1]) > 1e-12))


# ====================================================================
# 环境构建
# ====================================================================

# Part B 位置伺服的默认每关节增益 (Nm/rad) 与阻尼 (Nm·s/rad), 按机器人区分.
# 大臂/肘高增益 → 压入刚硬; 腕部低增益 (配合 dof_armature 转子惯量) → 稳定.
# UR12e 大而重, 需要高增益; UR3 小一个量级, UR12e 的 kp=5000 会让其塌缩,
# 实测 kp=3000 保持稳定且下垂 <3mm.
DEFAULT_ACT_KP = {
    'ur12e': [5000.0, 5000.0, 3000.0, 400.0, 250.0, 150.0],
    'ur3':   [3000.0, 3000.0, 1800.0, 200.0, 150.0, 80.0],
}
DEFAULT_ACT_KV = {
    'ur12e': [200.0, 200.0, 150.0, 20.0, 12.0, 8.0],
    'ur3':   [120.0, 120.0, 80.0, 15.0, 10.0, 6.0],
}
# 腕关节电机转子惯量 (kg·m²) — URDF 缺失腕部电机惯量, 使腕部惯量近零、
# 任何控制器都会失稳; 真实 UR 腕部有显著电机惯量, 故在此补上.
DEFAULT_WRIST_ARMATURE = 0.1


def build_environment(robot_name, ball_pos, ball_radius,
                      tool_length, tool_radius=0.01, tool_mass=0.05,
                      force_sensor=True, dt=0.001,
                      position_actuator=True, act_kp=None, act_kv=None,
                      wrist_armature=DEFAULT_WRIST_ARMATURE):
    """构建带接触环境的 MuJoCo 模型 + RobotModel (无控制器).

    :param position_actuator: 用 MuJoCo 内置 <position> 驱动器替换 <motor>.
        位置驱动器经隐式积分, 对近零惯量腕部稳定 (Part B 动态冲击必需);
        Part A 纯运动学压深只用 qpos + mj_forward, 不读 ctrl, 故不受影响.
    :param act_kp / act_kv: position actuator 每关节刚度/阻尼 (None→默认).
    :param wrist_armature: 腕部 dof_armature 电机转子惯量 (None→不设置).

    :returns: (model, data, robot, cfg, tip_body_id, tip_geom_id,
               ball_geom_id, ball_center)
    """
    cfg = get_robot_config(robot_name)
    urdf_path = get_urdf_path(robot_name)

    xml_str = urdf_joints_to_mujoco_xml(
        urdf_path, cfg['ee_frame'], timestep=dt,
        link_to_mesh=cfg['link_to_mesh'], mesh_subdir=cfg['mesh_subdir'],
        rigid_ball=(np.asarray(ball_pos, dtype=float), ball_radius),
        tool_tip={'length': tool_length, 'radius': tool_radius,
                  'mass': tool_mass},
        force_sensor=force_sensor)

    if position_actuator:
        kp = act_kp if act_kp is not None else DEFAULT_ACT_KP[robot_name]
        kv = act_kv if act_kv is not None else DEFAULT_ACT_KV[robot_name]
        if len(kp) != 6 or len(kv) != 6:
            raise ValueError('act_kp / act_kv 长度必须为 6')
        act_block = '<actuator>\n'
        for jn, kpv, kvv in zip(cfg['joint_names'], kp, kv):
            act_block += (f'  <position name="{jn}_act" joint="{jn}" '
                          f'kp="{kpv:g}" kv="{kvv:g}" ctrllimited="false" '
                          f'forcelimited="false"/>\n')
        act_block += '</actuator>'
        xml_str = re.sub(r'<actuator>.*?</actuator>', act_block, xml_str,
                         flags=re.S)

    tmpf = tempfile.NamedTemporaryFile(suffix='.xml', mode='w', delete=False)
    tmpf.write(xml_str)
    tmpf.close()
    try:
        model = mujoco.MjModel.from_xml_path(tmpf.name)
        if wrist_armature is not None:
            model.dof_armature[3:6] = float(wrist_armature)
        data = mujoco.MjData(model)
    finally:
        os.unlink(tmpf.name)

    robot = RobotModel(urdf_path, ee_frame_name=cfg['ee_frame'], verbose=False)

    body_names = [model.body(i).name for i in range(model.nbody)]
    geom_names = [model.geom(i).name for i in range(model.ngeom)]
    tip_body_id = body_names.index('tool_tip')
    tip_geom_id = geom_names.index('tool_tip')
    ball_geom_id = geom_names.index('rigid_ball')
    ball_center = np.asarray(ball_pos, dtype=float)
    return (model, data, robot, cfg,
            tip_body_id, tip_geom_id, ball_geom_id, ball_center)


# ====================================================================
# 量测工具
# ====================================================================

def tool_tip_pos(data, tip_body_id):
    """工具尖中心世界坐标."""
    return data.xpos[tip_body_id].copy()


def penetration_from_geom(tip_pos, ball_center, ball_radius, tip_radius):
    """几何压深: (球半径+尖半径) − 球心距, 非负."""
    d = np.linalg.norm(tip_pos - ball_center)
    return max(0.0, ball_radius + tip_radius - d)


def contact_force_mag(model, data, tip_geom_id):
    """工具尖所受接触力大小 (N), 从 mj_contactForce 累加.

    只统计与工具尖 geom 有关的接触 (臂 mesh 均为 contype=0, 不会参与).
    """
    F = np.zeros(3)
    for i in range(data.ncon):
        c = data.contact[i]
        if tip_geom_id not in (c.geom1, c.geom2):
            continue
        f = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, f)
        frame = c.frame.reshape(3, 3)
        F += frame @ f[:3]
    return float(np.linalg.norm(F))


# ====================================================================
# Part A — 静态压深扫描 + K_env 拟合
# ====================================================================

def _ik_press(robot, home_q, p_ee0, R_ee0, delta):
    """把末端下压 delta (m, 世界系 -z), 返回压入位形 q."""
    p_des = p_ee0 + np.array([0.0, 0.0, -delta])
    return robot.gauss_newton_IK(p_des, R_ee0, home_q,
                                 step_size=0.5, tol=1e-8, max_cnt=300)


def static_sweep(model, data, robot, cfg, tip_body_id, tip_geom_id,
                 ball_center, ball_radius, tip_radius, gap,
                 max_pen=0.010, n_steps=31, verbose=True):
    """纯运动学压深扫描: 直接设 qpos + mj_forward, 逐压深读接触力.

    工具在 home 位末端竖直朝下, 球顶在工具正下方 → 压入沿竖直方向.

    :returns: (pen_actual, F, pen_cmd)
    """
    home_q = cfg['home_q'][:robot.nv]
    data.qpos[:robot.nv] = home_q.copy()
    data.qvel[:robot.nv] = np.zeros(robot.nv)
    mujoco.mj_forward(model, data)
    robot.update(home_q)
    p_ee0, R_ee0 = robot.get_pose()

    # position actuator 在 ctrl=0 时施加 kp·(0-q) 的拉向 q=0 的巨大力矩,
    # 会污染静态接触力 → 置 ctrl=当前位形 q_ik 以零力; motor 型则 ctrl=0.
    is_pos = _is_position_actuator(model)

    pen_cmd = np.linspace(0.0, max_pen, n_steps)
    pen_actual = np.zeros(n_steps)
    F = np.zeros(n_steps)
    for k, pen in enumerate(pen_cmd):
        delta = gap + pen
        q_ik = _ik_press(robot, home_q, p_ee0, R_ee0, delta)
        data.qpos[:robot.nv] = q_ik.copy()
        data.qvel[:robot.nv] = np.zeros(robot.nv)
        data.ctrl[:robot.nv] = q_ik.copy() if is_pos else np.zeros(robot.nv)
        mujoco.mj_forward(model, data)
        tip = tool_tip_pos(data, tip_body_id)
        pen_actual[k] = penetration_from_geom(
            tip, ball_center, ball_radius, tip_radius)
        F[k] = contact_force_mag(model, data, tip_geom_id)
    return pen_actual, F, pen_cmd


def fit_k_env(pen, F):
    """对接触区 (pen > PEN_FIT_MIN) 线性拟合 F = K_env·pen + b.

    :returns: (K_env, intercept, R², mask) 或 None (样本不足)
    """
    mask = pen > PEN_FIT_MIN
    if mask.sum() < 3:
        return None
    A = np.column_stack([pen[mask], np.ones(mask.sum())])
    k, b = np.linalg.lstsq(A, F[mask], rcond=None)[0]
    Fhat = A @ np.array([k, b])
    ss_res = float(np.sum((F[mask] - Fhat) ** 2))
    ss_tot = float(np.sum((F[mask] - F[mask].mean()) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 1.0
    return float(k), float(b), float(r2), mask


def sweep_env_stiffness(model, data, robot, cfg, tip_body_id, tip_geom_id,
                        ball_center, ball_geom_id, ball_radius, tip_radius,
                        gap, solref_times, max_pen=0.010, n_steps=31):
    """扫刚体球接触刚度 (solref 时间常数) → 拟合 K_env 随刚度标尺的变化.

    :returns: [(timeconst, K_env, intercept, R²), ...]
    """
    results = []
    for tc in solref_times:
        # MuJoCo solref: [时间常数, 阻尼比]; 调小时间常数 → 变硬
        model.geom_solref[ball_geom_id, :] = np.array([tc, 1.0])
        pen, F, _ = static_sweep(model, data, robot, cfg, tip_body_id,
                                 tip_geom_id, ball_center, ball_radius,
                                 tip_radius, gap, max_pen=max_pen,
                                 n_steps=n_steps, verbose=False)
        fit = fit_k_env(pen, F)
        if fit is None:
            results.append((tc, None, None, None))
        else:
            results.append((tc, fit[0], fit[1], fit[2]))
    return results


# ====================================================================
# Part B — 动态冲击 (回跳特性)
# ====================================================================

def dynamic_press(model, data, robot, cfg, tip_body_id, tip_geom_id,
                  ball_center, ball_radius, tip_radius, tool_length,
                  approach_speed, press_pen=0.005, hold_time=0.5,
                  back_speed=0.05, verbose=True):
    """position actuator (MuJoCo 内置隐式积分) 恒速逼近并压入刚体球.

    轨迹三段 (工具尖沿世界 -z 竖直运动, 末端朝向保持 home):
      - 逼近段: 以 approach_speed 恒速下降, 直至几何穿透 press_pen
      - 保持段: 保持该压深 hold_time
      - 回退段: 以 back_speed 恒速回升至 home

    每步由 IK 求工具尖 z=zd 对应的关节位形, 喂给 position actuator 的 ctrl
    (ctrl 单位 = 关节弧度). position actuator 在 MuJoCo 内以隐式积分稳定
    跟踪, 腕部近零惯量不会失稳 (配合 dof_armature 转子惯量).

    :returns: dict 含 't','pen','F','vz','pz','metrics'
    """
    home_q = cfg['home_q'][:robot.nv]
    nv = robot.nv

    data.qpos[:nv] = home_q.copy()
    data.qvel[:nv] = np.zeros(nv)
    mujoco.mj_forward(model, data)
    robot.update(home_q)
    _p_ee0, R_ee0 = robot.get_pose()
    tip0 = tool_tip_pos(data, tip_body_id)

    # 接触平面: 工具在 (tip0.x, tip0.y) 竖直下落时首次触及球面的 tip z
    dx = tip0[0] - ball_center[0]
    dy = tip0[1] - ball_center[1]
    rsum = ball_radius + tip_radius
    if dx * dx + dy * dy > rsum * rsum:
        raise ValueError('工具水平位置在球体投影外, 无法竖直压入')
    z_contact = ball_center[2] + math.sqrt(rsum * rsum - dx * dx - dy * dy)
    z_end = z_contact - press_pen            # 工具尖目标 z (穿透 press_pen)

    press_dist = tip0[2] - z_end
    t_press = max(press_dist / approach_speed, 0.01)
    t_back = max(press_dist / back_speed, 0.01)
    T = int(np.ceil((t_press + hold_time + t_back) / model.opt.timestep))

    t = np.zeros(T)
    pen = np.zeros(T)
    F = np.zeros(T)
    vz = np.zeros(T)
    pz = np.zeros(T)

    q_cur = home_q.copy()
    z_prev = tip0[2]
    for i in range(T):
        ti = i * model.opt.timestep
        if ti < t_press:
            zd = tip0[2] - approach_speed * ti        # 恒速逼近
        elif ti < t_press + hold_time:
            zd = z_end
        else:
            u = ti - (t_press + hold_time)
            zd = min(z_end + back_speed * u, tip0[2]) # 恒速回升

        # 工具尖 z=zd → 末端位姿 (工具沿末端 -z 伸出 L)
        p_des = np.array([tip0[0], tip0[1], zd + tool_length])
        q_des = robot.gauss_newton_IK(p_des, R_ee0, q_cur,
                                      step_size=0.5, tol=1e-8, max_cnt=200)
        data.ctrl[:nv] = q_des
        mujoco.mj_step(model, data)
        q_cur = data.qpos[:nv].copy()

        t[i] = ti
        pen[i] = penetration_from_geom(
            tool_tip_pos(data, tip_body_id), ball_center,
            ball_radius, tip_radius)
        F[i] = contact_force_mag(model, data, tip_geom_id)
        pz[i] = data.xpos[tip_body_id, 2]
        # 工具世界 z 速度用 xpos 向后差分 (data.cvel 帧约定与世界速度
        # 不一致, 实测 cvel[tip,2] 在逼近时≈0 而有限差分=-v_app 吻合)
        vz[i] = 0.0 if i == 0 else (pz[i] - z_prev) / model.opt.timestep
        z_prev = pz[i]

    # 轻平滑 (5 点移动平均) 压掉差分噪声, 保留冲击瞬态
    k = min(5, T)
    if T >= k:
        vz = np.convolve(vz, np.ones(k) / k, mode='same')

    # 冲击瞬态统计窗 = [首次接触, 回退开始), 排除回退脱开的误判
    metrics = compute_impact_metrics(t, pen, F, vz, F_off=F_OFF,
                                     t_win_end=t_press + hold_time)
    return {'t': t, 'pen': pen, 'F': F, 'vz': vz, 'pz': pz,
            'metrics': metrics}


def compute_impact_metrics(t, pen, F, vz, F_off=F_OFF, t_win_end=None):
    """从冲击时间序列提取回跳特性指标.

    语义 (A.8 四阶段):
      - v_impact: 接触前工具的逼近速度 (取接触前 20ms 内最大向下速度;
        接触脉冲在同一 mj_step 内已把 vz 吸收, 故不能用接触后 vz)
      - F_peak:   窗内接触力峰值 (冲击尖峰, 含 solref 阻尼项)
      - F_ss:     保持段稳态力 (窗内后 20% 均值, 排除了回退段 F→0)
      - 超调:     (F_peak - F_ss)/F_ss
      - breaks:   窗内接触断开次数 (回弹跳离)
      - e_rest:   v_rebound / v_impact (恢复系数)
      - settle:   从接触到 |F−F_ss| 进入 ±10%F_ss 并保持 ≥20ms 的时刻

    :param t_win_end: 统计窗终点 (通常 = 保持段结束 = 回退开始), 排除
        回退脱开被误判为反弹; 缺省 = 全程.
    :returns: dict 或 None (未接触)
    """
    idx_c = np.where(pen > PEN_FIT_MIN)[0]
    if idx_c.size == 0:
        return None
    i_c = int(idx_c[0])
    t_c = t[i_c]

    # 时间窗: [首次接触, t_win_end] (缺省=全程)
    if t_win_end is not None:
        i_end = int(np.searchsorted(t, t_win_end))
    else:
        i_end = t.size
    i_end = max(i_end, i_c + 1)
    seg = np.arange(i_c, i_end)

    # 接触前逼近速度: 接触前 20ms 内最大向下速度 (负)
    i_pre = max(0, i_c - int(0.020 / max(t[1] - t[0], 1e-6)))
    v_pre = vz[i_pre:i_c + 1]
    v_impact = float(-np.min(v_pre)) if np.min(v_pre) < 0 else float(-np.min(np.abs(v_pre)))

    F_win = F[seg]
    F_peak = float(F_win.max())
    # 稳态力: 窗内后 20% 均值 (保持段)
    n_win = F_win.size
    F_ss = float(F_win[int(0.8 * n_win):].mean()) if n_win >= 5 else float(F_win.mean())
    overshoot = (F_peak - F_ss) / F_ss if F_ss > 1e-9 else 0.0

    # make-break: 窗内 F 低于 F_off 又回升的次数
    on = F >= F_off
    breaks = 0
    prev = True
    for k in seg[1:]:
        if prev and not on[k]:
            breaks += 1
        prev = on[k]

    # 回弹: 窗内最大向上 (正) 工具 z 速度
    v_win = vz[seg]
    v_rebound = float(v_win.max()) if v_win.max() > 0 else 0.0
    e_rest = (v_rebound / max(v_impact, 1e-9)
              if v_impact > 1e-3 else 0.0)

    # 调节时间: 从接触到 |F−F_ss| 进入 ±10%F_ss 且保持 ≥20ms (窗内)
    settle = None
    band = 0.10 * F_ss
    n_keep = max(int(0.020 / max(t[1] - t[0], 1e-6)), 3)
    for k in seg:
        jj = np.arange(k, min(k + n_keep, i_end))
        if jj.size and all(np.abs(F[j] - F_ss) < band for j in jj):
            settle = float(t[k] - t_c)
            break

    return {
        't_contact': float(t_c),
        'v_impact': float(v_impact),
        'F_peak': F_peak,
        'F_ss': F_ss,
        'overshoot': overshoot,
        'breaks': breaks,
        'v_rebound': v_rebound,
        'e_rest': e_rest,
        'settle': settle,
    }


# ====================================================================
# 报告与绘图
# ====================================================================

_CJK_FONT_CANDIDATES = [
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
    '/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc',
    '/usr/share/fonts/opentype/noto/NotoSansCJKsc-Regular.otf',
    '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc',
]


def _setup_matplotlib():
    """配置 matplotlib 使用 CJK 字体渲染中文图标签 (找不到则退回默认)."""
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import font_manager
    for fname in _CJK_FONT_CANDIDATES:
        if os.path.exists(fname):
            try:
                font_manager.fontManager.addfont(fname)
                fam = font_manager.FontProperties(fname=fname).get_name()
                matplotlib.rcParams['font.family'] = fam
            except Exception:
                continue
            break


def plot_partA(pen_actual, F, fit, pen_cmd, solref_results, save_dir):
    _setup_matplotlib()
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(7, 5))
    ax.scatter(pen_actual * 1000, F, s=28, label='静态压深 (直接 qpos)',
               zorder=3)
    if fit is not None:
        k, b, r2, mask = fit
        pp = np.linspace(0, pen_actual.max() * 1000, 50)
        ax.plot(pp, k * pp / 1000 + b, 'r--', lw=1.5,
                label=f'K_env={k/1000:.1f} kN/m (R²={r2:.4f})')
        ax.set_xlabel('压深 penetration (mm)')
    ax.set_ylabel('接触力 F (N)')
    ax.set_title('Part A — 静态压深-接触力 (K_env 标定)')
    ax.grid(alpha=0.3)
    ax.legend()

    # 次级图: K_env vs solref 时间常数
    if solref_results:
        ax2 = ax.twinx()
        tcs = [r[0] for r in solref_results if r[1] is not None]
        ks = [r[1] / 1000 for r in solref_results if r[1] is not None]
        ax2.plot(tcs, ks, 'g--o', alpha=0.7)
        ax2.set_xscale('log')
        ax2.set_ylabel('K_env (kN/m)', color='g')
        ax2.set_xlabel('solref 时间常数 (s, log)')

    os.makedirs(save_dir, exist_ok=True)
    path = os.path.join(save_dir, 'calibration_partA.png')
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return path


def plot_partB(runs, save_dir):
    _setup_matplotlib()
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
    for run in runs:
        lab = f"v={run['approach_speed']:.2f} m/s"
        axes[0].plot(run['t'], run['F'], lw=1.2, label=lab)
        axes[1].plot(run['t'], run['pen'] * 1000, lw=1.2, label=lab)
    axes[0].set_ylabel('接触力 F (N)')
    axes[0].legend(); axes[0].grid(alpha=0.3)
    axes[0].set_title('Part B — 动态冲击 (位置伺服, 无导纳/阻抗)')
    axes[1].set_ylabel('压深 (mm)')
    axes[1].set_xlabel('t (s)')
    axes[1].legend(); axes[1].grid(alpha=0.3)

    os.makedirs(save_dir, exist_ok=True)
    path = os.path.join(save_dir, 'calibration_partB.png')
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return path


def print_report(fit, solref_results, partB_runs, gap, ball_radius,
                 tool_radius, robot_name):
    print('\n' + '=' * 70)
    print(f'阶段 0 标定报告 — 接触模型 (robot={robot_name})')
    print('=' * 70)
    print(f'几何: 刚体球 r={ball_radius:.3f} m, 工具尖 r={tool_radius:.3f} m, '
          f'home 间隙={gap * 1000:.1f} mm')
    if fit is not None:
        k, b, r2, mask = fit
        print(f'\n[Part A] 静态压深-接触力 (默认 solref)')
        print(f'  K_env     = {k/1000:8.2f} kN/m  ({k:.1f} N/m)')
        print(f'  激活预载  = {b:8.2f} N   (pen=0 处接触力截距)')
        print(f'  线性 R²   = {r2:.4f}')
        print(f'  拟合点数  = {int(mask.sum())} / {mask.size}')
    print('\n[Part A] 环境刚度扫描 (solref 时间常数 → K_env):')
    for tc, k, b, r2 in solref_results:
        if k is not None:
            print(f'  tc={tc:.4f}s → K_env = {k/1000:8.2f} kN/m '
                  f'(预载 {b:6.1f} N, R²={r2:.4f})')
        else:
            print(f'  tc={tc:.4f}s → 拟合样本不足')

    print('\n[Part B] 动态冲击回跳特性 (位置伺服, 无控制器):')
    hdr = (f'  {"v指令":>7} {"v_impact":>8} {"F_peak(N)":>10} '
           f'{"F_ss(N)":>9} {"超调%":>7} {"断开":>4} {"e":>6} '
           f'{"settle(s)":>9}')
    print(hdr)
    for r in partB_runs:
        m = r['metrics']
        if m is None:
            print(f'  {r["approach_speed"]:>7.2f}  未接触')
            continue
        settle = '—' if m['settle'] is None else f'{m["settle"]:.3f}'
        print(f'  {r["approach_speed"]:>7.2f} {m["v_impact"]:>8.3f} '
              f'{m["F_peak"]:>10.1f} {m["F_ss"]:>9.1f} '
              f'{m["overshoot"]*100:>7.1f} {m["breaks"]:>4} '
              f'{m["e_rest"]:>6.3f} {settle:>9}')
    print('=' * 70)


# ====================================================================
# 主流程
# ====================================================================

def parse_args():
    p = argparse.ArgumentParser(
        description='阶段 0: 接触模型标定 (纯运动学压入, 无控制器)')
    p.add_argument('--robot', type=str, default='ur12e', choices=['ur12e', 'ur3'])
    p.add_argument('--ball-radius', type=float, default=0.12)
    p.add_argument('--ball-pos', type=float, nargs=3, default=None,
                   help='刚体球球心 (默认按 home 位工具正下方自动计算)')
    p.add_argument('--tool-length', type=float, default=0.10)
    p.add_argument('--tool-radius', type=float, default=0.01)
    p.add_argument('--tool-mass', type=float, default=0.05)
    p.add_argument('--gap', type=float, default=0.01,
                   help='home 位工具尖与球面的间隙 (m)')
    p.add_argument('--max-pen', type=float, default=0.010,
                   help='Part A 最大压深 (m)')
    p.add_argument('--solref-times', type=float, nargs='+', default=[0.02, 0.01, 0.005, 0.002],
                   help='环境刚度扫描的 solref 时间常数 (s)')
    p.add_argument('--approach-speeds', type=float, nargs='+',
                   default=[0.05, 0.10, 0.20],
                   help='Part B 逼近速度 (m/s)')
    p.add_argument('--press-pen', type=float, default=0.005,
                   help='Part B 保持段的几何穿透 (m)')
    p.add_argument('--wrist-armature', type=float, default=0.1,
                   help='腕部 dof_armature 电机转子惯量 (kg·m²), None 表示不设')
    p.add_argument('--act-kp', type=float, nargs=6, default=None,
                   help='position actuator 每关节刚度 (Nm/rad)')
    p.add_argument('--act-kv', type=float, nargs=6, default=None,
                   help='position actuator 每关节阻尼 (Nm·s/rad)')
    p.add_argument('--save-dir', type=str, default=None,
                   help='结果图目录 (默认 se3_control/figures/contact)')
    return p.parse_args()


def main():
    args = parse_args()
    cfg = get_robot_config(args.robot)

    # 默认球位: 工具正下方, 球顶在 tool0 下方 (gap+工具半径) 处
    if args.ball_pos is None:
        robot0 = RobotModel(get_urdf_path(args.robot),
                            ee_frame_name=cfg['ee_frame'], verbose=False)
        robot0.update(cfg['home_q'][:robot0.nv])
        p_ee, R_ee = robot0.get_pose()
        # home 下工具竖直朝下 → 工具尖 = p_ee + tool_axis·L
        tool_axis = R_ee @ np.array([0.0, 0.0, 1.0])   # 实际工具轴向 (朝下)
        tip0 = p_ee + tool_axis * args.tool_length
        # 球心: 工具尖沿 tool_axis 方向 (朝向球面) 移 (gap + 球半径 + 尖半径)
        ball_pos = tip0 + tool_axis * (args.gap + args.ball_radius
                                       + args.tool_radius)
        ball_pos = [float(v) for v in ball_pos]
        print(f'[Calib] 自动球位: {[round(v,3) for v in ball_pos]} '
              f'(球心在工具尖下方 {args.gap + args.ball_radius + args.tool_radius:.3f} m)')
    else:
        ball_pos = list(args.ball_pos)

    (model, data, robot, cfg, tip_body_id, tip_geom_id, ball_geom_id,
     ball_center) = build_environment(
        args.robot, ball_pos, args.ball_radius, args.tool_length,
        tool_radius=args.tool_radius, tool_mass=args.tool_mass,
        act_kp=args.act_kp, act_kv=args.act_kv,
        wrist_armature=args.wrist_armature)

    gap = args.gap
    tip_radius = args.tool_radius

    # ── Part A: 静态压深 (默认 solref) ──
    pen_actual, F, pen_cmd = static_sweep(
        model, data, robot, cfg, tip_body_id, tip_geom_id, ball_center,
        args.ball_radius, tip_radius, gap, max_pen=args.max_pen)
    fit = fit_k_env(pen_actual, F)

    # ── Part A: 环境刚度扫描 ──
    solref_results = sweep_env_stiffness(
        model, data, robot, cfg, tip_body_id, tip_geom_id, ball_center,
        ball_geom_id, args.ball_radius, tip_radius, gap,
        args.solref_times, max_pen=args.max_pen)
    # sweep 把球 solref 留在了最硬的 tc; 恢复默认, 让 Part B 用基准 K_env
    model.geom_solref[ball_geom_id, :] = np.array([0.02, 1.0])

    # ── Part B: 动态冲击 (扫逼近速度) ──
    partB_runs = []
    for v in args.approach_speeds:
        r = dynamic_press(model, data, robot, cfg, tip_body_id, tip_geom_id,
                          ball_center, args.ball_radius, tip_radius,
                          args.tool_length,
                          approach_speed=v, press_pen=args.press_pen)
        r['approach_speed'] = v
        partB_runs.append(r)
        if r['metrics'] is not None:
            m = r['metrics']
            print(f'[PartB] v={v:.2f} m/s: t_contact={m["t_contact"]:.3f}s, '
                  f'v_impact={m["v_impact"]:.3f} m/s, '
                  f'F_peak={m["F_peak"]:.1f} N, 断开={m["breaks"]}, '
                  f'e={m["e_rest"]:.3f}')

    # ── 报告 + 图 ──
    save_dir = args.save_dir or os.path.join(
        _PROJECT_ROOT, 'se3_control', 'figures', 'contact')
    print_report(fit, solref_results, partB_runs, gap, args.ball_radius,
                 tip_radius, args.robot)
    pA = plot_partA(pen_actual, F, fit, pen_cmd, solref_results, save_dir)
    pB = plot_partB(partB_runs, save_dir)
    print(f'[Figures] {pA}')
    print(f'[Figures] {pB}')


if __name__ == '__main__':
    main()
