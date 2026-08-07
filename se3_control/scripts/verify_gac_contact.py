#!/usr/bin/env python
"""阶段 2: GAC 压入接触 — Fe_raw 力反馈 + K_env × τ_delay 稳定域扫描.

计划 docs/plan/force_interaction_experiments_plan.md 附录 A.9 阶段 2:
  "GAC 压入: 从软 K_env 逐步加硬, 找失稳边界; 加延迟建模, 重扫稳定域,
   出硬件安全区间" (步骤 3-4), 以及 §3.1 的"动态接触刚度是接触稳定的主变量".

控制: **GAC 导纳 + 力反馈** (Fe_raw 接入). 接触力来自 ``ee_force`` 传感器
  (真实部署形态: 力传感器回读), Rᵀ 转到体坐标 → ``GACFilter`` → 轨迹修正
  → SE(3) 位置跟踪. 这是"位置内环 + 力外环 + 传感器延迟"的经典失稳结构本体,
  对应部署计划 M3 (FT 集成).

两个模式:
  1. **单次运行 (默认)**: 与阶段 1 相同的五相位全流程
     (逼近/接触/表面摩擦/离开/保持), 但 GAC 力反馈. 默认摩擦幅值
     θ_amp=0.12 > 阶段 1 GIC 被动上限 0.08 —— 验证 GAC 力反馈可突破
     GIC 被动的摩擦面积上限 (>0.87 cm²).
  2. **稳定域扫描 (--sweep)**: 扫 K_env (球 solref 时间常数 tc) ×
     τ_delay (FT 延迟), 每个点做 S1 垂直压入 + 保持, 分类
     稳定 / 极限环 / 发散. 横轴用 **§3.1 动态刚度标尺**
     (K_env_dyn = Fn_ss/pen_ss 在保持窗口实测, mj_step 动态),
     纵轴 τ_delay, 出硬件安全区间图.

用法示例:
  python se3_control/scripts/verify_gac_contact.py                  # 单次: 大摩擦面积
  python se3_control/scripts/verify_gac_contact.py --tau-delay 0.01 # 单次 + FT 延迟
  python se3_control/scripts/verify_gac_contact.py --sweep --no-viewer  # 稳定域扫描
  python se3_control/scripts/verify_gac_contact.py --sweep --no-viewer \
      --sweep-solref 1.0 0.5 0.2 0.1 0.05 0.02 --sweep-delay 0 0.005 0.01 0.02

产物 (se3_control/figures/contact/):
  单次: gac_contact.png / gac_contact_rub.png / gac_contact_surface.png
  扫描: gac_contact_stability.png (K_env_dyn × τ_delay 稳定域图) + .json
"""

import argparse
import math
import os
import sys
from collections import deque

import numpy as np

# ── 路径注入 (与 verify_gic_contact.py 相同的约定) ──
_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
for _p in (_PROJECT_ROOT,
           os.path.join(_PROJECT_ROOT, 'se3_control'),
           os.path.join(_PROJECT_ROOT, 'se3_control', 'scripts')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import mujoco

# 阶段 1 已建好的控制器无关设施: 环境构建 / 参数化球面轨迹 / 量测 / 指标
from verify_gic_contact import (
    build_environment, TipContactTrajectory,
    contact_force_vec, force_components,
    contact_establish_metrics, rub_metrics, depart_metrics,
    DEFAULT_WRIST_ARMATURE,
)
from verify_contact_calibration import (
    tool_tip_pos, penetration_from_geom, F_OFF, _setup_matplotlib,
)
from config.robot_configs import get_robot_config, get_urdf_path
from robot_model import RobotModel
from core.gac_controller import GACController


# ====================================================================
# S1 压入 + 保持轨迹 (K_env 扫描用, 只有 approach → hold)
# ====================================================================

class PressHoldTrajectory:
    """S1 垂直压入 + 保持: 沿球面法向匀速逼近, 接触后保持压深.

    (A.6 场景矩阵 S1/S4: 固定逼近速度扫 K_env → 稳定域边界.)
    只有 approach / hold 两个相位, 是稳定域判定的最小场景.
    """

    PHASES = ['approach', 'hold']

    def __init__(self, ball_center, r_start, r_des, approach_speed=0.006,
                 hold_time=3.0, dt=0.001):
        self.c = np.asarray(ball_center, dtype=float)
        self.r_start = float(r_start)
        self.r_des = float(r_des)
        self.approach_speed = float(approach_speed)
        self.tau_app = min(0.3, (self.r_start - self.r_des)
                           / max(self.approach_speed, 1e-6) / 4.0)
        self.t1 = (self.tau_app / 2.0 + (self.r_start - self.r_des)
                   / max(self.approach_speed, 1e-6))
        self.t_hold = float(hold_time)
        self.t_end = self.t1 + self.t_hold
        self.dt = float(dt)
        self.T = int(math.ceil(self.t_end / self.dt))

    def phase_at(self, t):
        return 'approach' if t <= self.t1 else 'hold'

    def eval(self, t):
        if t <= self.t1:                       # approach
            if t <= self.tau_app:              # 平滑加速段 (C²)
                r = (self.r_start - self.approach_speed
                     * (t - self.tau_app / (2.0 * math.pi)
                        * math.sin(2.0 * math.pi * t / self.tau_app)))
                rd = (-self.approach_speed
                      * (1.0 - math.cos(2.0 * math.pi * t / self.tau_app)))
                rdd = (-self.approach_speed * (2.0 * math.pi / self.tau_app)
                       * math.sin(2.0 * math.pi * t / self.tau_app))
            else:                              # 匀速巡航
                r = self.r_start - self.approach_speed * (t - self.tau_app / 2.0)
                rd = -self.approach_speed
                rdd = 0.0
        else:                                  # hold (接触保持)
            r, rd, rdd = self.r_des, 0.0, 0.0
        n = np.array([0.0, 0.0, 1.0])          # 极角 0 = 球顶正上方
        p = self.c + r * n
        v = rd * n
        a = rdd * n
        return p, v, a, self.phase_at(t)


# ====================================================================
# 环境初始化 + 力反馈延迟线
# ====================================================================

def init_contact_env(robot_name, ball_pos, ball_radius, tool_length,
                     tool_radius, tool_mass, wrist_armature,
                     ball_friction=0.15, tool_friction=0.15,
                     ball_solref=(1.0, 1.0), ball_solimp=None):
    """构建带接触环境的 MuJoCo 模型 + RobotModel, 并摆到 home.

    :returns: SimpleNamespace(model, data, robot, cfg, tip_body_id,
              tip_geom_id, ball_center, R0, L, r_start, nv, torque_lim)
    """
    from types import SimpleNamespace
    (model, data, robot, cfg, tip_body_id, tip_geom_id, ball_geom_id,
     ball_center) = build_environment(
        robot_name, ball_pos, ball_radius, tool_length,
        tool_radius=tool_radius, tool_mass=tool_mass, force_sensor=True,
        dt=0.001, wrist_armature=wrist_armature,
        ball_friction=ball_friction, tool_friction=tool_friction,
        ball_solref=ball_solref, ball_solimp=ball_solimp)
    nv = robot.nv
    home_q = cfg['home_q'][:nv]
    data.qpos[:nv] = home_q.copy()
    data.qvel[:nv] = np.zeros(nv)
    mujoco.mj_forward(model, data)
    robot.update(home_q)
    p0, R0 = robot.get_pose()
    tip0 = tool_tip_pos(data, tip_body_id)
    r_start = float(np.linalg.norm(tip0 - ball_center))
    return SimpleNamespace(
        model=model, data=data, robot=robot, cfg=cfg,
        tip_body_id=tip_body_id, tip_geom_id=tip_geom_id,
        ball_geom_id=ball_geom_id,
        ball_center=ball_center, R0=R0, L=float(tool_length),
        r_start=r_start, nv=nv,
        torque_lim=cfg['full_torque_limits'][:nv])


class FTDelayLine:
    """FT 传感器传输延迟线: 返回 n_delay 个控制周期前的 F_ext 读数.

    :param n_delay: 延迟步数 (τ_delay / dt 取整), 0 = 理想 (零延迟).
    """

    def __init__(self, n_delay: int):
        self.n_delay = int(max(n_delay, 0))
        self._buf = deque(maxlen=self.n_delay + 1)

    def __call__(self, F_ext_body: np.ndarray) -> np.ndarray:
        self._buf.append(np.asarray(F_ext_body, dtype=float).ravel())
        if len(self._buf) > self.n_delay:
            return self._buf[0].copy()
        return np.asarray(F_ext_body, dtype=float).ravel()


# ====================================================================
# 通用仿真核心 (GAC 力反馈闭环)
# ====================================================================

def run_loop(env, ctrl, traj, tau_delay, show_viewer=False, verbose=True):
    """沿 traj 运行 GAC 力反馈闭环 (环境已由 init_contact_env 建好).

    力反馈通路 (真实部署形态, 计划 §5.2 / A.4):
      ee_force 传感器 (世界系) → R_curᵀ → 体坐标系 → τ_delay 延迟线
      → GACFilter → 轨迹修正 → SE(3) 跟踪.

    :returns: (log dict, t_contact)
    """
    model, data = env.model, env.data
    R0, L = env.R0, env.L
    nv = env.nv
    delay = FTDelayLine(int(round(tau_delay / 0.001)))

    viewer = None
    if show_viewer:
        try:
            from mujoco.viewer import launch_passive
            viewer = launch_passive(model, data)
            import time as _t
            _t.sleep(0.3)
        except Exception as e:
            print(f'[Viewer] 启动失败: {e}')
            show_viewer = False

    T = traj.T
    log = {
        't': np.zeros(T), 'phase': np.full(T, '', dtype=object),
        'tip': np.zeros((T, 3)), 'tip_des': np.zeros((T, 3)),
        'pen': np.zeros(T), 'F': np.zeros(T),
        'Fn': np.zeros(T), 'Ft': np.zeros(T),
        'F_sensor': np.zeros(T),          # 传感器回读 (世界系力大小)
        'F_ext': np.zeros((T, 6)),        # 进控制器前 (延迟后体坐标)
        'x_corr': np.zeros((T, 6)),       # 导纳滤波器输出
        'tau': np.zeros((T, nv)),
    }
    t_contact = None
    for i in range(T):
        t = i * model.opt.timestep
        tip_des, v_tip, a_tip, phase = traj.eval(t)
        pd = tip_des - R0 @ np.array([0.0, 0.0, L])
        Rd = R0
        vd = Rd.T @ v_tip
        wd = np.zeros(3)
        dvd = Rd.T @ a_tip
        dwd = np.zeros(3)

        q = data.qpos[:nv].copy()
        dq = data.qvel[:nv].copy()

        # ── 力传感器回读 (世界系) → 体坐标 → 延迟 → GACFilter ──
        F_sensor_world = data.sensordata[0:3].copy()
        T_sensor_world = data.sensordata[3:6].copy()
        R_cur = data.site_xmat[0].reshape(3, 3)
        F_ext_body = np.concatenate((R_cur.T @ F_sensor_world,
                                     R_cur.T @ T_sensor_world))
        F_ext_ctrl = delay(F_ext_body)

        tau = ctrl.compute(q, dq, pd, Rd, vd, wd, dvd, dwd, F_ext=F_ext_ctrl)
        data.ctrl[:] = tau[:model.nu]
        mujoco.mj_step(model, data)

        tip = data.xpos[env.tip_body_id].copy()
        # 用实际球/尖半径 (geom size), 与轨迹 R_eff 一致
        ball_r = env.model.geom_size[env.ball_geom_id][0]
        tip_r = env.model.geom_size[env.tip_geom_id][0]
        pen = penetration_from_geom(tip, env.ball_center, ball_r, tip_r)
        Fvec = contact_force_vec(model, data, env.tip_geom_id)
        F = float(np.linalg.norm(Fvec))
        Fn, Ft = force_components(Fvec, tip, env.ball_center)
        F_sensor = float(np.linalg.norm(F_sensor_world))

        if t_contact is None and pen > 1e-4:
            t_contact = t

        log['t'][i] = t
        log['phase'][i] = phase
        log['tip'][i] = tip
        log['tip_des'][i] = tip_des
        log['pen'][i] = pen
        log['F'][i] = F
        log['Fn'][i] = Fn
        log['Ft'][i] = Ft
        log['F_sensor'][i] = F_sensor
        log['F_ext'][i] = F_ext_ctrl
        log['x_corr'][i] = ctrl.filter_state['X_corr']
        log['tau'][i] = tau

        if viewer is not None and i % 20 == 0:
            viewer.sync()
        if verbose and i % 1000 == 0:
            print(f'  t={t:6.2f}s [{phase:>8}] F={F:6.1f}N  '
                  f'pen={pen*1000:5.1f}mm  '
                  f'|x_corr|={np.linalg.norm(log["x_corr"][i])*1000:5.1f}mm')

    if viewer is not None:
        viewer.close()
    return log, t_contact


# ====================================================================
# 单次运行: 五相位 GAC 压入 (突破 GIC 摩擦面积上限)
# ====================================================================

def run_gac_contact(robot_name, ball_pos, ball_radius, tool_length,
                    tool_radius, tool_mass, delta_pen, approach_speed,
                    settle_time, theta_amp, phi_amp, rub_cycles, phi_cycles,
                    rub_mode, rub_duration, depart_speed,
                    bandwidth, damping, M_d, D_d, K_d, max_correction,
                    tau_delay, wrist_armature, save_dir,
                    show_viewer=False, verbose=True,
                    ball_friction=0.15, tool_friction=0.15,
                    ball_solref=(1.0, 1.0), ball_solimp=None):
    """GAC 力反馈压入 + 2D 表面摩擦 + 抬离全流程 (单次运行).

    摩擦幅值默认 θ_amp=0.12 (> 阶段 1 GIC 被动 0.08 上限) → 摩擦斑块面积
    ≈ φ·R²·θ² ≈ 1.9 cm², 验证 GAC 力反馈突破 GIC 被动摩擦面积上限.
    """
    env = init_contact_env(
        robot_name, ball_pos, ball_radius, tool_length,
        tool_radius, tool_mass, wrist_armature,
        ball_friction=ball_friction, tool_friction=tool_friction,
        ball_solref=ball_solref, ball_solimp=ball_solimp)
    R_eff = ball_radius + tool_radius
    r_des = R_eff - delta_pen

    traj = TipContactTrajectory(
        env.ball_center, env.r_start, r_des,
        approach_speed=approach_speed, settle_time=settle_time,
        theta_amp=theta_amp, phi_amp=phi_amp,
        rub_cycles=rub_cycles, phi_cycles=phi_cycles, rub_mode=rub_mode,
        rub_duration=rub_duration, depart_speed=depart_speed, dt=0.001)

    if verbose:
        print(f'[GAC] 球心={np.round(env.ball_center,3)}  R_eff={R_eff:.3f}  '
              f'r_des={r_des:.3f} (δ_pen={delta_pen:.4f}m)  '
              f'r_start={env.r_start:.3f}')
        print(f'[GAC] 相位边界: 逼近→{traj.t1:.2f}s 保持→{traj.t2:.2f}s  '
              f'摩擦→{traj.t3:.2f}s 离开→{traj.t4:.2f}s 结束→{traj.t_end:.2f}s')
        area = phi_amp * R_eff**2 * theta_amp**2 * 1e4
        print(f'[GAC] rub({rub_mode}): θ_amp={theta_amp:.3f} rad × '
              f'φ_amp={phi_amp:.3f} rad → 摩擦斑块面积 ≈ {area:.2f} cm² '
              f'(GIC 被动上限 0.87)')
        print(f'[GAC] 控制器: GAC 力反馈 (Fe_raw 传感器回读), '
              f'内环 ω={bandwidth}/{damping}, 导纳 K_d={np.round(K_d,1)}, '
              f'M_d={np.round(M_d,1)}, τ_delay={tau_delay*1000:.1f} ms')

    ctrl = GACController(env.robot, M_d=M_d, D_d=D_d, K_d=K_d, dt=0.001,
                         bandwidth=bandwidth, damping=damping,
                         torque_limits=env.cfg['full_torque_limits'],
                         max_correction=max_correction)
    log, t_contact = run_loop(env, ctrl, traj, tau_delay,
                              show_viewer=show_viewer, verbose=verbose)

    lim = env.torque_lim
    report = {}
    if t_contact is not None:
        report['contact'] = contact_establish_metrics(
            log['t'], log['pen'], log['F'], t_contact, traj.t2)
    report['rub'] = rub_metrics(
        log['t'], log['pen'], log['F'], log['Fn'], traj.t2, traj.t3)
    report['depart'] = depart_metrics(log['t'], log['F'], traj.t3)
    seg_app = (log['t'] <= traj.t1) & (log['pen'] < 1e-4)
    if np.any(seg_app):
        report['approach_err'] = float(np.linalg.norm(
            log['tip'][seg_app] - log['tip_des'][seg_app], axis=1).max())
    tau_max = np.abs(log['tau']).max(axis=0)
    report['torque_ok'] = bool(np.all(tau_max <= lim * 0.999))
    report['torque_max'] = tau_max
    report['stable'] = bool(np.isfinite(log['F']).all()
                            and log['F'].max() < 5e3)
    report['t_contact'] = t_contact
    report['phases'] = (traj.t1, traj.t2, traj.t3, traj.t4, traj.t_end)
    report['R_eff'] = R_eff
    report['r_des'] = r_des
    report['c'] = env.ball_center
    report['theta_amp'] = theta_amp
    report['phi_amp'] = phi_amp
    report['area_cm2'] = phi_amp * R_eff**2 * theta_amp**2 * 1e4
    report['tau_delay'] = tau_delay

    paths = plot_gac_contact(log, report, save_dir) if save_dir else []
    return log, report, paths


# ====================================================================
# 稳定域扫描: K_env (solref) × τ_delay
# ====================================================================

def classify_press_hold(log, hold_start, hold_end, max_correction):
    """S1 压入保持的稳定性分类 (A.7 硬门槛, 一票否决).

    :returns: 'stable' | 'limit_cycle' | 'diverged' | 'no_contact'
    """
    t = log['t']
    F = log['F']
    x = log['x_corr']
    if not np.isfinite(F).all() or F.max() > 5e3:
        return 'diverged'
    seg = (t >= hold_start) & (t <= hold_end)
    if not np.any(seg):
        return 'no_contact'
    Fw = F[seg]
    F_mean = float(Fw.mean())
    if F_mean < F_OFF:
        return 'no_contact'
    # X_corr 顶到限幅 → 力外环饱和发散前兆
    if np.any(np.abs(x[seg]).max(axis=1) >= max_correction * 0.99):
        return 'diverged'
    # 保持窗口后 40% 的振荡幅值 (极限环判据, A.7 > 5% F_ss)
    n = Fw.size
    tail = Fw[int(0.6 * n):]
    if tail.size < 3:
        return 'stable'
    pp = float((tail.max() - tail.min()) / max(F_mean, 1e-9))
    cv = float(Fw.std() / max(F_mean, 1e-9))
    if pp > 0.10 or cv > 0.15:
        return 'limit_cycle'
    # 回跳: 断开后重新建立次数 > 2 → 接触反复
    on = Fw >= F_OFF
    breaks = sum(1 for k in range(1, n) if on[k - 1] and not on[k])
    if breaks > 2:
        return 'limit_cycle'
    return 'stable'


def dynamic_k_env(log, hold_start, hold_end):
    """§3.1 动态刚度标尺: K_env_dyn = Fn_ss / pen_ss (保持窗口稳态段).

    用 mj_step 动态接触的实际稳态力/压深比, 捕捉近零压深隐式硬化.
    """
    seg = (log['t'] >= hold_start + 0.4 * (hold_end - hold_start)) \
          & (log['t'] <= hold_end)
    if not np.any(seg):
        return float('nan')
    Fn = log['Fn'][seg]
    pen = log['pen'][seg]
    pen_mean = float(pen.mean())
    if pen_mean < 1e-6:
        return float('nan')
    return float(Fn.mean()) / pen_mean


def run_stability_sweep(robot_name, ball_pos, ball_radius, tool_length,
                        tool_radius, tool_mass, delta_pen, approach_speed,
                        hold_time, bandwidth, damping, M_d, D_d, K_d,
                        max_correction, solref_tcs, delays,
                        wrist_armature, save_dir, verbose=True,
                        ball_friction=0.15, tool_friction=0.15):
    """扫 solref_tc × τ_delay → 每点 S1 压入 + 保持, 分类稳定/极限环/发散.

    :returns: (points, fig_paths)
    """
    R_eff = ball_radius + tool_radius
    r_des = R_eff - delta_pen

    points = []
    n_total = len(solref_tcs) * len(delays)
    k = 0
    for tc in solref_tcs:
        for td in delays:
            k += 1
            env = init_contact_env(
                robot_name, ball_pos, ball_radius, tool_length,
                tool_radius, tool_mass, wrist_armature,
                ball_friction=ball_friction, tool_friction=tool_friction,
                ball_solref=(tc, 1.0), ball_solimp=None)
            traj = PressHoldTrajectory(env.ball_center, env.r_start, r_des,
                                       approach_speed=approach_speed,
                                       hold_time=hold_time, dt=0.001)
            if verbose:
                print(f'\n[{k}/{n_total}] solref_tc={tc:.4f}  '
                      f'τ_delay={td*1000:.1f}ms  '
                      f'(t1={traj.t1:.2f}s, end={traj.t_end:.2f}s)')
            ctrl = GACController(env.robot, M_d=M_d, D_d=D_d, K_d=K_d,
                                 dt=0.001, bandwidth=bandwidth,
                                 damping=damping,
                                 torque_limits=env.cfg['full_torque_limits'],
                                 max_correction=max_correction)
            try:
                log, t_contact = run_loop(env, ctrl, traj, td,
                                          show_viewer=False, verbose=False)
            except Exception as e:
                print(f'  [跳过] 仿真失败: {e}')
                points.append({'tc': tc, 'tau_delay': td, 'class': 'diverged',
                               'K_env_dyn': float('nan'), 'F_ss': float('nan'),
                               't_contact': None})
                continue
            cls = classify_press_hold(log, traj.t1, traj.t_end, max_correction)
            K_env = dynamic_k_env(log, traj.t1, traj.t_end)
            seg_hold = (log['t'] >= traj.t1) & (log['t'] <= traj.t_end)
            F_ss = float(log['F'][seg_hold].mean()) if np.any(seg_hold) \
                else float('nan')
            points.append({'tc': tc, 'tau_delay': td, 'class': cls,
                           'K_env_dyn': K_env, 'F_ss': F_ss,
                           't_contact': t_contact})
            if verbose:
                ks = f'{K_env/1e3:.1f} kN/m' if np.isfinite(K_env) else '   n/a'
                print(f'  → class={cls}  K_env_dyn={ks}  F_ss={F_ss:.1f} N')

    if save_dir:
        path = plot_stability_map(points, save_dir)
        paths = [path]
    else:
        paths = []
    return points, paths


# ====================================================================
# 绘图
# ====================================================================

def _mark_phases(ax, t1, t2, t3, t4):
    for tx, lab in ((t1, '接触'), (t2, '摩擦'), (t3, '离开'), (t4, '保持')):
        ax.axvline(tx, color='gray', ls=':', lw=0.7)
        ax.text(tx, ax.get_ylim()[1], lab, fontsize=7, rotation=90,
                va='top', ha='right', color='gray')


def plot_gac_contact(log, report, save_dir):
    _setup_matplotlib()
    import matplotlib.pyplot as plt

    t = log['t']
    t1, t2, t3, t4, tend = report['phases']
    F_ss = report.get('contact', {}).get('F_ss')
    band = 0.10 * F_ss if F_ss else None

    fig, axes = plt.subplots(5, 1, figsize=(10, 13), sharex=True)

    # 1. 接触力 + 稳态带
    ax = axes[0]
    ax.plot(t, log['F'], 'b-', lw=1.1, label='|F_contact|')
    ax.plot(t, log['F_sensor'], 'g--', lw=0.8, alpha=0.6, label='ee_force 传感器')
    if F_ss:
        ax.axhline(F_ss, color='r', ls='--', lw=0.8,
                   label=f'F_ss={F_ss:.1f}N')
        ax.fill_between(t, F_ss - band, F_ss + band, color='r', alpha=0.08)
    _mark_phases(ax, t1, t2, t3, t4)
    ax.set_ylabel('接触力 F (N)')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(alpha=0.3)
    ax.set_title(f'GAC 压入接触 (Fe_raw 力反馈, τ_delay={report["tau_delay"]*1000:.0f}ms)')

    # 2. 压深
    ax = axes[1]
    ax.plot(t, log['pen'] * 1000, 'b-', lw=1.1)
    ax.axhline(0, color='k', lw=0.5)
    _mark_phases(ax, t1, t2, t3, t4)
    ax.set_ylabel('压深 pen (mm)')
    ax.grid(alpha=0.3)

    # 3. 法向/切向接触力
    ax = axes[2]
    ax.plot(t, log['Fn'], 'r-', lw=1.0, label='|F_normal|')
    ax.plot(t, log['Ft'], 'm-', lw=1.0, label='|F_tangent|')
    _mark_phases(ax, t1, t2, t3, t4)
    ax.set_ylabel('力分量 (N)')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(alpha=0.3)

    # 4. 导纳滤波器输出 |X_corr| (力反馈修正量 — GAC 新增信号)
    ax = axes[3]
    ax.plot(t, np.linalg.norm(log['x_corr'], axis=1) * 1000, 'c-', lw=1.0,
            label='|X_corr|')
    _mark_phases(ax, t1, t2, t3, t4)
    ax.set_ylabel('|X_corr| (mm)')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(alpha=0.3)

    # 5. 关节力矩 (max)
    ax = axes[4]
    tau_max = np.abs(log['tau']).max(axis=1)
    ax.plot(t, tau_max, 'k-', lw=0.9, label='max|τ|')
    lim = log['tau_lim'] if 'tau_lim' in log else None
    if lim is not None and np.isfinite(lim).all():
        ax.axhline(lim.max(), color='r', ls='--', lw=0.8, label='min 限幅')
    _mark_phases(ax, t1, t2, t3, t4)
    ax.set_xlabel('t (s)')
    ax.set_ylabel('max|τ| (Nm)')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(alpha=0.3)

    os.makedirs(save_dir, exist_ok=True)
    path = os.path.join(save_dir, 'gac_contact.png')
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)

    path2 = _plot_rub_tangent(log, report, save_dir, 'gac_contact_rub.png')
    path3 = _plot_surface_traj(log, report, save_dir, 'gac_contact_surface.png')

    return [path, path2, path3]


def _plot_rub_tangent(log, report, save_dir, fname):
    _setup_matplotlib()
    import matplotlib.pyplot as plt
    c = report['c']
    R = report['R_eff']
    theta_amp = report.get('theta_amp', 0.08)
    seg = log['phase'] == 'rub'
    tr = log['t'][seg]
    tip = log['tip'][seg]
    top = c + np.array([0.0, 0.0, R])
    d = tip - top
    fig, ax = plt.subplots(figsize=(6.5, 6.5))
    sc = ax.scatter(d[:, 0], d[:, 1], c=tr, cmap='viridis', s=6, linewidths=0)
    thc = np.linspace(0, 2 * np.pi, 100)
    r_patch = R * max(theta_amp, 1e-4)
    ax.plot(r_patch * np.sin(thc), r_patch * np.cos(thc), 'k--', lw=0.9,
            alpha=0.6, label=f'摩擦范围 (r=R·θ_amp={r_patch*1e3:.0f} mm)')
    ax.plot(0, 0, 'r+', ms=14, mew=2, label='球顶 (接触中心)')
    ax.set_xlabel('x 切向偏移 (m)')
    ax.set_ylabel('y 切向偏移 (m)')
    ax.set_title('GAC 摩擦段接触点轨迹 (球顶切平面俯视, 按时间着色)')
    ax.legend(fontsize=8); ax.grid(alpha=0.3); ax.axis('equal')
    cb = fig.colorbar(sc, ax=ax, shrink=0.8, label='t (s)')
    _ = cb
    path = os.path.join(save_dir, fname)
    fig.tight_layout(); fig.savefig(path, dpi=120); plt.close(fig)
    return path


def _plot_surface_traj(log, report, save_dir, fname):
    _setup_matplotlib()
    import matplotlib.pyplot as plt
    c = report['c']
    R = report['R_eff']
    theta_amp = report.get('theta_amp', 0.08)
    fig = plt.figure(figsize=(7.5, 7))
    ax = fig.add_subplot(111, projection='3d')
    uu = np.linspace(0, 2 * np.pi, 40)
    vv = np.linspace(0, np.pi, 20)
    xs = c[0] + R * np.outer(np.sin(uu), np.sin(vv))
    ys = c[1] + R * np.outer(np.cos(uu), np.sin(vv))
    zs = c[2] + R * np.outer(np.ones_like(uu), np.cos(vv))
    ax.plot_wireframe(xs, ys, zs, color='k', alpha=0.12, lw=0.4)
    seg = log['phase'] == 'rub'
    tr = log['t'][seg]
    tip = log['tip'][seg]
    step = max(int(tip.shape[0] // 3000), 1)
    sc = ax.scatter(tip[::step, 0], tip[::step, 1], tip[::step, 2],
                    c=tr[::step], cmap='viridis', s=8, depthshade=False)
    cb = fig.colorbar(sc, ax=ax, shrink=0.55, pad=0.1)
    cb.set_label('t (s)')
    for ph, col, lab in (('approach', 'b', '逼近'), ('depart', 'r', '抬离')):
        m = log['phase'] == ph
        if np.any(m):
            ax.plot(log['tip'][m, 0], log['tip'][m, 1], log['tip'][m, 2],
                    color=col, lw=2.0, alpha=0.9, label=lab)
    ax.scatter(*c, marker='+', s=120, color='k', label='球心')
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.legend(fontsize=8)
    ax.set_title('GAC 球面接触点轨迹 3D (摩擦段按时间着色)')
    top = c + np.array([0.0, 0.0, R])
    span = max(0.05, 3.5 * R * max(theta_amp, 1e-3))
    ax.set_xlim(top[0] - span, top[0] + span)
    ax.set_ylim(top[1] - span, top[1] + span)
    ax.set_zlim(top[2] - span, top[2] + span)
    try:
        ax.set_box_aspect((1, 1, 1))
    except Exception:
        pass
    path = os.path.join(save_dir, fname)
    fig.tight_layout(); fig.savefig(path, dpi=120); plt.close(fig)
    return path


_CLASS_STYLE = {
    'stable':      ('green', 'o'),
    'limit_cycle': ('orange', '^'),
    'diverged':    ('red', 'x'),
    'no_contact':  ('gray', 's'),
}


def plot_stability_map(points, save_dir):
    """K_env_dyn (x, log) × τ_delay (y) 稳定域图: 稳定/极限环/发散."""
    _setup_matplotlib()
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(9, 6))
    shown = set()
    for p in points:
        k = p['K_env_dyn']
        if not np.isfinite(k):
            continue
        col, mark = _CLASS_STYLE.get(p['class'], ('gray', 's'))
        lab = p['class'] if p['class'] not in shown else None
        shown.add(p['class'])
        ax.scatter(k / 1e3, p['tau_delay'] * 1000, c=col, marker=mark,
                   s=70, zorder=3, edgecolors='k', linewidths=0.5, label=lab)

    # 稳定边界: 每个 τ_delay 的最大稳定 K_env (折线)
    delays = sorted({p['tau_delay'] for p in points})
    bx, by = [], []
    for td in delays:
        st = [p for p in points if p['tau_delay'] == td
              and p['class'] == 'stable' and np.isfinite(p['K_env_dyn'])]
        if st:
            bx.append(max(p['K_env_dyn'] / 1e3 for p in st))
            by.append(td * 1000)
    if bx:
        ax.plot(bx, by, 'k--', lw=1.4, alpha=0.8,
                label='稳定边界 (max 稳定 K_env)')

    x_vals = [p['K_env_dyn'] / 1e3 for p in points
              if np.isfinite(p['K_env_dyn'])]
    if x_vals:
        ax.axvspan(0, min(x_vals), color='green', alpha=0.07,
                   label='扫描最软端')
    ax.set_xscale('log')
    ax.set_xlabel('动态环境刚度 K_env_dyn = Fn_ss/pen_ss (kN/m, log) — §3.1 动态标尺')
    ax.set_ylabel('FT 传感器延迟 τ_delay (ms)')
    ax.set_title('GAC 压入稳定域: K_env × τ_delay (S1 垂直压入 + 保持)')
    ax.grid(alpha=0.3, which='both')
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys(), fontsize=8, loc='lower left')
    os.makedirs(save_dir, exist_ok=True)
    path = os.path.join(save_dir, 'gac_contact_stability.png')
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path


# ====================================================================
# 报告
# ====================================================================

def print_report(report, robot_name):
    print('\n' + '=' * 70)
    print(f'阶段 2 报告 — GAC 压入接触 (robot={robot_name}, '
          f'Fe_raw 力反馈, τ_delay={report["tau_delay"]*1000:.0f}ms)')
    print('=' * 70)
    t1, t2, t3, t4, tend = report['phases']
    print(f'相位: 逼近[0,{t1:.2f}] 接触保持[{t1:.2f},{t2:.2f}] '
          f'摩擦[{t2:.2f},{t3:.2f}] 离开[{t3:.2f},{t4:.2f}] 保持[{t4:.2f},{tend:.2f}]')
    if report.get('t_contact') is not None:
        print(f'首次接触 t={report["t_contact"]:.3f}s')
    print(f'摩擦斑块面积 ≈ {report["area_cm2"]:.2f} cm² '
          f'(GIC 被动上限 0.87 cm²)')
    print(f'\n[稳定性] 全程力有界: {"✓" if report["stable"] else "✗ 发散"}')

    print('\n[逼近] 自由空间最大轨迹误差: '
          f'{report.get("approach_err", float("nan"))*1000:.2f} mm')
    if report.get('contact'):
        c = report['contact']
        settle = '—' if c['settle'] is None else f'{c["settle"]:.3f}s'
        print('\n[接触建立] (A.8: 反弹小、稳定)')
        print(f'  F_peak   = {c["F_peak"]:7.1f} N')
        print(f'  F_ss     = {c["F_ss"]:7.1f} N')
        print(f'  超调     = {c["overshoot"]*100:6.1f} %   '
              f'(<30% {"✓" if c["overshoot"] < 0.30 else "✗"})')
        print(f'  断开次数 = {c["breaks"]}   (理想 = 1 次建立不回跳)')
        print(f'  调节时间 = {settle}   '
              f'(<1s {"✓" if (c["settle"] is not None and c["settle"] < 1.0) else "✗"})')
    if report.get('rub'):
        r = report['rub']
        print('\n[表面摩擦] (A.8: 不掉球、无极限环)')
        print(f'  F 均值/最小值 = {r["F_mean"]:6.1f} / {r["F_min"]:6.1f} N')
        print(f'  F 变异系数   = {r["F_cv"]*100:5.1f} %   '
              f'(<10% {"✓" if r["F_cv"] < 0.10 else "✗"})')
        print(f'  法向力变异系数 = {r["Fn_cv"]*100:5.1f} %')
        print(f'  最小压深     = {r["min_pen"]*1000:5.1f} mm   '
              f'(未脱离 {"✓" if r["min_pen"] > 1e-4 else "✗ 脱离"})')
        print(f'  脱离时长     = {r["detach_time"]*1000:.0f} ms')
        print(f'  末段力峰-峰  = {r["pp_tail_rel"]*100:5.1f} % F_mean   '
              f'(极限环判据)')
    if report.get('depart'):
        d = report['depart']
        t_off = '—' if d['t_to_off'] is None else f'{d["t_to_off"]*1000:.0f}ms'
        print('\n[离开] (A.8: 抬离干净、无振铃)')
        print(f'  力归零耗时 = {t_off}')
        print(f'  再次误碰   = {d["recontact"]}   '
              f'({"✓ 无" if d["recontact"] == 0 else "✗ 有"})')
        print(f'  离开后尾段 max F = {d["tail_F_max"]:.2f} N   '
              f'({"✓ 干净" if d["tail_F_max"] < F_OFF else "✗ 振铃"})')
    tm = report['torque_max']
    print(f'\n[力矩] max|τ| = {np.round(tm,1)} Nm, 饱和: '
          f'{"✓ 无" if report["torque_ok"] else "✗ 饱和"}')
    print('=' * 70)


def print_sweep_summary(points):
    print('\n' + '=' * 70)
    print('阶段 2 报告 — GAC 稳定域扫描 (K_env × τ_delay)')
    print('=' * 70)
    from collections import Counter
    cnt = Counter(p['class'] for p in points)
    print(f'总点数 {len(points)}: '
          + '  '.join(f'{k}={v}' for k, v in cnt.items()))
    print('\n   tc       τ_delay  class      K_env_dyn    F_ss')
    for p in points:
        k = p['K_env_dyn']
        ks = f'{k/1e3:8.1f} kN/m' if np.isfinite(k) else '     n/a  '
        fc = f'{p["F_ss"]:6.1f} N' if np.isfinite(p.get('F_ss', float('nan'))) \
            else '  n/a'
        print(f'  {p["tc"]:6.3f}   {p["tau_delay"]*1000:7.1f}ms  '
              f'{p["class"]:>11}  {ks}  {fc}')
    print('=' * 70)


# ====================================================================
# 主流程
# ====================================================================

def parse_args():
    p = argparse.ArgumentParser(
        description='阶段 2: GAC 压入接触 — Fe_raw 力反馈 + K_env×τ_delay 稳定域扫描')
    p.add_argument('--robot', type=str, default='ur12e',
                   choices=['ur12e', 'ur3'])
    p.add_argument('--ball-radius', type=float, default=0.12)
    p.add_argument('--ball-pos', type=float, nargs=3, default=None,
                   help='刚体球球心 (默认按 home 位工具正下方自动计算)')
    p.add_argument('--tool-length', type=float, default=0.10)
    p.add_argument('--tool-radius', type=float, default=0.01)
    p.add_argument('--tool-mass', type=float, default=0.05)
    p.add_argument('--wrist-armature', type=float, default=0.1,
                   help='腕部 dof_armature 电机转子惯量 (kg·m²)')
    p.add_argument('--ball-friction', type=float, default=0.15)
    p.add_argument('--tool-friction', type=float, default=0.15)
    p.add_argument('--ball-solref', type=float, nargs=2, default=[1.0, 1.0],
                   help='球接触 solref (时间常数 阻尼比). 默认 [1.0,1.0] 动态'
                        '接触刚度 ~36 kN/m (§3.1 动态标尺)')
    # 轨迹
    p.add_argument('--delta-pen', type=float, default=0.008,
                   help='接触/摩擦目标压深 (m). 导纳平衡实际压深 = F_ss/K_env')
    p.add_argument('--approach-speed', type=float, default=0.006)
    p.add_argument('--settle-time', type=float, default=1.2)
    p.add_argument('--theta-amp', type=float, default=0.12,
                   help='摩擦斑块经向半宽 (rad). 默认 0.12 > GIC 被动上限 0.08'
                        '→ GAC 力反馈突破摩擦面积上限 (≈1.9 cm²)')
    p.add_argument('--phi-amp', type=float, default=0.8)
    p.add_argument('--rub-cycles', type=int, default=2)
    p.add_argument('--phi-cycles', type=int, default=3)
    p.add_argument('--rub-mode', type=str, default='lissajous',
                   choices=['lissajous', 'cap'])
    p.add_argument('--rub-duration', type=float, default=16.0)
    p.add_argument('--depart-speed', type=float, default=0.05)
    # GAC
    p.add_argument('--bandwidth', type=float, default=90.0,
                   help='GAC 内环期望带宽 ω_des (rad/s), 与阶段 1 稳定配方一致')
    p.add_argument('--damping', type=float, default=4.0,
                   help='GAC 内环期望阻尼比 ζ')
    p.add_argument('--M-d', type=float, nargs=6,
                   default=[10.0, 10.0, 10.0, 1.0, 1.0, 1.0])
    p.add_argument('--D-d', type=float, nargs=6, default=None,
                   help='虚拟阻尼 (None = 按 K_d/M_d 临界阻尼)')
    p.add_argument('--K-d', type=float, nargs=6,
                   default=[5000.0, 5000.0, 5000.0, 200.0, 200.0, 200.0],
                   help='虚拟刚度: 平动 5000 N/m (力反馈下接触"手感"), '
                        '转动 200 Nm/rad (维持朝向)')
    p.add_argument('--max-correction', type=float, default=0.1,
                   help='导纳滤波器最大修正量 (m/rad)')
    p.add_argument('--tau-delay', type=float, default=0.0,
                   help='FT 传感器传输延迟 τ_delay (s), 0 = 理想零延迟')
    # 稳定域扫描
    p.add_argument('--sweep', action='store_true',
                   help='稳定域扫描模式: 扫 K_env × τ_delay (替代单次运行)')
    p.add_argument('--sweep-solref', type=float, nargs='+',
                   default=[2.0, 1.0, 0.5, 0.3, 0.2, 0.1, 0.05, 0.02],
                   help='球 solref 时间常数序列 (tc↓ → 动态 K_env↑)')
    p.add_argument('--sweep-delay', type=float, nargs='+',
                   default=[0.0, 0.002, 0.005, 0.010, 0.020],
                   help='FT 延迟序列 (s)')
    p.add_argument('--sweep-hold', type=float, default=3.0,
                   help='扫描点 S1 接触保持时长 (s)')
    # 运行
    p.add_argument('--no-viewer', action='store_true')
    p.add_argument('--save-dir', type=str, default=None,
                   help='结果图目录 (默认 se3_control/figures/contact)')
    return p.parse_args()


def main():
    args = parse_args()
    cfg = get_robot_config(args.robot)

    if args.ball_pos is None:
        robot0 = RobotModel(get_urdf_path(args.robot),
                            ee_frame_name=cfg['ee_frame'], verbose=False)
        robot0.update(cfg['home_q'][:robot0.nv])
        p_ee, R_ee = robot0.get_pose()
        tool_axis = R_ee @ np.array([0.0, 0.0, 1.0])
        tip0 = p_ee + tool_axis * args.tool_length
        gap = 0.02 + args.delta_pen
        ball_pos = tip0 + tool_axis * (gap + args.ball_radius + args.tool_radius)
        ball_pos = [float(v) for v in ball_pos]
        print(f'[GAC] 自动球位: {[round(v,3) for v in ball_pos]}')
    else:
        ball_pos = list(args.ball_pos)

    # 导纳参数
    M_d = np.asarray(args.M_d, dtype=float)
    K_d = np.asarray(args.K_d, dtype=float)
    if args.D_d is not None:
        D_d = np.asarray(args.D_d, dtype=float)
    else:
        D_d = np.array([2 * np.sqrt(K_d[i] * M_d[i]) if K_d[i] > 0
                        else 2 * np.sqrt(500.0 * M_d[i])
                        for i in range(6)], dtype=float)

    save_dir = args.save_dir or os.path.join(
        _PROJECT_ROOT, 'se3_control', 'figures', 'contact')

    if args.sweep:
        points, paths = run_stability_sweep(
            args.robot, ball_pos, args.ball_radius, args.tool_length,
            args.tool_radius, args.tool_mass, args.delta_pen,
            args.approach_speed, args.sweep_hold,
            args.bandwidth, args.damping, M_d, D_d, K_d,
            args.max_correction, args.sweep_solref, args.sweep_delay,
            args.wrist_armature, save_dir,
            ball_friction=args.ball_friction,
            tool_friction=args.tool_friction)
        print_sweep_summary(points)
        for path in paths:
            print(f'[Figure] {path}')
        import json
        data_file = os.path.join(save_dir, 'gac_contact_stability.json')
        os.makedirs(save_dir, exist_ok=True)
        with open(data_file, 'w') as f:
            json.dump([{**p,
                        'K_env_dyn': (None if not np.isfinite(p['K_env_dyn'])
                                      else float(p['K_env_dyn'])),
                        'F_ss': float(p['F_ss'])}
                       for p in points], f, indent=2)
        print(f'[JSON] {data_file}')
        return

    log, report, paths = run_gac_contact(
        args.robot, ball_pos, args.ball_radius, args.tool_length,
        args.tool_radius, args.tool_mass, args.delta_pen,
        args.approach_speed, args.settle_time, args.theta_amp, args.phi_amp,
        args.rub_cycles, args.phi_cycles, args.rub_mode, args.rub_duration,
        args.depart_speed, args.bandwidth, args.damping, M_d, D_d, K_d,
        args.max_correction, args.tau_delay, args.wrist_armature, save_dir,
        show_viewer=not args.no_viewer,
        ball_friction=args.ball_friction,
        tool_friction=args.tool_friction,
        ball_solref=(tuple(args.ball_solref) if args.ball_solref else None),
        ball_solimp=None)

    report.setdefault('c', np.asarray(ball_pos, dtype=float))
    report.setdefault('theta_amp', args.theta_amp)
    report.setdefault('phi_amp', args.phi_amp)
    report.setdefault('tau_delay', args.tau_delay)
    print_report(report, args.robot)
    for path in paths:
        print(f'[Figure] {path}')


if __name__ == '__main__':
    main()
