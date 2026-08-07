#!/usr/bin/env python
"""阶段 1: GIC 被动接触 — 逼近 / 接触 / 表面摩擦 / 离开 全流程验证.

计划 docs/plan/force_interaction_experiments_plan.md 附录 A.9 阶段 1 (GIC 被动接触):
  "GIC 被动接触先做: 环境基础设施最简单、被动性验证最容易过,
   先把环境 + 传感器 + 分析库跑通."

目标 (A.8 理想状态的定量落点):
  1. 沿给定路径逐渐逼近刚体球 — 逼近段无提前抖动, 轨迹跟踪误差小;
  2. 接触建立反弹小且稳定 — F_peak 超调 < 30%, make-break = 1, 调节时间 < 1 s;
  3. 沿球面来回摩擦不掉球 — 接触力波动 < 10% F_ss, 径向不脱离, 无极限环;
  4. 离开球面干净 — 抬离后力立即归零, 无再次误碰, 无振铃.

控制: **GIC 被动响应** (Fe_raw=None). 控制器不读接触力, 撞上去靠阻抗律
 (K_adapt = ω²M̃, D_adapt = 2ζωM̃, 自适应操作空间惯性) 自然让位 —
 这是无源性验证: 不加力反馈也要不发散、稳定、可摩擦、可抬离.

环境: 复用 verify_gac_mujoco.urdf_joints_to_mujoco_xml 的
  rigid_ball (可碰撞刚体球) + tool_tip (带质量工具尖) + force_sensor.
  `<motor>` 力矩驱动 (GIC 是力矩控制), 腕部补 dof_armature 电机转子惯量
  (修正 URDF 缺失腕部电机惯量导致的近零惯量失稳, 阶段 0 结论).

轨迹 (工具尖中心, 球心在工具正下方 → 逼近沿球面法向):
  - 逼近: 半径 r 从 r_start 匀速 (平滑加速) 收缩到 r_des = R_eff − δ_pen
          (R_eff = 球半径+尖半径, 收缩 = 沿法向压入, 接触时 v≈approach_speed);
  - 保持: r = r_des 静止, 接触力建立并落入 ±10% 稳态带;
  - 表面摩擦: r = r_des 固定, 极角 θ 在 ±θ_amp 间光滑往复 (N 个来回),
          工具尖沿球面弧线运动, 保持恒压深 (来回摩擦);
  - 离开: r 回到 r_start (沿法向抬离), 接触干净断开;
  - 保持: 离开后静止, 验证无振铃、无再次误碰.

极角坐标系 (yz 平面内扫掠): n(θ)=[0, sinθ, cosθ], 从球顶 (θ=0) 向 ±y 摆.

用法示例:
  python se3_control/scripts/verify_gic_contact.py
  python se3_control/scripts/verify_gic_contact.py --robot ur3
  python se3_control/scripts/verify_gic_contact.py --approach-speed 0.08 \
      --delta-pen 0.004 --rub-cycles 3 --no-viewer
  # 扩大表面摩擦面积: 增大 θ_amp / φ_amp (2D 球面 Lissajous 斑块)
  python se3_control/scripts/verify_gic_contact.py --theta-amp 0.10 \
      --phi-amp 1.5708 --no-viewer

产物:
  控制台指标报告 + se3_control/figures/contact/:
    gic_contact.png           — 接触力/压深/力分量/力矩时间序列 (四阶段标注)
    gic_contact_rub.png       — 摩擦段接触轨迹球顶切平面俯视 (显示覆盖面积)
    gic_contact_surface.png   — 球面接触点轨迹 3D 视图 (球 + 轨迹按时间着色)
"""

import argparse
import math
import os
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
from verify_contact_calibration import (
    contact_force_mag, penetration_from_geom, tool_tip_pos, F_OFF,
    _setup_matplotlib, _CJK_FONT_CANDIDATES,
)
from config.robot_configs import get_robot_config, get_urdf_path
from robot_model import RobotModel
from core.gic_controller import GICController

# 接触判定阈值 (F 低于此视为"断开")
# (F_OFF 已从 verify_contact_calibration 导入, 这里不再重复定义)


# ====================================================================
# 光滑轨迹工具
# ====================================================================

def _cos_step(s):
    """C² 平滑步进: s∈[0,1] → [0,1], 端点速度/加速度为零.
    ṡ = 1−cos(2πs), s̈ = 2π sin(2πs).
    """
    return s - math.sin(2.0 * math.pi * s) / (2.0 * math.pi)


def _cos_step_vel(s, T):
    """d/dt[_cos_step(s(t))] = ṡ/T (s 线性推进, 总时长 T)."""
    return (1.0 - math.cos(2.0 * math.pi * s)) / T


def _cos_step_acc(s, T):
    """d²/dt²[_cos_step(s(t))] = s̈/T²."""
    return 2.0 * math.pi * math.sin(2.0 * math.pi * s) / (T * T)


def _smoothstep(x):
    """C¹ 平滑步进 (用于 rub 幅值 bump): x∈[0,1] → [0,1]."""
    x = min(1.0, max(0.0, x))
    return x * x * (3.0 - 2.0 * x)


def _smoothstep_d(x):
    x = min(1.0, max(0.0, x))
    return 6.0 * x * (1.0 - x)


def _bump(tp, T, tau):
    """0→1 平滑上升 (τ 内) → 平台 1 → 平滑下降 (τ 内). tp∈[0,T]."""
    if tau <= 0:
        return 1.0
    s = tp / T
    r = tau / T
    if s < r:
        return _smoothstep(s / r)
    if s > 1.0 - r:
        return _smoothstep((1.0 - s) / r)
    return 1.0


def _bump_dot(tp, T, tau):
    """d/dt _bump."""
    if tau <= 0:
        return 0.0
    s = tp / T
    r = tau / T
    if s < r:
        return _smoothstep_d(s / r) / (r * T)
    if s > 1.0 - r:
        return -_smoothstep_d((1.0 - s) / r) / (r * T)
    return 0.0


def _bump_ddot(tp, T, tau):
    """d²/dt² _bump."""
    if tau <= 0:
        return 0.0
    s = tp / T
    r = tau / T
    if s < r:
        x = s / r
        # d²/ds² smoothstep = 6(1−2x); 链式两次除以 rT
        return 6.0 * (1.0 - 2.0 * x) / (r * r * T * T)
    if s > 1.0 - r:
        x = (1.0 - s) / r
        return 6.0 * (1.0 - 2.0 * x) / (r * r * T * T)
    return 0.0


# ====================================================================
# 轨迹: 工具尖沿球面法向逼近 → 恒压深表面摩擦 → 法向抬离
# ====================================================================

class TipContactTrajectory:
    """工具尖中心的参数化轨迹 (球心在工具正下方).

    球面极坐标扫掠: n(θ,φ) = [sinθ·sinφ, sinθ·cosφ, cosθ]
      (θ = 极角, φ = 方位角; θ=φ=0 为球顶). 位置 p_tip = c + r(t)·n(θ(t),φ(t)).
    相位: approach → settle → rub → depart → hold.

    摩擦段默认 **球冠螺旋 (cap)**: θ 沿经向缓慢 0→θ_amp→0 往复 (来回),
    φ 沿纬向匀速整圈旋转 → 在球顶填充半径为 R·θ_amp 的 **2D 球冠面积**
    (≫ 1D 弧线), 便于表面运行面积观察且保持恒定切向速率 (无急转 → 被动稳定).
    可选 `rub_mode='lissajous'`: θ, φ 各自正弦往复且不同频, 填充 Lissajous 斑块.

    :param ball_center: 球心 (3,)
    :param r_start:     逼近起点半径 (m, 应 > R_eff, 保证未接触)
    :param r_des:       接触/摩擦目标半径 (m, = R_eff − δ_pen < R_eff, 压入)
    :param approach_speed: 逼近巡航速度 (m/s, 接触时工具沿法向速度 ≈ 此值)
    :param settle_time: 接触建立保持时间 (s)
    :param theta_amp:   表面摩擦极角幅值 (rad) — 摩擦斑块经向半宽 / 球冠半角
    :param phi_amp:     表面摩擦方位角幅值 (rad, ±) — 仅 lissajous 模式用
                         (φ_amp=π/2 时切平面内为半径 R·θ_amp 的半圆斑块; 0 = 1D 弧)
    :param rub_cycles:  表面摩擦 θ 向往返次数 (整周期数)
    :param phi_cycles:  表面摩擦 φ 向往返次数 (整周期数). cap 模式 = 整圈旋转数;
                         lissajous 模式 = 与 rub_cycles 不同频填面积
    :param rub_mode:    'cap' (默认, 球冠螺旋) | 'lissajous'
    :param rub_duration: 表面摩擦段总时长 (s)
    :param rub_ramp:    rub 幅值平滑上升/下降时长 (s)
    :param depart_speed: 抬离速度 (m/s)
    :param dt:          控制周期 (s)
    """

    PHASES = ['approach', 'settle', 'rub', 'depart', 'hold']

    def __init__(self, ball_center, r_start, r_des,
                 approach_speed=0.05, settle_time=0.6,
                 theta_amp=0.4, phi_amp=0.0, rub_cycles=2, phi_cycles=3,
                 rub_mode='cap', rub_duration=4.0, rub_ramp=0.4,
                 depart_speed=0.05, dt=0.001):
        self.c = np.asarray(ball_center, dtype=float)
        self.r_start = float(r_start)
        self.r_des = float(r_des)
        self.approach_speed = float(approach_speed)
        self.depart_speed = float(depart_speed)
        self.theta_amp = float(theta_amp)
        self.phi_amp = float(phi_amp)
        self.rub_mode = rub_mode
        self.rub_duration = float(rub_duration)
        self.rub_ramp = float(rub_ramp)

        # 逼近: 平滑加速 ramp (τ_app) + 匀速巡航, 总距离 r_start − r_des
        self.tau_app = min(0.3, (self.r_start - self.r_des)
                           / max(self.approach_speed, 1e-6) / 4.0)
        dist = self.r_start - self.r_des
        self.t1 = self.tau_app / 2.0 + dist / max(self.approach_speed, 1e-6)
        self.t2 = self.t1 + settle_time
        self.t3 = self.t2 + self.rub_duration
        # 离开: 平滑步进抬离 (C²), 峰值速度 ≈ 2·dist/T_dep
        self.T_dep = 2.0 * dist / max(self.depart_speed, 1e-6)
        self.t4 = self.t3 + self.T_dep
        self.t_end = self.t4 + 0.5          # 离开后保持 0.5 s
        self.dt = float(dt)
        self.T = int(math.ceil(self.t_end / self.dt))

        # rub θ 角频率 (经向): 平台期完成 N 个整周期
        self.omega = (2.0 * math.pi * max(int(rub_cycles), 1)
                      / max(self.rub_duration - 2.0 * self.rub_ramp, 1e-6))
        # rub φ 角频率 (纬向): 与 ω_θ 不同频 → 球面 Lissajous, 填充 2D 面积
        self.omega_phi = (2.0 * math.pi * max(int(phi_cycles), 1)
                          / max(self.rub_duration - 2.0 * self.rub_ramp, 1e-6))

    # ── 位置/速度/加速度 (工具尖, 3D 球面坐标) ──
    def _r_theta(self, t):
        """返回 (r, ṙ, r̈, θ, θ̇, θ̈, φ, φ̇, φ̈)."""
        if t <= self.t1:                       # approach
            s_ramp = t / self.tau_app
            if t <= self.tau_app:
                # 平滑加速段: ṙ: 0 → −v_app (C²)
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
            _ = s_ramp
            return r, rd, rdd, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

        if t <= self.t2:                       # settle (接触建立)
            return self.r_des, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

        if t <= self.t3:                       # rub (恒压深 2D 球面往复)
            tp = t - self.t2
            u = _bump(tp, self.rub_duration, self.rub_ramp)
            ud = _bump_dot(tp, self.rub_duration, self.rub_ramp)
            udd = _bump_ddot(tp, self.rub_duration, self.rub_ramp)
            if self.rub_mode == 'cap':
                # ── 球冠螺旋: θ 沿经向 0→θ_amp→0 平滑往复 (sin² 波形),
                #    φ 沿纬向匀速整圈旋转 → 填充半径为 R·θ_amp 的球冠面积.
                #    无方向急转、切向速率平滑 → GIC 被动稳定摩擦.
                s = math.sin(self.omega * tp)
                c = math.cos(self.omega * tp)
                th = self.theta_amp * u * s * s
                thd = self.theta_amp * (ud * s * s
                                        + u * 2.0 * s * c * self.omega)
                thdd = self.theta_amp * (
                    udd * s * s
                    + 4.0 * ud * s * c * self.omega
                    + u * 2.0 * self.omega * self.omega * (c * c - s * s))
                ph = self.omega_phi * tp            # 连续整圈旋转 (无需回零)
                phd = self.omega_phi
                phdd = 0.0
            else:
                # ── 球面 Lissajous: θ, φ 各自正弦往复且不同频
                th = self.theta_amp * u * math.sin(self.omega * tp)
                thd = self.theta_amp * (ud * math.sin(self.omega * tp)
                                        + u * self.omega * math.cos(self.omega * tp))
                thdd = self.theta_amp * (
                    udd * math.sin(self.omega * tp)
                    + 2.0 * ud * self.omega * math.cos(self.omega * tp)
                    - u * self.omega * self.omega * math.sin(self.omega * tp))
                ph = self.phi_amp * u * math.sin(self.omega_phi * tp)
                phd = self.phi_amp * (ud * math.sin(self.omega_phi * tp)
                                      + u * self.omega_phi * math.cos(self.omega_phi * tp))
                phdd = self.phi_amp * (
                    udd * math.sin(self.omega_phi * tp)
                    + 2.0 * ud * self.omega_phi * math.cos(self.omega_phi * tp)
                    - u * self.omega_phi * self.omega_phi * math.sin(self.omega_phi * tp))
            return self.r_des, 0.0, 0.0, th, thd, thdd, ph, phd, phdd

        if t <= self.t4:                       # depart (法向抬离, 平滑)
            s = (t - self.t3) / self.T_dep
            r = self.r_des + (self.r_start - self.r_des) * _cos_step(s)
            rd = (self.r_start - self.r_des) * _cos_step_vel(s, self.T_dep)
            rdd = (self.r_start - self.r_des) * _cos_step_acc(s, self.T_dep)
            return r, rd, rdd, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

        # hold (离开后保持)
        return self.r_start, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

    def phase_at(self, t):
        if t <= self.t1:
            return 'approach'
        if t <= self.t2:
            return 'settle'
        if t <= self.t3:
            return 'rub'
        if t <= self.t4:
            return 'depart'
        return 'hold'

    def eval(self, t):
        """返回 (p_tip, v_tip, a_tip, phase).  3D 球面坐标.

        位置 p = c + r·n(θ,φ),  n = [sinθ·sinφ, sinθ·cosφ, cosθ].
        速度/加速度用球面坐标基链式求导 (含曲率项):
          n_θθ = −n,  n_φφ = −sinθ·[sinφ,cosφ,0],  n_θφ = cosθ·[cosφ,−sinφ,0].
        φ_amp=0 时退化为原 yz 平面 1D 弧 (n=[0,sinθ,cosθ]).
        """
        r, rd, rdd, th, thd, thdd, ph, phd, phdd = self._r_theta(t)
        st, ct = math.sin(th), math.cos(th)
        sf, cf = math.sin(ph), math.cos(ph)
        n = np.array([st * sf, st * cf, ct])
        n_th = np.array([ct * sf, ct * cf, -st])
        n_ph = np.array([st * cf, -st * sf, 0.0])
        # 曲率项 (显式向量, 避免符号歧义)
        curv = (-r * thd * thd * n
                - r * phd * phd * st * np.array([sf, cf, 0.0])
                + 2.0 * r * thd * phd * ct * np.array([cf, -sf, 0.0]))
        p = self.c + r * n
        v = rd * n + r * thd * n_th + r * phd * n_ph
        a = ((rdd - r * thd * thd) * n
             + (2.0 * rd * thd + r * thdd) * n_th
             + (2.0 * rd * phd + r * phdd) * n_ph
             + curv)
        return p, v, a, self.phase_at(t)


# ====================================================================
# 环境构建 (GIC 力矩控制 + 接触环境)
# ====================================================================

DEFAULT_WRIST_ARMATURE = 0.1   # 腕部 dof_armature 电机转子惯量 (阶段 0 结论)


def build_environment(robot_name, ball_pos, ball_radius,
                      tool_length, tool_radius=0.01, tool_mass=0.05,
                      force_sensor=True, dt=0.001,
                      wrist_armature=DEFAULT_WRIST_ARMATURE,
                      ball_friction=0.5, tool_friction=None,
                      ball_solref=None, ball_solimp=None):
    """构建带接触环境的 MuJoCo 模型 + RobotModel (GIC 用 <motor> 力矩驱动).

    :param ball_friction: 刚体球摩擦系数 (0.5 → 表面摩擦平滑滑移; 0.8+ → 粘滞/抓附).
    :param tool_friction: 工具尖摩擦系数 (None → MuJoCo 默认 1.0). MuJoCo 组合
        摩擦为几何平均 sqrt(mu_geom1·mu_geom2), 单设球摩擦并不能真正降低
        表面摩擦; 摩擦跟随需同时调低两者 (如 0.3/0.3 → 有效 0.3).
    :param ball_solref: 球接触 solref 2元组 (时间常数, 阻尼比), None=默认 [0.02,1].
    :param ball_solimp: 球接触 solimp 5元组, None=默认. 默认 width=0.001 造成
        近零压深超硬拐点 (0.19mm 内 ~350 kN/m), 摩擦跟随需增大 width 平滑.

    :returns: (model, data, robot, cfg, tip_body_id, tip_geom_id,
               ball_geom_id, ball_center)
    """
    cfg = get_robot_config(robot_name)
    urdf_path = get_urdf_path(robot_name)

    xml_str = urdf_joints_to_mujoco_xml(
        urdf_path, cfg['ee_frame'], timestep=dt,
        link_to_mesh=cfg['link_to_mesh'], mesh_subdir=cfg['mesh_subdir'],
        rigid_ball=(np.asarray(ball_pos, dtype=float), ball_radius,
                    ball_friction),
        tool_tip={'length': tool_length, 'radius': tool_radius,
                  'mass': tool_mass,
                  **({} if tool_friction is None
                     else {'friction': tool_friction})},
        force_sensor=force_sensor)

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

    # 接触柔度覆盖: 默认 solref=[0.02,1] → K_env≈19 kN/m (Phase 0 标定);
    # 近零压深的超硬拐点来自默认 solimp width=0.001. 增大 width 平滑拐点,
    # 让 GIC 能在 ~1mm 量级压深下稳定控制接触力 (摩擦跟随所需).
    if ball_solref is not None:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM,
                                'rigid_ball')
        if bid >= 0:
            model.geom_solref[bid, 0] = float(ball_solref[0])
            model.geom_solref[bid, 1] = float(ball_solref[1])
    if ball_solimp is not None:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM,
                                'rigid_ball')
        if bid >= 0:
            model.geom_solimp[bid, :] = np.asarray(ball_solimp, dtype=float)

    robot = RobotModel(urdf_path, ee_frame_name=cfg['ee_frame'], verbose=False)

    body_names = [model.body(i).name for i in range(model.nbody)]
    geom_names = [model.geom(i).name for i in range(model.ngeom)]
    tip_body_id = body_names.index('tool_tip')
    tip_geom_id = geom_names.index('tool_tip')
    ball_geom_id = geom_names.index('rigid_ball')
    ball_center = np.asarray(ball_pos, dtype=float)
    return (model, data, robot, cfg,
            tip_body_id, tip_geom_id, ball_geom_id, ball_center)


def contact_force_vec(model, data, tip_geom_id):
    """工具尖所受接触力向量 (世界系, 带方向), 从 mj_contactForce 累加.

    mj_contactForce 给出作用于 geom1 的力 (接触帧); geom2 取反.
    """
    F = np.zeros(3)
    for i in range(data.ncon):
        c = data.contact[i]
        if tip_geom_id not in (c.geom1, c.geom2):
            continue
        f = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, f)
        frame = c.frame.reshape(3, 3)
        fw = frame @ f[:3]
        F += fw if c.geom1 == tip_geom_id else -fw
    return F


def force_components(F, tip, ball_center):
    """把接触力分解为 (法向 |F_n|, 切向 |F_t|). 法向 = 沿 球心→尖 径向."""
    n_hat = tip - ball_center
    d = np.linalg.norm(n_hat)
    if d < 1e-9:
        return 0.0, float(np.linalg.norm(F))
    n_hat /= d
    Fn = float(np.dot(F, n_hat))
    Ft = F - Fn * n_hat
    return abs(Fn), float(np.linalg.norm(Ft))


# ====================================================================
# 指标
# ====================================================================

def contact_establish_metrics(t, pen, F, t_contact, t_settle_end, F_off=F_OFF):
    """接触建立段 [t_contact, t_settle_end]: 超调/稳态/断开/调节时间.

    :returns: dict 或 None (段内未接触)
    """
    seg = (t >= t_contact) & (t <= t_settle_end)
    if not np.any(seg):
        return None
    Fw = F[seg]
    tw = t[seg]
    F_peak = float(Fw.max())
    n_win = Fw.size
    F_ss = float(Fw[int(0.8 * n_win):].mean()) if n_win >= 5 else float(Fw.mean())
    if F_ss < 1e-9:
        return None
    overshoot = (F_peak - F_ss) / F_ss

    # make-break: 段内 F 低于 F_off 又回升的次数 (理想 = 1 次建立)
    on = Fw >= F_off
    breaks = 0
    prev = True
    for k in range(1, n_win):
        if prev and not on[k]:
            breaks += 1
        prev = on[k]

    # 调节时间: 从接触到 |F−F_ss| 进入 ±10%F_ss 且保持 ≥20ms
    band = 0.10 * F_ss
    n_keep = max(int(0.020 / max(tw[1] - tw[0], 1e-6)), 3)
    settle = None
    for k in range(n_win):
        jj = np.arange(k, min(k + n_keep, n_win))
        if jj.size and all(abs(Fw[j] - F_ss) < band for j in jj):
            settle = float(tw[k] - t_contact)
            break
    return {'F_peak': F_peak, 'F_ss': F_ss, 'overshoot': overshoot,
            'breaks': breaks, 'settle': settle}


def rub_metrics(t, pen, F, Fn, t_rub_start, t_rub_end, F_off=F_OFF):
    """表面摩擦段: 接触力保持/波动/径向不脱离/无极限环.

    :returns: dict
    """
    seg = (t >= t_rub_start) & (t <= t_rub_end)
    tw = t[seg]
    Fw = F[seg]
    Fnw = Fn[seg]
    penw = pen[seg]
    if Fw.size == 0:
        return None
    F_mean = float(Fw.mean())
    F_min = float(Fw.min())
    detach_time = float((tw[Fw < F_off].size) * (tw[1] - tw[0])) if tw.size > 1 else 0.0
    # 径向不脱离: 压深恒为正 (几何压深 = R_eff − 球心距)
    min_pen = float(penw.min())
    # 接触力波动 (变异系数) — A.7 阈值 < 10% F_ss
    cv = float(Fw.std() / F_mean) if F_mean > 1e-9 else float('inf')
    # 法向力波动 (更直接反映"按在球面上"的稳定性)
    Fn_mean = float(Fnw.mean()) if Fnw.size else 0.0
    Fn_cv = float(Fnw.std() / Fn_mean) if Fn_mean > 1e-9 else float('inf')
    # 极限环检测: 摩擦段后 30% 的力振荡幅值 (pp = 峰值-峰谷), 相对 F_mean
    n_tail = max(int(0.30 * Fw.size), 3)
    F_tail = Fw[-n_tail:]
    pp = float(F_tail.max() - F_tail.min())
    pp_rel = pp / F_mean if F_mean > 1e-9 else float('inf')
    return {'F_mean': F_mean, 'F_min': F_min, 'F_cv': cv,
            'Fn_mean': Fn_mean, 'Fn_cv': Fn_cv,
            'detach_time': detach_time, 'min_pen': min_pen,
            'pp_tail_rel': pp_rel}


def depart_metrics(t, F, t_depart_start, F_off=F_OFF):
    """离开段: 力归零快慢、是否再次误碰、离开后力保持零."""
    seg = (t >= t_depart_start)
    tw = t[seg]
    Fw = F[seg]
    if Fw.size == 0:
        return None
    # 首次 F 降到 < F_off 的时刻 (相对 depart 开始)
    off_idx = np.where(Fw < F_off)[0]
    t_off = float(tw[off_idx[0]] - t_depart_start) if off_idx.size else None
    # 离开后是否再次接触 (F 回升 >= F_off 之后又断开的次数 > 0 → 误碰/振铃)
    if off_idx.size:
        after = Fw[off_idx[0]:]
        recontact = int(np.any(after[1:] >= F_off))
    else:
        recontact = None
    # 离开后 (后半段) 力最大值
    n_tail = max(int(0.5 * Fw.size), 3)
    F_tail = Fw[-n_tail:]
    return {'t_to_off': t_off, 'recontact': recontact,
            'tail_F_max': float(F_tail.max())}


# ====================================================================
# 主仿真
# ====================================================================

def run_contact_sim(robot_name, ball_pos, ball_radius, tool_length,
                    tool_radius, tool_mass, delta_pen, approach_speed,
                    settle_time, theta_amp, phi_amp, rub_cycles, phi_cycles,
                    rub_mode, rub_duration, depart_speed, bandwidth, damping,
                    wrist_armature, save_dir, show_viewer=False, verbose=True,
                    ball_friction=0.5, tool_friction=None,
                    ball_solref=None, ball_solimp=None):
    """运行 GIC 被动接触全流程, 返回 (log, report_dict, save_paths)."""
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
    L = tool_length
    R_eff = ball_radius + tool_radius
    r_des = R_eff - delta_pen

    # 工具尖起点 (home) — 逼近起点半径取实测值 (自动球位下 ≈ R_eff+gap)
    tip0 = tool_tip_pos(data, tip_body_id)
    d0 = np.linalg.norm(tip0 - ball_center)
    r_start = d0

    # 轨迹 (球心在工具正下方 → 极角 0 = 球顶 = 逼近方向)
    traj = TipContactTrajectory(
        ball_center, r_start, r_des,
        approach_speed=approach_speed, settle_time=settle_time,
        theta_amp=theta_amp, phi_amp=phi_amp,
        rub_cycles=rub_cycles, phi_cycles=phi_cycles, rub_mode=rub_mode,
        rub_duration=rub_duration, depart_speed=depart_speed, dt=0.001)
    T = traj.T

    if verbose:
        print(f'[GIC] 球心={np.round(ball_center,3)}  R_eff={R_eff:.3f}  '
              f'r_des={r_des:.3f} (δ_pen={delta_pen:.4f}m)  r_start={r_start:.3f}')
        print(f'[GIC] home 尖距球心={d0:.4f} m (应≈{r_start:.3f})')
        print(f'[GIC] 相位边界: 逼近→{traj.t1:.2f}s 保持→{traj.t2:.2f}s  '
              f'摩擦→{traj.t3:.2f}s 离开→{traj.t4:.2f}s 结束→{traj.t_end:.2f}s')
        if rub_mode == 'cap':
            print(f'[GIC] rub(cap 球冠螺旋): θ_amp={theta_amp:.3f} rad '
                  f'({rub_cycles} 经向起伏) × φ 整圈旋转 {phi_cycles} 圈 → '
                  f'填充球冠面积 π·(R·θ_amp)²≈{math.pi*(0.13*theta_amp)**2*1e4:.1f} cm²')
        else:
            print(f'[GIC] rub(lissajous): θ_amp=±{theta_amp:.3f} rad × '
                  f'φ_amp=±{phi_amp:.3f} rad, ω_θ={traj.omega:.2f} '
                  f'({rub_cycles}) × ω_φ={traj.omega_phi:.2f} ({phi_cycles})')
        print(f'[GIC] 控制器: GIC 被动 (Fe_raw=None), ω_des={bandwidth} rad/s, '
              f'ζ={damping}')

    # GIC 控制器 (被动阻抗, 不读力)
    ctrl = GICController(robot, bandwidth=bandwidth, damping=damping,
                         torque_limits=cfg['full_torque_limits'])

    # viewer (可选)
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

    # 日志
    log = {
        't': np.zeros(T), 'phase': np.full(T, '', dtype=object),
        'tip': np.zeros((T, 3)), 'tip_des': np.zeros((T, 3)),
        'pd': np.zeros((T, 3)), 'p': np.zeros((T, 3)),
        'pen': np.zeros(T), 'F': np.zeros(T),
        'Fn': np.zeros(T), 'Ft': np.zeros(T),
        'F_sensor': np.zeros(T), 'tau': np.zeros((T, nv)),
        'tau_lim': np.full(nv, np.nan),
    }
    lim = cfg['full_torque_limits'][:nv]
    log['tau_lim'][:] = lim

    t_contact = None
    for i in range(T):
        t = i * model.opt.timestep
        tip_des, v_tip, a_tip, phase = traj.eval(t)
        # 期望 EE 位姿: 朝向恒为 home, pd = tip_des − R0@[0,0,L]
        pd = tip_des - R0 @ np.array([0.0, 0.0, L])
        Rd = R0
        # 期望体速度/加速度 (世界→体: Rd.T)
        vd = Rd.T @ v_tip
        wd = np.zeros(3)
        dvd = Rd.T @ a_tip
        dwd = np.zeros(3)

        q = data.qpos[:nv].copy()
        dq = data.qvel[:nv].copy()
        tau = ctrl.compute(q, dq, pd, Rd, vd, wd, dvd, dwd)
        data.ctrl[:] = tau[:model.nu]
        mujoco.mj_step(model, data)

        tip = data.xpos[tip_body_id].copy()
        pen = penetration_from_geom(tip, ball_center, ball_radius, tool_radius)
        Fvec = contact_force_vec(model, data, tip_geom_id)
        F = float(np.linalg.norm(Fvec))
        Fn, Ft = force_components(Fvec, tip, ball_center)
        F_sensor = float(np.linalg.norm(data.sensordata[0:3])) \
            if model.nsensor >= 3 else 0.0

        if t_contact is None and pen > 1e-4:
            t_contact = t

        log['t'][i] = t
        log['phase'][i] = phase
        log['tip'][i] = tip
        log['tip_des'][i] = tip_des
        log['pd'][i] = pd
        log['p'][i] = data.site_xpos[0].copy()
        log['pen'][i] = pen
        log['F'][i] = F
        log['Fn'][i] = Fn
        log['Ft'][i] = Ft
        log['F_sensor'][i] = F_sensor
        log['tau'][i] = tau

        if viewer is not None and i % 20 == 0:
            viewer.sync()
        if verbose and i % 1000 == 0:
            print(f'  t={t:6.2f}s [{phase:>8}] F={F:6.1f}N  '
                  f'pen={pen*1000:5.1f}mm  |tip−tip_des|={np.linalg.norm(tip-tip_des)*1000:5.1f}mm')

    if viewer is not None:
        viewer.close()

    if verbose:
        print(f'[GIC] 首次接触 t={t_contact:.3f}s' if t_contact is not None
              else '[GIC] 未接触!')

    # ── 指标 ──
    report = {}
    if t_contact is not None:
        report['contact'] = contact_establish_metrics(
            log['t'], log['pen'], log['F'], t_contact, traj.t2)
    report['rub'] = rub_metrics(
        log['t'], log['pen'], log['F'], log['Fn'], traj.t2, traj.t3)
    report['depart'] = depart_metrics(log['t'], log['F'], traj.t3)
    # 轨迹跟踪 (逼近段自由空间: 无接触前的路径误差)
    seg_app = (log['t'] <= traj.t1) & (log['pen'] < 1e-4)
    if np.any(seg_app):
        report['approach_err'] = float(
            np.linalg.norm(log['tip'][seg_app] - log['tip_des'][seg_app], axis=1).max())
    # 力矩饱和
    tau_max = np.abs(log['tau']).max(axis=0)
    report['torque_ok'] = bool(np.all(tau_max <= lim * 0.999))
    report['torque_max'] = tau_max
    # 稳定性: 全程力/位移无发散 (末段力有界)
    report['stable'] = bool(np.isfinite(log['F']).all()
                            and log['F'].max() < 5e3)
    report['t_contact'] = t_contact
    report['phases'] = (traj.t1, traj.t2, traj.t3, traj.t4, traj.t_end)
    report['R_eff'] = R_eff
    report['r_des'] = r_des
    report['c'] = ball_center
    report['theta_amp'] = theta_amp
    report['phi_amp'] = phi_amp

    paths = plot_contact(log, report, save_dir) if save_dir else []
    return log, report, paths


# ====================================================================
# 绘图
# ====================================================================

def plot_contact(log, report, save_dir):
    _setup_matplotlib()
    import matplotlib.pyplot as plt

    t = log['t']
    t1, t2, t3, t4, tend = report['phases']
    F_ss = report.get('contact', {}).get('F_ss')
    band = 0.10 * F_ss if F_ss else None

    fig, axes = plt.subplots(4, 1, figsize=(10, 11), sharex=True)

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
    ax.set_title('GIC 被动接触 — 逼近/接触/表面摩擦/离开')

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

    # 4. 关节力矩 (max) + 工具尖 yz 轨迹
    ax = axes[3]
    tau_max = np.abs(log['tau']).max(axis=1)
    ax.plot(t, tau_max, 'k-', lw=0.9, label='max|τ|')
    lim = log['tau_lim']
    if np.isfinite(lim).all():
        ax.axhline(lim.max(), color='r', ls='--', lw=0.8, label='min 限幅')
    _mark_phases(ax, t1, t2, t3, t4)
    ax.set_xlabel('t (s)')
    ax.set_ylabel('max|τ| (Nm)')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(alpha=0.3)

    os.makedirs(save_dir, exist_ok=True)
    path = os.path.join(save_dir, 'gic_contact.png')
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)

    # 副图 1: 摩擦段接触点轨迹 — 球顶切平面 (x-y) 俯视图, 显示 2D 覆盖面积
    path2 = _plot_rub_tangent(log, report, save_dir)
    # 副图 2: 球面接触点轨迹 3D 视图 (球 + 轨迹按时间着色)
    path3 = _plot_surface_traj(log, report, save_dir)

    return [path, path2, path3]


def _plot_rub_tangent(log, report, save_dir):
    """摩擦段接触点轨迹: 球顶切平面 (x-y) 俯视, 直接显示覆盖面积."""
    _setup_matplotlib()
    import matplotlib.pyplot as plt

    c = report['c']
    R = report['R_eff']
    theta_amp = report.get('theta_amp', 0.08)
    seg = log['phase'] == 'rub'
    tr = log['t'][seg]
    tip = log['tip'][seg]
    # 相对球顶的切平面位移 (顶 = 球心 + R·[0,0,1])
    top = c + np.array([0.0, 0.0, R])
    d = tip - top

    fig, ax = plt.subplots(figsize=(6.5, 6.5))
    sc = ax.scatter(d[:, 0], d[:, 1], c=tr, cmap='viridis', s=6, linewidths=0)
    # 摩擦范围参考圆 (切平面内半径 R·θ_amp)
    thc = np.linspace(0, 2 * np.pi, 100)
    r_patch = R * max(theta_amp, 1e-4)
    ax.plot(r_patch * np.sin(thc), r_patch * np.cos(thc), 'k--', lw=0.9,
            alpha=0.6, label=f'摩擦范围 (r=R·θ_amp={r_patch*1e3:.0f} mm)')
    ax.plot(0, 0, 'r+', ms=14, mew=2, label='球顶 (接触中心)')
    ax.set_xlabel('x 切向偏移 (m)')
    ax.set_ylabel('y 切向偏移 (m)')
    ax.set_title('摩擦段接触点轨迹 (球顶切平面俯视, 按时间着色)')
    ax.legend(fontsize=8); ax.grid(alpha=0.3); ax.axis('equal')
    cb = fig.colorbar(sc, ax=ax, shrink=0.8, label='t (s)')
    _ = cb
    path = os.path.join(save_dir, 'gic_contact_rub.png')
    fig.tight_layout(); fig.savefig(path, dpi=120); plt.close(fig)
    return path


def _plot_surface_traj(log, report, save_dir):
    """球面接触点轨迹 3D 视图: 刚体球线框 + 摩擦段轨迹 (按时间着色) + 逼近/抬离."""
    _setup_matplotlib()
    import matplotlib.pyplot as plt

    c = report['c']
    R = report['R_eff']
    theta_amp = report.get('theta_amp', 0.08)

    fig = plt.figure(figsize=(7.5, 7))
    ax = fig.add_subplot(111, projection='3d')

    # 接触面 (半径 R_eff) 球线框
    uu = np.linspace(0, 2 * np.pi, 40)
    vv = np.linspace(0, np.pi, 20)
    xs = c[0] + R * np.outer(np.sin(uu), np.sin(vv))
    ys = c[1] + R * np.outer(np.cos(uu), np.sin(vv))
    zs = c[2] + R * np.outer(np.ones_like(uu), np.cos(vv))
    ax.plot_wireframe(xs, ys, zs, color='k', alpha=0.12, lw=0.4)

    # 摩擦段接触轨迹 (按时间着色; 抽稀以加速渲染)
    seg = log['phase'] == 'rub'
    tr = log['t'][seg]
    tip = log['tip'][seg]
    step = max(int(tip.shape[0] // 3000), 1)
    sc = ax.scatter(tip[::step, 0], tip[::step, 1], tip[::step, 2],
                    c=tr[::step], cmap='viridis', s=8, depthshade=False)
    cb = fig.colorbar(sc, ax=ax, shrink=0.55, pad=0.1)
    cb.set_label('t (s)')

    # 逼近 / 抬离 轨迹
    for ph, col, lab in (('approach', 'b', '逼近'), ('depart', 'r', '抬离')):
        m = log['phase'] == ph
        if np.any(m):
            ax.plot(log['tip'][m, 0], log['tip'][m, 1], log['tip'][m, 2],
                    color=col, lw=2.0, alpha=0.9, label=lab)

    ax.scatter(*c, marker='+', s=120, color='k', label='球心')
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.legend(fontsize=8)
    ax.set_title('球面接触点轨迹 3D (摩擦段按时间着色)')

    # 聚焦接触区: 球顶附近 ±span
    top = c + np.array([0.0, 0.0, R])
    span = max(0.05, 3.5 * R * max(theta_amp, 1e-3))
    ax.set_xlim(top[0] - span, top[0] + span)
    ax.set_ylim(top[1] - span, top[1] + span)
    ax.set_zlim(top[2] - span, top[2] + span)
    try:
        ax.set_box_aspect((1, 1, 1))
    except Exception:
        pass

    path = os.path.join(save_dir, 'gic_contact_surface.png')
    fig.tight_layout(); fig.savefig(path, dpi=120); plt.close(fig)
    return path


def _mark_phases(ax, t1, t2, t3, t4):
    for tx, lab in ((t1, '接触'), (t2, '摩擦'), (t3, '离开'), (t4, '保持')):
        ax.axvline(tx, color='gray', ls=':', lw=0.7)
        ax.text(tx, ax.get_ylim()[1], lab, fontsize=7, rotation=90,
                va='top', ha='right', color='gray')


# ====================================================================
# 报告
# ====================================================================

def print_report(report, robot_name):
    print('\n' + '=' * 70)
    print(f'阶段 1 报告 — GIC 被动接触 (robot={robot_name})')
    print('=' * 70)
    t1, t2, t3, t4, tend = report['phases']
    print(f'相位: 逼近[0,{t1:.2f}] 接触保持[{t1:.2f},{t2:.2f}] '
          f'摩擦[{t2:.2f},{t3:.2f}] 离开[{t3:.2f},{t4:.2f}] 保持[{t4:.2f},{tend:.2f}]')
    if report.get('t_contact') is not None:
        print(f'首次接触 t={report["t_contact"]:.3f}s')
    print(f'\n[稳定性] 全程力有界: {"✓" if report["stable"] else "✗ 发散"}')

    print('\n[逼近] 自由空间最大轨迹误差: '
          f'{report.get("approach_err", float("nan"))*1000:.2f} mm')
    if report.get('contact'):
        c = report['contact']
        settle = '—' if c['settle'] is None else f'{c["settle"]:.3f}s'
        overshoot_ok = c['overshoot'] < 0.30
        settle_ok = (c['settle'] is not None and c['settle'] < 1.0)
        print('\n[接触建立] (A.8: 反弹小、稳定)')
        print(f'  F_peak   = {c["F_peak"]:7.1f} N')
        print(f'  F_ss     = {c["F_ss"]:7.1f} N')
        print(f'  超调     = {c["overshoot"]*100:6.1f} %   '
              f'(<30% {"✓" if overshoot_ok else "✗"})')
        print(f'  断开次数 = {c["breaks"]}   (理想 = 1 次建立不回跳)')
        print(f'  调节时间 = {settle}   (<1s {"✓" if settle_ok else "✗"})')
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
        print('\n[离开] (A.8: 抬离干净、无振铃)')
        t_off = '—' if d['t_to_off'] is None else f'{d["t_to_off"]*1000:.0f}ms'
        print(f'  力归零耗时 = {t_off}')
        print(f'  再次误碰   = {d["recontact"]}   '
              f'({"✓ 无" if d["recontact"] == 0 else "✗ 有"})')
        print(f'  离开后尾段 max F = {d["tail_F_max"]:.2f} N   '
              f'({"✓ 干净" if d["tail_F_max"] < F_OFF else "✗ 振铃"})')
    tm = report['torque_max']
    print(f'\n[力矩] max|τ| = {np.round(tm,1)} Nm, 饱和: '
          f'{"✓ 无" if report["torque_ok"] else "✗ 饱和"}')
    print('=' * 70)


# ====================================================================
# 主流程
# ====================================================================

def parse_args():
    p = argparse.ArgumentParser(
        description='阶段 1: GIC 被动接触 — 逼近/接触/表面摩擦/离开')
    p.add_argument('--robot', type=str, default='ur12e',
                   choices=['ur12e', 'ur3'])
    p.add_argument('--ball-radius', type=float, default=0.12)
    p.add_argument('--ball-pos', type=float, nargs=3, default=None,
                   help='刚体球球心 (默认按 home 位工具正下方自动计算)')
    p.add_argument('--tool-length', type=float, default=0.10)
    p.add_argument('--tool-radius', type=float, default=0.01)
    p.add_argument('--tool-mass', type=float, default=0.05)
    p.add_argument('--wrist-armature', type=float, default=0.1,
                   help='腕部 dof_armature 电机转子惯量 (kg·m²), None 不设')
    p.add_argument('--ball-friction', type=float, default=0.15,
                   help='刚体球摩擦系数 (阶段 1 标定: 0.15 稳定摩擦; 0.8+ 粘滞)')
    p.add_argument('--tool-friction', type=float, default=0.15,
                   help='工具尖摩擦系数 (默认 0.15; MuJoCo 组合摩擦 = '
                        'sqrt(球×尖), 需同时调低两者才能真正减摩)')
    p.add_argument('--ball-solref', type=float, nargs=2, default=[1.0, 1.0],
                   help='球接触 solref (时间常数 阻尼比). 默认 [1.0,1.0]: 动态接触'
                        '刚度 ~36 kN/m (接近 Phase 0 标定 17.8 kN/m), 静态默认 '
                        '[0.02,1] 在 mj_step 近零压深处 ~6 MN/m, GIC 无法稳定摩擦')
    p.add_argument('--ball-solimp-width', type=float, default=None,
                   help='球接触 solimp[2] 宽度 (默认 0.001; 近零压深超硬拐点 '
                        '主要来自 mj_step 约束解算, 此参数效果有限)')
    # 轨迹
    p.add_argument('--delta-pen', type=float, default=0.008,
                   help='接触/摩擦目标压深 (m). 阻抗受限于 K_rad, 实际压深为平衡值')
    p.add_argument('--approach-speed', type=float, default=0.006,
                   help='逼近巡航速度 (m/s) (慢 → 接触冲击超调小)')
    p.add_argument('--settle-time', type=float, default=1.2,
                   help='接触建立保持时长 (s)')
    p.add_argument('--theta-amp', type=float, default=0.08,
                   help='表面摩擦极角幅值 (rad) = 摩擦斑块经向半宽.'
                        '默认 0.08 (稳定域内最大经向半宽, 见 exp3 报告 §4.3)')
    p.add_argument('--phi-amp', type=float, default=0.8,
                   help='表面摩擦方位角幅值 (rad, ±). 默认 0.8 (46°) → 与 θ 组成 '
                        '2D 球面 Lissajous, 球顶切平面内为半径 R·θ_amp 的扇形斑块 '
                        '(面积 ≈ φ·R²·θ_amp² ≈ 0.9 cm²); 0 = 1D 弧线')
    p.add_argument('--rub-cycles', type=int, default=2,
                   help='表面摩擦 θ (经向) 往返次数')
    p.add_argument('--phi-cycles', type=int, default=3,
                   help='表面摩擦 φ (纬向) 往返次数 (与 rub-cycles 不同频 → 填充面积)')
    p.add_argument('--rub-mode', type=str, default='lissajous',
                   choices=['lissajous', 'cap'],
                   help='表面摩擦模式: lissajous(默认, θ×φ 双向正弦, 高频带宽下 '
                        'F_cv<10% 稳定) | cap(球冠螺旋, 填充整圆但 F_cv 偏高)')
    p.add_argument('--rub-duration', type=float, default=16.0,
                   help='表面摩擦段总时长 (s)')
    p.add_argument('--depart-speed', type=float, default=0.05,
                   help='抬离速度 (m/s)')
    # GIC
    p.add_argument('--bandwidth', type=float, default=90.0,
                   help='GIC 期望带宽 ω_des (rad/s). 2D 摩擦需高带宽: K_tan=ω²M̃ '
                        '加硬 → 切向跟踪误差小, F_cv 低 (默认 90, 见 exp3 §4.3)')
    p.add_argument('--damping', type=float, default=4.0,
                   help='GIC 期望阻尼比 ζ (大 → 摩擦段径向振动衰减, 力更稳; '
                        '2D 摩擦稳定域需 ζ≈4)')
    # 运行
    p.add_argument('--no-viewer', action='store_true',
                   help='无头模式 (SSH/服务器)')
    p.add_argument('--save-dir', type=str, default=None,
                   help='结果图目录 (默认 se3_control/figures/contact)')
    return p.parse_args()


def main():
    args = parse_args()
    cfg = get_robot_config(args.robot)

    if args.ball_pos is None:
        # 自动球位: 工具正下方, 球面在工具尖下方 2cm (与逼近几何一致)
        robot0 = RobotModel(get_urdf_path(args.robot),
                            ee_frame_name=cfg['ee_frame'], verbose=False)
        robot0.update(cfg['home_q'][:robot0.nv])
        p_ee, R_ee = robot0.get_pose()
        tool_axis = R_ee @ np.array([0.0, 0.0, 1.0])
        tip0 = p_ee + tool_axis * args.tool_length
        gap = 0.02 + args.delta_pen          # 逼近起点球面外 2cm (压入目标在球面内)
        # 球心 = 尖端下方 (gap + 球半径 + 尖半径) — 使 r_start = R_eff + 0.02
        ball_pos = tip0 + tool_axis * (gap + args.ball_radius + args.tool_radius)
        ball_pos = [float(v) for v in ball_pos]
        print(f'[GIC] 自动球位: {[round(v,3) for v in ball_pos]}')
    else:
        ball_pos = list(args.ball_pos)

    save_dir = args.save_dir or os.path.join(
        _PROJECT_ROOT, 'se3_control', 'figures', 'contact')
    log, report, paths = run_contact_sim(
        args.robot, ball_pos, args.ball_radius, args.tool_length,
        args.tool_radius, args.tool_mass, args.delta_pen,
        args.approach_speed, args.settle_time, args.theta_amp, args.phi_amp,
        args.rub_cycles, args.phi_cycles, args.rub_mode, args.rub_duration,
        args.depart_speed, args.bandwidth, args.damping, args.wrist_armature,
        save_dir, show_viewer=not args.no_viewer,
        ball_friction=args.ball_friction,
        tool_friction=args.tool_friction,
        ball_solref=(tuple(args.ball_solref) if args.ball_solref else None),
        ball_solimp=([0.9, 0.95, args.ball_solimp_width, 0, 2]
                     if args.ball_solimp_width is not None else None))

    # 报告里补上绘图用的球心/幅值
    report.setdefault('c', np.asarray(ball_pos, dtype=float))
    report.setdefault('theta_amp', args.theta_amp)
    report.setdefault('phi_amp', args.phi_amp)
    print_report(report, args.robot)
    for path in paths:
        print(f'[Figure] {path}')


if __name__ == '__main__':
    main()
