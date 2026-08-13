#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tests/test_servo_bridge.py — ServoJTorqueBridge 力矩→伺服关节目标位桥接测试

背景
----
CB3 classic 版 UR（本仓库 UR3，固件 < 5.23）没有 directTorque，ur_rtde 1.6.3
会对 <5.23 整段删除 direct_torque 命令（静默空操作）。ServoJTorqueBridge 把
GICController 算出的关节力矩折算成 servoJ 可以跟踪的关节目标位:

  τ_imp = τ − bias,  ddq = M⁻¹·τ_imp,  dq_des += (ddq + ref_damp·(dq−dq_des))·dt,
  q_des += dq_des·dt,  servoJ(q_des, ...)

覆盖范围
--------
  - reset():          积分器状态初始化（含 dq 限幅 / dq=None 时置零）
  - 任务空间一致性:    Jb·M⁻¹·τ_imp ≈ dVd* − 2ζω·ev − ω²·e_op
                       （桥接的数学根基：折算后末端行为正是 GIC 目标二阶闭环）
  - Regulation 3cm 步进: GIC→桥接→PD 伺服级联收敛 < 5mm
  - Circle 跟踪:        级联稳定不发散、误差有界、q_target 有限且在关节限位内

仿真模型
--------
内层 UR 伺服用二阶 PD 近似（带宽 ws，阻尼比 ζs），在桥接层每步 dt_ctrl 内
跑多个伺服子步 dt_servo，复现实机「外环阻抗参考 + 内环位置伺服」级联结构。

运行:
    conda activate roboarm
    cd /media/lj252/Data/catkin_ws/roboarm_test/SE3_roboarm_control
    python3 -m pytest tests/test_servo_bridge.py -v
"""

import os
import sys
import unittest

import numpy as np

# 添加项目根 (tests/ 的上一级)
_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from se3_control.config.robot_configs import get_robot_config, get_urdf_path
from se3_control.core.gic_controller import GICController
from se3_control.core.servo_bridge import ServoJTorqueBridge
from se3_control.core.se3_math import (vee_map, adjoint_g_ed, adjoint_g_ed_deriv,
                                       rotmat_slerp)
from se3_control.core.trajectory import build_trajectory, eval_body_twist
from se3_control.robot_model.robot_model import RobotModel


# ====================================================================
# 共享 UR3 模型 (模块级缓存, 避免每个测试重复加载 URDF/Pinocchio)
# ====================================================================

_ROBOT = None
_ROBOT_CFG = None


def _robot():
    global _ROBOT, _ROBOT_CFG
    if _ROBOT is None:
        _ROBOT_CFG = get_robot_config('ur3')
        _ROBOT = RobotModel(get_urdf_path('ur3'),
                            ee_frame_name=_ROBOT_CFG['ee_frame'],
                            robot_name=_ROBOT_CFG['name'], verbose=False)
    return _ROBOT


# ====================================================================
# 内层 UR 伺服模型: 二阶 PD, dt_servo 内子步积分
# ====================================================================

def servo_step(robot, q, dq, q_des, dt_servo, ws, zeta_s):
    """二阶 PD 伺服一个子步 (semi-implicit Euler).

    ddq = ws²·(q_des − q) − 2ζs·ws·dq  — 近似 UR 内层位置伺服闭环
    (伺服自带重力补偿, 故不显式出现重力项).

    :returns: (q, dq) 更新后的关节状态
    """
    ddq = ws * ws * (q_des - q) - 2.0 * zeta_s * ws * dq
    dq = dq + ddq * dt_servo
    q = q + dq * dt_servo
    return q, dq


def run_cascade(robot, bridge, pd, Rd, q0, dq0,
                duration, dt_ctrl, ws=15.0, zeta_s=1.0,
                dt_servo=0.002, traj=None, blend_time=0.0):
    """运行「GIC 桥接 → PD 伺服」级联闭环仿真.

    :param traj: TrajectoryFuncs — 轨迹任务时传入 (每周期按真实时间求值);
                 None = regulation (pd/Rd 恒定, 零前馈).
    :returns: dict — t/p/err/q/q_target/tau
    """
    q = np.asarray(q0, dtype=float).ravel().copy()
    dq = np.asarray(dq0, dtype=float).ravel().copy()
    n_steps = int(duration / dt_ctrl)
    n_sub = max(1, int(round(dt_ctrl / dt_servo)))

    p_start, R_start = None, None
    if traj is not None and blend_time > 0:
        robot.update(q, dq)
        p_start, R_start = robot.get_pose()

    t_log, p_log, err_log, q_target_log, tau_log = [], [], [], [], []

    t = 0.0
    for i in range(n_steps):
        # ── 期望轨迹 (真实时间求值 + 起步混合) ──
        if traj is not None:
            bf = 1.0 if blend_time <= 0 else min(1.0, t / blend_time)
            pd_ref = traj.pd_t(t).ravel()
            Rd_ref = traj.Rd_t(t).reshape(3, 3)
            if bf < 1.0:
                pd = (1.0 - bf) * p_start + bf * pd_ref
                Rd = rotmat_slerp(R_start, Rd_ref, bf)
            else:
                pd, Rd = pd_ref, Rd_ref
            vd, wd, dvd, dwd = eval_body_twist(traj, t, Rd, bf)
        else:
            vd = wd = dvd = dwd = np.zeros(3)

        # ── GIC 桥接: 力矩 → 关节目标位 ──
        robot.update(q, dq)
        q_servo, tau = bridge.compute(q, dq, pd, Rd,
                                      vd, wd, dvd, dwd)

        # ── 内层 PD 伺服: 一个控制周期内跑多个子步 ──
        for _ in range(n_sub):
            q, dq = servo_step(robot, q, dq, q_servo, dt_servo, ws, zeta_s)

        # ── 记录 ──
        robot.update(q, dq)
        p_cur, _ = robot.get_pose()
        err_ref = traj.pd_t(t).ravel() if traj is not None else pd
        t_log.append(t)
        p_log.append(p_cur.copy())
        err_log.append(float(np.linalg.norm(p_cur - err_ref)))
        q_target_log.append(q_servo.copy())
        tau_log.append(np.asarray(tau, dtype=float).ravel().copy())
        t += dt_ctrl

    return dict(
        t=np.asarray(t_log), p=np.asarray(p_log),
        err=np.asarray(err_log),
        q_target=np.asarray(q_target_log),
        tau=np.asarray(tau_log),
    )


def _make_ctrl(robot, bandwidth, torque_limits=None):
    return GICController(robot, bandwidth=bandwidth, damping=1.0,
                         torque_limits=torque_limits)


# ====================================================================
# 1. reset: 积分器状态初始化
# ====================================================================

class TestServoBridgeReset(unittest.TestCase):
    """reset(): 每个相位开始时把积分器复位到当前关节状态."""

    def setUp(self):
        self.robot = _robot()
        cfg = _ROBOT_CFG
        self.ctrl = _make_ctrl(self.robot, bandwidth=10.0,
                               torque_limits=cfg['torque_limits'])
        self.bridge = ServoJTorqueBridge(self.robot, self.ctrl, dt=0.004)

    def test_reset_initializes_q_target_to_clipped_q(self):
        """reset 后 q_target == clip(q, qlo, qhi), dq_des 为零."""
        q = _ROBOT_CFG['home_q'].copy()
        dq = np.zeros(self.robot.nv)
        self.bridge.reset(q, dq)
        q_target = self.bridge.q_target
        lo = self.bridge._qlo
        hi = self.bridge._qhi
        self.assertTrue(np.allclose(q_target, np.clip(q, lo, hi)),
                        f"q_target={q_target} 应等于 clip(q)")
        # 内部期望速度应置零
        self.assertTrue(np.allclose(self.bridge._dq_des, 0))

    def test_reset_without_dq(self):
        """dq=None 时期望速度置零而非报错."""
        self.bridge.reset(_ROBOT_CFG['home_q'].copy())
        self.assertTrue(np.allclose(self.bridge._dq_des, 0))

    def test_reset_clips_dq(self):
        """dq 超限时被限幅到 ±dq_max."""
        big = np.full(self.robot.nv, 99.0)
        self.bridge.reset(_ROBOT_CFG['home_q'].copy(), big)
        dq_max = self.bridge._dq_max
        self.assertTrue(np.allclose(np.abs(self.bridge._dq_des), dq_max))

    def test_compute_before_reset_auto_initializes(self):
        """未显式 reset 时 compute 自动用当前状态初始化 (不抛异常)."""
        q = _ROBOT_CFG['home_q'].copy()
        dq = np.zeros(self.robot.nv)
        self.robot.update(q, dq)
        p, R = self.robot.get_pose()
        pd, Rd = p.copy(), R.copy()
        q_servo, tau = self.bridge.compute(q, dq, pd, Rd,
                                           np.zeros(3), np.zeros(3),
                                           np.zeros(3), np.zeros(3))
        self.assertEqual(q_servo.shape, (self.robot.nv,))
        self.assertTrue(np.all(np.isfinite(q_servo)))
        self.assertTrue(np.all(np.isfinite(tau)))


# ====================================================================
# 1b. 速度缩放感知 (方案 A, real_vs_sim_diagnostics.md §8.4-A)
# ====================================================================

class TestServoBridgeSpeedFraction(unittest.TestCase):
    """实机降速 (RTDE speed_scaling < 1.0) 时参考上限按 s 缩放, 防参考
    积分跑赢被限速的臂而发散 (run_04 根因: s=0.24 时 dq_max 仍 2.0 rad/s)."""

    def setUp(self):
        self.robot = _robot()
        cfg = _ROBOT_CFG
        self.ctrl = _make_ctrl(self.robot, bandwidth=10.0,
                               torque_limits=cfg['torque_limits'])
        # 默认 dt=0.004 → 上升爬升速率 = dt·_SPEED_RAMP_UP = 0.002/步

    def _bridge(self, **kw):
        return ServoJTorqueBridge(self.robot, self.ctrl, dt=0.004, **kw)

    def test_constructor_scales_limits(self):
        """构造时传 speed_fraction=s → dq_max/qdd_max 立即按 s 缩放."""
        s = 0.25
        bridge = self._bridge(speed_fraction=s)
        self.assertAlmostEqual(bridge.speed_fraction, s, places=6)
        np.testing.assert_allclose(bridge._dq_max, bridge._dq_max_base * s)
        np.testing.assert_allclose(bridge._qdd_max, bridge._qdd_max_base * s)
        # 基值本身不受影响 (可恢复)
        self.assertTrue(np.all(bridge._dq_max_base > bridge._dq_max))

    def test_set_speed_fraction_fast_fall(self):
        """下降立即生效: 单步 s:1.0→0.24 后 speed_fraction 直接到 0.24."""
        bridge = self._bridge(speed_fraction=1.0)
        out = bridge.set_speed_fraction(0.24)
        self.assertAlmostEqual(out, 0.24, places=6)
        self.assertAlmostEqual(bridge.speed_fraction, 0.24, places=6)
        np.testing.assert_allclose(bridge._dq_max, bridge._dq_max_base * 0.24)

    def test_set_speed_fraction_slow_rise(self):
        """上升缓慢爬升: 单步 0.24→1.0 只爬到 0.24+dt·0.5, 不突跳."""
        bridge = self._bridge(speed_fraction=0.24)
        out = bridge.set_speed_fraction(1.0)
        expected = 0.24 + bridge.dt * 0.5
        self.assertAlmostEqual(out, expected, places=6)
        self.assertAlmostEqual(bridge.speed_fraction, expected, places=6)

    def test_set_speed_fraction_eventually_converges(self):
        """持续请求 1.0 时爬升至满速 (多步收敛)."""
        bridge = self._bridge(speed_fraction=0.0)
        for _ in range(1000):   # 1000 步 × 0.002 = 2s > (1-0)/0.5
            bridge.set_speed_fraction(1.0)
        self.assertGreaterEqual(bridge.speed_fraction, 1.0 - 1e-9)

    def test_reset_uses_scaled_dq_max(self):
        """降速下 reset 的 dq 限幅用缩放后的上限 (而非满速)."""
        bridge = self._bridge(speed_fraction=0.24)
        big = np.full(self.robot.nv, 99.0)
        bridge.reset(_ROBOT_CFG['home_q'].copy(), big)
        np.testing.assert_allclose(np.abs(bridge._dq_des), bridge._dq_max)
        np.testing.assert_allclose(np.abs(bridge._dq_des),
                                   bridge._dq_max_base * 0.24)

    def test_compute_reference_cap_reflects_fraction(self):
        """compute 步进下参考速度被缩放后的 dq_max 限幅 (防跑赢限速臂)."""
        s = 0.2
        bridge = self._bridge(speed_fraction=s)
        q = _ROBOT_CFG['home_q'].copy()
        dq = np.zeros(self.robot.nv)
        bridge.reset(q, dq)
        # 持续请求大步进 (大误差 → 参考积分持续逼近 dq_max)
        robot = self.robot
        robot.update(q, dq)
        p0, R0 = robot.get_pose()
        pd = p0 + np.array([0.10, 0.0, 0.0])
        Rd = R0.copy()
        for _ in range(200):
            bridge.compute(q, dq, pd, Rd,
                           np.zeros(3), np.zeros(3),
                           np.zeros(3), np.zeros(3))
        self.assertLessEqual(float(np.max(np.abs(bridge._dq_des))),
                             float(np.max(bridge._dq_max)) + 1e-9)
        self.assertLessEqual(float(np.max(np.abs(bridge._dq_des))),
                             float(np.max(bridge._dq_max_base * s)) + 1e-9)

    def test_clip_out_of_range(self):
        """越界输入被夹到 [1e-3, 1.0]."""
        bridge = self._bridge(speed_fraction=1.0)
        bridge.set_speed_fraction(-5.0)
        self.assertGreaterEqual(bridge.speed_fraction, 1e-3)
        bridge.set_speed_fraction(3.0)
        self.assertLessEqual(bridge.speed_fraction, 1.0)

    def test_ref_damp_not_scaled_by_speed_fraction(self):
        """ref_damp 不随 speed_fraction 缩放 (保持基值).

        run_05b 实测否决了 1/s 放大: 24% 降速下臂刹不住、以超参考的速度下坠
        (dq 0.65 > 上限 0.48) 时, 放大的 ref_damp 把参考拉向实测 dq → 参考满速
        往下追 → GIC 纠正加速度被压过 → 坠向基部. 保持基值: 参考不镜像下坠,
        GIC 能把它拉回. 级联稳定性由带宽×s 与参考上限×s 保证.
        """
        base = 15.0
        bridge = self._bridge(speed_fraction=1.0)
        self.assertAlmostEqual(bridge._ref_damp, base, places=6)
        bridge.set_speed_fraction(0.24)
        self.assertAlmostEqual(bridge._ref_damp, base, places=6)   # 不放大
        for _ in range(100):
            bridge.set_speed_fraction(0.077)
        self.assertAlmostEqual(bridge._ref_damp, base, places=6)   # 极低速也不变

    def test_ref_damp_used_in_compute(self):
        """compute 的参考积分使用基值阻尼; 参考速度/加速度上限仍按 s 缩放."""
        s = 0.24
        bridge = self._bridge(speed_fraction=s)
        self.assertAlmostEqual(bridge._ref_damp, 15.0, places=6)   # 阻尼不缩放
        self.assertAlmostEqual(bridge._dq_max[0], 2.0 * s, places=6)  # 上限缩放
        self.assertAlmostEqual(bridge._qdd_max[0], 20.0 * s, places=6)

    def test_track_gate_blocks_chase_when_servo_lost(self):
        """伺服丢跟踪 (|参考−臂|>0.15 rad) 时, 参考只刹不追 — 防坠落时参考被拖走.

        run_05b 根因: 24% 降速下臂刹不住、以超参考的速度下坠 (dq 0.65>上限 0.48),
        对称阻尼把参考拉向实测 dq → 参考满速往下追 → GIC 纠正加速度被压过 → 坠向基部.
        门限后: 臂领先下坠时 (lag>0) 阻尼项 = 0, 参考只随 GIC 命令走, 不放大速度.
        """
        bridge = self._bridge(speed_fraction=0.24)
        self.assertAlmostEqual(bridge._dq_track_gate, 0.15, places=6)
        robot = bridge.robot
        nv = robot.nv
        q = _ROBOT_CFG['home_q'].copy()
        robot.update(q, np.zeros(nv))
        p0, R0 = robot.get_pose()

        def one_step(q_des_err, dqd=0.05, dq_fall=0.2):
            """同 q/dq 状态, 只变参考-臂误差, 返回 dq_des 的 Δ."""
            b = self._bridge(speed_fraction=0.24)
            b.reset(q, np.zeros(nv))
            b._q_des = q + q_des_err
            b._dq_des = np.full(nv, dqd)
            before = b._dq_des.copy()
            b.compute(q, np.full(nv, dq_fall), p0, R0,
                      np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3))
            return b._dq_des - before

        d_lost = one_step(0.2)     # 参考-臂误差 0.2 > 门限 0.15 → 只刹不追
        d_ok = one_step(0.02)      # 0.02 < 门限 → 对称阻尼 (允许追臂耦合)
        # 丢跟踪: 阻尼项=0, 参考不因臂领先而加速 → 明显减速 (对称会把追臂 +15*0.15*dt≈+0.009 加回)
        self.assertLess(d_lost[2], -0.006, f"丢跟踪后参考仍在追下坠臂: Δdq_des={d_lost}")
        # 正常跟踪: 保留对称耦合 → 比丢跟踪情况更不减速 (追臂 boost 在)
        self.assertGreater(d_ok[2], d_lost[2], "正常跟踪的对称耦合应比丢跟踪更不减速")


# ====================================================================
# 2. 任务空间一致性 (桥接数学根基)
# ====================================================================

class TestTorqueTaskConsistency(unittest.TestCase):
    """Jb·M⁻¹·(τ−bias) ≈ dVd* − 2ζω·ev − ω²·e_op — 折算后末端闭环正是
    GIC 期望的二阶响应 (servoJ 紧密跟踪时)."""

    def setUp(self):
        self.robot = _robot()
        # 纯数学检查: 不限幅, 避免饱和破坏恒等式
        self.ctrl = _make_ctrl(self.robot, bandwidth=10.0, torque_limits=None)
        self.bridge = ServoJTorqueBridge(self.robot, self.ctrl, dt=0.004)

    def _check(self, q, dq, pd, Rd):
        robot, ctrl, bridge = self.robot, self.ctrl, self.bridge
        robot.update(q, dq)
        p, R = robot.get_pose()
        M = robot.get_full_inertia()
        bias = robot.get_bias_torque()
        Jb = robot.get_body_jacobian()
        Vb = robot.get_body_ee_velocity()

        vd = wd = dvd = dwd = np.zeros(3)
        tau = ctrl.compute(q, dq, pd, Rd, vd, wd, dvd, dwd)
        tau_imp = tau - bias

        # LHS: Jb·M⁻¹·τ_imp (桥接折算出的末端任务空间加速度)
        lhs = Jb @ np.linalg.solve(M, tau_imp)

        # RHS: 复刻 GIC 内部 (dVd* − 2ζω·ev − ω²·e_op)
        g = np.eye(4); g[:3, :3] = R; g[:3, 3] = p
        gd = np.eye(4); gd[:3, :3] = Rd; gd[:3, 3] = pd
        g_ed = np.linalg.inv(g) @ gd
        Vd = np.hstack((vd, wd)).reshape((-1, 1))
        dVd = np.hstack((dvd, dwd)).reshape((-1, 1))
        Vd_star = adjoint_g_ed(g_ed) @ Vd
        dVd_star = (adjoint_g_ed_deriv(g, gd, Vb[:3], Vb[3:], vd, wd) @ Vd
                    + adjoint_g_ed(g_ed) @ dVd)
        e_pos = R.T @ (p - pd).reshape((-1, 1))
        e_rot = vee_map(Rd.T @ R - R.T @ Rd)
        e_op = np.vstack((e_pos, e_rot))
        ev = Vb.reshape((-1, 1)) - Vd_star
        w = ctrl._w_des
        z2w = 2.0 * ctrl._zeta_des * w
        w2 = w * w
        rhs = dVd_star - z2w * ev - w2 * e_op

        # SVD 阻尼伪逆使 M̃_inv·M̃ ≈ I 而非严格相等 → 用相对容差
        np.testing.assert_allclose(lhs, rhs.ravel(),
                                   rtol=2e-2, atol=1e-2,
                                   err_msg=f"q={np.round(q,3)}")

    def test_home_q_regulation(self):
        """home_q, 零误差期望 (regulation) 恒等式成立."""
        q = _ROBOT_CFG['home_q'].copy()
        self.robot.update(q, np.zeros(self.robot.nv))
        p, R = self.robot.get_pose()
        self._check(q, np.zeros(self.robot.nv), p, R)

    def test_home_q_perturbed_error(self):
        """home_q, 期望位姿偏移 3cm + 转动 → 有误差时恒等式成立."""
        q = _ROBOT_CFG['home_q'].copy()
        self.robot.update(q, np.zeros(self.robot.nv))
        p, R = self.robot.get_pose()
        pd = p + np.array([0.03, -0.02, 0.01])
        Rd = R @ _rotmat_z(0.1)
        self._check(q, np.zeros(self.robot.nv), pd, Rd)

    def test_nonzero_velocity_state(self):
        """带关节速度的状态 (有阻尼项) 恒等式成立."""
        q = _ROBOT_CFG['home_q'].copy()
        self.robot.update(q, np.zeros(self.robot.nv))
        p, R = self.robot.get_pose()
        dq = np.array([0.1, -0.2, 0.3, 0.1, -0.1, 0.2])
        self._check(q, dq, p, R)


# ====================================================================
# 3. Regulation 3cm 步进 — 级联稳定性
# ====================================================================

class TestServoBridgeRegulation(unittest.TestCase):
    """GIC(ω=10) → 桥接 → PD 伺服(ws=15) 级联下, 3cm 步进应收敛 < 5mm.
    (无 ref_damp 且 ω 过高时参考积分会跑赢伺服而发散 — 本测试守卫该回归.)"""

    def setUp(self):
        self.robot = _robot()
        cfg = _ROBOT_CFG
        self.ctrl = _make_ctrl(self.robot, bandwidth=10.0,
                               torque_limits=cfg['torque_limits'])
        self.bridge = ServoJTorqueBridge(self.robot, self.ctrl, dt=0.004,
                                         ref_damp=15.0)

    def test_regulation_step_converges(self):
        robot, bridge, ctrl = self.robot, self.bridge, self.ctrl
        q = cfg_q0 = _ROBOT_CFG['home_q'].copy()
        dq = np.zeros(robot.nv)
        robot.update(q, dq)
        p0, R0 = robot.get_pose()

        # 3cm 位置步进 (x 方向)
        pd = p0 + np.array([0.03, 0.0, 0.0])
        Rd = R0.copy()

        bridge.reset(q, dq)
        sim = run_cascade(robot, bridge, pd, Rd, q, dq,
                          duration=4.0, dt_ctrl=0.004)

        err = sim['err']
        final_err = float(err[-1])
        max_err_after = float(np.max(err[int(0.5 / 0.004):]))  # 0.5s 后
        self.assertLess(final_err, 0.005,
                        f"regulation 3cm 步进未收敛: final_err={final_err*1000:.1f}mm")
        self.assertLess(max_err_after, 0.010,
                        f"0.5s 后最大误差过大: {max_err_after*1000:.1f}mm")

        # q_target 有限且在关节限位内
        q_target = sim['q_target']
        self.assertTrue(np.all(np.isfinite(q_target)))
        lo = bridge._qlo
        hi = bridge._qhi
        self.assertGreaterEqual(np.min(q_target - lo), -1e-9)
        self.assertGreaterEqual(np.min(hi - q_target), -1e-9)

    def test_no_drift_when_holding(self):
        """保持当前位姿 (零误差) 不漂移: 稳态 q_target 收敛."""
        robot, bridge = self.robot, self.bridge
        q = _ROBOT_CFG['home_q'].copy()
        dq = np.zeros(robot.nv)
        robot.update(q, dq)
        p0, R0 = robot.get_pose()
        bridge.reset(q, dq)
        sim = run_cascade(robot, bridge, p0, R0, q, dq,
                          duration=2.0, dt_ctrl=0.004)
        self.assertLess(float(sim['err'][-1]), 0.001,
                        "保持当前位姿不应漂移")


# ====================================================================
# 4. Circle 跟踪 — 级联稳定不发散
# ====================================================================

class TestServoBridgeTrack(unittest.TestCase):
    """Circle 轨迹跟踪: 误差有界 (<30mm), q_target 有限且在限位内.
    (若 ref_damp 被移除或带宽超伺服上限, 参考积分漂移会使误差爆炸到数百 mm —
    本测试用 <30mm 守卫该回归.)"""

    def setUp(self):
        self.robot = _robot()
        cfg = _ROBOT_CFG
        self.ctrl = _make_ctrl(self.robot, bandwidth=10.0,
                               torque_limits=cfg['torque_limits'])
        self.bridge = ServoJTorqueBridge(self.robot, self.ctrl, dt=0.004,
                                         ref_damp=15.0)

    def _start_on_circle(self, traj):
        """IK 到 circle 起点, 使跟踪从轨迹上开始 (无需起步混合)."""
        robot = self.robot
        pd_start = traj.pd_t(0.0).ravel()
        Rd_start = traj.Rd_t(0.0).reshape(3, 3)
        q0 = robot.gauss_newton_IK(pd_start, Rd_start,
                                   _ROBOT_CFG['home_q'],
                                   step_size=0.5, tol=1e-6,
                                   max_cnt=300, verbose=False)
        return q0

    def test_circle_track_bounded(self):
        from se3_control.config import task_config

        robot, bridge = self.robot, self.bridge
        traj = build_trajectory('circle', cfg=task_config)
        q0 = self._start_on_circle(traj)
        dq0 = np.zeros(robot.nv)
        bridge.reset(q0, dq0)

        sim = run_cascade(robot, bridge, None, None, q0, dq0,
                          duration=6.0, dt_ctrl=0.004, traj=traj,
                          blend_time=0.0)

        err = sim['err']
        # 稳态 (1.5s 后, 去掉起步瞬态)
        mask = sim['t'] > 1.5
        err_ss = err[mask]
        self.assertGreater(len(err_ss), 0)
        max_err = float(np.max(err_ss))
        mean_err = float(np.mean(err_ss))
        self.assertLess(max_err, 0.030,
                        f"circle 跟踪误差过大: max={max_err*1000:.1f}mm "
                        f"mean={mean_err*1000:.1f}mm")

        # q_target 有限且在关节限位内
        q_target = sim['q_target']
        self.assertTrue(np.all(np.isfinite(q_target)))
        lo = bridge._qlo
        hi = bridge._qhi
        self.assertGreaterEqual(np.min(q_target - lo), -1e-9)
        self.assertGreaterEqual(np.min(hi - q_target), -1e-9)

        # 力矩不超过 UR3 限幅 (不饱和失控)
        cfg = _ROBOT_CFG
        limits = cfg['torque_limits']
        self.assertLessEqual(np.max(np.abs(sim['tau'])),
                             1.5 * float(np.max(limits)),
                             "力矩应被限幅约束")


def _rotmat_z(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0.0],
                     [s, c, 0.0],
                     [0.0, 0.0, 1.0]])


if __name__ == '__main__':
    unittest.main()
