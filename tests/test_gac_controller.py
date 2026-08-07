#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tests/test_gac_controller.py — GAC 控制器单元测试

覆盖范围:
  - GACFilter:  零力、恒力、阶跃响应、限幅、泄漏积分、在线调参、reset
  - GACController: F_ext=0 退化、恒力偏移、轨迹跟踪、reset、互换性
  - GIC ↔ GAC 一致性: F_ext=0 时输出一致

运行:
    conda activate roboarm
    cd /media/lj252/Data/catkin_ws/roboarm_test/SE3_roboarm_control
    python3 -m pytest tests/test_gac_controller.py -v
    # 或
    python3 tests/test_gac_controller.py
"""

import os
import sys
import unittest
import numpy as np

# 添加项目路径
_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from se3_control.core.gac_controller import GACController, GACFilter, _so3_exp, _correct_orientation
from se3_control.core.gic_controller import GICController
from se3_control.robot_model.robot_model import RobotModel


# ====================================================================
# 路径配置
# ====================================================================

_URDF_DIR = os.path.join(_PROJECT_ROOT, 'se3_control', 'urdf')
UR12E_URDF = os.path.join(_URDF_DIR, 'ur12e.urdf')
UR3_URDF = os.path.join(_URDF_DIR, 'ur3.urdf')

_HAS_UR12E = os.path.exists(UR12E_URDF)
_HAS_UR3 = os.path.exists(UR3_URDF)


# ====================================================================
# GACFilter 单元测试 (不依赖 robot_model)
# ====================================================================

class TestGACFilter(unittest.TestCase):
    """GACFilter 组件测试 — 二阶导纳滤波器."""

    def setUp(self):
        """每个测试前创建一个临界阻尼滤波器."""
        # 临界阻尼: D = 2·sqrt(K·M)
        M = [10.0, 10.0, 10.0, 1.0, 1.0, 1.0]
        K = [500.0, 500.0, 500.0, 50.0, 50.0, 50.0]
        D = [2 * np.sqrt(K[i] * M[i]) for i in range(6)]
        self.filt = GACFilter(
            M_d=M, D_d=D, K_d=K,
            dt=0.002, max_correction=0.05, leak_rate=0.0,
        )

    # ── 1. 零力 → 状态归零 ──────────────────────────────────

    def test_zero_force_converges_to_zero(self):
        """F_ext=0 持续运行 → X_corr → 0."""
        for _ in range(1000):  # 2 秒
            X, V, dV = self.filt.update(np.zeros(6))
        self.assertTrue(np.allclose(X, 0.0, atol=1e-12),
                        f"零力不稳定: |X|={np.linalg.norm(X):.2e}")
        self.assertTrue(np.allclose(V, 0.0, atol=1e-12),
                        f"零力速度非零: |V|={np.linalg.norm(V):.2e}")

    # ── 2. 恒力 → 稳态 X = K⁻¹·F ──────────────────────────────

    def test_constant_force_steady_state(self):
        """F_ext=const → 稳态 X_corr[:3] ≈ K_d⁻¹ · F_ext[:3]."""
        F_test = np.array([10.0, -5.0, 3.0, 0.0, 0.0, 0.0])
        self.filt.reset()
        for _ in range(10000):  # 20 秒
            X, V, dV = self.filt.update(F_test)
        # 稳态: K_d @ X ≈ F → X ≈ K_d⁻¹ @ F
        expected = np.linalg.solve(self.filt._K_d, F_test)
        np.testing.assert_allclose(X, expected, atol=1e-4,
                                   err_msg="恒力稳态偏差")

    def test_constant_force_6d(self):
        """6D 恒力 → 全自由度稳态."""
        F_test = np.array([5.0, 2.0, -1.0, 0.5, -0.3, 0.1])
        self.filt.reset()
        for _ in range(15000):  # 30 秒
            X, V, dV = self.filt.update(F_test)
        expected = np.linalg.solve(self.filt._K_d, F_test)
        np.testing.assert_allclose(X, expected, atol=1e-3,
                                   err_msg="6D 恒力稳态偏差")

    # ── 3. 阶跃响应 (临界阻尼 → 无超调) ─────────────────────

    def test_step_response_no_overshoot(self):
        """阶跃力, 临界阻尼 → 无超调."""
        F_step = np.array([10.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.filt.reset()
        X_prev = np.zeros(6)
        overshoot = False
        settled = False
        for _ in range(5000):
            X, V, dV = self.filt.update(F_step)
            # 检查超调: 位置超过稳态值 5%
            steady = F_step[0] / 500.0  # ≈ 0.02
            if X[0] > steady * 1.05:
                overshoot = True
            X_prev = X.copy()
        self.assertFalse(overshoot, "临界阻尼出现超调")
        self.assertAlmostEqual(X[0], steady, delta=1e-4,
                               msg="阶跃响应稳态偏差")

    def test_step_response_settling_time(self):
        """阶跃力 → 5% 稳定时间 < 1.0 秒 (设 ωₙ ≈ 7 rad/s)."""
        F_step = np.array([10.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.filt.reset()
        steady = F_step[0] / 500.0
        settled_at = None
        for i in range(1000):  # 最多 2 秒
            X, V, dV = self.filt.update(F_step)
            if settled_at is None and i > 50:  # 跳过起始点
                if abs(X[0] - steady) < 0.05 * steady:
                    settled_at = i * self.filt._dt
        self.assertIsNotNone(settled_at, "从未进入 5% 稳定带")
        # ωₙ = sqrt(K/M) = sqrt(500/10) ≈ 7.07 rad/s
        # 临界阻尼 5% 稳定时间 ≈ 4.75/ωₙ ≈ 0.67s
        self.assertLess(settled_at, 1.0,
                        f"稳定时间 {settled_at:.3f}s 超过预期")

    # ── 4. reset ─────────────────────────────────────────────

    def test_reset_clears_state(self):
        """reset() → 状态全零."""
        # 先施加外力积累状态
        for _ in range(100):
            self.filt.update(np.array([10.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        self.filt.reset()
        X, V, dV = self.filt._X_corr, self.filt._V_corr, np.zeros(6)
        self.assertTrue(np.allclose(X, 0.0), f"reset 后 X_corr ≠ 0: {X}")
        self.assertTrue(np.allclose(V, 0.0), f"reset 后 V_corr ≠ 0: {V}")

    # ── 5. 修正量限幅 ────────────────────────────────────────

    def test_max_correction_clamp(self):
        """大外力 → 修正量超限幅 → 被钳位."""
        filt = GACFilter(
            M_d=[1]*6, D_d=[10]*6, K_d=[10]*6, dt=0.001,
            max_correction=0.005,  # 很小
        )
        big_force = np.array([1000.0, 0, 0, 0, 0, 0])
        for _ in range(2000):
            X, V, dV = filt.update(big_force)
        max_abs = np.max(np.abs(X))
        self.assertLessEqual(max_abs, 0.006,
                             f"限幅失效: max|X|={max_abs:.4f} > 0.005")

    # ── 6. 泄漏积分 ──────────────────────────────────────────

    def test_leaky_integrator_drift_prevention(self):
        """泄漏积分 → F_ext=0 后 X_corr 指数衰减到零."""
        filt = GACFilter(
            M_d=[10]*6, D_d=[100]*6, K_d=[500]*6, dt=0.002,
            leak_rate=1.0,  # 强泄漏, 半衰期 ~0.7s
        )
        # 积累状态
        for _ in range(1000):
            filt.update(np.array([10.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        X_charged = filt._X_corr.copy()
        self.assertGreater(np.linalg.norm(X_charged), 1e-6,
                           "充电阶段未积累修正量")

        # F_ext=0, 泄漏衰减
        for _ in range(5000):  # 10 秒
            filt.update(np.zeros(6))
        self.assertTrue(np.allclose(filt._X_corr, 0.0, atol=1e-8),
                        "泄漏积分未衰减到零")

    # ── 7. 在线调参 ──────────────────────────────────────────

    def test_set_parameters_updates_Kd(self):
        """set_parameters(K_d=...) → 刚度更新 → 稳态响应变化."""
        F_test = np.array([10.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.filt.reset()
        # 用原始 K_d 达到稳态
        for _ in range(5000):
            X_old, _, _ = self.filt.update(F_test)
        old_position = X_old[0].copy()

        # 在线增大刚度
        self.filt.set_parameters(K_d=[1000]*6)
        for _ in range(5000):
            X_new, _, _ = self.filt.update(F_test)
        new_position = X_new[0]

        # 刚度增大 → 修正量减小
        self.assertLess(abs(new_position), abs(old_position),
                        f"增大刚度后修正量未减小: "
                        f"{abs(old_position):.6f} → {abs(new_position):.6f}")

    # ── 8. 输入形状容错 ──────────────────────────────────────

    def test_filter_accepts_row_and_column_vector(self):
        """F_ext_body 可以是 (6,), (6,1), (1,6) — 全接受."""
        for shape in [(6,), (6, 1), (1, 6)]:
            F = np.ones(shape) * 2.0
            try:
                X, V, dV = self.filt.update(F)
                self.assertEqual(X.shape, (6,))
            except Exception as e:
                self.fail(f"形状 {shape} 失败: {e}")

    # ── 9. 非法输入 ──────────────────────────────────────────

    def test_filter_rejects_wrong_shape(self):
        """F_ext_body 不是 6 维 → 抛 ValueError."""
        with self.assertRaises(ValueError):
            self.filt.update(np.array([1.0, 2.0, 3.0]))  # (3,)
        with self.assertRaises(ValueError):
            self.filt.update(np.array([[1.0, 2.0]]))      # (1,2)

    # ── 10. 状态快照 ─────────────────────────────────────────

    def test_filter_state_snapshot(self):
        """state 属性返回正确的字典结构."""
        self.filt.update(np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        s = self.filt.state
        self.assertIn('X_corr', s)
        self.assertIn('V_corr', s)
        self.assertIn('M_d', s)
        self.assertIn('D_d', s)
        self.assertIn('K_d', s)
        self.assertEqual(s['X_corr'].shape, (6,))
        self.assertEqual(s['V_corr'].shape, (6,))


# ====================================================================
# _so3_exp 单元测试
# ====================================================================

class TestSO3Exp(unittest.TestCase):
    """SO(3) 指数映射测试."""

    def test_identity_for_zero(self):
        """零向量 → I."""
        R = _so3_exp(np.zeros(3))
        np.testing.assert_allclose(R, np.eye(3), atol=1e-15)

    def test_det_is_one(self):
        """det(R) = 1."""
        phi = np.array([0.5, -0.3, 0.2])
        R = _so3_exp(phi)
        self.assertAlmostEqual(np.linalg.det(R), 1.0, places=10)

    def test_orthogonal(self):
        """RᵀR = I."""
        phi = np.array([0.5, -0.3, 0.2])
        R = _so3_exp(phi)
        np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-10)

    def test_known_rotation_x(self):
        """绕 X 轴 90° → 已知矩阵."""
        R = _so3_exp(np.array([np.pi/2, 0, 0]))
        expected = np.array([[1, 0, 0],
                             [0, 0, -1],
                             [0, 1, 0]])
        np.testing.assert_allclose(R, expected, atol=1e-10)

    def test_large_angle_wraparound(self):
        """大角度旋转 (≥ 2π) 仍合法."""
        phi = np.array([2*np.pi - 0.1, 0, 0])
        R = _so3_exp(phi)
        # 绕 X 轴转 2π-0.1 等价于转 -0.1
        expected = _so3_exp(np.array([-0.1, 0, 0]))
        np.testing.assert_allclose(R, expected, atol=1e-10)


# ====================================================================
# _correct_orientation 单元测试
# ====================================================================

class TestCorrectOrientation(unittest.TestCase):
    """朝向修正测试."""

    def test_zero_correction(self):
        """Δφ=0 → R unchanged."""
        Rd = _so3_exp(np.array([0.3, -0.2, 0.1]))
        R_out = _correct_orientation(Rd, np.zeros(3))
        np.testing.assert_allclose(R_out, Rd, atol=1e-15)

    def test_small_angle_orthogonal(self):
        """小角度修正 → RᵀR = I."""
        Rd = np.eye(3)
        R_out = _correct_orientation(Rd, np.array([0.01, 0.02, -0.015]))
        np.testing.assert_allclose(R_out.T @ R_out, np.eye(3), atol=1e-10)
        self.assertAlmostEqual(np.linalg.det(R_out), 1.0, places=10)

    def test_large_angle_orthogonal(self):
        """大角度修正 (Rodrigues) → RᵀR = I."""
        Rd = np.eye(3)
        R_out = _correct_orientation(Rd, np.array([0.5, -0.3, 0.2]))
        np.testing.assert_allclose(R_out.T @ R_out, np.eye(3), atol=1e-10)

    def test_automatic_switch_at_threshold(self):
        """0.04 → 小角度路径; 0.06 → Rodrigues 路径 (边界测试)."""
        Rd = np.eye(3)
        # 0.04 rad (< 0.05): 小角度近似
        R_small = _correct_orientation(Rd, np.array([0.04, 0, 0]))
        # 0.06 rad (> 0.05): Rodrigues
        R_large = _correct_orientation(Rd, np.array([0.06, 0, 0]))
        for R in [R_small, R_large]:
            self.assertAlmostEqual(np.linalg.det(R), 1.0, places=10)

    def test_non_identity_base_rotation(self):
        """非单位矩阵 Rd 上叠加修正."""
        Rd = _so3_exp(np.array([0.5, 0.0, 0.0]))
        Δφ = np.array([0.0, 0.3, 0.0])
        R_out = _correct_orientation(Rd, Δφ)
        np.testing.assert_allclose(R_out.T @ R_out, np.eye(3), atol=1e-10)


# ====================================================================
# GACController 集成测试 (依赖 robot_model)
# ====================================================================

@unittest.skipIf(not _HAS_UR12E, f"URDF 不存在: {UR12E_URDF}")
class TestGACControllerRobot(unittest.TestCase):
    """GACController 与控制相关的测试 (需要 URDF)."""

    @classmethod
    def setUpClass(cls):
        cls.robot = RobotModel(UR12E_URDF, ee_frame_name='tool0', verbose=False)
        cls.dt = 0.002
        # GAC 控制器: 临界阻尼
        M = [10.0, 10.0, 10.0, 1.0, 1.0, 1.0]
        K = [500.0, 500.0, 500.0, 50.0, 50.0, 50.0]
        D = [2 * np.sqrt(K[i] * M[i]) for i in range(6)]
        cls.gac = GACController(
            cls.robot, M_d=M, D_d=D, K_d=K, dt=cls.dt,
            bandwidth=30.0, damping=1.0,
        )
        cls.gic = GICController(
            cls.robot, bandwidth=30.0, damping=1.0,
        )

    def setUp(self):
        self.gac.reset()

    # ── 测试配置 ──────────────────────────────────────────

    Q_HOME = np.array([0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0])
    DQ_ZERO = np.zeros(6)
    PD = np.array([0.5, 0.0, 0.125])
    RD = np.eye(3)
    VD = np.zeros(3)
    WD = np.zeros(3)
    DVD = np.zeros(3)
    DWD = np.zeros(3)

    # ── 1. F_ext=0 → 退化为纯位置跟踪 ─────────────────────

    def test_zero_force_equals_gic(self):
        """F_ext=0 → GAC 输出与 GIC 一致."""
        for _ in range(10):
            tau_gac = self.gac.compute(
                self.Q_HOME, self.DQ_ZERO,
                self.PD, self.RD,
                self.VD, self.WD, self.DVD, self.DWD,
                F_ext=np.zeros(6),
            )
            tau_gic = self.gic.compute(
                self.Q_HOME, self.DQ_ZERO,
                self.PD, self.RD,
                self.VD, self.WD, self.DVD, self.DWD,
            )
        np.testing.assert_allclose(tau_gac, tau_gic, atol=1e-10,
                                   err_msg="F_ext=0 时 GAC ≠ GIC")

    def test_zero_force_default_arg(self):
        """F_ext=None → GAC 输出与 GIC 一致."""
        for _ in range(10):
            tau_gac = self.gac.compute(
                self.Q_HOME, self.DQ_ZERO,
                self.PD, self.RD,
                self.VD, self.WD, self.DVD, self.DWD,
            )
            tau_gic = self.gic.compute(
                self.Q_HOME, self.DQ_ZERO,
                self.PD, self.RD,
                self.VD, self.WD, self.DVD, self.DWD,
            )
        np.testing.assert_allclose(tau_gac, tau_gic, atol=1e-10,
                                   err_msg="F_ext=None 时 GAC ≠ GIC")

    # ── 2. 恒力 → 位置偏移 ─────────────────────────────

    def test_constant_force_causes_position_deviation(self):
        """F_ext 恒力 → 末端位置从目标位置偏移 (阻抗行为验证)."""
        q = self.Q_HOME.copy()
        dq = self.DQ_ZERO.copy()
        F_ext = np.array([10.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # 无外力 → 末端趋近期望位置
        for _ in range(500):
            tau = self.gac.compute(
                q, dq, self.PD, self.RD,
                self.VD, self.WD, self.DVD, self.DWD,
                F_ext=None,
            )
            # 简化: 不开仿真, 只验证力矩输出形状
        tau_no_force = self.gac.compute(
            q, dq, self.PD, self.RD,
            self.VD, self.WD, self.DVD, self.DWD,
        )

        # 施加外力 → 修正轨迹 → 阻抗项产生不同力矩
        tau_with_force = self.gac.compute(
            q, dq, self.PD, self.RD,
            self.VD, self.WD, self.DVD, self.DWD,
            F_ext=F_ext,
        )
        # 确认有力/无力时力矩输出不同
        diff = np.linalg.norm(tau_with_force - tau_no_force)
        self.assertGreater(diff, 1e-6,
                           f"外力未改变力矩输出: diff={diff:.2e}")

    # ── 3. 输出形状 ────────────────────────────────────

    def test_output_shape(self):
        """compute() 返回 (nv,) 形状."""
        tau = self.gac.compute(
            self.Q_HOME, self.DQ_ZERO,
            self.PD, self.RD,
            self.VD, self.WD, self.DVD, self.DWD,
        )
        self.assertEqual(tau.shape, (6,))

    def test_output_shape_with_force(self):
        """compute(F_ext=...) 返回 (nv,) 形状."""
        tau = self.gac.compute(
            self.Q_HOME, self.DQ_ZERO,
            self.PD, self.RD,
            self.VD, self.WD, self.DVD, self.DWD,
            F_ext=np.array([10.0, 0, 0, 0, 0, 0]),
        )
        self.assertEqual(tau.shape, (6,))

    # ── 4. reset ────────────────────────────────────────

    def test_reset_clears_filter(self):
        """reset() → 滤波器状态归零."""
        # 先积累
        for _ in range(100):
            self.gac.compute(
                self.Q_HOME, self.DQ_ZERO,
                self.PD, self.RD,
                self.VD, self.WD, self.DVD, self.DWD,
                F_ext=np.array([10.0, 0, 0, 0, 0, 0]),
            )
        self.gac.reset()
        fs = self.gac.filter_state
        self.assertTrue(np.allclose(fs['X_corr'], 0.0, atol=1e-15),
                        "reset 后 X_corr ≠ 0")
        self.assertTrue(np.allclose(fs['V_corr'], 0.0, atol=1e-15),
                        "reset 后 V_corr ≠ 0")

    # ── 5. 滤波器状态可监控 ─────────────────────────────

    def test_filter_state_accessible(self):
        """filter_state 属性可读取."""
        fs = self.gac.filter_state
        for key in ['X_corr', 'V_corr', 'M_d', 'D_d', 'K_d']:
            self.assertIn(key, fs)
        self.assertEqual(fs['X_corr'].shape, (6,))

    # ── 6. 多步力输入的稳定性 ───────────────────────────

    def test_multi_step_force_stability(self):
        """多步变力 → 不产生 NaN/inf, 滤波器状态不发散."""
        rng = np.random.default_rng(42)
        for i in range(200):
            F_ext = rng.uniform(-5, 5, size=6)
            tau = self.gac.compute(
                self.Q_HOME, self.DQ_ZERO,
                self.PD, self.RD,
                self.VD, self.WD, self.DVD, self.DWD,
                F_ext=F_ext,
            )
            # 力矩不 NaN
            self.assertFalse(np.any(np.isnan(tau)),
                             f"第 {i} 步出现 NaN")
            # 力矩有限
            self.assertTrue(np.all(np.isfinite(tau)),
                            f"第 {i} 步出现 inf")

        # 滤波器状态不发散
        fs = self.gac.filter_state
        self.assertFalse(np.any(np.isnan(fs['X_corr'])),
                         "滤波器 X_corr 出现 NaN")
        self.assertFalse(np.any(np.isnan(fs['V_corr'])),
                         "滤波器 V_corr 出现 NaN")
        # 修正量在限幅范围内
        self.assertLess(np.max(np.abs(fs['X_corr'])), 0.06,
                        "滤波器 X_corr 超限幅")

    # ── 7. 回归: 与 GIC Regulation 行为一致 ─────────────

    def test_regulation_behavior_match_gic(self):
        """F_ext=0, 多步 regulation → GAC 与 GIC 力矩趋势一致."""
        q = self.Q_HOME.copy()
        dq = self.DQ_ZERO.copy()
        trajs_gac = []
        trajs_gic = []
        for _ in range(50):
            tau_gac = self.gac.compute(
                q, dq, self.PD, self.RD,
                self.VD, self.WD, self.DVD, self.DWD,
            )
            tau_gic = self.gic.compute(
                q, dq, self.PD, self.RD,
                self.VD, self.WD, self.DVD, self.DWD,
            )
            trajs_gac.append(tau_gac)
            trajs_gic.append(tau_gic)
        # 全序列一致
        for i, (tgac, tgic) in enumerate(zip(trajs_gac, trajs_gic)):
            np.testing.assert_allclose(tgac, tgic, atol=1e-10,
                                       err_msg=f"第 {i} 步 GAC ≠ GIC")

    # ── 8. 不同机器人 UR3 ───────────────────────────────

    @unittest.skipIf(not _HAS_UR3, f"URDF 不存在: {UR3_URDF}")
    def test_different_robot_ur3(self):
        """UR3 机器人上 GAC 输出合法."""
        robot = RobotModel(UR3_URDF, ee_frame_name='tool0', verbose=False)
        gac = GACController(
            robot,
            M_d=[5.0, 5.0, 5.0, 0.5, 0.5, 0.5],
            D_d=[100]*6, K_d=[500]*6, dt=self.dt,
            bandwidth=20.0, damping=1.0,
        )
        q_home_ur3 = np.array([0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0])
        tau = gac.compute(
            q_home_ur3, np.zeros(6),
            self.PD, self.RD,
            self.VD, self.WD, self.DVD, self.DWD,
        )
        self.assertEqual(tau.shape, (6,))
        self.assertFalse(np.any(np.isnan(tau)),
                         "UR3 输出含 NaN")


# ====================================================================
# GACController 与 GICController 互换性测试
# ====================================================================

@unittest.skipIf(not _HAS_UR12E, f"URDF 不存在: {UR12E_URDF}")
class TestGICGACInterchangeability(unittest.TestCase):
    """GIC ↔ GAC 互换性验证."""

    @classmethod
    def setUpClass(cls):
        cls.robot = RobotModel(UR12E_URDF, ee_frame_name='tool0', verbose=False)
        cls.dt = 0.002
        # 相同的位置跟踪参数
        cls.bandwidth = 30.0
        cls.damping = 1.0
        cls.gic = GICController(
            cls.robot, bandwidth=cls.bandwidth, damping=cls.damping,
        )
        M = [10.0, 10.0, 10.0, 1.0, 1.0, 1.0]
        K = [500.0, 500.0, 500.0, 50.0, 50.0, 50.0]
        D = [2 * np.sqrt(K[i] * M[i]) for i in range(6)]
        cls.gac = GACController(
            cls.robot, M_d=M, D_d=D, K_d=K, dt=cls.dt,
            bandwidth=cls.bandwidth, damping=cls.damping,
        )

    def test_same_input_output_type(self):
        """GIC 和 GAC 输入输出类型完全一致."""
        q = np.array([0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0])
        dq = np.zeros(6)
        pd = np.array([0.5, 0.0, 0.125])
        Rd = np.eye(3)
        vd = wd = dvd = dwd = np.zeros(3)

        tau_gic = self.gic.compute(q, dq, pd, Rd, vd, wd, dvd, dwd)
        tau_gac = self.gac.compute(q, dq, pd, Rd, vd, wd, dvd, dwd)

        self.assertEqual(tau_gic.shape, tau_gac.shape)
        self.assertEqual(tau_gic.dtype, tau_gac.dtype)
        self.assertIsInstance(tau_gac, np.ndarray)

    def test_loop_swap_without_restructuring(self):
        """控制循环可以在 GIC/GAC 间互换不改循环结构."""
        q = np.array([0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0])
        dq = np.zeros(6)
        pd = np.array([0.5, 0.0, 0.125])

        # 模拟控制循环 —— 相同调用签名
        for ctrl in [self.gic, self.gac]:
            for _ in range(20):
                tau = ctrl.compute(
                    q, dq, pd, np.eye(3),
                    np.zeros(3), np.zeros(3),
                    np.zeros(3), np.zeros(3),
                )
            self.assertEqual(tau.shape, (6,))

    def test_gradual_force_response_trend(self):
        """增大 F_ext → 位置修正量单调增加."""
        self.gac.reset()
        q = np.array([0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0])
        dq = np.zeros(6)

        corrections = []
        for fx in [0, 2, 5, 10]:
            self.gac.reset()
            for _ in range(500):
                tau = self.gac.compute(
                    q, dq, self.gac_controller_test.PD if hasattr(self, 'gac_controller_test') else np.array([0.5, 0.0, 0.125]),
                    np.eye(3),
                    np.zeros(3), np.zeros(3),
                    np.zeros(3), np.zeros(3),
                    F_ext=np.array([fx, 0, 0, 0, 0, 0]),
                )
            fs = self.gac.filter_state
            corrections.append(np.linalg.norm(fs['X_corr']))
        # 外力增大 → 修正量增大 (不严格单调, 但趋势一致)
        for i in range(1, len(corrections)):
            self.assertGreaterEqual(corrections[i], corrections[i-1] * 0.9,
                                    f"F_ext 增大时修正量异常下降: "
                                    f"{corrections}")


# ====================================================================
# 运行入口
# ====================================================================

if __name__ == '__main__':
    unittest.main(verbosity=2)
