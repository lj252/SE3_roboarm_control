"""
GIC 控制律 — 自适应操作空间惯性整形
=====================================

Geometric Impedance Controller (GIC) 的核心实现。
只依赖 ``core.se3_math`` (纯 NumPy) 和 ``robot_model.RobotModel`` (Pinocchio)，
与具体机器人硬件无关。

自适应原理
-----------
操作空间惯性矩阵 M̃(q) 在平移和旋转分量间差异可达 10⁵ 倍，
固定增益会使腕关节过刚/过阻尼。本实现根据期望带宽 ω_des 和阻尼比 ζ 自适应:

  K_adapt = ω² · M̃(q)
  D_adapt = 2ζω · M̃(q)

使控制性能在不同位形下保持一致。

用法::

    from core.se3_math import ...
    from core.gic_controller import GICController
    from robot_model.robot_model import RobotModel

    robot = RobotModel(urdf_path, ee_frame_name='tool0')
    ctrl = GICController(robot, bandwidth=30.0, damping=1.0,
                         torque_limits=np.array([165.0]*6))

    tau = ctrl.compute(q, dq, pd, Rd, vd, wd, dvd, dwd)
"""

import numpy as np

from .se3_math import vee_map, adjoint_g_ed, adjoint_g_ed_deriv


class GICController:
    """GIC 控制律 — 自适应 M_tilde 增益.

    :param robot_model: RobotModel 实例 (Pinocchio 封装)
    :param bandwidth:   期望控制带宽 ω_des (rad/s), 默认 30.0 ≈ 5 Hz
    :param damping:     期望阻尼比 ζ, 默认 1.0 (临界阻尼)
    :param torque_limits: 关节力矩限幅 (nv,), 默认 None (不限幅)

    Fe_raw 参数保留接口供 GUFIC 子类使用，GIC 本身不使用外力反馈。
    """

    def __init__(self, robot_model,
                 bandwidth: float = 30.0,
                 damping: float = 1.0,
                 torque_limits: np.ndarray = None):
        self.robot = robot_model
        self._w_des = float(bandwidth)
        self._zeta_des = float(damping)
        self._tau_limits = (np.asarray(torque_limits, dtype=float).ravel()
                            if torque_limits is not None else None)

    # ── 公共接口 ──────────────────────────────────────────────

    def compute(self, q, dq, pd, Rd, vd, wd, dvd, dwd, Fe_raw=None):
        """GIC 控制律单步计算.

        :param q:   关节位置 (nv,)
        :param dq:  关节速度 (nv,)
        :param pd:  期望位置 (3,)
        :param Rd:  期望朝向 (3,3)
        :param vd:  期望线速度 (3,)
        :param wd:  期望角速度 (3,)
        :param dvd: 期望线加速度 (3,)
        :param dwd: 期望角加速度 (3,)
        :param Fe_raw: 外力矩传感器读数 (6,) — GIC 不使用, 保留给 GUFIC
        :returns: 关节力矩指令 (nv,)
        """
        # ── 1. 正运动学 ────────────────────────────────────────
        self.robot.update(q, dq)
        p, R = self.robot.get_pose()
        M = self.robot.get_full_inertia()
        nv = M.shape[0]
        qfrc_bias = self.robot.get_bias_torque()
        Jb = self.robot.get_body_jacobian()

        # ── 2. SE(3) 位姿变换 ──────────────────────────────────
        g = np.eye(4)
        g[:3, :3] = R
        g[:3, 3] = p

        gd = np.eye(4)
        gd[:3, :3] = Rd
        gd[:3, 3] = pd

        g_ed = np.linalg.inv(g) @ gd

        # ── 3. 期望速度变换到体坐标系 ──────────────────────────
        Vd = np.hstack((vd, wd)).reshape((-1, 1))
        dVd = np.hstack((dvd, dwd)).reshape((-1, 1))

        Vd_star = adjoint_g_ed(g_ed) @ Vd
        dVd_star = (adjoint_g_ed_deriv(g, gd, vd, wd, dvd, dwd) @ Vd
                     + adjoint_g_ed(g_ed) @ dVd)

        # ── 4. SE(3) 误差 (体坐标系) ────────────────────────────
        # e_pos = Rᵀ @ (p - pd)
        e_pos = R.T @ (p - pd).reshape((-1, 1))
        # e_rot = vee(Rdᵀ @ R - Rᵀ @ Rd)
        e_rot = vee_map(Rd.T @ R - R.T @ Rd)
        e_op = np.vstack((e_pos, e_rot))

        # ── 5. 速度误差 ────────────────────────────────────────
        Vb = self.robot.get_body_ee_velocity()
        ev = Vb - Vd_star

        # ── 6. 操作空间惯性 ────────────────────────────────────
        M_inv = np.linalg.solve(M, np.eye(nv))
        M_tilde_inv = Jb @ M_inv @ Jb.T

        U_t, s_t, Vt_t = np.linalg.svd(M_tilde_inv)
        damp_sv = max(1e-6, 0.1 * s_t[-1]) if len(s_t) > 0 else 1e-6
        s_damped = s_t / (s_t**2 + damp_sv**2)
        M_tilde = (Vt_t.T * s_damped) @ U_t.T

        # ── 7. 自适应阻抗 ──────────────────────────────────────
        w2 = self._w_des ** 2
        z2w = 2 * self._zeta_des * self._w_des
        K_adapt = w2 * M_tilde
        D_adapt = z2w * M_tilde

        # ── 8. 控制律 ──────────────────────────────────────────
        # τ̃ = M̃·dVd* - D·ev - K·e_op  (负反馈)
        tau_tilde = M_tilde @ dVd_star - D_adapt @ ev - K_adapt @ e_op

        # ── 9. 关节力矩 ────────────────────────────────────────
        tau_cmd = (Jb.T @ tau_tilde + qfrc_bias.reshape((-1, 1))).ravel()

        if self._tau_limits is not None:
            limits = self._tau_limits[:nv]
            tau_cmd = np.clip(tau_cmd, -limits, limits)

        return tau_cmd
