"""
GAC 控制律 — SE(3) 几何导纳控制（自包含实现）
=============================================

Geometric Admittance Controller (GAC) 的核心实现。
只依赖 ``core.se3_math`` (纯 NumPy) 和 ``robot_model.RobotModel`` (Pinocchio)，
与具体机器人硬件无关，**不导入 GICController**。

三层结构
---------
  1. GACFilter  — 二阶导纳滤波器: M_d·dV + D_d·V + K_d·X = F_ext
  2. 轨迹修正     — 将滤波器输出叠加到期望轨迹 (pd, Rd → pd', Rd')
  3. 位置跟踪     — SE(3) 自适应 M_tilde 跟踪修正后的轨迹 → τ

当 F_ext = None 时退化为纯位置跟踪（阻抗模式）。

用法::

    from core.gac_controller import GACController

    ctrl = GACController(robot,
        M_d=[10.0, 10.0, 10.0, 1.0, 1.0, 1.0],
        D_d=[100.0, 100.0, 100.0, 10.0, 10.0, 10.0],
        K_d=[500.0, 500.0, 500.0, 50.0, 50.0, 50.0],
        dt=0.002, bandwidth=30.0, damping=1.0)

    # 纯位置跟踪 (F_ext 可选)
    tau = ctrl.compute(q, dq, pd, Rd, vd, wd, dvd, dwd)

    # 导纳模式 (有力传感器)
    tau = ctrl.compute(q, dq, pd, Rd, vd, wd, dvd, dwd, F_ext=f_ext)
"""

import sys
from typing import Tuple

import numpy as np

# 当作为脚本运行时，添加项目路径以便相对导入
if __name__ == '__main__':
    import os
    _script_dir = os.path.dirname(os.path.abspath(__file__))
    _project_dir = os.path.dirname(_script_dir)  # se3_control/
    if _project_dir not in sys.path:
        sys.path.insert(0, _project_dir)

from .se3_math import vee_map, adjoint_g_ed, adjoint_g_ed_deriv, hat_map


# ====================================================================
# GACFilter — 导纳滤波器内部组件
# ====================================================================

class GACFilter:
    """SE(3) 体坐标系导纳滤波器 — 外力 → 轨迹修正量.

    实现虚拟二阶动力学:
      M_d · dV_corr + D_d · V_corr + K_d · X_corr = F_ext_body

    状态量 X_corr, V_corr 在体坐标系中定义。
    泄漏积分 (leaky integrator) 防止力传感器零漂导致的位置漂移。
    修正量限幅防止超出安全工作空间。

    :param M_d: 虚拟质量 (6,) 对角值 或 (6,6) 矩阵
    :param D_d: 虚拟阻尼 (6,) 对角值 或 (6,6) 矩阵
    :param K_d: 虚拟刚度 (6,) 对角值 或 (6,6) 矩阵
    :param dt: 控制周期 (s)
    :param max_correction: 最大修正量 (m/rad), 默认 0.05
    :param leak_rate: 泄漏率, 0=无泄漏, 建议 0.01~0.1 防漂移, 默认 0.0
    """

    def __init__(self,
                 M_d: np.ndarray,
                 D_d: np.ndarray,
                 K_d: np.ndarray,
                 dt: float,
                 max_correction: float = 0.05,
                 leak_rate: float = 0.0):
        # ── 将输入统一为 (6,6) 矩阵 ──────────────────────────
        M_d = np.asarray(M_d, dtype=float).ravel()
        D_d = np.asarray(D_d, dtype=float).ravel()
        K_d = np.asarray(K_d, dtype=float).ravel()

        if M_d.size == 6:
            self._M_d = np.diag(M_d)
        elif M_d.shape == (6, 6):
            self._M_d = M_d
        else:
            raise ValueError(f"M_d 应为 (6,) 或 (6,6), 实际 {M_d.shape}")

        if D_d.size == 6:
            self._D_d = np.diag(D_d)
        elif D_d.shape == (6, 6):
            self._D_d = D_d
        else:
            raise ValueError(f"D_d 应为 (6,) 或 (6,6), 实际 {D_d.shape}")

        if K_d.size == 6:
            self._K_d = np.diag(K_d)
        elif K_d.shape == (6, 6):
            self._K_d = K_d
        else:
            raise ValueError(f"K_d 应为 (6,) 或 (6,6), 实际 {K_d.shape}")

        self._dt = float(dt)
        self._max_correction = float(max_correction)
        self._leak_rate = float(leak_rate)

        # ── 滤波器状态 (体坐标系) ──────────────────────────────
        self._X_corr = np.zeros(6)   # 位姿修正量 [Δp; Δφ]
        self._V_corr = np.zeros(6)   # 速度修正量

    # ── 公共接口 ──────────────────────────────────────────────

    def update(self, F_ext_body: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """一步滤波更新.

        :param F_ext_body: 体坐标系外力/力矩 (6,)
        :returns: (X_corr, V_corr, dV_corr) — 修正量、速度、加速度，均为 (6,)
        """
        F_ext_body = np.asarray(F_ext_body, dtype=float).ravel()
        if F_ext_body.shape != (6,):
            raise ValueError(f"F_ext_body 应为 (6,), 实际 {F_ext_body.shape}")

        # M_d · dV_corr + D_d · V_corr + K_d · X_corr = F_ext
        # → dV_corr = M_d⁻¹ · (F_ext - D_d·V_corr - K_d·X_corr)
        dV_corr = np.linalg.solve(
            self._M_d,
            F_ext_body - self._D_d @ self._V_corr - self._K_d @ self._X_corr,
        )

        # 显式积分 (前向欧拉)
        self._V_corr += dV_corr * self._dt
        self._X_corr += self._V_corr * self._dt

        # 泄漏积分 (防零漂)
        if self._leak_rate > 0.0:
            leak_factor = 1.0 - self._leak_rate * self._dt
            leak_factor = np.clip(leak_factor, 0.0, 1.0)
            self._X_corr *= leak_factor

        # 修正量限幅
        self._X_corr = np.clip(
            self._X_corr,
            -self._max_correction,
            self._max_correction,
        )

        return self._X_corr.copy(), self._V_corr.copy(), dV_corr

    def reset(self):
        """重置滤波器状态为零."""
        self._X_corr[:] = 0.0
        self._V_corr[:] = 0.0

    def set_parameters(self, M_d=None, D_d=None, K_d=None):
        """在线更新虚拟阻抗参数.

        :param M_d: (6,) 或 (6,6), None=保持原值
        :param D_d: (6,) 或 (6,6), None=保持原值
        :param K_d: (6,) 或 (6,6), None=保持原值
        """
        if M_d is not None:
            M_d_a = np.asarray(M_d, dtype=float).ravel()
            self._M_d = np.diag(M_d_a) if M_d_a.size == 6 else M_d
        if D_d is not None:
            D_d_a = np.asarray(D_d, dtype=float).ravel()
            self._D_d = np.diag(D_d_a) if D_d_a.size == 6 else D_d
        if K_d is not None:
            K_d_a = np.asarray(K_d, dtype=float).ravel()
            self._K_d = np.diag(K_d_a) if K_d_a.size == 6 else K_d

    # ── 属性 ──────────────────────────────────────────────────

    @property
    def state(self) -> dict:
        """当前滤波器状态快照 (用于监控/记录)."""
        return {
            'X_corr': self._X_corr.copy(),
            'V_corr': self._V_corr.copy(),
            'M_d': self._M_d.copy(),
            'D_d': self._D_d.copy(),
            'K_d': self._K_d.copy(),
        }


# ====================================================================
# SO(3) 指数映射 (Rodrigues 公式)
# ====================================================================

def _so3_exp(phi):
    """ℝ³ → SO(3) 指数映射 (Rodrigues 旋转公式).

    :param phi: ndarray (3,) — 旋转向量
    :returns: ndarray (3,3) — 旋转矩阵
    """
    theta = np.linalg.norm(phi)
    if theta < 1e-10:
        return np.eye(3)
    axis = phi / theta
    K = hat_map(axis)
    return (np.eye(3)
            + np.sin(theta) * K
            + (1.0 - np.cos(theta)) * (K @ K))


def _correct_orientation(Rd, Δφ):
    """将体坐标系旋转修正量 Δφ 叠加到期望朝向 Rd.

    小角度近似 (||Δφ|| < 0.05 rad): R' = Rd + hat(Δφ) @ Rd
    大角度 (≥ 0.05): 完整 Rodrigues 公式
    修正后 SVD 重正交化保证 SO(3) 性质.

    :param Rd: 期望朝向 (3,3)
    :param Δφ: 体坐标系旋转修正量 (3,)
    :returns: 修正后的朝向 (3,3)
    """
    theta = np.linalg.norm(Δφ)
    if theta < 1e-12:
        return Rd.copy()

    if theta < 0.05:
        # 小角度近似: exp(hat(Δφ)) ≈ I + hat(Δφ)
        Rd_corrected = Rd + hat_map(Δφ) @ Rd
    else:
        # 完整 Rodrigues
        Rd_corrected = Rd @ _so3_exp(Δφ)

    # SVD 重正交化保证 det(R) = 1, RᵀR = I
    U, _, Vt = np.linalg.svd(Rd_corrected)
    return U @ Vt


# ====================================================================
# GACController — 主控制器
# ====================================================================

class GACController:
    """SE(3) 导纳控制器 (Geometric Admittance Controller).

    自包含实现，不依赖 GICController。
    三层流程: 导纳滤波 → 轨迹修正 → SE(3) 位置跟踪

    当 F_ext = None 时退化为纯位置跟踪（退化模式），
    此时 GAC 行为等效于 GIC 位置跟踪器。

    :param robot_model: RobotModel 实例 (Pinocchio 封装)
    :param M_d: 虚拟质量 (6,) 对角值
    :param D_d: 虚拟阻尼 (6,) 对角值
    :param K_d: 虚拟刚度 (6,) 对角值
    :param dt: 控制周期 (s)
    :param bandwidth: 内环期望带宽 ω_des (rad/s), 默认 30.0
    :param damping: 内环期望阻尼比 ζ, 默认 1.0 (临界阻尼)
    :param torque_limits: 关节力矩限幅 (nv,), 默认 None (不限幅)
    :param max_correction: 最大修正量 (m/rad), 默认 0.05
    :param filter_leak_rate: 滤波器泄漏率, 默认 0.0
    """

    def __init__(self,
                 robot_model,
                 M_d: np.ndarray,
                 D_d: np.ndarray,
                 K_d: np.ndarray,
                 dt: float,
                 bandwidth: float = 30.0,
                 damping: float = 1.0,
                 torque_limits: np.ndarray = None,
                 max_correction: float = 0.05,
                 filter_leak_rate: float = 0.0):
        self.robot = robot_model
        self._w_des = float(bandwidth)
        self._zeta_des = float(damping)
        self._tau_limits = (np.asarray(torque_limits, dtype=float).ravel()
                            if torque_limits is not None else None)

        self._filter = GACFilter(
            M_d=M_d, D_d=D_d, K_d=K_d, dt=dt,
            max_correction=max_correction,
            leak_rate=filter_leak_rate,
        )

    # ── 公共接口 ──────────────────────────────────────────────

    def compute(self, q, dq, pd, Rd, vd, wd, dvd, dwd, F_ext=None):
        """GAC 控制律单步计算.

        :param q:   关节位置 (nv,)
        :param dq:  关节速度 (nv,)
        :param pd:  期望位置 (3,)
        :param Rd:  期望朝向 (3,3)
        :param vd:  期望线速度 (3,)
        :param wd:  期望角速度 (3,)
        :param dvd: 期望线加速度 (3,)
        :param dwd: 期望角加速度 (3,)
        :param F_ext: 体坐标系外力/力矩 (6,), None=纯位置跟踪
        :returns: 关节力矩指令 (nv,)
        """
        # ── 1. 导纳滤波 + 轨迹修正 ───────────────────────────
        if F_ext is not None:
            # 先做 FK 获取当前位姿 R (用于体→惯性系变换)
            self.robot.update(q, dq)
            R_current = self.robot.get_pose()[1]

            # 1a. 导纳滤波
            X_corr, V_corr, dV_corr = self._filter.update(F_ext)

            # 1b. 轨迹修正 (体坐标系修正量 → 惯性系叠加)
            pd, Rd, vd, wd, dvd, dwd = self._correct_trajectory(
                pd, Rd, vd, wd, dvd, dwd,
                X_corr, V_corr, dV_corr, R_current,
            )

        # ── 2. SE(3) 位置跟踪 (内嵌, 与 GIC 同公式但独立代码) ─
        tau_cmd = self._compute_tracking(q, dq, pd, Rd, vd, wd, dvd, dwd)

        return tau_cmd

    def reset(self):
        """重置滤波器状态 (例如 F_ext 断开后调用)."""
        self._filter.reset()

    # ── 属性 ──────────────────────────────────────────────────

    @property
    def filter_state(self) -> dict:
        """当前滤波器状态快照 (用于监控/记录)."""
        return self._filter.state

    # ── 内部方法 ──────────────────────────────────────────────

    def _correct_trajectory(self, pd, Rd, vd, wd, dvd, dwd,
                             X_corr, V_corr, dV_corr, R):
        """将体坐标系修正量叠加到期望轨迹 (内部方法).

        :param pd:  期望位置 (3,)
        :param Rd:  期望朝向 (3,3)
        :param vd:  期望线速度 (3,)
        :param wd:  期望角速度 (3,)
        :param dvd: 期望线加速度 (3,)
        :param dwd: 期望角加速度 (3,)
        :param X_corr: 体坐标系位姿修正量 (6,) [Δp_body; Δφ_body]
        :param V_corr: 体坐标系速度修正量 (6,) [Δv_body; Δw_body]
        :param dV_corr: 体坐标系加速度修正量 (6,) [Δa_body; Δα_body]
        :param R: 当前末端朝向 (3,3) — 用于体→惯性系变换
        :returns: (pd', Rd', vd', wd', dvd', dwd') — 修正后的轨迹
        """
        Δp = X_corr[:3]
        Δφ = X_corr[3:]
        # 确保修正量为列向量 (3,1), 避免广播错误
        Δv = np.atleast_2d(V_corr[:3]).reshape((-1, 1))
        Δw = np.atleast_2d(V_corr[3:]).reshape((-1, 1))
        Δa = np.atleast_2d(dV_corr[:3]).reshape((-1, 1))
        Δα = np.atleast_2d(dV_corr[3:]).reshape((-1, 1))

        # 输入 vd/wd/dvd/dwd 也统一为列向量 (3,1) 再计算
        _vd = np.atleast_2d(vd).reshape((-1, 1))
        _wd = np.atleast_2d(wd).reshape((-1, 1))
        _dvd = np.atleast_2d(dvd).reshape((-1, 1))
        _dwd = np.atleast_2d(dwd).reshape((-1, 1))

        # 位置修正: pd' = pd + R @ Δp
        pd_corrected = pd + R @ Δp

        # 朝向修正
        Rd_corrected = _correct_orientation(Rd, Δφ)

        # 速度修正
        vd_corrected = _vd + R @ Δv
        wd_corrected = _wd + Δw

        # 加速度修正 (简化: 直接叠加)
        dvd_corrected = _dvd + R @ Δa
        dwd_corrected = _dwd + Δα

        return (pd_corrected, Rd_corrected,
                vd_corrected, wd_corrected,
                dvd_corrected, dwd_corrected)

    def _compute_tracking(self, q, dq, pd, Rd, vd, wd, dvd, dwd):
        """SE(3) 位置跟踪 — 与 GIC 同公式但独立代码.

        注意: 此方法内部调用 robot.update(), 即使 compute() 中已调用过。
        这是有意的设计: 保持 _compute_tracking 可独立使用, 不依赖外部状态。
        双 FK 开销 (~10μs) 远小于控制周期 (2000μs), 可忽略。

        :param q:   关节位置 (nv,)
        :param dq:  关节速度 (nv,)
        :param pd:  期望位置 (3,)
        :param Rd:  期望朝向 (3,3)
        :param vd:  期望线速度 (3,)
        :param wd:  期望角速度 (3,)
        :param dvd: 期望线加速度 (3,)
        :param dwd: 期望角加速度 (3,)
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

    def compute_state_feedback(self, q, dq, t, trajectory_funcs, F_ext=None):
        """基于轨迹函数的便捷方法: 从时间 t 自动计算期望轨迹并调用 compute.

        用法示例::

            ctrl = GACController(robot, ...)
            traj = build_trajectory('circle')

            for t in np.arange(0, 5.0, 0.001):
                tau = ctrl.compute_state_feedback(q, dq, t, traj, F_ext=F_ext)

        :param q:  关节位置 (nv,)
        :param dq: 关节速度 (nv,)
        :param t:  当前时间 (s)
        :param trajectory_funcs: TrajectoryFuncs 实例
        :param F_ext: 体坐标系外力/力矩 (6,), 可选
        :returns: 关节力矩指令 (nv,)
        """
        pd = trajectory_funcs.pd_t(t).ravel()
        Rd = trajectory_funcs.Rd_t(t).reshape(3, 3)
        vd = trajectory_funcs.dpd_t(t).ravel()
        wd = trajectory_funcs.dRd_t(t).reshape(3, 3)
        dvd = trajectory_funcs.ddpd_t(t).ravel()
        dwd = trajectory_funcs.ddRd_t(t).reshape(3, 3)
        # 将 dRd (3,3) → wd (3,) 使用 vee_map
        wd_vec = vee_map(Rd.T @ wd).ravel()
        dwd_vec = vee_map(Rd.T @ dwd).ravel()
        # (注: 简化版, 期望角速度和角加速度用符号微分结果而非 vee)
        # 更好的方式: trajectory_funcs 应直接提供 wd, dwd 向量.
        # 当前 trajectory 输出的 Rd_t 是旋转矩阵, dRd_t 是矩阵导数.
        # vee(Rdᵀ @ dRd) → 体坐标系角速度
        wd_body = vee_map(Rd.T @ wd).ravel()
        dwd_body = vee_map(Rd.T @ dwd).ravel()

        return self.compute(q, dq, pd, Rd, vd, wd_body, dvd, dwd_body, F_ext)


# ====================================================================
# 自检 (不依赖 robot_model)
# ====================================================================

if __name__ == '__main__':
    np.set_printoptions(precision=6, suppress=True)
    print("=" * 60)
    print("GAC 控制器自检")
    print("=" * 60)

    # ── 测试 GACFilter ────────────────────────────────────────
    print("\n[GACFilter]")

    # 临界阻尼设置: D = 2·sqrt(K·M)
    filt = GACFilter(
        M_d=[10.0, 10.0, 10.0, 1.0, 1.0, 1.0],
        D_d=[2*np.sqrt(500*10), 2*np.sqrt(500*10), 2*np.sqrt(500*10),
             2*np.sqrt(50*1), 2*np.sqrt(50*1), 2*np.sqrt(50*1)],
        K_d=[500.0, 500.0, 500.0, 50.0, 50.0, 50.0],
        dt=0.002,
        max_correction=0.05,
    )

    # 测试: 零力 → 状态归零
    for _ in range(100):
        X, V, dV = filt.update(np.zeros(6))
    assert np.allclose(X, 0.0, atol=1e-12), "零力: X_corr ≠ 0"
    assert np.allclose(V, 0.0, atol=1e-12), "零力: V_corr ≠ 0"
    print("  ✅ 零力 → 状态归零")

    # 测试: 恒力 → 稳态 X = K⁻¹·F
    F_test = np.array([10.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    filt.reset()
    for _ in range(5000):  # 10 秒 @ 2ms
        X, V, dV = filt.update(F_test)
    X_expected = F_test / np.array([500, 500, 500, 50, 50, 50])
    assert np.allclose(X[:3], X_expected[:3], atol=1e-4), "恒力稳态偏差"
    print(f"  ✅ 恒力 → 稳态 X ≈ K⁻¹·F: X={X[:3].ravel()} (期望 {X_expected[:3]})")

    # 测试: reset
    filt.reset()
    assert np.allclose(filt._X_corr, 0.0)
    assert np.allclose(filt._V_corr, 0.0)
    print("  ✅ reset() → 状态全零")

    # 测试: 限幅
    filt_max = GACFilter(
        M_d=[1]*6, D_d=[10]*6, K_d=[10]*6, dt=0.001,
        max_correction=0.01,
    )
    big_force = np.array([1000.0, 0, 0, 0, 0, 0])
    for _ in range(1000):
        X, V, dV = filt_max.update(big_force)
    assert np.all(np.abs(X) <= 0.011), f"限幅失效: max|X|={np.max(np.abs(X))}"
    print(f"  ✅ 限幅有效: max|X|={np.max(np.abs(X)):.4f} (限 0.01)")

    # 测试: set_parameters
    filt.set_parameters(K_d=[200.0, 200.0, 200.0, 20.0, 20.0, 20.0])
    assert np.allclose(np.diag(filt._K_d[:3]), 200.0), "set_parameters 失败"
    print("  ✅ set_parameters → 在线更新成功")

    # ── 测试 _so3_exp ──────────────────────────────────────────
    print("\n[_so3_exp]")
    phi_test = np.array([0.3, -0.2, 0.1])
    R_exp = _so3_exp(phi_test)
    assert np.allclose(np.linalg.det(R_exp), 1.0), "det(R) ≠ 1"
    assert np.allclose(R_exp.T @ R_exp, np.eye(3)), "RᵀR ≠ I"
    assert np.allclose(_so3_exp(np.zeros(3)), np.eye(3)), "零向量 ≠ I"
    print(f"  ✅ Rodrigues 正确: det(R)={np.linalg.det(R_exp):.6f}")
    print(f"  ✅ 零向量 → I")

    # ── 测试 _correct_orientation ──────────────────────────────
    print("\n[_correct_orientation]")
    Rd_test = np.eye(3)
    # 小角度修正
    Rd_small = _correct_orientation(Rd_test, np.array([0.01, 0.02, 0.03]))
    det_small = np.linalg.det(Rd_small)
    assert abs(det_small - 1.0) < 1e-10, f"小角度 det(R)={det_small}"
    print(f"  ✅ 小角度修正: det(R)={det_small:.10f}")

    # 大角度修正
    Rd_large = _correct_orientation(Rd_test, np.array([0.5, 0.3, 0.2]))
    det_large = np.linalg.det(Rd_large)
    assert abs(det_large - 1.0) < 1e-10, f"大角度 det(R)={det_large}"
    print(f"  ✅ 大角度修正 (Rodrigues): det(R)={det_large:.10f}")

    # ── 测试 _correct_trajectory ───────────────────────────────
    print("\n[_correct_trajectory]")
    ctrl_dummy = GACController.__new__(GACController)
    R_identity = np.eye(3)
    pd_test = np.array([0.5, 0.0, 0.2])
    Rd_test = np.eye(3)
    X_test = np.array([0.01, 0.0, 0.0, 0.0, 0.0, 0.0])  # 沿 x 移 1cm
    V_test = np.zeros(6)
    dV_test = np.zeros(6)

    result = ctrl_dummy._correct_trajectory(
        pd_test, Rd_test,
        np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3),
        X_test, V_test, dV_test, R_identity,
    )
    pd_c, Rd_c, vd_c, wd_c, dvd_c, dwd_c = result
    assert np.allclose(pd_c, np.array([0.51, 0.0, 0.2])), f"位置修正偏差: {pd_c}"
    print(f"  ✅ 位置修正: pd {pd_test} → {pd_c}")
    print(f"  ✅ 朝向修正: det(R')={np.linalg.det(Rd_c):.6f}")

    print("\n所有自检通过 ✅")
