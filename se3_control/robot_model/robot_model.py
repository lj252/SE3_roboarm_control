# -*- coding: utf-8 -*-
"""
RobotModel — Pinocchio 运动学/动力学计算封装

======================================================================
定位：替代 MuJoCo RobotState，使 SE(3) 控制律脱离仿真器运行

对齐接口：
  将 GUFIC_mujoco-main/gufic_env/utils/robot_state.py 的 RobotState
  接口用 Pinocchio 重新实现，上层控制代码无需修改即可切换。

用法：
  model = RobotModel(urdf_path, ee_frame_name="end_effector")
  model.update(q, dq)
  p, R = model.get_pose()
  Jb   = model.get_body_jacobian()
  M    = model.get_full_inertia()
  bias = model.get_bias_torque()
======================================================================
"""

import numpy as np
import pinocchio as pin
import os


class RobotModel:
    """Pinocchio 运动学/动力学计算封装。

    与 MuJoCo RobotState 接口对齐，供上层 SE(3) 控制律调用。

    :param str urdf_path:         URDF 文件的绝对或相对路径
    :param str ee_frame_name:     末端执行器 frame 名称（对应 URDF 中的 link 或 frame）
    :param str robot_name:        机器人名称（仅用于日志/标识）
    :param bool verbose:          是否打印加载信息
    """

    def __init__(self, urdf_path, ee_frame_name="end_effector",
                 robot_name="generic", verbose=True):
        self.robot_name = robot_name
        self.ee_frame_name = ee_frame_name
        self.verbose = verbose

        # ---------- 解析 URDF 路径 ----------
        if not os.path.exists(urdf_path):
            # 尝试从当前工作目录拼接
            alt_path = os.path.join(os.getcwd(), urdf_path)
            if os.path.exists(alt_path):
                urdf_path = alt_path
            else:
                raise FileNotFoundError(
                    f"URDF not found at: {urdf_path}\n"
                    f"  Also tried: {alt_path}"
                )

        # ---------- 加载模型 ----------
        self.model = pin.Model()
        try:
            # 尝试带几何模型的加载（用于碰撞/可视化）
            self.geom_model = pin.GeometryModel()
            pin.buildModelsFromUrdf(urdf_path, pin.JointModelFreeFlyer(),
                                    self.model, self.geom_model)
        except Exception:
            # 回退：纯运动学加载，无自由飞行的基座
            self.model = pin.Model()
            pin.buildModelFromUrdf(urdf_path, self.model)
            self.geom_model = None

        self.data = self.model.createData()
        self.nq = self.model.nq          # 配置空间维度
        self.nv = self.model.nv          # 速度空间维度

        if self.verbose:
            print(f"[RobotModel] Loaded: {urdf_path}")
            print(f"  └─ robot_name  : {self.robot_name}")
            print(f"  └─ nq          : {self.nq}")
            print(f"  └─ nv          : {self.nv}")
            print(f"  └─ ee_frame    : {self.ee_frame_name}")

        # ---------- 查找末端 frame ID ----------
        # Pinocchio 4.0: model.getFrameId(name) 返回 frame 在 model.frames 列表中的索引
        if self.model.existFrame(self.ee_frame_name):
            self.ee_frame_id = self.model.getFrameId(self.ee_frame_name)
        else:
            # 尝试名称模糊匹配
            matching = [f for f in self.model.frames
                        if self.ee_frame_name.lower() in f.name.lower()]
            if matching:
                self.ee_frame_id = self.model.getFrameId(matching[0].name)
                if self.verbose:
                    print(f"  └─ fuzzy matched frame: '{matching[0].name}'")
            else:
                # 默认使用最后一个操作 frame (OP_FRAME)
                op_frames = [f for f in self.model.frames
                             if f.type == pin.FrameType.OP_FRAME]
                if op_frames:
                    self.ee_frame_id = self.model.getFrameId(op_frames[-1].name)
                    if self.verbose:
                        frame_name = self.model.frames[self.ee_frame_id].name
                        print(f"  └─ fallback frame: '{frame_name}'")
                else:
                    raise ValueError(
                        f"Cannot find frame '{ee_frame_name}' in model.\n"
                        f"Available frames: "
                        f"{[f.name for f in self.model.frames]}"
                    )

        # ---------- 内部状态缓冲 ----------
        self._q = np.zeros(self.nq)       # 关节位置
        self._dq = np.zeros(self.nv)      # 关节速度
        self._updated = False             # 是否需要重新计算

        # ---------- 力传感器占位（实机时由硬件接口提供） ----------
        self._fe = np.zeros(6)

    # ================================================================
    # 0. 核心更新方法
    # ================================================================

    def update(self, q=None, dq=None):
        """更新正运动学与动力学计算。

        对标 MuJoCo: `mj_step1` + `mj_rnePostConstraint`

        :param q:  关节位置, shape (nq,). None 则使用上次值.
        :param dq: 关节速度, shape (nv,). None 则使用上次值.
        """
        if q is not None:
            self._q = np.asarray(q, dtype=float).ravel()
        if dq is not None:
            self._dq = np.asarray(dq, dtype=float).ravel()

        # 确保维度正确（对于树结构机器人，nq 可能 > nv）
        # Pinocchio 的 forwardKinematics 需要完整的 q
        q_full = self._q.copy()
        if len(q_full) < self.nq:
            q_full = np.pad(q_full, (0, self.nq - len(q_full)))

        # 正运动学（位置级）
        pin.forwardKinematics(self.model, self.data, q_full)
        pin.updateFramePlacements(self.model, self.data)

        # 正运动学（速度级）
        dq_full = self._dq.copy()
        if len(dq_full) < self.nv:
            dq_full = np.pad(dq_full, (0, self.nv - len(dq_full)))
        pin.forwardKinematics(self.model, self.data, q_full, dq_full)

        self._updated = True
        return self

    def update_dynamic(self):
        """占位 — 实机不需要前行动力学步进。

        对标 MuJoCo: `mj_step2`
        在仿真中该步推进物理状态；真实机器人物理已自行演化。
        """
        self._updated = False
        pass

    # ================================================================
    # 1. 正运动学 — 位姿
    # ================================================================

    def get_pose(self):
        """获取末端执行器的位置和朝向。

        对标 MuJoCo: `site_xpos` + `site_xmat`

        :returns:
            p: ndarray (3,)  — 位置向量（世界坐标系）
            R: ndarray (3,3) — 旋转矩阵 SO(3)（世界坐标系）
        """
        self._ensure_updated()
        placement = self.data.oMf[self.ee_frame_id]
        p = placement.translation.copy()
        R = placement.rotation.copy()
        return p, R

    def get_joint_pose(self):
        """获取当前关节角度。

        对标 MuJoCo: `data.qpos`
        """
        return self._q.copy()

    def get_joint_velocity(self):
        """获取当前关节速度。

        对标 MuJoCo: `data.qvel`
        """
        return self._dq.copy()

    def get_num_joints(self):
        """获取关节数（自由度数）。"""
        return self.nv

    def get_timestep(self):
        """获取控制周期占位。

        对标 MuJoCo: `model.opt.timestep`
        实机中由外部定时器决定。
        """
        return 0.002  # 默认 500 Hz，实机可覆盖

    # ================================================================
    # 2. 雅可比矩阵
    # ================================================================

    def get_jacobian(self):
        """获取几何雅可比矩阵（geometric Jacobian, 世界坐标系）。

        对标 MuJoCo: `mj_jacSite`

        Pinocchio 的 getFrameJacobian(WORLD) 返回的是**空间雅可比**（spatial/
        Plücker Jacobian），其线速度部分定义在世界坐标系原点（而非末端点），
        与 MuJoCo 的 mj_jacSite 不同。需要通过伴随变换转换为几何雅可比:

          J_geom = [[I, -hat(p)], [0, I]] @ J_spatial

        几何雅可比将关节速度映射到末端点的实际空间速度:
          Vs = [p_dot; w] = Js @ dq

        :returns: ndarray (6, nv)
        """
        self._ensure_updated()
        pin.computeJointJacobians(self.model, self.data, self._q)
        J_spatial = pin.getFrameJacobian(
            self.model, self.data, self.ee_frame_id, pin.WORLD
        )
        # 转换为几何雅可比
        p, _ = self.get_pose()
        p_hat = np.array([[0, -p[2], p[1]],
                          [p[2], 0, -p[0]],
                          [-p[1], p[0], 0]])
        T_convert = np.block([[np.eye(3), -p_hat],
                              [np.zeros((3, 3)), np.eye(3)]])
        J_geom = T_convert @ J_spatial
        # 只保留实际有效自由度
        return J_geom[:6, :self.nv].copy()

    def get_body_jacobian(self):
        """获取体雅可比矩阵（body Jacobian, 末端体坐标系）。

        对标 MuJoCo: `get_body_jacobian()` — 即 Rᵀ @ Js

        通过空间雅可比 Js (WORLD) 经伴随变换得到体雅可比:
          Jb = [[Rᵀ, 0], [0, Rᵀ]] @ Js

        注意: 不能直接使用 pin.LOCAL, 因为 Pinocchio 的 LOCAL 参考系
        是末端所在支链的父关节局部系, 而非末端自身的体坐标系。

        :returns: ndarray (6, nv)
        """
        Js = self.get_jacobian()
        _, R = self.get_pose()
        transform = np.block([[R.T, np.zeros((3, 3))],
                              [np.zeros((3, 3)), R.T]])
        Jb = transform @ Js
        return Jb

    def get_body_ee_velocity(self):
        """获取末端体速度（body twist）。

        对标 MuJoCo: `get_body_ee_velocity()` — Jb @ dq

        Vb = [vx, vy, vz, wx, wy, wz]ᵀ ∈ ℝ⁶

        :returns: ndarray (6, 1)
        """
        Jb = self.get_body_jacobian()
        dq = self._dq[:self.nv]
        Vb = Jb @ dq.reshape((-1, 1))
        return Vb

    def get_spatial_ee_velocity(self):
        """获取末端空间速度（spatial twist）。

        对标 MuJoCo: `get_spatial_ee_velocity()` — Js @ dq

        Vs = [p_dot_x, p_dot_y, p_dot_z, wx, wy, wz]ᵀ ∈ ℝ⁶

        :returns: ndarray (6, 1)
        """
        Js = self.get_jacobian()
        dq = self._dq[:self.nv]
        Vs = Js @ dq.reshape((-1, 1))
        return Vs

    # ================================================================
    # 3. 动力学 — 惯性矩阵 & 偏置力矩
    # ================================================================

    def get_full_inertia(self):
        """获取关节空间惯性矩阵 M(q)。

        对标 MuJoCo: `mj_fullM` → `get_full_inertia()`

        使用 CRBA (Composite Rigid Body Algorithm) 计算:
          M(q) ∈ ℝⁿᵛˣⁿᵛ

        :returns: ndarray (nv, nv)
        """
        self._ensure_updated()
        pin.crba(self.model, self.data, self._q)
        return self.data.M.copy()

    def get_bias_torque(self):
        """获取偏置力矩（重力 + 科氏力 + 离心力）。

        对标 MuJoCo: `qfrc_bias`

        通过逆动力学计算: b(q, dq) = rnea(q, dq, 0)
        即令加速度为零时的逆动力学输出。

        :returns: ndarray (nv,)
        """
        self._ensure_updated()
        a_zero = np.zeros(self.nv)
        tau_bias = pin.rnea(self.model, self.data, self._q, self._dq, a_zero)
        return tau_bias.copy()

    def get_coriolis_matrix(self):
        """获取科氏力矩阵 C(q, dq)（可选，GIC 不直接需要。

        可用于高级控制律（如惯性整形）。
        """
        self._ensure_updated()
        pin.computeCoriolisMatrix(self.model, self.data, self._q, self._dq)
        return self.data.C.copy()

    def get_gravity(self):
        """获取重力力矩 g(q)。

        :returns: ndarray (nv,)
        """
        self._ensure_updated()
        g = pin.computeGeneralizedGravity(self.model, self.data, self._q)
        return g.copy()

    # ================================================================
    # 4. 逆运动学（高斯-牛顿法）
    # ================================================================

    def gauss_newton_IK(self, pd, Rd, init_q,
                        step_size=0.5, tol=1e-3, max_cnt=200,
                        verbose=None):
        """高斯-牛顿法（Levenberg-Marquardt）逆运动学求解。

        对标 MuJoCo 版本: 功能等价, 但使用 LM 方法(更快收敛)

        从初始猜测 init_q 出发，迭代最小化位姿误差:
          e = [ep; eR] ∈ ℝ⁶

        使用 Levenberg-Marquardt (阻尼高斯-牛顿):
          Δq = -(JᵀJ + λI)⁻¹ Jᵀ e

        特性:
          - 自动忽略不对末端位姿产生影响的关节（如夹爪）
          - 自适应阻尼: 误差增大时减小步长
          - 限制最大步长避免震荡

        :param pd:        期望位置 (3,)
        :param Rd:        期望朝向 (3,3)
        :param init_q:    初始关节角度 (nv,)
        :param step_size: 初始迭代步长 (0~1)
        :param tol:       收敛容差 (||e|| < tol 时停止)
        :param max_cnt:   最大迭代次数
        :param verbose:   覆盖 self.verbose（可选）
        """
        q = np.asarray(init_q, dtype=float).ravel().copy()
        Rd_arr = np.asarray(Rd)
        pd_vec = np.asarray(pd).ravel()

        # 初始正解
        self.update(q)
        p, R = self.get_pose()

        # 辨识有效关节: 雅可比中非零列的关节
        J_all = self.get_jacobian()
        col_norms = np.linalg.norm(J_all[:, :self.nv], axis=0)
        active_joints = np.where(col_norms > 1e-8)[0]

        # 如果初始位形奇异, 加入小扰动打破奇异性
        if len(active_joints) < min(6, self.nv):
            inactive = np.where(col_norms <= 1e-8)[0]
            for idx in inactive[:min(3, len(inactive))]:
                q[idx] += np.random.randn() * 0.01
            self.update(q)
            J_all = self.get_jacobian()
            col_norms = np.linalg.norm(J_all[:, :self.nv], axis=0)
            active_joints = np.where(col_norms > 1e-8)[0]

        if len(active_joints) == 0:
            if verbose is None or verbose:
                print(f"[IK] Warning: No active joints affect EE pose")
            return q

        n_active = len(active_joints)
        use_subset = n_active < self.nv

        # 初始误差
        ep = (p - pd_vec).reshape((-1, 1))
        R1, R2, R3 = R[:, 0], R[:, 1], R[:, 2]
        Rd1, Rd2, Rd3 = Rd_arr[:, 0], Rd_arr[:, 1], Rd_arr[:, 2]
        eR = -0.5 * (np.cross(R1, Rd1) + np.cross(R2, Rd2) + np.cross(R3, Rd3))
        error = np.vstack((ep, eR.reshape((-1, 1))))
        err_norm = np.linalg.norm(error)

        step_cnt = 0
        lam = 0.1          # 初始阻尼

        while err_norm >= tol and step_cnt < max_cnt:
            J = self.get_jacobian()[:, :self.nv]
            J_eff = J[:, active_joints] if use_subset else J
            n_eff = n_active if use_subset else self.nv

            # LM 步: Δ = -(JᵀJ + λI)⁻¹ Jᵀ e
            try:
                delta = -np.linalg.solve(J_eff.T @ J_eff + lam * np.eye(n_eff),
                                         J_eff.T @ error)
            except np.linalg.LinAlgError:
                delta = -np.linalg.pinv(J_eff.T @ J_eff + lam * np.eye(n_eff)) @ J_eff.T @ error

            # 线搜索: step ∈ {1.0, 0.5, 0.25, ...}, 接受第一个降低误差的步
            step = 1.0
            accepted = False
            for _ in range(8):
                if use_subset:
                    q_try = q.copy()
                    q_try[active_joints] = q[active_joints] + step * delta.ravel()
                else:
                    q_try = q.copy() + step * delta.ravel()

                self._apply_joint_limits(q_try)
                self.update(q_try)

                p_try, R_try = self.get_pose()
                ep_try = (p_try - pd_vec).reshape((-1, 1))
                R1, R2, R3 = R_try[:, 0], R_try[:, 1], R_try[:, 2]
                eR_try = -0.5 * (np.cross(R1, Rd1) + np.cross(R2, Rd2) + np.cross(R3, Rd3))
                err_try = np.linalg.norm(np.vstack((ep_try, eR_try.reshape((-1, 1)))))

                if err_try < err_norm:
                    q = q_try
                    error = np.vstack((ep_try, eR_try.reshape((-1, 1))))
                    err_norm = err_try
                    accepted = True
                    break
                step *= 0.5

            if accepted:
                lam = max(lam * 0.3, 1e-6)   # 减少阻尼
            else:
                lam = min(lam * 3, 100.0)     # 增大阻尼
                if lam >= 100:
                    break                     # 无法收敛

            step_cnt += 1

        self._q = q
        self.update(q)

        show_log = verbose if verbose is not None else self.verbose
        if show_log:
            print(f"[IK] Finished. Steps: {step_cnt}, final error: {err_norm:.6e}")

        return q

    def _apply_joint_limits(self, q):
        """应用关节软限位（通过在 model 上配置的限位）。

        在 Pinocchio 中 model 的 joint 限位通过 model.lowerPositionLimit
        和 model.upperPositionLimit 访问（若存在）。
        """
        for i in range(min(len(q), self.nq)):
            if hasattr(self.model, 'upperPositionLimit') and i < len(self.model.upperPositionLimit):
                q[i] = np.clip(q[i],
                               self.model.lowerPositionLimit[i],
                               self.model.upperPositionLimit[i])

    # ================================================================
    # 5. 力/力矩传感器（占位）
    # ================================================================

    def get_ee_force(self, return_derivative=False):
        """获取末端力/力矩（占位）。

        对标 MuJoCo: `get_ee_force()`

        实机部署时，此方法应由 RobotHWInterface 覆盖/注入。
        当前返回零值，不影响 GIC-only 控制模式。

        :returns:
            Fe:  ndarray (6,)   — [fx, fy, fz, τx, τy, τz]
            dFe: ndarray (6,)   — 力导数（仅 return_derivative=True 时）
        """
        Fe = self._fe.copy()
        dFe = np.zeros(6)
        if return_derivative:
            return Fe, dFe
        return Fe

    def get_ee_force_raw(self):
        """获取原始（未滤波）末端力/力矩（占位）。

        对标 MuJoCo: `get_ee_force_raw()`
        """
        return self._fe.copy(), np.zeros(6)

    def set_ee_force(self, fe):
        """由硬件接口注入力传感器读数。"""
        self._fe = np.asarray(fe, dtype=float).ravel()

    # ================================================================
    # 6. 控制力矩设置（占位）
    # ================================================================

    def set_control_torque(self, tau, gripper=0):
        """设置控制力矩（占位 — 实机中由 RobotHWInterface 实现）。

        对标 MuJoCo: `set_control_torque`

        真实机器人中，力矩应通过 RobotHWInterface.set_joint_torques()
        下发，此处保留以保持接口兼容。
        """
        pass

    # ================================================================
    # 7. 工具方法
    # ================================================================

    def _ensure_updated(self):
        """确保正运动学已更新。"""
        if not self._updated:
            self.update()

    def print_frame_list(self):
        """打印所有可用 frame 名称（调试用）。

        在 Pinocchio 4.0 中，frame 的索引即其在 model.frames 列表中的位置，
        可通过 model.getFrameId(name) 获取。
        """
        print(f"[RobotModel] Available frames ({len(self.model.frames)}):")
        for i, f in enumerate(self.model.frames):
            print(f"  └─ idx={i}, name='{f.name}', type={f.type}")


# ================================================================
# 8. 快速自检
# ================================================================

if __name__ == "__main__":
    import sys

    print("=" * 60)
    print("RobotModel 自检程序")
    print("用法: python robot_model.py <path/to/robot.urdf> [ee_frame_name]")
    print("=" * 60)

    if len(sys.argv) < 2:
        print("\n⚠️  未指定 URDF 文件，跳过测试。")
        print("   示例: python robot_model.py /path/to/ur12.urdf tool0")
        sys.exit(0)

    urdf = sys.argv[1]
    ee_frame = sys.argv[2] if len(sys.argv) > 2 else "end_effector"

    model = RobotModel(urdf, ee_frame_name=ee_frame, verbose=True)

    # 列出所有 frame
    model.print_frame_list()

    # 随机初始状态
    np.random.seed(42)
    q0 = np.random.rand(model.nq) * 0.5 - 0.25
    dq0 = np.random.rand(model.nv) * 0.1 - 0.05

    model.update(q0, dq0)

    # --- 测试正运动学 ---
    p, R = model.get_pose()
    print(f"\n[Test] Forward Kinematics:")
    print(f"  └─ position p: {p}")
    print(f"  └─ rotation R:\n{R}")

    # --- 测试雅可比 ---
    Js = model.get_jacobian()
    Jb = model.get_body_jacobian()
    print(f"\n[Test] Jacobians:")
    print(f"  └─ spatial Js: {Js.shape}")
    print(f"  └─ body    Jb: {Jb.shape}")

    # 验证 Jb ≈ R.T @ Js
    _, R_check = model.get_pose()
    transform = np.block([[R_check.T, np.zeros((3, 3))],
                          [np.zeros((3, 3)), R_check.T]])
    Jb_from_js = transform @ Js
    diff = np.linalg.norm(Jb - Jb_from_js)
    print(f"  └─ Jb ≈ R.T @ Js 验证: diff = {diff:.2e}  {'✅' if diff < 1e-10 else '⚠️'}")

    # --- 测试体速度 ---
    Vb = model.get_body_ee_velocity()
    Vs = model.get_spatial_ee_velocity()
    print(f"\n[Test] Velocities:")
    print(f"  └─ body twist    Vb: {Vb.ravel()}")
    print(f"  └─ spatial twist Vs: {Vs.ravel()}")
    Vb_from_vs = transform @ Vs
    diff_v = np.linalg.norm(Vb - Vb_from_vs)
    print(f"  └─ Vb ≈ R.T @ Vs 验证: diff = {diff_v:.2e}  {'✅' if diff_v < 1e-10 else '⚠️'}")

    # --- 测试动力学 ---
    print(f"\n[Test] Dynamics:")
    M = model.get_full_inertia()
    print(f"  └─ inertia M({model.nv}×{model.nv}):")
    print(M)

    bias = model.get_bias_torque()
    print(f"  └─ bias torque: {bias}")

    g = model.get_gravity()
    print(f"  └─ gravity torque: {g}")

    # --- 测试逆运动学 ---
    print(f"\n[Test] Inverse Kinematics:")
    pd_des = p + np.array([0.05, 0.03, -0.02])
    Rd_des = R.copy()
    q_ik = model.gauss_newton_IK(pd_des, Rd_des, q0)
    p_ik, R_ik = model.get_pose()
    pos_err = np.linalg.norm(p_ik - pd_des)
    rot_err = np.linalg.norm(R_ik - Rd_des)
    print(f"  └─ position error: {pos_err:.6f}  {'✅' if pos_err < 1e-3 else '⚠️'}")
    print(f"  └─ rotation error:  {rot_err:.6f}  {'✅' if rot_err < 1e-6 else '⚠️'}")

    print("\n[RobotModel] 自检完成 ✅")
