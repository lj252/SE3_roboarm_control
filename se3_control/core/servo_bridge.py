"""
ServoJTorqueBridge — 把 GIC 关节力矩折算成 servoJ 关节目标位
================================================================

定位
----
CB3 classic 版 UR（本仓库 UR3，固件 < 5.23）**没有直接力矩控制**：
ur_rtde 1.6.3 会对 <5.23 控制器整段删除 ``direct_torque`` 命令
（cmd 66 静默空操作），因此 ``URHW.set_joint_torques()`` 在 CB3 上
臂纹丝不动、也不报错。

本模块提供**力矩→关节目标位**的桥接：把 GICController 算出的关节
力矩，折算成 servoJ（关节位置伺服，CB3 全版本支持）可以跟踪的
关节目标位。内层 UR 伺服以高增益紧密跟踪该参考，外层的等效任务
空间闭环恰好就是 GIC 期望的二阶响应：

  τ_imp = τ − bias                          # 去掉重力/科氏（UR 伺服自带重力补偿）
  ddq   = M⁻¹·τ_imp                         # 惯量补偿 → 期望关节加速度
  dq_des += ddq·dt,  q_des += dq_des·dt     # 半隐式欧拉积分
  servoJ(q_des, ..., time=dt, gain=高)      # 紧密跟踪

推导（任务空间一致性）::

  Jb·M⁻¹·τ_imp
    = Jb·M⁻¹·Jbᵀ·(M̃·dVd* − D·ev − K·e_op)
    = M̃⁻¹·(M̃·dVd* − D·ev − K·e_op)
    = dVd* − 2ζω·ev − ω²·e_op               # 正是 GIC 目标二阶闭环

故 servoJ 紧密跟踪 q_des 时，末端行为 ≈ 直接力矩控制下的 GIC 阻抗。

级联稳定性（关键）
------------------
这是**外环阻抗参考 + 内环位置伺服**的级联::

  误差 → GIC → ddq_des ──积分──► q_des ──伺服跟踪──► 臂

若参考积分器 (带宽 ≈ ω_des) 比内层 UR 伺服 (带宽 ≈ ω_servo, 常 10–20 rad/s)
跑得还快, 参考会因臂追不上而积分漂移 → 发散。两个稳定化手段:

  1. **参考速度阻尼 ref_damp** (默认 15, **非对称**): 把实测关节速度耦合进参考,
     ``dq_des += (ddq + ref_damp·min(dq − dq_des, 0))·dt`` — 只在臂落后时减速参考
     (防积分漂移); 臂领先时**不**放大参考速度. 对称阻尼会让参考"追"一个跑得比它快的
     下坠臂 → 坠向基部 (run_05b 实测, 见 set_speed_fraction 注释).
  2. **带宽限制**: servoJ 模式下 GIC 的 ω_des 应低于内层伺服带宽
     (经验上限 ≈ 10–12 rad/s, 视伺服带宽而定). run_se3_control 的
     --servo-bandwidth-cap 会限制有效带宽并告警.

离线仿真验证: servo ω=15 时, 无阻尼仅 ω_des≤6 稳定; 加 ref_damp=15 后
ω_des≤12 稳定 (regulation 步进 3cm 收敛到 0).

用法::

  from core.gic_controller import GICController
  from core.servo_bridge import ServoJTorqueBridge

  ctrl = GICController(robot, bandwidth=20.0, damping=1.0, torque_limits=...)
  bridge = ServoJTorqueBridge(robot, ctrl, dt=0.004)
  bridge.reset(q, dq)                        # 每相位开始时调用一次
  q_servo, tau = bridge.compute(q, dq, pd, Rd, vd, wd, dvd, dwd)
  hw.set_servo_joint_positions(q_servo, gain=1000, lookahead=0.1)
"""

import numpy as np

# 速度缩放感知 (方案 A): 实机在 REDUCED/降速时 UR 伺服实际最大关节速度 = s × 额定,
# 参考积分器按满速 (dq_max=2 rad/s) 前进会跑赢被限速的臂 → 积分漂移 → 发散.
# 把参考上限乘上 s 即可在任意降速下稳定 (详见 real_vs_sim_diagnostics.md §8).
_SPEED_FRACTION_MIN = 1e-3   # 参考上限下限 (保留微小量, 防全冻结/除零)
_SPEED_RAMP_UP = 0.5         # 速度缩放恢复时参考上限的爬升速率 (1/s, 防参考突加速抖动)


class ServoJTorqueBridge:
    """力矩 → servoJ 关节目标位的桥接（有状态，跨周期保持积分器状态）。

    :param robot_model:  RobotModel 实例 (Pinocchio 封装), 提供 M(q)/bias/限位
    :param controller:   GICController 实例, compute() 返回关节力矩
    :param float dt:     控制周期 (s)
    :param float joint_margin: 关节限位安全边距 (rad), 默认 0.05
    :param np.ndarray qdd_max: 期望关节加速度限幅 (nv,), rad/s²; None=自动
    :param np.ndarray dq_max:  期望关节速度限幅 (nv,), rad/s; None=自动
    :param float speed_fraction: 实机速度缩放系数 s (0–1), 默认 1.0 = 满速.
        实机在 REDUCED/降速时 UR 伺服实际最大关节速度 = s × 额定; 参考上限
        ``dq_max/qdd_max`` 会按 s 缩放, 防参考积分跑赢被限速的臂而发散.
        运行中可用 :meth:`set_speed_fraction` 连续更新 (真机从 RTDE 读).

    .. note::
        积分器状态 (q_des, dq_des) 在每个相位开始时必须 reset() 到当前关节状态,
        否则会带着上一相位的参考继续积分.
    """

    def __init__(self, robot_model, controller, dt,
                 joint_margin: float = 0.05,
                 qdd_max: np.ndarray = None,
                 dq_max: np.ndarray = None,
                 ref_damp: float = 15.0,
                 speed_fraction: float = 1.0):
        self.robot = robot_model
        self.ctrl = controller
        self.dt = float(dt)
        nv = robot_model.nv
        # 参考速度阻尼 (1/s), 见类注释. 注意: 不随 speed_fraction 缩放!
        # run_05b 实测: 曾按 1/s 放大 (15→62.5), 臂降速下刹不住、以超参考的速度下坠时,
        # ref_damp 把参考拉向实测 dq → 参考满速往下追, GIC 纠正加速度被压过 → 坠向基部.
        # 保持基值即可: 参考上限已按 s 缩放防 windup, 带宽已按 s 缩放保级联稳定.
        self._ref_damp = float(ref_damp)
        # "伺服丢跟踪"门限 (rad): |参考−臂| 超此值时参考阻尼切到只刹不追,
        # 防参考被下坠臂拖走 (run_05b 坠向基部根因). 正常跟踪 (误差 < 门限) 时
        # 对称阻尼保收敛. 经验值: 正常伺服跟踪误差 ~10–50 mrad, 24% 降速坠落 >150 mrad.
        self._dq_track_gate = 0.15

        # 关节限位 (留边距, 防撞限位)
        lo = np.asarray(robot_model.model.lowerPositionLimit[:nv], dtype=float)
        hi = np.asarray(robot_model.model.upperPositionLimit[:nv], dtype=float)
        span = hi - lo
        margin = np.full(nv, joint_margin)
        margin = np.minimum(margin, 0.5 * span)   # 防限位带过窄时边距超范围
        self._qlo = lo + margin
        self._qhi = hi - margin

        # 期望加速度/速度限幅 (安全网) — 基值存一份, 有效值按 speed_fraction 缩放
        if qdd_max is None:
            qdd_max = np.full(nv, 20.0)
        if dq_max is None:
            dq_max = np.full(nv, 2.0)
        self._qdd_max_base = np.asarray(qdd_max, dtype=float).ravel()
        self._dq_max_base = np.asarray(dq_max, dtype=float).ravel()
        self._speed_fraction = 1.0
        self._dq_max = self._dq_max_base.copy()
        self._qdd_max = self._qdd_max_base.copy()
        self.set_speed_fraction(speed_fraction)

        # 积分器状态
        self._q_des: np.ndarray = None
        self._dq_des: np.ndarray = None
        self._ddq_last: np.ndarray = None   # 最近一次限幅后的期望加速度

    # ── 状态 ────────────────────────────────────────────────

    def reset(self, q, dq=None):
        """复位积分器到当前关节状态. 每个相位开始必须调用一次."""
        q = np.asarray(q, dtype=float).ravel()
        self._q_des = np.clip(q, self._qlo, self._qhi)
        if dq is None:
            self._dq_des = np.zeros_like(q)
        else:
            self._dq_des = np.clip(np.asarray(dq, dtype=float).ravel(),
                                   -self._dq_max, self._dq_max)

    @property
    def q_target(self) -> np.ndarray:
        """最近一次 compute() 产出的关节目标位 (nv,)."""
        return self._q_des.copy()

    @property
    def dq_target(self) -> np.ndarray:
        """最近一次 compute() 的期望关节速度 (nv,).

        供仿真内层伺服做速度前馈 (模拟 UR servoJ 内层位置伺服).
        """
        return self._dq_des.copy()

    @property
    def speed_fraction(self) -> float:
        """当前生效的速度缩放系数 (0–1). 1.0 = 满速, <1.0 = 参考上限被缩放."""
        return self._speed_fraction

    def set_speed_fraction(self, s: float) -> float:
        """按实机实际速度缩放系数缩放参考上限, 防参考积分跑赢被限速的臂.

        实机在 REDUCED/降速状态 (RTDE ``target_speed_fraction`` /
        ``speed_scaling_combined`` < 1.0) 时, UR 伺服实际关节速度能力按 s 缩小;
        而参考积分器默认按满速 (dq_max=2 rad/s) 前进 → q_servo 跑赢实际臂 →
        参考积分漂移 → 力矩饱和 → 发散. 把 ``dq_max/qdd_max`` 乘上 s, 参考速度
        上限 ≤ 实际能力, 即可在任意降速下稳定. 级联稳定性另由 run_se3_control
        的 ``resolve_main_bandwidth`` 把外环 GIC 带宽按 s 缩小保证 (详见
        real_vs_sim_diagnostics §8).

        下降立即生效 (安全优先), 上升缓慢爬升 (避免参考突然加速造成抖动).

        :param s: RTDE 速度缩放 (0–1), 如 run_04 的 0.24.
        :returns: 实际生效的系数 (可能因爬升而小于输入 s).

        .. note::
            ``ref_damp`` **不随 s 缩放**. 曾尝试按 1/s 放大 (run_05_forced/05b),
            结果: 24% 降速下臂刹不住、以超参考的速度下坠 (dq 0.65 > 上限 0.48),
            ref_damp 把参考拉向实测 dq → 参考满速往下追 → GIC 纠正被压过 → 坠向基部.
            保持基值 15: 参考不镜像臂的下坠, GIC 能把它拉回.
        """
        s = float(np.clip(s, _SPEED_FRACTION_MIN, 1.0))
        if s < self._speed_fraction:
            self._speed_fraction = s                # 安全优先: 下降立即生效
        else:
            self._speed_fraction = min(
                s, self._speed_fraction + self.dt * _SPEED_RAMP_UP)
        self._dq_max = self._dq_max_base * self._speed_fraction
        self._qdd_max = self._qdd_max_base * self._speed_fraction
        return self._speed_fraction

    @property
    def ddq_target(self) -> np.ndarray:
        """最近一次 compute() 的期望关节加速度 (nv,), 已限幅.

        供仿真内层伺服做加速度前馈; 未 compute 前返回零.
        """
        if self._ddq_last is None:
            return np.zeros(self.robot.nv)
        return self._ddq_last.copy()

    # ── 计算 ────────────────────────────────────────────────

    def compute(self, q, dq, pd, Rd, vd, wd, dvd, dwd):
        """单步: 力矩 → 关节目标位.

        :param q,dq:  当前关节位置/速度 (nv,)
        :param pd,Rd,vd,wd,dvd,dwd: GIC 期望轨迹 (同 GICController.compute)
        :returns: (q_servo, tau)
            - q_servo: ndarray (nv,) — 下发给 servoJ 的关节目标位
            - tau:     ndarray (nv,) — GIC 算出的关节力矩 (供记录/分析)
        """
        if self._q_des is None:
            self.reset(q, dq)

        # 1. GIC 力矩 (内部已 update(q,dq), 并含重力补偿 bias)
        tau = self.ctrl.compute(q, dq, pd, Rd, vd, wd, dvd, dwd)

        # 2. 去掉重力/科氏 → 纯阻抗部分 (UR 内层伺服自带重力补偿)
        bias = self.robot.get_bias_torque()
        tau_imp = tau - bias

        # 3. 惯量补偿: ddq = M⁻¹·τ_imp
        M = self.robot.get_full_inertia()
        ddq = np.linalg.solve(M, tau_imp)

        # 4. 三重限幅 + 参考速度阻尼 (级联稳定, 见类注释)
        ddq = np.clip(ddq, -self._qdd_max, self._qdd_max)
        self._ddq_last = ddq.copy()
        # 参考阻尼 (带"伺服丢跟踪"门限): 正常跟踪时对称阻尼保证收敛;
        # 伺服丢跟踪 (|参考−臂| > 门限, 如 24% 降速下臂刹不住、以超参考的速度下坠)
        # 时只"刹车"不"追臂" — 对称阻尼会把参考拉向实测 dq → 参考满速往下追 →
        # GIC 纠正加速度被压过 → 坠向基部 (run_05b 实测). 丢跟踪后不追, GIC 拉回参考.
        lag = dq - self._dq_des
        lost = np.abs(self._q_des - q) > self._dq_track_gate
        brake = self._ref_damp * np.where(lost, np.minimum(lag, 0.0), lag)
        self._dq_des = np.clip(
            self._dq_des + (ddq + brake) * self.dt,
            -self._dq_max, self._dq_max)
        self._q_des = np.clip(self._q_des + self._dq_des * self.dt,
                              self._qlo, self._qhi)

        return self._q_des.copy(), np.asarray(tau, dtype=float).ravel()
