# Phase 1: RobotHWInterface — Hardware Abstraction Layer

> 关联文档: [deploy_se3_to_hardware_plan.md](../../../docs/deploy_se3_to_hardware_plan.md)
> 首个部署目标: **UR12e**（使用 `ur_rtde`）

---

## 1. 设计原则

### P1. 薄层原则
硬件接口层必须极薄。每个具体实现（`UR12eHW` / `FrankaHW`）控制在 **100–200 行**以内。超过 300 行说明抽象层泄漏了机器人细节。

### P2. 机器人无关
- **禁止**任何上层代码（GIC/GUFIC 控制器、主循环）引用 `ur_rtde`、`libfranka` 等具体驱动。
- 所有机器人相关代码**只允许**出现在 `hardware/ur12e_hw.py` 和 `hardware/franka_hw.py` 中。
- 上层代码通过 `RobotHWInterface` 接口操作硬件，完全不知道底层用的是 RTDE 还是 FCI。

### P3. 生命周期安全
- 必须支持 Python 上下文管理器（`with` 语句），确保异常退出时安全释放硬件连接。
- `initialize()` / `shutdown()` 可多次调用（幂等性）。

### P4. 类型安全
- 所有方法使用 `numpy.ndarray` + `typing` 类型标注。
- 关节状态返回原生 numpy 数组（无额外封装）。

### P5. 容错与超时
- 读状态/发力矩方法可接受 `timeout` 参数。
- 通信超时时抛出特定异常（`HardwareTimeoutError`），而非静默忽略。

---

## 2. RobotHWInterface 完整 API

```python
class RobotHWInterface(ABC):
    """硬件接口抽象基类。
    
    所有真实机械臂的驱动都继承此类，上层 SE(3) 控制律通过此接口
    与控制循环交互，不感知具体机器人。
    """

    # ── 生命周期 ──────────────────────────────────────────────

    @abstractmethod
    def initialize(self) -> None:
        """初始化硬件连接。
        
        职责:
          1. 建立与机器人的通信连接（TCP/RTDE/FCI）
          2. 配置控制模式（力矩前馈/纯力矩）
          3. 读取初始关节状态
          4. 状态自检
        
        幂等: 可多次调用，第二次调用时自动先 shutdown() 再重连。
        """

    @abstractmethod
    def shutdown(self) -> None:
        """安全断开硬件连接。
        
        职责:
          1. 停止控制模式
          2. 释放关节制动
          3. 关闭通信连接
        
        幂等: 可多次调用，重复调用不报错。
        """

    # ── 状态读取 ──────────────────────────────────────────────

    @abstractmethod
    def get_joint_states(self) -> Tuple[np.ndarray, np.ndarray]:
        """获取当前关节状态。
        
        :returns:
            q:  ndarray (nv,) — 关节位置 (rad)
            dq: ndarray (nv,) — 关节速度 (rad/s)
        
        频率考虑:
          - 此方法会被控制循环以 250–1000 Hz 高频调用
          - 实现必须轻量（不分配大对象、不写日志）
          - 推荐缓存上一帧结果，通信失败时返回缓存
        
        对标 MuJoCo: `data.qpos`, `data.qvel`
        """

    @abstractmethod
    def get_ft_sensor(self) -> np.ndarray:
        """获取末端力/力矩传感器读数。
        
        :returns: ndarray (6,) — [fx, fy, fz, tx, ty, tz]
        
        注意:
          - GIC-only 模式下返回零向量（不需要传感器）
          - GUFIC 模式下需要真实传感器数据
          - Franka 可基于关节扭矩估计末端力; UR12e 需要外部 FT 传感器
        """

    # ── 执行 ──────────────────────────────────────────────────

    @abstractmethod
    def set_joint_torques(self, tau: np.ndarray) -> None:
        """下发关节力矩指令。
        
        :param tau: ndarray (nv,) — 期望关节力矩 (Nm)
        
        安全约束（实现层必须执行）:
          1. 力矩限幅: 每个关节力矩不得超过 `torque_limit`
          2. 变化率限幅: 力矩变化率不得超过 `torque_rate_limit`
          3. 急停检查: 如果 `emergency_stop()` 被触发过，禁止发力矩
        
        对标 MuJoCo: `data.ctrl`
        """

    # ── 定时 ──────────────────────────────────────────────────

    @abstractmethod
    def get_timestep(self) -> float:
        """获取标称控制周期。
        
        :returns: float — 控制周期 (秒)
        
        举例:
          UR12e (ur_rtde RTDE):  250–500 Hz → 0.002–0.004 s
          Franka (libfranka):    1000 Hz     → 0.001 s
        """

    @abstractmethod
    def wait_next_cycle(self) -> float:
        """等待下一个控制周期，返回实际经过的时间。
        
        :returns: float — 自上次调用以来实际经过的时间 (秒)
        
        设计意图:
          控制循环不假定严格固定步长。每次迭代通过此方法获取
          实际 dt，用于状态预测和积分。
          对于没有硬件定时器的模拟环境，此方法返回 `get_timestep()`。
        """

    # ── 安全 ──────────────────────────────────────────────────

    @abstractmethod
    def emergency_stop(self) -> None:
        """触发急停。
        
        行为:
          1. 立即将所有关节力矩置零
          2. 退出控制模式
          3. 设置内部急停标志位（此后 `set_joint_torques` 不生效）
          4. 记录时间戳
        
        恢复: 调用 `reset_emergency_stop()` 清除标志位。
        """

    @abstractmethod
    def reset_emergency_stop(self) -> None:
        """重置急停状态，允许继续发力矩。"""

    # ── 状态查询 ──────────────────────────────────────────────

    @abstractmethod
    def is_connected(self) -> bool:
        """检查与机器人的通信连接是否正常。"""

    @abstractmethod
    def is_enabled(self) -> bool:
        """检查控制模式是否激活（力矩模式已使能）。"""

    @abstractmethod
    def get_error_state(self) -> int:
        """获取机器人错误状态码。
        
        :returns:
            0 = 无错误
            >0 = 错误码（具体值由实现定义）
        """

    # ── 配置 ──────────────────────────────────────────────────

    @abstractmethod
    def set_torque_limits(self, limits: np.ndarray) -> None:
        """设置关节力矩限幅值（实现层硬限幅的软上限）。
        
        :param limits: ndarray (nv,) — 每个关节的最大力矩 (Nm)
        """

    @abstractmethod
    def get_joint_names(self) -> List[str]:
        """获取关节名称列表。
        
        :returns: [str, ...] — 长度为 nv 的关节名列表
        
        用途:
          用于将 URDF 中的关节顺序与硬件接口的关节顺序对齐。
        """
```

### 2.1 数据流契约

```
控制循环 (250–500 Hz)
    │
    ├─ hardware.get_joint_states()   → q, dq    (nv,)
    ├─ robot_model.update(q, dq)     → p, R, Jb  (3,)/(3,3)/(6,nv)
    ├─ controller.compute(p, R, Vb, Jb, t) → tau  (nv,)
    └─ hardware.set_joint_torques(tau)          → None
    
    时序约束:
      [get_joint_states] → [compute] → [set_joint_torques] → [wait_next_cycle]
      ├──────────── 一个控制周期 dt ────────────┤
      get_joint_states 读到的是 "当前" 状态;
      发出的力矩会在下一个周期生效（一个周期延迟）。
```

---

## 3. 首个部署目标: UR12e 实现指南

### 3.1 ur_rtde 要点

| 接口 | RTDE 方法 | 频率 | 说明 |
|---|---|---|---|
| 关节位置 | `rtde_receive.getActualQ()` | 500 Hz | 6 维向量 (rad) |
| 关节速度 | `rtde_receive.getActualQd()` | 500 Hz | 6 维向量 (rad/s) |
| 关节力矩指令 | `rtde_control.setTargetTorque(...)` | 500 Hz | **力矩前馈**模式 |
| 控制模式 | `rtde_control.modeSwitch(...)` | 100 Hz | 需先切换到力矩模式 |
| 安全状态 | `rtde_receive.getSafetyStatus()` | 10 Hz | 用于急停检测 |

### 3.2 力矩前馈 vs 纯力矩模式

| 模式 | UR 支持 | 说明 |
|---|---|---|
| **力矩前馈 (Torque Feedforward)** | ✅ 推荐 | `setTargetTorque` + `setTargetQ`。UR 内部仍有位置闭环，力矩作为前馈叠加。（**更安全**，失控时位置环兜底） |
| **纯力矩模式 (Pure Torque)** | ⚠️ 有限 | `setTargetTorque` 在非仿真模式下需要特殊配置，限制较多 |

**推荐**: 首期使用**力矩前馈模式**。UR 内部的位置环作为安全兜底，GIC 控制的力矩作为前馈叠加。

### 3.3 关节顺序对齐

UR12e 的 URDF 与 RTDE 关节顺序必须一致:

| 索引 | URDF 关节名 | RTDE 对应 |
|---|---|---|
| 0 | `shoulder_pan_joint` | `getActualQ()[0]` |
| 1 | `shoulder_lift_joint` | `getActualQ()[1]` |
| 2 | `elbow_joint` | `getActualQ()[2]` |
| 3 | `wrist_1_joint` | `getActualQ()[3]` |
| 4 | `wrist_2_joint` | `getActualQ()[4]` |
| 5 | `wrist_3_joint` | `getActualQ()[5]` |

> ⚠️ **注意**: ur_rtde 的关节顺序与 `ur_description` URDF 中的顺序一致。如果使用第三方 URDF，务必验证顺序。

### 3.4 安全软限位

UR12e 初始部署时的软限位（比硬件限位更保守）:

```python
JOINT_LIMITS_SOFT = {
    'position': (np.deg2rad([-360, -360, -360, -360, -360, -360]),  # 下界
                 np.deg2rad([ 360,  360,  360,  360,  360,  360])), # 上界
    'velocity': np.deg2rad([180, 180, 180, 180, 180, 180]),   # 关节速度限幅 (rad/s)
    'torque':   np.array([150, 150, 100, 50, 50, 50]),        # 关节力矩限幅 (Nm)
}
```

### 3.5 重力补偿策略

| 方式 | 优点 | 缺点 | 推荐 |
|---|---|---|---|
| UR 内置重力补偿 | 无需额外计算 | 补偿参数无法在线调整 | Phase 1 |
| Pinocchio 计算 + 前馈 | 与仿真一致，可使用 URDF 参数 | 增加控制延迟 | Phase 2+ |
| 混合: Pinocchio 计算，UR 内置兜底 | 两者兼顾 | 实现略复杂 | 最终 |

**Phase 1 建议**: 使用 UR 内置的重力补偿。控制初始阶段先让臂"保持不动"，确认通信/读数/发力矩链路正常后，再切换为 Pinocchio 补偿。

---

## 4. 控制循环设计

### 4.1 伪代码

```python
def run_control_loop(hardware: RobotHWInterface,
                     robot_model: RobotModel,
                     controller: GICController,
                     duration: float):
    """通用控制循环（机器人无关）。"""
    dt = hardware.get_timestep()
    t = 0.0
    
    # 读取初始关节状态
    q, dq = hardware.get_joint_states()
    robot_model.update(q, dq)
    
    # 计算初始力矩（保持不动）
    tau = np.zeros(robot_model.nv)
    hardware.set_joint_torques(tau)
    
    while t < duration and hardware.is_connected():
        cycle_start = time.perf_counter()
        
        # 1. 读状态
        q, dq = hardware.get_joint_states()
        robot_model.update(q, dq)
        
        # 2. 计算控制律
        p, R = robot_model.get_pose()
        Jb   = robot_model.get_body_jacobian()
        Vb   = robot_model.get_body_ee_velocity()
        tau  = controller.compute(p, R, Vb, Jb, t)
        
        # 3. 发力矩
        hardware.set_joint_torques(tau)
        
        # 4. 等待下一周期
        actual_dt = hardware.wait_next_cycle()
        t += actual_dt
        
        # 5. 安全检查
        if hardware.get_error_state() != 0:
            hardware.emergency_stop()
            break
```

---

## 5. 测试流程

### 5.1 无负载测试（Phase 1.1）

| 步骤 | 操作 | 预期结果 |
|---|---|---|
| 1 | `hardware.initialize()` | 连接成功，`is_connected()` → True |
| 2 | `hardware.get_joint_states()` | 返回合理的 q/dq 值 |
| 3 | `hardware.set_joint_torques(np.zeros(6))` | 无异常，臂保持原位 |
| 4 | `hardware.wait_next_cycle()` | 返回 ≈ `get_timestep()` |
| 5 | `hardware.shutdown()` | 连接断开，`is_connected()` → False |

### 5.2 重力补偿验证（Phase 1.2）

1. 运行控制循环，仅下发偏置力矩 `tau_bias`（重力补偿）
2. 臂应在重力补偿生效时保持静止（漂移 < 5 mm/min）
3. 移除重力补偿 → 臂应缓慢下落

### 5.3 Regulation 模式验证（Phase 1.3）

1. 运行简化 GIC（仅位置调节，极低增益 `Kp=50`）
2. 末端应稳定在初始位置附近（抖动 < 1 mm）
3. 逐步增加 Kp: 50 → 200 → 500 → 1000
4. 记录每个增益下的稳态误差和抖动幅度
5. 对外加微小推力的响应（阻尼比验证）

### 5.4 急停测试（Phase 1.4）

1. 运行中硬件按下 E-stop → 臂立即停止
2. 软件调用 `emergency_stop()` → 力矩置零
3. `reset_emergency_stop()` → 恢复控制

---

## 6. 文件结构（Phase 1 完成后）

```
se3_control/
├── core/                          # 待 Phase 2 实现
│   ├── __init__.py
│   ├── se3_math.py                # SE(3) 数学工具
│   ├── gic_controller.py          # GIC 控制律
│   └── trajectory.py              # 轨迹生成
├── robot_model/
│   ├── __init__.py
│   └── robot_model.py             # Pinocchio 封装（已完成 ✅）
├── hardware/                      # 本 Phase
│   ├── __init__.py
│   ├── interface.py               # RobotHWInterface 抽象基类 ← ✅
│   └── ur12e_hw.py                # UR12e 具体实现（后续实现）
├── config/
│   ├── __init__.py
│   ├── task_config.py             # 任务参数
│   └── ur12e.yaml                 # UR12e 机器人配置（后续实现）
├── scripts/
│   ├── run_se3_control.py         # 主入口（后续实现）
│   └── calibrate_gains.py         # 参数标定（后续实现）
└── docs/
    ├── deploy_se3_to_hardware_plan.md
    └── plan.md                    # ← 本文件
```

---

## 7. 实施 roadmap

| 子阶段 | 产出 | 预计工时 |
|---|---|---|
| 1.0 | `interface.py` 抽象基类完成 | 0.5 天 |
| 1.1 | `ur12e_hw.py` 基础实现（连接/读状态/发力矩） | 1 天 |
| 1.2 | 无负载测试 + 重力补偿验证 | 0.5 天 |
| 1.3 | 控制循环集成 + Regulation 测试 | 1 天 |
| 1.4 | 安全测试（急停/限幅/异常恢复） | 0.5 天 |

---

*文档创建日期: 2026-07-26*
*编写依据: [deploy_se3_to_hardware_plan.md](../../../docs/deploy_se3_to_hardware_plan.md) Phase 1*
