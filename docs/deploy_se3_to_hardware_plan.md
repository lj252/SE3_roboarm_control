# SE(3) 几何控制在真实机械臂上的部署计划

> 目标：将 [GUFIC_mujoco-main](..) 的 GIC/GUFIC 控制算法部署到**多种真实机械臂**（UR12, Franka Emika Panda），保持控制核心代码完全与机器人无关

---

## 0. 设计哲学：Write Once, Run on Any Arm

```
┌─────────────────────────────────────────┐
│           应用层 (任务/轨迹)              │  ← 机器人无关
├─────────────────────────────────────────┤
│         SE(3) 控制核心 (GIC/GUFIC)       │  ← 机器人无关 ← 这是你的核心资产
├─────────────────────────────────────────┤
│        运动学/动力学抽象层 (Pinocchio)    │  ← 机器人无关（加载不同URDF即可）
├─────────────────────────────────────────┤
│          硬件接口抽象层 (RobotHW)         │  ← 机器人相关 ← 唯一需要换的部分
├──────────────┬──────────────────────────┤
│   UR12 驱动  │   Franka 驱动             │  ← 具体实现
└──────────────┴──────────────────────────┘
```

**核心原则**：SE(3) 控制律代码**零修改**切换机械臂。唯一需要替换的是最底层的硬件接口。

---

## 1. 仿真代码 vs 实机部署 — 逐层映射

### 仿真中的依赖 & 实机的替代方案

| 层 | 仿真 (MuJoCo) | 实机方案 | 机器人相关？ |
|---|---|---|---|
| 正运动学 `p,R = f(q)` | `mj_step1` → `site_xpos/xmat` | **Pinocchio `forwardKinematics()`** + `frames()` | ❌ 无关（URDF 决定） |
| 空间雅可比 `Js(q)` | `mj_jacSite` | **Pinocchio `computeJointJacobians()`** + `getFrameJacobian()` | ❌ 无关 |
| 体雅可比 `Jb(q)` | `Js` → `Rᵀ` 变换 | 同一公式，用 Pinocchio 位姿计算 | ❌ 无关 |
| 体速度 `Vb` | `Jb @ dq` | `Jb @ dq`（从编码器读dq） | ❌ 无关 |
| 惯性矩阵 `M(q)` | `mj_fullM` | **Pinocchio `crba()`** | ❌ 无关 |
| 偏置力矩 `b(q, dq)` | `qfrc_bias` | **Pinocchio `rnea()`** 或 `computeCoriolisMatrix()` + `computeGeneralizedGravity()` | ❌ 无关 |
| 力矩执行 | `ctrl` → `mj_step2` | **RobotHW 接口** → 具体机器人驱动 | ✅ **相关** |
| 关节状态读取 | `data.qpos`, `data.qvel` | **RobotHW 接口** → 编码器 | ✅ **相关** |
| 力/力矩传感 | MuJoCo 传感器 | 独立 FT 传感器驱动 | ✅ **相关**（传感器品牌相关） |
| 控制循环定时 | MuJoCo 步进回调 | **实时定时器** / RT 线程 | ❌ 无关（OS 相关） |

**结论**：除了最底层的"读关节/发力矩"和 FT 传感器驱动，其余全部可以用 **Pinocchio**（一个开源库）统一完成。

---

## 2. 推荐软件架构

### 2.1 核心依赖（机器人无关）

```
Python:
  - pinocchio         # 运动学、动力学、雅可比（支持任何URDF）
  - numpy, scipy      # 数值计算
  - control           # 滤波器设计（可选）

C++（生产环境）:
  - pinocchio         # 同上
  - Eigen             # 矩阵运算
```

### 2.2 硬件接口抽象层设计

```python
# robot_hw_interface.py  — 这是唯一需要为不同机械臂实现的接口
class RobotHWInterface(ABC):
    @abstractmethod
    def get_joint_states(self) -> Tuple[np.ndarray, np.ndarray]:
        """返回: (q: np.ndarray[7], dq: np.ndarray[7]) — 关节位置和速度"""
        pass

    @abstractmethod
    def set_joint_torques(self, tau: np.ndarray):
        """下发关节力矩指令"""
        pass

    @abstractmethod
    def get_ft_sensor(self) -> np.ndarray:
        """返回: (fx, fy, fz, tx, ty, tz) 末端力/力矩（可选）"""
        pass

    @abstractmethod
    def get_timestep(self) -> float:
        """返回控制周期（秒）"""
        pass

    @abstractmethod
    def emergency_stop(self):
        """急停"""
        pass
```

**UR12 实现**：底层可用 `ur_rtde`（Python 绑定），但**仅限这一层调用**。上层 SE(3) 控制核心完全不知 `ur_rtde` 的存在。

**Franka 实现**：底层用 `libfranka`（Python 封装 `franka_ros2` 或 `panda-py`），同样，上层控制代码不感知 Franka 的存在。

### 2.3 控制核心代码结构

```
se3_control/
├── core/
│   ├── se3_math.py          # SE(3) 数学: hat_map, vee_map, adjoint, expm  ← 0依赖
│   ├── gic_controller.py     # GIC 控制律                              ← 只依赖 se3_math
│   ├── gufic_controller.py   # GUFIC 控制律                            ← 只依赖 se3_math
│   └── trajectory.py         # 轨迹生成（与仿真中 initialize_trajectory 同思路）
├── robot_model/
│   └── robot_model.py        # 封装 Pinocchio 正解/雅可比/惯性/偏置   ← 只依赖 URDF
├── hardware/
│   ├── interface.py          # RobotHWInterface 抽象基类
│   ├── ur12_hw.py            # UR12 具体实现（使用 ur_rtde）
│   └── franka_hw.py          # Franka 具体实现（使用 libfranka）
├── config/
│   ├── ur12_panda.yaml       # UR12 URDF 路径 + 增益参数（注意：这是Panda的配置）
│   └── franka.yaml           # Franka URDF 路径 + 增益参数
├── scripts/
│   ├── run_se3_control.py     # 主入口
│   └── calibrate_gains.py    # 参数标定辅助脚本
└── docs/
    └── deploy_se3_to_hardware_plan.md  # 本文件
```

---

## 3. 实施困难分析

> 困难等级：🔴 H (High) / 🟡 M (Medium) / 🟢 L (Low)

---

### 🔴 H1. 控制循环的重构 — 从"回调步进"到"读写异步"

**核心矛盾**：仿真中 `mj_step1` → 控制 → `mj_step2` 是同步串行的。实机中"读状态-算控制-发力矩"是**异步循环**，状态和指令之间有**一个周期的延迟**。

**影响**：
- SE(3) 控制律的 `ev = Vb - Vd_star` 中的 `Vb` 永远是"上一帧"的速度
- 阻尼项 `Kd @ ev` 可能在低控制频率（500 Hz）下引入不稳定
- **解决方法**：在控制循环中引入状态预测（简单的欧拉外推）或降低对阻尼精度的依赖

**改造程度**：中。控制律数学公式不变，但控制循环结构需要重写。

---

### 🔴 H2. 动力学补偿的准确性

**困难**：GIC 控制律中 `tau_cmd = Jbᵀ @ tau_tilde + qfrc_bias` 的 `qfrc_bias` 补偿。

- 仿真中：MuJoCo 计算的是**完美模型**的重力 + 科氏力
- 实机中：Pinocchio 计算的是**URDF 标称参数**的补偿

**问题**：
- UR12 和 Franka 的标称惯性参数 ≠ 真实值（尤其是摩擦、负载变化）
- Franka 有**关节扭矩传感器**，可以测量外力和摩擦，但 UR12 没有
- 不准确的补偿 → 稳态误差 / 跟踪抖动

**解决方案**：
- 在 GIC 力矩输出上加入**摩擦力补偿**（库仑 + 粘滞模型）
- 或 Franka 利用关节扭矩传感器的读数替代模型补偿的 `qfrc_bias`

---

### 🔴 H3. 安全 — 从仿真到实物的跨越

**比上一版文档更具体地列出每种机械臂的安全差异**：

| 安全机制 | UR12 | Franka Panda |
|---|---|---|
| 关节力矩限幅 | UR 安全配置（默认较高） | Franka 有**碰撞检测**自动停止 |
| 软限位 | 需自己实现 | Franka 有 `joint_limits` 保护 |
| 奇异点保护 | 需自己实现 | 需自己实现 |
| 急停 | 硬件 E-stop 接口 | 硬件 E-stop 接口 |
| 力矩模式限速 | UR 安全配置可设 | Franka `move_j` 有速度限制 |

**Franka 对算法开发者更友好**：6 轴全都有**高精度关节扭矩传感器** + **libfranka** 原生支持力矩模式。

---

### 🔴 H4. 多机械臂的增益普适性问题

SE(3) 控制律本身是机器人无关的，但**阻抗增益参数**与机械臂物理特性强相关：

```python
Kp = diag(2500, 2500, 1500)  # 对 UR12 可能太大，对 Franka 可能太小
```

| 机械臂 | 重量 | 负载 | 惯量特点 | 预期增益范围 |
|---|---|---|---|---|
| UR12 | ~33 kg | 12 kg | 大臂、高惯量 | Kp 可较高（刚性结构） |
| Franka Panda | ~18 kg | 3 kg | 轻量、柔性关节 | Kp 需较低（避免震荡） |

**需要为每种机械臂独立调参**。控制律数学相同，但增益参数需要在 `config/*.yaml` 中分别配置。

---

### 🟡 M1. 控制频率上限差异

| 机械臂 | 控制模式 | 可达频率 | 接口库 |
|---|---|---|---|
| UR12 | 力矩前馈 | **500 Hz** (ur_rtde) | C++ 稳定，Python 不稳定 |
| Franka | 力矩控制 | **1000 Hz** (libfranka) | C++ 原生，Python 封装约 200-500 Hz |

**影响**：
- Franka 的控制频率更高，对控制律的稳定更有利
- UR12 需要更保守的增益（因为控制频率低，延迟影响更大）
- 两种机器人用同一套控制代码时，控制循环的**定时机制**需要可配置

---

### 🟡 M2. 通信延迟与抖动（Jitter）

- 仿真：零延迟
- UR12 (RTDE over Ethernet): ~1-2 ms 通信延迟，抖动 ±0.5 ms
- Franka (libfranka, FCI over Ethernet): ~1 ms 通信延迟，抖动 ±0.2 ms

**建议**：在控制循环中始终记录 `timestamp`，分析延迟抖动，必要时引入**固定步长**控制（不管实际帧何时到达，控制律按固定 dt 计算）。

---

### 🟢 M3. FT 传感器集成

**如果你只做 GIC（无力控）**：FT 传感器不是必须的。GIC 控制律不需要外力反馈。

**如果你后续要做 GUFIC**：
- Franka：**自带关节扭矩传感器**，可以计算末端力（不需要外加 FT 传感器）
- UR12：**没有关节扭矩传感器**，需要外加 FT 传感器（Onrobot / Robotiq）

**这对架构设计的影响**：`get_ft_sensor()` 对 Franka 可以直接用 libfranka 的关节扭矩 → 末端力映射；对 UR12 需要额外传感器驱动。接口抽象层天然解耦。

---

## 4. 推荐实施路径

### Phase 0：Pinocchio 验证（~3 天，纯仿真）

```
目标：确认 Pinocchio 能完整替代 MuJoCo 的运动学/动力学功能
步骤：
  1. 安装 Pinocchio + hpp-fcl
  2. 下载 UR12 和 Franka 的 URDF（ros-industrial / franka_ros）
  3. 编写 robot_model.py：
     - forward_kinematics(q) → p, R
     - body_jacobian(q)     → Jb
     - inertia_matrix(q)    → M
     - bias_torque(q, dq)   → qfrc_bias
  4. 在仿真代码中用 Pinocchio 替换 MuJoCo 调用的部分，对比输出的一致性
```

### Phase 1：硬件接口层实现（~1 周，选一种臂先做）

```
目标：跑通"读-算-发"闭环，末端保持位置不动
步骤：
  1. 实现 RobotHWInterface 的具体类（UR12 或 Franka）
  2. 关节状态读取验证（q, dq）
  3. 力矩下发验证（先发零力矩 → 确认重力补偿正确 → 臂保持静止）
  4. 运行简化 GIC：只有位置保持（regulation 模式），极低增益 Kp=50
```

### Phase 2：SE(3) 控制核心适配（~1 周）

```
目标：将仿真中的 GIC 控制律移植到实机
步骤：
  1. 将 gic_controller.py 代码从仿真工程中抽离，只依赖 se3_math + robot_model
  2. 在仿真环境中用"纯 Pinocchio"版本跑通验证
  3. 移植到实机，从 regulation 任务开始
  4. 逐步调参：位置阶跃响应 → 优化阻尼比
```

### Phase 3：轨迹跟踪（~1 周）

```
目标：末端跟踪简单轨迹
步骤：
  1. 简单轨迹：直线（line）→ 圆（circle）
  2. 实机验证轨迹跟踪精度
  3. 记录跟踪误差、力矩输出，与仿真对比
  4. 排查"仿真→实机"的差异源（摩擦、延迟、参数误差）
```

### Phase 4：第二机械臂移植（~1 周）

```
目标：同一套代码在 Franka 上运行
步骤：
  1. 实现 FrankaHW（共一个文件 ~100 行）
  2. 加载 Franka 的 URDF 配置
  3. 调参（Franka 的初始增益应比 UR12 低）
  4. 验证 regulation + 轨迹跟踪
```

### Phase 5（可选）：GUFIC 力-阻抗控制

```
FT 传感器要求：
  - Franka：自带关节扭矩传感器 → 不需要额外硬件
  - UR12：需要外加 FT 传感器
步骤：
  1. 实现 force_observer（Franka 用 libfranka 关节扭矩；UR12 用 FT 传感器数据）
  2. GUFIC 控制律中的力项（力跟踪 + 能量油箱）
  3. 接触任务验证（恒力跟踪、曲面跟随）
```

---

## 5. 关键建议汇总

| # | 建议 |
|---|---|
| 1 | **先做 Pinocchio 验证（Phase 0）** — 这是整个架构的基石。确认 Pinocchio 输出与 MuJoCo 一致后再部署到实机 |
| 2 | **第一台臂选 Franka** — Franka 的 libfranka 原生力矩控制、关节扭矩传感器、1000 Hz 频率，对 SE(3) 控制算法部署友好得多。UR12 的限制更多 |
| 3 | **如果先做 UR12，从力矩前馈模式起步** — UR12 的纯力矩模式限制多，用 `setTargetTorque` + 位置参考更安全 |
| 4 | **硬件接口层必须薄** — `UR12HW` 和 `FrankaHW` 各自控制在 100-150 行。如果超过 300 行，说明抽象层泄漏了机器人细节 |
| 5 | **用 YAML 配置文件管理所有机器人参数** — URDF 路径、增益、控制频率、关节名称映射、力矩限幅值等，不要在代码里硬编码 |
| 6 | **实机从"保持不动"开始** — 不要急着做轨迹跟踪。先在 regulation 模式下验证重力补偿和控制律稳定性。这一步走稳了，后面的轨迹跟踪自然水到渠成 |
| 7 | **所有控制代码先用仿真环境验证** — 在 MuJoCo 中创建 UR12/Franka 模型，用 Pinocchio + GIC 控制，验证与原生 MuJoCo 控制律行为一致后，再上实机 |
| 8 | **放弃 C++ 方案** — 鉴于你要写通用代码、快速迭代，Python 路径更现实。在 Python 中将控制频率定为：UR12 250 Hz / Franka 500 Hz 即可 |

---

## 6. 架构详图：代码流程

```
┌──────────────────────────────────────────────────────────┐
│  main.py                                                 │
│                                                          │
│  1. 加载配置 (config/ur12.yaml)                          │
│  2. 初始化 robot_model = RobotModel(urdf_path)          │
│  3. 初始化 hardware = UR12HW(ip)  # 或 FrankaHW(host)    │
│  4. 初始化 controller = GICController(robot_model, Kp/KR/Kd) │
│  5. 进入控制循环:                                         │
│     while running:                                       │
│       q, dq = hardware.get_joint_states()               │
│       p, R   = robot_model.forward_kinematics(q)        │
│       Jb     = robot_model.body_jacobian(q)             │
│       Vb     = Jb @ dq                                  │
│                                                          │
│       tau = controller.compute(p, R, Vb, Jb, t)         │
│       hardware.set_joint_torques(tau)                   │
│       wait_for_next_cycle()                             │
│                                                          │
│  GICController.compute():                                │
│     gd, Vd, dVd = self.trajectory(t)                    │
│     g_ed = inv(g) @ gd                                  │
│     Vd_star = adjoint(g_ed) @ Vd                        │
│     fp = Rᵀ @ Rd @ Kp @ Rdᵀ @ (p - pd)                  │
│     fR = vee(KR @ Rdᵀ @ R - Rᵀ @ Rd @ KR)               │
│     fg = [fp; fR]                                       │
│     M_tilde = (Jb @ inv(M) @ Jbᵀ)⁻¹                     │
│     tau_tilde = M_tilde @ dVd_star - Kd @ (Vb-Vd_star) - fg │
│     tau_bias = robot_model.bias_torque(q, dq)          │
│     tau_cmd = Jbᵀ @ tau_tilde + tau_bias                │
│     return tau_cmd                                      │
└──────────────────────────────────────────────────────────┘
```

---

## 7. 参考资源

| 资源 | 用途 |
|---|---|
| **Pinocchio** https://stack-of-tasks.github.io/pinocchio/ | 运动学/动力学计算的核心库 |
| **ros-industrial/ur_description** | UR12 URDF 文件 |
| **franka_ros** / **libfranka** | Franka 接口库和 URDF |
| **ur_rtde** | UR12 RTDE 通信（仅底层接口使用） |
| **论文: GUFIC** | Seo et al., "Geometric Unified Force-Impedance Control" |

---

*文档创建日期: 2026-07-25*
*更新说明：采用跨机械臂通用架构，以 Pinocchio 替代 MuJoCo 和机器人特定库*
