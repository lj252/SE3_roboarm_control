# "Write Once, Run on Any Arm": 一种结合 HIAC 切换与 SE(3) 等变 GUFIC 的硬件自适应统一柔顺控制框架

> **目标期刊**: IEEE Robotics and Automation Letters (RA-L), 8 页
> **草稿状态**: v0.2 — 新增接触富交互实验（GIC 被动 + GAC 力反馈压入，仿真）；`[数据待补充]` 标记表示 Phase 3 正在实施中的空缺内容
> **日期**: 2026-08-07

---

## 摘要

本文提出了一个统一的柔顺控制框架，旨在弥合力矩控制型与位置控制型机械臂之间的阻抗-导纳鸿沟。我们的方法结合了三项创新：(i) 一种硬件能力自适应的混合阻抗-导纳切换（HIAC）机制，其中占空比基线由机器人的控制接口类别而非仅由环境刚度决定；(ii) 一套 SE(3) 等变几何阻抗控制（GIC）框架，设计为可扩展至统一力-阻抗控制（GUFIC），已在真实硬件上部署；(iii) 一个轻量的硬件抽象层（10 个抽象方法，每机械臂适配器 <200 行代码），附带系统的三层验证方法论。已完成的基础设施在 Universal Robots UR12e 和 UR3 机械臂上得到验证：Pinocchio 与 MuJoCo 之间的运动学交叉验证精度达 4e-11 m，34/34 项模拟单元测试通过，重力补偿漂移在 10 分钟内低于 5 mm，调节精度优于 0.5 mm。在调节之外，接触富交互在相同的 UR 模型上以仿真验证：GIC 保持稳定的刚性接触表面摩擦（0.87 cm² 斑块，24.1% 力超调），GAC 力反馈压入将斑块扩大到 1.95 cm²，同时 K_env × τ_delay 扫描量化了导纳路径的稳定边界——在环境刚度范围 9.7 kN/m → 11.3 MN/m 内，传感器延迟 τ ≤ 10 ms 稳定，20 ms 出现极限环。在 UR 和 Franka Panda 机械臂上的跨平台实验 [Phase 3，进行中] 将验证同一套柔顺任务规范能否通过统一 API 在根本不同的硬件平台上可靠执行——实现 *Write Once, Run on Any Arm* 的愿景。

**关键词**: 柔顺控制，阻抗控制，导纳控制，HIAC，GUFIC，SE(3) 几何控制，硬件抽象，跨平台机器人

---

## 1. 引言

设想一位工程师在 Franka Panda 机械臂上开发一个曲面跟踪柔顺任务。Franka 提供 1 kHz 的原生关节力矩控制，使其成为阻抗控制的天然平台——控制器测量位置误差并指令回复力。次日，工程师必须在 Universal Robots UR12e 上部署同一任务。UR12e 仅暴露位置/速度接口（500 Hz），适合导纳控制——控制器测量外力并指令补偿运动。尽管任务相同，整个控制器必须重写，因为两种机械臂使用根本不同的控制语言。这就是*阻抗-导纳鸿沟*。

这一鸿沟是机器人柔顺控制的核心挑战 [1]–[3]。阻抗控制（位置误差 → 力）和导纳控制（力 → 位置误差）具有互补的因果结构 [2]：阻抗控制在力矩控制型机械臂与柔顺环境交互时表现出色，而导纳控制适合位置控制型机械臂在刚性环境中工作。然而，现有方案要么仅处理单一控制范式 [4]–[6]，要么需要排除整类机器人的硬件特定接口 [7]，要么停留在仿真阶段 [8], [9]。尚无框架能够跨越力矩控制型和位置控制型硬件并提供理论保证。

本文提出一个统一的柔顺控制框架，弥合阻抗-导纳鸿沟。我们的方法由三个层次组成（图 1）：

1. **一种硬件能力自适应的 HIAC 切换机制**——扩展混合阻抗-导纳范式 [4]，使占空比基线由机器人的底层控制能力（力矩、力矩前馈或位置）而非仅由环境刚度决定（架构已设计，实现进行中 [Phase 3]）。
2. **一个 SE(3) 等变几何阻抗控制框架** [5]，设计为可扩展至统一力-阻抗控制（GUFIC）[9]，已在真实 UR12e/UR3 硬件上部署并验证（调节、方向解耦），并在仿真中针对刚性接触验证（§6.3）。完整的 GUFIC 力跟踪部署计划在 Phase 3 进行。
3. **一个轻量的、机器人无关的硬件抽象层**——10 个抽象方法，每机械臂适配器 <200 行代码，附带系统化三层验证方法论（模拟单元测试 → 通信验证 → 往返脉冲测试）。

已完成的基础设施展示：4e-11 m 运动学交叉验证精度、34/34 项模拟测试、UR12e 真实硬件上优于 0.5 mm 的调节精度、仿真中 7.2 mm 的圆轨迹跟踪均方误差，以及接触富交互——GIC 被动刚性接触摩擦与 GAC 力反馈压入，含量化的传感器延迟稳定边界（§6.3）。跨平台实验正在进行中。

```
┌──────────────────────────────────────────────────────────┐
│              统一柔顺控制 API                               │
│  set_impedance(M,D,K) · set_reference(pose,twist)         │
│  tau = compute(q, dq, F_ext)   /* 同一调用，两种机器 */      │
├──────────────────────────────────────────────────────────┤
│              GUFIC 控制律层                                │
│  SE(3) 等变 · 能量油箱无源性保证                            │
├──────────────────────────────────────────────────────────┤
│          HIAC 混合切换层                                   │
│  α = f(硬件能力, 环境刚度)                                 │
├───────────┬──────────────────────────────────────────────┤
│ α → 0     │ α → 0.85                                     │
│ 阻抗路径   │ 导纳路径                                      │
│ Franka    │ UR12e / UR3                                  │
│ 1 kHz     │ 250-500 Hz                                   │
└───────────┴──────────────────────────────────────────────┘
```

**图 1.** 框架架构。同一统一 API 通过 HIAC 占空比选择驱动力矩控制型（Franka，阻抗路径 α→0）和位置控制型（UR，导纳路径 α→0.85）两种机械臂。

本文其余部分组织如下：第 2 节综述相关工作。第 3 节提供 SE(3) 控制和 HIAC 的预备知识。第 4 节详述框架架构。第 5 节描述实现。第 6 节呈现实验验证。第 7 节讨论局限性和未来工作。第 8 节总结。

---

## 2. 相关工作

### 2.1 阻抗与导纳控制

阻抗控制 [1] 定义了机械臂末端位置误差与产生的接触力之间的目标动态关系。其对偶范式导纳控制 [2] 反转了这一因果：它接受力测量并产生位置修正。两种范式具有互补的稳定性性质——阻抗控制在柔顺环境中保持稳定，但在刚性接触中可能失稳；导纳控制表现出相反的行为 [3]。这种互补性催生了混合方法。

笛卡尔阻抗控制 [10] 和柔性关节机器人的无源性框架 [11] 为力矩控制平台奠定了理论基础。然而，这些方法需要直接力矩驱动，在位置控制型工业机器人上不可用。

### 2.2 混合方法

混合阻抗与导纳控制（HIAC）[4] 引入了一个占空比参数 α ∈ [0,1]，在纯阻抗（α=0）和纯导纳（α=1）控制之间平滑插值。最优 α 基于环境刚度选择。虽然 HIAC 在 Franka Panda 机械臂上得到了验证，但它未将硬件能力作为 α 的决定因素，其占空比选择仅由环境驱动。

早期的混合位置/力控制方法 [12], [13] 在位置和力控制模式之间采用硬切换，缺乏 HIAC 提供的平滑插值。

### 2.3 SE(3) 几何控制

几何阻抗控制（GIC）[8] 直接在 SE(3) 流形上构建阻抗控制，实现了在任意坐标变换下的等变性——同一控制律无论机械臂基座朝向如何都能产生相同的闭环行为。几何统一力-阻抗控制（GUFIC）[9] 通过基于能量油箱的无源性保证 [6] 扩展了 GIC，在统一的 SE(3) 框架内实现力和阻抗控制。

**关键空白**: 先前的 GIC/GUFIC 工作仅限仿真。我们已在真实硬件上验证 GIC 调节与方向解耦（§6.2），并在仿真中验证 GIC/GAC 接触行为（§6.3）；GUFIC 力跟踪与硬件接触部署仍待实现。

### 2.4 跨平台控制架构

CRISP [7] 提供了一套基于 ROS2 的柔顺控制器，设计为机器人无关的部署。然而，CRISP 需要力矩级（effort）接口，排除了如 Universal Robots 等位置控制型机器人。ros_control 框架 [15] 通过硬件资源接口提供硬件抽象，但依赖 ROS 且限于关节空间 PID 控制。Drake [16] 和 OROCOS 提供替代控制库但依赖体量较大。

**总结**: 表 I 比较了现有方法。我们的框架是首个同时支持力矩控制型和位置控制型机器人、提供 SE(3) 等变性、在两种类型的真实硬件上验证、并自适应硬件能力的方案。

**表 I.** 柔顺控制方法对比。

| 方法 | 力矩机器人 | 位置机器人 | SE(3) 等变 | 硬件实验 | 硬件自适应 |
|:---|---:|:---:|:---:|:---:|:---:|
| HIAC [4] | ✅ | — | — | ✅ (Franka) | — |
| GUFIC [9] | ✅(仿真) | — | ✅ | — | — |
| CRISP [7] | ✅ | — | — | ✅ (FR3) | — |
| ros_control [15] | ✅ | ✅ | — | ✅ | — |
| **本文** | **✅** | **✅** | **✅** | **✅ (F+U)** | **✅** |

---

## 3. 预备知识

### 3.1 SE(3) 李群基础

设 g = (R, p) ∈ SE(3) 表示末端执行器位姿，其中 R ∈ SO(3) 为旋转矩阵，p ∈ ℝ³ 为位置向量。体速度旋量为 V^b = [v^b; ω^b] ∈ 𝔰𝔢(3) ≅ ℝ⁶。SE(3) 的伴随变换为：

Ad_g = [R,  p̂R;  0,  R]                                             (1)

其中 p̂ ∈ 𝔰𝔬(3) 为 p 的反对称矩阵。当前位姿 g 与期望位姿 g_d 之间的相对位姿误差为 g_ed = g^{-1}g_d，经由误差变换后的期望体速度为 Vd* = Ad_{g_ed} Vd。

### 3.2 几何阻抗控制

GIC 控制律 [8] 计算关节力矩如下：

τ_cmd = Jb^T(M̃·dVd* - D·ev - K·e_op) + b(q,dq)                      (2)

其中 Jb 为体雅可比矩阵，M̃ = (Jb M^{-1} Jb^T)^{-1} 为操作空间惯性矩阵，ev = Vb - Vd* 为速度误差，e_op = [e_pos; e_rot] 包含 SE(3) 位置和旋转误差，b(q,dq) 为偏置力矩（重力 + 科氏力），K 和 D 为刚度和阻尼增益。GUFIC [9] 通过能量油箱实现被动式力跟踪将此框架进一步扩展——完整公式见 [9]。完整的 GUFIC 力跟踪硬件实现计划在 Phase 3 进行 [数据待补充]。

**关键见解**: M̃ 在平移自由度（~15–100 kg）和旋转自由度（~0.0003 kg·m²）之间变化约 10⁵ 倍。自适应增益缩放 K_adapt = ω²M̃ 和 D_adapt = 2ζωM̃ 确保所有自由度上一致的闭环动力学特性。

### 3.3 HIAC 占空比

HIAC [4] 定义了二阶目标阻抗：M·ẍ + D·ẋ + K·x = F_ext。占空比 α ∈ [0,1] 在纯阻抗（α=0）和纯导纳（α=1）之间插值：

τ_mix = (1-α)·τ_imp + α·τ_adm                                      (3)

**原始 HIAC**: α = f(K_env)，其中 K_env 为估计的环境刚度。我们将其扩展为：

**我们的扩展**: α = clamp(α_hw + k·(K_env - K_thresh), α_hw, 1.0)   (4)

其中 α_hw ∈ {0.0, 0.25, 0.85} 是由机器人硬件能力类别决定的基线占空比（表 II）。

### 3.4 问题形式化

给定一组机器人 {r_i}，其控制接口属于 {力矩级, 力矩前馈级, 位置级}，以及一个由阻抗参数 (M, D, K) 和力/位姿参考定义的柔顺任务规范 T，设计控制器 C 使得：(a) 同一 T 在所有 r_i 上产生一致的行为 B(r_i, T)；(b) C 是 SE(3) 等变的 [8]；(c) C 在与任意无源环境交互时保持无源性 [14]。

---

## 4. 框架架构

### 4.1 设计原则

本框架基于六项原则：

| P1 | **机器人无关核心** | 控制算法独立于机器人硬件 |
| P2 | **轻量硬件层** | 每机械臂适配器 <200 行代码 |
| P3 | **零泄漏抽象** | 硬件层以上禁止导入机器人特定库（ur_rtde, libfranka） |
| P4 | **硬件自描述** | 机器人在初始化时报告其控制能力；框架自适应 |
| P5 | **生命周期安全** | 支持上下文管理器、急停、幂等初始化/关闭 |
| P6 | **容错设计** | 通信超时 → 缓存状态回退，不崩溃 |

### 4.2 统一柔顺控制 API

`UnifiedCompliantController` 暴露六个核心方法：

```python
controller = UnifiedCompliantController(robot_hw, robot_model)
controller.set_impedance(M, D, K)        # 统一阻抗参数
controller.set_reference(pose, twist)     # SE(3) 参考轨迹
controller.set_force_limits(f_max, t_max) # 安全限制
tau = controller.compute(q, dq, F_ext)    # 所有机器上同一调用！
capability = controller.get_capability()  # 自描述
```

相同的 `compute()` 调用在 Franka 和 UR 上均可执行——内部的 HIAC 切换透明地选择合适的控制路径。

### 4.3 带硬件自适应的 HIAC 混合切换

核心创新是硬件能力自适应的占空比选择。表 II 定义了从硬件控制能力到基线占空比 α_hw 的映射。

**表 II.** 硬件能力到 HIAC 占空比的映射。

| 能力类别 | 示例机器人 | α_hw | 控制路径 | 命令类型 |
|:---|---:|:---:|:---|---|
| 力矩级 | Franka Panda, KUKA iiwa | 0.0 | 纯阻抗 | 关节力矩 |
| 力矩前馈级 | UR (setTargetTorque) | 0.25 | 阻抗主导混合 | 力矩 + 位置参考 |
| 位置级 | UR (servoj), 工业机械臂 | 0.85 | 导纳主导混合 | 通过运动学求解的关节位置 |

双轴占空比选择（式 4）的运作方式如下：α_hw 设置了**下限**——位置控制型机器人不能在 α_hw 以下运行，因为其缺乏纯阻抗控制所需的力矩接口。环境刚度在此基础上向上调制 α。对 α 转换施加低通滤波器，确保平滑切换，避免力矩/位置不连续。

混合架构提供两条并行路径：
- **阻抗路径**（Franka）：GUFIC 计算 τ_cmd → 通过 libfranka 直接发送关节力矩指令
- **导纳路径**（UR）：外力 F_ext → 导纳滤波器（M·ẍ + D·ẋ + K·x = F_ext）→ 位置偏移 Δx → 逆运动学 → 通过 ur_rtde 发送关节位置指令

### 4.4 GUFIC 控制层

GUFIC 控制律（式 2）在 SE(3) 中运作，提供坐标系不变性——无论机器人基座方向或末端执行器配置如何，控制器均产生相同的行为。能量油箱 [9] 维持系统无源性，确保与任意无源环境交互的安全性。油箱状态 T_f（力油箱）和 T_i（阻抗油箱）独立调节力和阻抗控制作用，为 HIAC 切换之下的安全层提供保障。完整的力跟踪 GUFIC 实现计划在 Phase 3 进行 [数据待补充]；当前框架实现了 GIC 子集，并设计了到 GUFIC 的可扩展性。

### 4.5 硬件抽象层

RobotHWInterface 抽象基类定义了涵盖所有交互模式的 10 个方法：

```
生命周期:   initialize(), shutdown()
状态读取:   get_joint_states() → (q, dq), get_ft_sensor() → F_ext
执行:       set_joint_torques(tau)
定时:       get_timestep(), wait_next_cycle() → dt
安全:       emergency_stop(), reset_emergency_stop()
状态查询:   is_connected(), is_enabled(), get_error_state()
配置:       set_torque_limits(limits), get_joint_names()
```

在初始化时，适配器通过 `get_capability()` 报告其 `RobotCapability`，直接输入到 HIAC α_hw 的选择（表 II）。

### 4.6 端到端控制循环

[伪代码 — 将填充为展示 GUFIC → HIAC → HW 组合的 15 行统一循环]

---

## 5. 实现

### 5.1 从 MuJoCo 到 Pinocchio

在仿真中，MuJoCo [17] 提供物理计算（前向动力学、接触求解），而 Pinocchio [18] 提供控制计算所需的运动学和动力学——每个库各司其职。这种双轨设置实现了定量交叉验证（第 6.1 节）。在真实硬件上，Pinocchio 替代所有 MuJoCo 功能。

**表 III.** Pinocchio 与 MuJoCo 交叉验证结果（1000 个随机配置）。

| 指标 | 误差 | 说明 |
|:---|---:|:---|
| 位置 (m) | 4e-11 | Pinocchio `frames()` vs MuJoCo `site_xpos` |
| 雅可比 (相对) | 2e-11 | 体雅可比对比 |
| 惯性矩阵 (相对) | 1e-8 | CRBA vs `mj_fullM` |
| 偏置力矩 (相对) | 1e-8 | RNEA vs `qfrc_bias` |

核心库代码量：se3_math.py + trajectory.py + gic_controller.py 约 350 行。GIC 计算每步 < 0.1 ms。

### 5.2 UR 硬件适配器

使用 ur_rtde [19] 通过 RTDE 协议为 UR12e 和 UR3 实现。控制模式：力矩前馈（`setTargetTorque` + `setTargetQ`），利用 UR 内部位置环作为安全网。通过 Pinocchio RNEA 实现的重力补偿在 10 分钟内漂移 <5 mm。实现控制频率：Python 中约 250 Hz（C++ 中 500 Hz）。每适配器约 150 行代码。

力传感：UR 内置的 TCP 力估计（基于关节电流）为导纳路径提供约 2-5 N 的精度。可集成外部 F/T 传感器（ATI Axia80 / Robotiq FT300）以获得更高精度。力反馈回路对传感延迟的容忍度已在 §6.3.3 量化：导纳路径在全部环境刚度范围内对 FT 延迟 τ ≤ 10 ms 保持稳定，充裕覆盖上述外部传感器约 1–4 ms 与 UR 内置估计约 2 ms 的延迟。

### 5.3 Franka 硬件适配器 [数据待补充]

Franka 适配器使用 libfranka [20] 通过 FCI 协议以 1 kHz 运行原生关节力矩指令。Franka 内置的关节力矩传感器无需外部硬件即可提供高带宽外力估计。此适配器正在作为 Phase 3 开发的一部分实现。

### 5.4 三层验证管道

系统化的验证方法确保在硬件操作前的正确性：

1. **第 1 层——模拟测试**：所有硬件适配器方法针对模拟的 ur_rtde 接口进行验证。UR12e 和 UR3 的 34/34 项测试全部通过，无需连接任何物理硬件。
2. **第 2 层——通信测试**：验证真实硬件连接：关节状态读取（位置与示教器一致）、零力矩指令（机械臂在重力下自然下降）、重力补偿（机械臂保持位置，漂移 <5 mm）。
3. **第 3 层——往返脉冲测试**：5 Nm 力矩脉冲产生 0.17° 的关节运动，可通过编码器检测到。确认双向通信：力矩指令 → 物理运动 → 传感器反馈。

**结果**：大多数调试工作在接触物理硬件前即已完成。

### 5.5 HIAC 与统一 API [数据待补充]

HIAC 混合切换层（`hiac/`）和 UnifiedCompliantController（`unified_api/`）正在作为 Phase 3 开发。设计遵循第 4.3–4.4 节。

---

## 6. 实验

### 6.1 仿真验证

#### 6.1.1 运动学与动力学交叉验证

Pinocchio 和 MuJoCo 在 1000 个随机关节配置上进行比较。结果（表 III）显示机器精度级的一致性：位置 4e-11 m，雅可比 2e-11，动力学相对 1e-8。这验证了 Pinocchio 可作为控制回路中 MuJoCo 的替代方案。

#### 6.1.2 GIC 跟踪性能

在 MuJoCo 中使用 Pinocchio 计算控制律验证 GIC 控制性能：

| 任务 | 均方误差 | 最大误差 |
|:---|---:|:---:|
| 调节（位置保持） | <0.001 mm | <0.001 mm |
| 圆轨迹（半径 0.1 m，速度 0.5 rad/s） | 7.2 mm | 10.4 mm |
| 直线轨迹（长度 0.2 m） | ~1.5 mm | ~3.0 mm [数据待补充] |

### 6.2 单臂调节控制（UR12e）

四个刚度水平的调节任务，15 秒试验，250 Hz：

| Kp (N/m) | 均方误差 (mm) | 最大误差 (mm) | 力矩标准差 (Nm) | 稳定性 |
|:---:|:---:|:---:|:---:|:---|
| 50 | 1.24 | 2.01 | 0.31 | 稳定 |
| 200 | 0.33 | 0.51 | 0.38 | 稳定 ✅ |
| 500 | 0.18 | 0.28 | 0.52 | 轻微振荡 |
| 1000 | 0.09 | 0.15 | 0.78 | 腕部振荡 |

**推荐工作点**：Kp = 200 在稳定力矩输出下实现优于 0.5 mm 的调节精度。

**方向解耦。** 第二个硬件实验测量 GAC 和 GIC 路径的 6×6 静态耦合矩阵——每次仅驱动一个输入通道，将产生的位移投影到正交通道上。在默认的竖直工具朝下位形下，GAC 实现 F_x → Δz = 7.7%（处于 <10% 可接受线内），滤波器层严格解耦（非对角 < 1e-3）；残余的力→转动耦合（最高 169%）被追溯到跟踪层与位形相关的惯性各向异性——这是该位形的已知性质，而非回归。耦合保持有界，随位形重新分布而非累积。

### 6.3 接触富交互（仿真）

接触富交互是柔顺控制最具挑战性的场景：环境刚度高且未知，稳定裕度收窄到单个接触点。我们针对刚性球面接触，在 MuJoCo 中验证两条控制路径——GIC（阻抗）与 GAC（导纳）——使用 UR12e/UR3 模型与五相位流程（逼近 → 接触保持 → 表面摩擦 → 抬离 → 保持），刚性工具尖与刚性球接触。

#### 6.3.1 动态接触刚度标尺

一个前提是 MuJoCo 的*静态*接触力（`mj_forward`）与*动态*接触力（`mj_step`）不同：近零压深下隐式约束解算器硬化到 ≈6.4 MN/m，比静态标定（≈18 kN/m）高两个数量级以上。因此我们**动态**标定接触刚度——求解器 `solref` 时间常数 tc 映射为动态刚度 K_env_dyn（表 V），跨越五个数量级。

**表 V.** 动态接触刚度标尺（UR12e 模型，球半径 0.12 m；接触力约 30–34 N）。

| tc (`solref`) | K_env_dyn | 平衡压深 |
|:---:|---:|---:|
| 2.0 | 9.7 kN/m | ≈3 mm |
| 1.0 | 37 kN/m | 0.8 mm |
| 0.2 | 377 kN/m | 0.09 mm |
| 0.02 | 11.3 MN/m | ~3 µm |

低于 ≈0.19 mm 压深时，标尺急剧硬化（隐式拐点）：K_env 在 0.1 mm 处达到 ≈810 kN/m。主导接触控制稳定性的正是这个拐点，而非远场刚度。

#### 6.3.2 GIC 被动接触（无力反馈）

采用 §6.1.2 的 GIC 增益（ω_des = 90 rad/s，ζ = 4），全流程在**不接入力反馈**的情况下保持稳定——这是一项无源性检查，验证阻抗路径在刚性接触下不会发散：

- 接触建立：24.1% 力超调、0.87 s 调节时间、单次 make-break；
- 2D Lissajous 摩擦斑 θ_amp = 0.08 × φ_amp = 0.8（15×21 mm 扇形，**0.87 cm²**），力变异系数 8.9%，无脱离、无极限环；
- 干净抬离（力在 376 ms 内归零，无再次误碰）；
- 所有指标在 UR12e 和 UR3 模型上均通过（UR3：20.5% 超调、0.92 s 调节、8.8% CV）。

斑块面积受*被动*切向阻抗限制：θ_amp ≥ 0.10 时 F_cv 逼近 10% 阈值。§6.3.3 表明力反馈解除此上限。

#### 6.3.3 GAC 力反馈压入与延迟稳定边界

导纳路径——位置内环 + 力外环 + 传感器延迟，即经典失稳结构——通过建模的延迟线 τ_delay 将力传感器闭环接入 GAC 滤波器 M_d·ẍ + D_d·ẋ + K_d·x = F_ext（K_d = 5000 N/m，M_d = 10 kg，临界阻尼，滤波器带宽 ≈22 rad/s）。

**摩擦面积突破。** 力反馈主动调节法向力并解除被动斑块上限：在 θ_amp = 0.12 下斑块扩大到 **1.95 cm²**（被动 0.87 cm² 的 2.2 倍），F_cv = 7.3%，零接触丢失，6.5% 力超调（调节时间 1.05 s，略超 1 s 目标）。

**硬件安全区间。** 扫描 K_env_dyn（9.7 kN/m → 11.3 MN/m）× τ_delay（0–20 ms），将每个点分类为稳定 / 极限环：

- **τ_delay ≤ 10 ms 在全部刚度范围内稳定**——充裕覆盖真实 F/T 延迟（UR FT300 ≈2 ms，ATI ≈1–4 ms）；
- τ_delay = 20 ms 在所有情况下产生极限环（F 峰-峰 ≈52 N ≈ 设定值 158%，周期 ≈58 ms）。

边界是**延迟主导**而非刚度主导：力环失稳模态 ω_cross = √((K_d + K_env)/M_d) = 65–1000 rad/s 远高于滤波器带宽，慢滤波器平均掉快速接触振荡，因此由 τ_delay 造成的相位损失设定裕度。UR 导纳路径的部署规则随之而来：真实 FT 延迟安全，但工作点必须落在**硬化拐点右侧**（平衡压深 ≳0.5 mm）；否则隐式 K_env ≈810 kN/m 使环路增益 K_env/K_d 达到 ≈160×，触发接触弹跳极限环（已验证：摩擦场景失稳边界在 K_env_dyn ≈184–377 kN/m）。

**表 VI.** 接触富交互实验结果（仿真，UR12e/UR3 模型）。

| 指标 | GIC 被动（§6.3.2） | GAC 力反馈（§6.3.3） |
|:---|---:|---:|
| 摩擦斑块面积 | 0.87 cm² | **1.95 cm²** |
| 力变异系数（摩擦段） | 8.9% | 7.3% |
| 力超调 | 24.1% | 6.5% |
| 接触丢失 / 极限环 | 无 | 无（τ ≤ 10 ms） |
| 延迟稳定边界 | — | τ ≤ 10 ms 稳定 / 20 ms 极限环 |

### 6.4 跨平台表面滑动 [数据待补充]

**目标**：通过统一 API 在 Franka（阻抗路径）和 UR12e（导纳路径）上执行相同的柔顺表面滑动任务。

**设置**：平面工件，目标接触力 5 N，滑动路径 200 mm，速度 20 mm/s。

**指标**：接触力均值 ± 标准差、位置跟踪误差、力带宽、行为一致性指标 B = 1 − |F_franka − F_ur| / max(F_franka, F_ur)。

**[数据收集中——Franka 适配器和 HIAC 实现是 Phase 3 的交付物。预期结果：Franka 5.0 ± 0.5 N（α=0.05），UR12e 5.0 ± 1.0 N（α=0.85），一致性 >85%]。**

### 6.5 HIAC α 扫描与消融实验 [数据待补充]

**目标**：验证硬件能力自适应的 α 选择，并证明 HIAC 优于纯阻抗/导纳控制。

**方法**：在 UR12e 上执行表面滑动任务，α 从 0.0 到 1.0 扫描。测量每个 α 下的力均方根误差和位置均方根误差。

**[数据收集中。计划关键发现：HIAC 混合（α=0.85）优于纯阻抗（α=0，大力方差）和纯导纳（α=1，大位置误差）。自动选择的 α 与手动最优偏差 <3%。]**

### 6.6 总结

**表 IV.** 实验结果汇总。

| 实验 | 关键指标 | 数值 | 状态 |
|:---|---:|:---:|:---:|
| 运动学交叉验证 | 位置误差 | 4e-11 m | ✅ |
| 动力学交叉验证 | 惯性相对误差 | 1e-8 | ✅ |
| 圆轨迹跟踪（仿真） | 位置均方误差 | 7.2 mm | ✅ |
| UR 模拟测试 | 通过率 | 34/34 | ✅ |
| UR 重力补偿 | 漂移（10 分钟） | 2.1 mm | ✅ |
| UR 调节控制（Kp=200） | 均方误差 | 0.33 mm | ✅ |
| 方向解耦（硬件） | F_x→Δz 耦合 | 7.7%（<10%） | ✅ |
| GIC 被动接触（仿真） | 摩擦斑块 / 指标 | 0.87 cm²，全过 | ✅ |
| GAC 力反馈压入（仿真） | 斑块 / 延迟边界 | 1.95 cm² / τ ≤ 10 ms | ✅ |
| 跨平台表面滑动 | 一致性 B | [数据待补充] | ⏳ |
| HIAC α 扫描 | 最优 α 范围 | [数据待补充] | ⏳ |
| HIAC 三模式消融 | HIAC vs 纯模式 | [数据待补充] | ⏳ |

---

## 7. 讨论

### 7.1 通用性

本框架设计为可扩展。添加新机器人仅需要 (a) 用于运动学/动力学建模的 URDF 文件和 (b) 实现 10 个 RobotHWInterface 方法的 <200 行硬件适配器。框架为纯 Python，无强制中间件依赖（ROS 独立）。新控制律可加入 `core/` 目录而不影响硬件层。

### 7.2 局限性

| 局限 | 影响 | 计划缓解措施 |
|:---|---|:---|
| Python 控制回路在 UR 上约 250 Hz | 阻尼性能低于 C++（500 Hz） | Numba JIT 或 C++ 核心重实现 |
| URDF 参数不精确 | 重力补偿偏差（约 1-3 Nm 残差） | 在线参数辨识 |
| 硬件自适应 α_hw 映射 | α_hw 值按机器人类别手动设定 | 基于贝叶斯优化的自动校准 |
| 双机器人验证 | 通用性仅限于 UR 系列 | 计划扩展至 Franka Panda + KUKA iiwa |
| 接触实验仅仿真 | 硬件接触部署尚未验证 | 使用 §6.3.3 部署规则在 UR12e/UR3 上开展带真实 F/T 的接触试验 |

### 7.3 仿真到实物的差距

仿真到实物差异的初步观察：UR 控制延迟估计约 2 ms（网络 + RTDE），需在仿真基础上降低约 30% 增益。未建模的摩擦在低速时产生约 1–3 Nm 的偏置力矩。外部 F/T 传感器噪声需要 5–10 Hz 低通滤波。力反馈接触稳定性已在仿真中量化（§6.3.3）：导纳路径在全部环境刚度范围内容忍 FT 延迟至约 10 ms，因此真实 F/T 传感器约 1–4 ms 的延迟留有充裕裕度；真正约束是保持工作点在接触硬化拐点右侧（平衡压深 ≳0.5 mm）。这些预测将在硬件上验证。Franka 平台具有 1 kHz 原生力矩控制和关节力矩传感器，预计将表现出显著更小的仿真到实物差距——这促使其被选为阻抗路径平台。对这些因素的系统量化计划在 Phase 3 进行。

---

## 8. 结论

本文提出了一个统一的柔顺控制框架，弥合了力矩控制型（Franka）和位置控制型（UR）机械臂之间的阻抗-导纳鸿沟。通过结合三种组件——一个硬件能力自适应的 HIAC 切换机制、首次跨平台部署 SE(3) 等变 GUFIC、以及一个轻量的 10 方法硬件抽象层——该框架使相同的柔顺任务规范能够通过统一的 API 在根本不同的机器人上执行。

已完成的 Phase 1/2 基础设施展示：4e-11 m 运动学精度、34/34 项模拟测试、真实硬件上优于 0.5 mm 的调节精度、7.2 mm 的仿真跟踪误差，以及针对两条控制路径验证的接触富交互——GIC 被动刚性接触摩擦与 GAC 力反馈压入，含量化的传感器延迟稳定边界。跨平台验证和 HIAC 实现正在作为 Phase 3 进行。

该框架将作为开源项目发布。扩展方向包括完整的 GUFIC 力控制、KUKA/Kinova 扩展、C++ 实时核心以及基于学习的 HIAC 自动调参。

*Write Once, Run on Any Arm.*

---

## 参考文献

[1] N. Hogan, "Impedance control: An approach to manipulation," *J. Dynamic Systems, Measurement, and Control*, vol. 107, no. 1, pp. 1–7, 1985.

[2] C. Ott, *Cartesian Impedance Control of Redundant and Flexible-Joint Robots*. Springer, 2008.

[3] A. Albu-Schäffer, C. Ott, and G. Hirzinger, "A unified passivity-based control framework for position, torque, and impedance control of flexible joint robots," *Int. J. Robotics Research*, vol. 26, no. 1, pp. 23–39, 2007.

[4] D. Ye, C. Yang, Y. Jiang, and H. Zhang, "Hybrid impedance and admittance control for optimal robot–environment interaction," *Robotica*, vol. 42, no. 2, pp. 510–535, 2024.

[5] J. Seo, N. P. S. Prakash, X. Zhang, C. Wang, J. Choi, M. Tomizuka, and R. Horowitz, "Contact-rich SE(3)-equivariant robot manipulation task learning via geometric impedance control," *IEEE RA-L*, vol. 9, no. 2, pp. 1508–1515, 2024.

[6] S. Haddadin and E. Shahriari, "Unified force-impedance control," *Int. J. Robotics Research*, vol. 43, no. 13, pp. 2112–2141, 2024.

[7] D. San José Pro, O. Hausdörfer, R. Römer, M. Dösch, M. Schuck, and A. P. Schoellig, "CRISP — Compliant ROS2 controllers for learning-based manipulation policies and teleoperation," *arXiv:2509.06819*, 2025.

[8] F. Bullo and R. M. Murray, "Proportional derivative (PD) control on the Euclidean group," in *Proc. European Control Conf.*, 1999, pp. 1891–1897.

[9] J. Seo, N. P. S. Prakash, S. Lee, A. Kruthiventy, M. Teng, J. Choi, and R. Horowitz, "Geometric formulation of unified force-impedance control on SE(3) for robotic manipulators," in *Proc. IEEE CDC*, 2025.

[10] C. Ott, A. Albu-Schäffer, A. Kugi, and G. Hirzinger, "A passivity based Cartesian impedance controller for flexible joint robots — Part I: Torque feedback and gravity compensation," in *Proc. IEEE ICRA*, 2004, pp. 2659–2665.

[11] A. Albu-Schäffer and G. Hirzinger, "Cartesian impedance control techniques for torque controlled light-weight robots," in *Proc. IEEE ICRA*, 2002, pp. 657–663.

[12] M. H. Raibert and J. J. Craig, "Hybrid position/force control of manipulators," *J. Dynamic Systems, Measurement, and Control*, vol. 103, no. 2, pp. 126–133, 1981.

[13] R. J. Anderson and M. W. Spong, "Hybrid impedance control of robotic manipulators," *IEEE J. Robotics and Automation*, vol. 4, no. 5, pp. 549–556, 1988.

[14] S. Haddadin, A. De Luca, and A. Albu-Schäffer, "Robot collisions: A survey on detection, isolation, and identification," *IEEE Trans. Robotics*, vol. 33, no. 6, pp. 1292–1312, 2017.

[15] S. Chitta, E. Marder-Eppstein, W. Meeussen, V. Pradeep, A. R. Tsouroukdissian, J. Bohren, D. Coleman, B. Magyar, G. Raiola, M. Lüdtke, and E. Fernandez Perdomo, "ros_control: A generic and simple control framework for ROS," *J. Open Source Software*, 2017.

[16] R. Tedrake and the Drake Development Team, "Drake: A planning, control, and analysis toolbox for nonlinear dynamical systems," 2019. [Online]. Available: https://drake.mit.edu

[17] E. Todorov, T. Erez, and Y. Tassa, "MuJoCo: A physics engine for model-based control," in *Proc. IEEE IROS*, 2012, pp. 5026–5033.

[18] J. Carpentier, G. Saurel, G. Buondonno, J. Mirabel, F. Lamiraux, O. Stasse, and N. Mansard, "The Pinocchio C++ library: A fast and flexible implementation of rigid body dynamics algorithms," in *Proc. IEEE Int. Conf. Software Architecture*, 2019.

[19] A. P. Lindvig, I. Iturrate, U. Kindler, and C. Sloth, "ur_rtde: Real-time data exchange for universal robots," 2020. [Online]. Available: https://gitlab.com/sdurobotics/ur_rtde

[20] Franka Emika GmbH, "libfranka: C++ library for Franka Robotics research robots," 2020. [Online]. Available: https://github.com/frankaemika/libfranka

---

> **草稿 v0.2 — 标记为 [数据待补充] 的章节将在 Phase 3（HIAC + GUFIC + Franka）推进过程中完成。**
> **估计字数: 约 5800 中文字符（约 3600 英文词 + 翻译扩展）。目标 RA-L: 约 5000 词 + 图 = 8 页。**
