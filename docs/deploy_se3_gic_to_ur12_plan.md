# SE(3) 几何阻抗控制部署到 UR12 实机 — 实施计划与困难分析

> 基于 [GUFIC_mujoco-main](..) 的 GIC (Geometric Impedance Control) 仿真代码，部署到 Universal Robots UR12 真实机械臂

---

## 1. 概述

### 1.1 仿真代码的核心结构

GIC 仿真代码的核心是 **SE(3) 李群框架下的几何阻抗控制律**：

```python
# 1. 获取机器人状态
p, R       = robot_state.get_pose()         # SE(3) 位姿
Vb         = robot_state.get_body_ee_velocity()  # 体速度 (se(3))
Jb         = robot_state.get_body_jacobian()
M          = robot_state.get_full_inertia()
qfrc_bias  = robot_state.get_bias_torque()

# 2. 计算SE(3)误差与期望
g_ed = inv(g) @ gd
Vd_star = Ad(g_ed) @ Vd

# 3. 阻抗弹簧力
fp = Rᵀ @ Rd @ Kp @ Rdᵀ @ (p - pd)    # 位置弹簧
fR = vee(KR @ Rdᵀ @ R - Rᵀ @ Rd @ KR)  # 朝向弹簧(SO(3))

# 4. 操作空间控制律
M_tilde = (Jb @ M⁻¹ @ Jbᵀ)⁻¹
tau_tilde = M_tilde @ dVd_star - Kd @ ev - fg

# 5. 映射到关节空间
tau_cmd = Jbᵀ @ tau_tilde + qfrc_bias
```

**仿真中由 MuJoCo 提供的关键要素：**

| 要素 | MuJoCo 调用 | 替代方案（实机） |
|---|---|---|
| 正运动学 (p, R) | `mj_step1` → `site_xpos/site_xmat` | UR 正向运动学（DH参数或URDF） |
| 体雅可比 Jb | `mj_jacSite` + 旋转变换 | URDF 运动学库（KDL, Pinocchio） |
| 体速度 Vb | Jb @ dq | KDL + 关节速度读数 |
| 惯性矩阵 M | `mj_fullM` | URDF 惯性参数 / 辨识 |
| 偏置力矩 (重力+科氏) | `qfrc_bias` | URDF 动力学 + 递推牛顿-欧拉 |
| 力/力矩传感器 | MuJoCo 传感器 | 实装 FT 传感器 (Onrobot, Robotiq) |
| 力矩执行 | `ctrl` → `mj_step2` | UR 力矩接口 / 前馈 |

### 1.2 UR12 特点

- 6-DOF 协作机器人，最大负载 12 kg
- 控制频率上限：**500 Hz**（RTDE）/ 125 Hz（Dashboard）
- 原生支持：**位置控制**、**速度控制**，**力矩控制**需在 `ur_control` 或 `ur_rtde` 中启用
- 接口选项：
  - **ur_rtde** (C++/Python): 最推荐，500 Hz 双向通信，支持力矩前馈
  - **ur_driver** (ROS): 集成方便，基于 `ros_control`
  - **UR Script**: 原生脚本语言，灵活性较低

---

## 2. 实施困难分析

### 🔴 困难等级说明
- **H (High)**: 核心难点，不做则无法实现控制
- **M (Medium)**: 有成熟方案但需适配
- **L (Low)**: 工作量问题，方案明确

---

### H1. 实时控制环替代 MuJoCo 物理引擎

**难度: H | 优先级: 最高**

仿真中 MuJoCo 在一个循环中完成：
```
update() [mj_step1: 运动学] → 控制律计算 → update_dynamic() [mj_step2: 动力学]
```

**实机情况**：没有 MuJoCo 来推进状态。你需要：

1. **读取** → 从 UR 获取关节位置/速度（RTDE）
2. **计算** → 正运动学、雅可比、控制律（你自己的代码）
3. **发送** → 通过 RTDE 下发力矩/速度指令
4. **等待** → 等待下一个控制周期

**问题**：
- MuJoCo 与实机的"步进"语义完全不同。仿真中 `mj_step1` 和 `mj_step2` 同步完成；实机发指令后机械臂物理上运动，你需要在下个周期再读状态。
- 没有 `qfrc_bias`（MuJoCo 的内部计算），重力/科氏补偿需要自己算。
- 没有 `mj_fullM` 给你惯性矩阵。

---

### H2. 运动学与雅可比计算

**难度: H | 优先级: 最高**

`RobotState` 中的核心方法都需要改造：

| 方法 | 仿真 (MuJoCo) | 实机方案 |
|---|---|---|
| `get_pose()` | `site_xpos`, `site_xmat` | DH参数正解 或 URDF + KDL |
| `get_body_jacobian()` | `mj_jacSite` + Rᵀ 变换 | KDL `Jacobian()` 或 Pinocchio |
| `get_body_ee_velocity()` | Jb @ dq | Jb @ dq（同上） |
| `gauss_newton_IK()` | 内部迭代 + `mj_jacSite` | KDL `ChainFkSolverPos` / TRAC-IK |

**UR12 的 DH 参数**虽有官方文档，但实际组装公差会导致小偏差。更稳健的方式是用 **URDF + ur_description** 包通过 KDL/Pinocchio 计算运动学。

---

### H3. 动力学补偿（重力/科氏/惯性）

**难度: M | 优先级: 高**

GIC 控制律中：
```python
tau_cmd = Jbᵀ @ tau_tilde + qfrc_bias
```

`qfrc_bias` 包含重力 + 科氏力 + 离心力。MuJoCo 自动计算。实机需要：

**选项 A: UR 内部补偿**
- UR 控制器**默认已做重力补偿**。此时 `qfrc_bias` 不应额外加，否则双重补偿。
- 可通过 `ur_rtde` 的 `set_gravity()` 调节。

**选项 B: 自己计算**
- URDF → Pinocchio 的 `rnea()` / `computeGeneralizedGravity()`。
- 自由度6，计算量不大，可在控制循环内实时跑。

**⚠️ 关键**：必须清楚 UR 控制器的内部补偿策略，避免双重补偿。

---

### H4. 力/力矩传感（如果要做力控）

**难度: H | 优先级: 高（视应用）**

GUFIC 的核心就是力-阻抗控制。如果要部署 GUFIC（含力跟踪），需要实装 FT 传感器。

**UR12 本身无内置关节扭矩传感器**（UR10/UR5 系列同），必须外加：

| 方案 | 成本 | 精度 | 集成难度 |
|---|---|---|---|
| Onrobot HEX 系列 | ~$5000+ | 高 | 中（Ethernet 通信） |
| Robotiq FT 300 | ~$4000+ | 高 | 中（Ethernet / ROS） |
| 自组（应变片+采集卡） | 低 | 低 | 高 |
| 仅用 UR 电流估计 | 0 | 很低 | 低（但不可用于力控） |

**如果只部署 GIC（纯阻抗，无力跟踪）**，FT 传感器不是必须的。但需注意：
- GIC 代码中 `get_FT_value()` 被调用但仅在 GUFIC 中真正用于力反馈。
- 在 GIC-only 模式下，`Fe` 和 `Fe_raw` 项不影响控制律，可以忽略。

---

### H5. 实时性要求与控制频率

**难度: M | 优先级: 高**

| 项目 | 仿真中 | 实机要求 |
|---|---|---|
| 控制频率 | MuJoCo 模拟器时间步（通常 1-2 kHz） | UR RTDE 上限 **500 Hz** |
| 计算延迟 | 模拟器立即响应 | 通信 + 计算延迟 |
| 确定性 | 完全确定 | 受网络、系统调度影响 |

**影响**：
- 仿真中 GIC 的增益 (Kp=1500~2500) 是针对模拟环境调的，实机上直接用会**不稳定**。需要从低增益开始重新调参。
- 500 Hz 对应 2 ms 周期。Python 控制循环在非实时 Linux 上很难稳定达到 500 Hz（尤其是力矩模式）。**强烈建议：**
  - 控制逻辑用 **C++** 写在 `ur_rtde` 中
  - 或 Python 控制在 100-250 Hz 跑简单模式（位置/速度环）
  - 用实时内核 (RT-Preempt) 提升稳定性

---

### H6. 安全与保护机制

**难度: H | 优先级: 最高（安全永远是第一位）**

仿真撞了不会坏。实机撞了会：
- 损坏机械臂
- 损坏夹具/工件
- 伤人

**需要实现的安全机制**：

1. **力矩限幅** — 设置 `tau_cmd` 的绝对上限（如 UR12 的额定力矩）
2. **速度限幅** — 末端线速度上限（如 0.5 m/s 开始调试）
3. **位置边界** — 软限位（工作空间约束）
4. **奇异点检测** — 判断雅可比条件数，接近奇异时切换阻尼最小二乘
5. **急停** — UR 的 emergency stop 硬接线
6. **力矩模式保护** — UR 的 `tcp_screw` / 安全配置中降低力矩模式限值

---

### M1. UR 控制模式选择

**难度: M | 优先级: 中**

| 控制模式 | RTDE 接口 | 适合 GIC? | 说明 |
|---|---|---|---|
| **位置控制** | `speedJ` / `moveJ` | ❌ 不适合 | 位置模式无法实现阻抗行为 |
| **速度控制** | `speedJ` / `speedL` | ⚠️ 勉强 | 需要外环位置修正，响应慢 |
| **力矩前馈 + 位置环** | `setTargetTorque` + `targetJointPosition` | ✅ 可以 | UR 内部位置环 + 你的阻抗力矩前馈 |
| **纯力矩控制** | `setTargetTorque`（Mode 0x0E） | ✅ 最好 | 完全接管。但 UR 力矩模式限制大 |

**推荐**：在初期使用 **力矩前馈模式**（ur_rtde 的 `setTargetTorque` 配合位置/速度参考），稳定性好；成熟后再考虑纯力矩模式。

---

### M2. 初始位姿与 IK

**难度: M | 优先级: 中**

仿真中 `reset()` 调用高斯-牛顿 IK 初始化位姿。实机上：

- UR12 从当前位置启动，不需要每次做 IK
- 首次启动时需要**安全回零**（Home position）
- UR 的 `moveJ` 和 `moveL` 自带 IK，可以用脚本把臂移到起始点
- 如果要自己实现 IK（比如部署过程中用到），推荐用 **TRAC-IK** 或 **KDL**，不要用仿真里的高斯-牛顿迭代（收敛慢且不稳定）

---

### M3. 坐标系对齐

**难度: M | 优先级: 高**

MuJoCo 模型有自己的坐标系约定，UR12 也有自己的基坐标系和工具坐标系。

- 仿真中 `p_plate` 世界坐标写死为 `[0.50, 0.00, 0.11]`
- **需要标定 UR12 的基坐标系**相对于工作台的变换
- 需要**工具坐标系标定**（TCP）— UR 的 `TCP Configuration`
- 推荐使用 UR 自带的 TCP 标定功能（四点法）

---

### M4. 仿真到实机的参数迁移

**难度: L | 优先级: 中**

仿真中的增益 `Kp`, `KR`, `Kd` 是针对 Indy7 虚拟模型调的。UR12 动力学参数完全不同：

| 参数 | 仿真 (Indy7) | UR12 预期 | 建议 |
|---|---|---|---|
| Kp (位置刚度) | 1500-2500 | **200-800** 起步 | 从 1/10 开始，逐渐增大 |
| KR (朝向刚度) | 1500-2000 | **100-500** 起步 | 同位置刚度 |
| Kd (阻尼) | 500 | **50-200** 起步 | 先从临界阻尼调 |

---

## 3. 推荐实施路径

### Phase 1: 环境搭建与基础通信（~1 周）

```
[ ] 安装 ur_rtde (Python/C++)
[ ] 验证 RTDE 通信：读取关节角度、速度（500 Hz）
[ ] UR12 安全配置：降低力矩模式最大力矩、限速
[ ] 编写"读-算-发"最小循环骨架
```

### Phase 2: 运动学与动力学计算（~1 周）

```
[ ] 安装 Pinocchio 或 KDL
[ ] 导入 UR12 URDF（ros-industrial/ur_description）
[ ] 实现正运动学 get_pose()  —— 验证与 UR 实际 TCP 读数一致
[ ] 实现体雅可比 get_body_jacobian()
[ ] 实现重力补偿计算
[ ] 实现 IK 求解器（TRAC-IK）
```

### Phase 3: 纯位置阻抗控制（~2 周）

```
[ ] 在速度模式下实现简化 GIC（外环修正）
[ ] 极低增益启动，末端无负载
[ ] 逐步增加增益
[ ] 验证跟踪性能（阶跃响应、正弦轨迹）
[ ] 加入动力学补偿（重力）
```

### Phase 4: 完整力矩模式 GIC（~2 周）

```
[ ] UR 力矩模式配置（安全第一）
[ ] GIC 完整控制律在实机运行
[ ] 从低增益开始调参
[ ] 实现安全保护（力矩限幅、速度限幅、奇异检测）
[ ] 对比仿真与实机的跟踪性能
```

### Phase 5: 力-阻抗控制 GUFIC（~3 周，需要 FT 传感器）

```
[ ] 安装 FT 传感器 + 通信集成
[ ] 实现力传感器读取 + 滤波（复用 ButterLowPass 思路）
[ ] GUFIC 完整控制律
[ ] 力跟踪调参
[ ] 能量油箱机制验证
```

### Phase 6: 应用集成（~1-2 周）

```
[ ] 标定工件坐标系
[ ] 任务级轨迹生成
[ ] 监控与日志
[ ] 故障恢复策略
```

---

## 4. 关键建议汇总

| # | 建议 |
|---|---|
| 1 | **安全第一** — 力矩限幅、速度限幅、急停、软限位是必须的，不是可选的 |
| 2 | **从速度模式起步** — 不要一上来就纯力矩模式。先用 `speedL` 验证正运动学和轨迹跟踪 |
| 3 | **用 ROS 生态** — `ur_robot_driver` + `ros_control` + `moveit` 可以让你快速跳过运动学/IK 的重复劳动 |
| 4 | **用 Pinocchio 替代 MuJoCo 动力学** — 它是专门为机器人实时控制设计，支持 RNE、雅可比、惯性矩阵计算 |
| 5 | **增益降 10 倍起步** — 仿真的阻抗刚度在实机上会导致振荡甚至失稳 |
| 6 | **Python 不够快** — 500 Hz 力矩控制建议用 C++ (ur_rtde C++ 版本)。Python 在 250 Hz 以内尚可 |
| 7 | **优先部署 GIC (无力控)** — 如果你手头没有 FT 传感器，先把纯几何阻抗控制部署通。GIC 是 GUFIC 的子集 |
| 8 | **仿真验证流程** — 修改仿真代码以输出 UR12 的控制指令格式，先在 MuJoCo 中验证"UR12 版本"控制律的正确性，再部署到实机 |
| 9 | **日志记录** — 实机上每步都要记录关节位置/速度/力矩/时间戳，方便事后分析 |
| 10 | **不要相信仿真参数** — 阻抗增益、滤波器截止频率、前馈项全部需要实机重调 |

---

## 5. 关键技术决策树

```
是否有 FT 传感器？
├─ 否 → 部署 GIC 纯阻抗控制
│        └─ 控制模式：力矩前馈 + 位置环 或 速度环外环
│
└─ 是 → 部署 GUFIC 全功能
         ├─ 控制模式：纯力矩控制 (ur_rtde setTargetTorque)
         └─ 需要实现：力滤波、能量油箱、力控制律

控制循环用什么写？
├─ Python + ur_rtde → 250 Hz 上限，原型开发快
├─ C++ + ur_rtde   → 500 Hz，生产部署
└─ ROS + ur_robot_driver → 生态好，但延迟略高

运动学库选择？
├─ Pinocchio → 快速、现代、支持 RNE/雅可比/惯性
├─ KDL       → ROS 标配，成熟，性能够用
└─ Custom DH → 不推荐（易出错，调试困难）
```

---

## 6. 参考资源

| 资源 | 链接 |
|---|---|
| ur_rtde (推荐) | https://sdurobotics.gitlab.io/ur_rtde/ |
| Universal Robots ROS Driver | https://github.com/UniversalRobots/Universal_Robots_ROS_Driver |
| ur_description (URDF) | https://github.com/ros-industrial/ur_description |
| Pinocchio 动力学库 | https://stack-of-tasks.github.io/pinocchio/ |
| TRAC-IK 运动学求解 | https://bitbucket.org/traclabs/trac_ik |
| 论文: GUFIC (Seo et al.) | "Geometric Unified Force-Impedance Control" |

---

*文档创建日期: 2026-07-25*
*基于 GUFIC_mujoco-main 仓库的 GIC 仿真代码分析*
