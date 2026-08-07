# SE3 RoboArm Control — 项目全面介绍与使用文档

> 本文档是该项目的综合导览：简介、构建思路、架构、脚本与文档体系、全部实验、
> 启动命令与可调参数。目标是让任何读者（含几个月后的自己）通读一遍即可对该项目
> 形成系统认识，并知道每个实验怎么跑、改哪些参数。
>
> 适用范围：`se3_control/` 的全部仿真代码，以及 `docs/`、`se3_control/docs/` 的全部文档。
> 最后更新：2026-08-07

---

## 目录

1. [项目简介](#1-项目简介)
2. [构建思路](#2-构建思路)
3. [项目架构](#3-项目架构)
4. [代码结构总览](#4-代码结构总览)
5. [脚本说明与用法](#5-脚本说明与用法)
6. [文档体系与作用](#6-文档体系与作用)
7. [全部实验介绍](#7-全部实验介绍)
8. [实验启动命令与参数](#8-实验启动命令与参数)
9. [配置文件详解](#9-配置文件详解)
10. [运行环境与测试](#10-运行环境与测试)
11. [已知问题与后续计划](#11-已知问题与后续计划)

---

## 1. 项目简介

**SE3 RoboArm Control** 是一个基于 **SE(3) 几何控制** 的机械臂柔顺控制（导纳/阻抗）
研究与开发项目。它用 SE(3) 李群/李代数描述机械臂末端的位姿与速度，用 **Pinocchio**
做运动学与动力学计算，在 **MuJoCo** 中完成物理仿真验证，最终目标是部署到真实机械臂
（**UR12e**、**UR3**，未来加入 **Franka Panda**）上运行。

项目将柔顺控制拆成三个相互独立、可互换的控制器模块：

| 模块 | 全称 | 含义 | 状态 |
|---|---|---|---|
| **GIC** | Geometric Impedance Control | 几何阻抗控制（力→位移被动响应） | ✅ 已实现 + 仿真验证 |
| **GAC** | Geometric Admittance Control | 几何导纳控制（力→位移主动修正） | ✅ 已实现 + 仿真验证 |
| **GUFIC** | Geometric Unified Force-Impedance Control | 混合力-阻抗控制 | ⏳ 预留，未实现 |

三个模块是**对等的独立模块**（peer modules），互不 import，通过统一的接口互换。
首套实机方案是**纯 GIC**（阻抗）。Franka 推迟到 UR12e / UR3 全部通过仿真 + 实机
验证之后再接入。

### 1.1 核心控制思想

**SE(3) 几何控制**：不用传统的 RPY 欧拉角，而是把末端位姿视为 SE(3) 群元
`g = (R, p)`，误差与速度都用李代数 `se(3)` 表示。这样：

- 姿态误差是**全局定义的**（不是欧拉角的近似差），无奇异；
- 控制器同时处理平动 + 转动，统一 6 维；
- 直积空间（平动 + 转动）退化为经典笛卡尔控制，因此本框架是通用笛卡尔控制的推广。

**自适应操作空间惯性**（GIC / GAC 内环共用）：经典阻抗控制常取固定对角刚度
`K_p/K_R/K_d`，但机械臂在不同位形的**操作空间惯性** `M̃(q) = (Jb·M⁻¹·Jbᵀ)⁻¹`
变化巨大，固定增益会造成某些位形过刚、某些位形过柔。本项目用**自适应增益**：

```
K_adapt = ω² · M̃(q)            # 位置/姿态刚度
D_adapt = 2ζω · M̃(q)          # 阻尼
M̃(q)   = (Jb·M⁻¹·Jbᵀ)⁻¹       # 操作空间惯性（SVD 阻尼伪逆，damp_sv=0.1·s_t[-1]）
```

其中 `ω` 是闭环带宽（rad/s），`ζ` 是阻尼比。这样在任意位形下，闭环动力学
都被整形为**相同的目标特性**（带宽 `ω`、阻尼 `ζ`），与位形无关。`M̃` 通过
`Jb·M⁻¹·Jbᵀ` 的 SVD 截断奇异值求阻尼伪逆得到，避免近奇异位形下增益爆炸。

### 1.2 控制律（GIC）

```
误差（李代数）:  e_op = [e_p; e_R]，e_p = p − p_d，e_R = vee(R_dᵀR − RᵀR_d)/2
速度误差:        e_v = Vb − Vd*  （体坐标系 twist）
期望加速度:      Vd* = Ad(g_ed⁻¹)·[vd; wd]，含前馈
力矩指令:        τ = Jbᵀ ( M̃·dVd* − D_adapt·e_v − K_adapt·e_op ) + bias
```

- `Jb` 为**体坐标系**雅可比（`Jb = diag(Rᵀ,Rᵀ)·Js`，由 Pinocchio 空间雅可比 `Js` 变换）。
- `bias` 为重力/科氏补偿（Pinocchio `rnea(q, 0, 0)`）。
- 力矩受 `torque_limits` 限幅。

### 1.3 控制律（GAC 双层结构）

GAC 在 GIC 内环跟踪之上，叠加一层**体坐标系导纳滤波器**：

```
滤波器层（核心）:  M_d·dV_corr + D_d·V_corr + K_d·X_corr = F_ext_body
```

- `M_d, D_d, K_d ∈ ℝ⁶ˣ⁶`：用户指定的虚拟质量/阻尼/刚度（导纳特性，可在线改）；
- 外用力 `F_ext`（体坐标系）经滤波器 → 位置修正量 `X_corr`，**叠加到参考轨迹**上；
- 修正后的轨迹交给内环（与 GIC 同公式的 SE(3) 位置跟踪）执行。

```
  F_ext ──► [GACFilter] ──► 修正量 ΔX/ΔR ──► 参考轨迹修正 ──► [GIC 式内环跟踪] ──► τ
```

**为什么不用 GIC 作内环**：GIC 是阻抗（被动让位），内环需要的是**位置跟踪**
（主动跟踪修正后的参考），语义相反；复用会引入延迟与刚度叠加，且违背“对等模块”
原则。GIC 与 GAC 共享 `bandwidth`、`damping`、`torque_limits` 三个参数，GAC 额外
需要 `M_d/D_d/K_d/dt`。

---

## 2. 构建思路

### 2.1 Write Once, Run on Any Arm

项目的最高设计原则：**控制核心与硬件解耦**，一份代码在 MuJoCo 仿真、UR12e、UR3、
Franka 上无需改动即可运行。

```
  MuJoCo 仿真（data.ctrl / data.qpos / mj_contactForce）
  UR12e / UR3（ur_rtde: directTorque + RTDE 状态）
  Franka   （libfranka，预留）
        │  通过同一个 RobotHWInterface 抽象接口
        ▼
  ┌─────────────────────────────────────┐
  │  相同的 SE(3) 控制律（GIC/GAC）      │
  └─────────────────────────────────────┘
```

具体落地是 **`core/` 模块抽离**：早期仿真脚本（`verify_gic_mujoco.py`）把控制器、
轨迹、数学公式内联在脚本里，并依赖外部 `GUFIC_mujoco-main` 仓库。项目把这部分
**迁移到 `core/`**，使项目完全脱离外部仓库、可独立运行（本次迁移已完成并验证）。

### 2.2 三层递进验证（仿真 → 实机）

| 阶段 | 内容 | 状态 |
|---|---|---|
| Phase 0 | 接触模型标定（MuJoCo 接触数值特性） | ✅ 已完成 |
| Phase 1 | GIC 被动接触全流程（逼近/接触/摩擦/离开） | ✅ 已完成 |
| Phase 2 | GAC 力模式实验 + 方向解耦 + 扫频 + 负载突变 | 🚧 部分完成 |
| Phase 3 | 实机部署（UR12e 首个） | ⏳ 规划中 |
| Phase 4 | UR3 / Franka 扩展 | ⏳ 规划中 |

### 2.3 自适应增益 vs 固定增益

早期用固定对角增益 `K_p/K_R/K_d`（现 `task_config.gains` 中仍保留，但已是**死代码**）。
推导后改为**自适应操作空间惯性增益**（见 §1.1）：刚度随位形变化的操作空间惯性
`M̃(q)` 缩放，闭环响应处处一致。仿真验证中固定增益已从验证脚本移除，配置保留
仅作文档参考。

### 2.4 关键设计决策

- **GIC/GAC 对等独立**：互不 import，接口互换只需换构造函数（见 GAC_plan.md §3）。
- **薄层硬件接口**：`RobotHWInterface` 具体实现控制在 100–200 行，上层禁止引用驱动库。
- **无源性验证优先**：实验三（被动接触）特意**不读力反馈**（`Fe_raw=None`），
  靠阻抗律自然让位，验证“不加力反馈也不发散”。
- **接触稳定性先标定**：实验三之前先做 Phase 0 接触标定，避免失稳被接触求解器
  行为污染（`solref=[1.0,1.0]` → 动态接触刚度约 36 kN/m）。

---

## 3. 项目架构

### 3.1 分层架构图

```
┌──────────────────────────────────────────────────────────────────────┐
│  应用层  scripts/                                                      │
│  ┌──────────────────────────────────────────────────────────────────┐ │
│  │ run_se3_control.py        仿真主入口（GIC 常规任务）              │ │
│  │ verify_gic_mujoco.py      GIC 验证（任务 + 方向解耦实验）          │ │
│  │ verify_gac_mujoco.py      GAC 验证（5 种力模式 + 方向解耦）        │ │
│  │ verify_gic_contact.py     实验三：GIC 被动接触全流程               │ │
│  │ verify_contact_calibration.py  Phase 0：接触模型标定              │ │
│  └──────────────────────────────────────────────────────────────────┘ │
│                              │ 调用                                     │
├──────────────────────────────┼─────────────────────────────────────────┤
│  核心层  core/                                                          │
│  ┌──────────────┬──────────────┬────────────────┬───────────────────┐  │
│  │ se3_math.py  │ trajectory.py│ gic_controller │ gac_controller.py│  │
│  │ hat/vee/adjoint│ 轨迹生成     │ GIC 控制律      │ GACFilter+控制律  │  │
│  │ slerp/rpy    │ (SymPy→NumPy)│ (自适应惯量)     │ (双层结构)         │  │
│  └──────────────┴──────────────┴────────────────┴───────────────────┘  │
│  ┌──────────────────────────────────────────────────────────────────┐  │
│  │ experiment_analysis.py  实验分析（解耦矩阵提取/绘图/回归断言）    │  │
│  └──────────────────────────────────────────────────────────────────┘  │
│                              │ 调用                                     │
├──────────────────────────────┼─────────────────────────────────────────┤
│  机器人模型层  robot_model/                                              │
│  ┌──────────────────────────────────────────────────────────────────┐ │
│  │ robot_model.py   Pinocchio 封装：FK / IK / 雅可比 / crba / rnea  │ │
│  └──────────────────────────────────────────────────────────────────┘ │
│                              │ 调用                                     │
├──────────────────────────────┼─────────────────────────────────────────┤
│  硬件抽象层  hardware/（实机，预留）                                     │
│  ┌─────────────┬─────────────┬─────────────┬─────────────┐           │
│  │ interface.py│ ur_hw.py    │ ur12e_hw.py │ ur3_hw.py   │           │
│  │ RobotHWInterface│ URHW(ur_rtde)│ UR12e 子类 │ UR3 子类    │           │
│  └─────────────┴─────────────┴─────────────┴─────────────┘           │
├──────────────────────────────┼─────────────────────────────────────────┤
│  配置层  config/  ── task_config.py（轨迹/控制器/实验参数）              │
│                      robot_configs.py（机器人 ip/力矩限幅/home_q）      │
└──────────────────────────────┼─────────────────────────────────────────┘
                               ▼
              ┌──────────────────────────────┐
              │  tests/   48 个 pytest 测试   │
              └──────────────────────────────┘
```

### 3.2 依赖方向（不可反向）

```
scripts → core → robot_model → (Pinocchio)
scripts → config → robot_configs / task_config
hardware → interface（ABC）
```

- `core/` 不依赖任何脚本、不依赖 MuJoCo、不依赖硬件驱动——纯 NumPy + Pinocchio。
- 仿真脚本依赖 MuJoCo；实机脚本依赖 ur_rtde；`RobotHWInterface` 屏蔽差异。
- 数据流：脚本（XML 构建 / MuJoCo 步进）→ 控制器（`core/`）→ 机器人模型
  （Pinocchio）→ 力矩 → 脚本写回 `data.ctrl`。

---

## 4. 代码结构总览

```
SE3_roboarm_control/
├── readme.md                          # 仓库级占位 README（内容已过时，以本文档为准）
├── docs/                              # 根级文档（部署计划 + 本文档）
│   ├── project_overview.md            # ← 本文档
│   ├── deploy_se3_to_hardware_plan.md # 部署总计划（Write Once Run Any Arm）
│   └── deploy_se3_gic_to_ur12_plan.md # GIC 专属实机部署计划
│
├── se3_control/                       # 主代码包
│   ├── config/
│   │   ├── task_config.py             # 轨迹/控制器/实验参数（可改）
│   │   └── robot_configs.py           # 各机器人配置（ip/力矩限幅/home_q）
│   ├── core/
│   │   ├── se3_math.py                # SE(3) 数学工具
│   │   ├── trajectory.py              # 轨迹生成
│   │   ├── gic_controller.py          # GIC 控制律
│   │   ├── gac_controller.py          # GAC 控制律（含 GACFilter）
│   │   └── experiment_analysis.py     # 实验分析与回归断言
│   ├── robot_model/
│   │   └── robot_model.py             # Pinocchio 封装
│   ├── hardware/
│   │   ├── interface.py               # RobotHWInterface ABC（实机）
│   │   ├── ur_hw.py                   # URHW（ur_rtde 通用 UR）
│   │   ├── ur12e_hw.py                # UR12e 子类
│   │   └── ur3_hw.py                  # UR3 子类
│   ├── scripts/
│   │   ├── run_se3_control.py         # 仿真主入口
│   │   ├── verify_gic_mujoco.py       # GIC 验证
│   │   ├── verify_gac_mujoco.py       # GAC 验证
│   │   ├── verify_gic_contact.py      # 实验三
│   │   ├── verify_contact_calibration.py  # Phase 0
│   │   └── usages.md                  # 脚本使用速查（最全参数表）
│   ├── docs/
│   │   ├── plan/                      # 设计计划
│   │   ├── usages/                    # 使用/报告
│   │   └── verification/              # 验证记录
│   ├── urdf/                          # 机器人 URDF 模型
│   └── figures/                       # 实验输出图
│
└── tests/                             # 48 个 pytest 测试
```

---

## 5. 脚本说明与用法

所有脚本统一约定：

- 需要 `conda activate roboarm`（依赖全在 conda 环境 `roboarm` 中）；
- 需要在 `se3_control/` 目录下运行（脚本内相对路径以它为准）；
- 都支持 `--robot`（ur12e / ur3）、`--no-viewer`（无头模式，适合 SSH/服务器）。

| 脚本 | 作用 | 关键参数 |
|---|---|---|
| `run_se3_control.py` | 仿真主入口：URDF→XML、MuJoCo 步进、可视化、记录。跑 regulation/circle/line 常规任务 | `--task --max-time --save-plot --cross-validate` |
| `verify_gic_mujoco.py` | GIC 完整验证：常规任务 + `--experiment decouple` 方向解耦 | `--experiment --decouple-*` |
| `verify_gac_mujoco.py` | GAC 验证：5 种外力模式（zero/constant/pulse/spring/tangent）+ 方向解耦 | `--force-mode --force-amplitude --M-d --D-d --K-d` |
| `verify_gic_contact.py` | 实验三：GIC 被动接触（逼近/接触/摩擦/离开） | `--ball-* --delta-pen --theta-amp --phi-amp --bandwidth --damping` |
| `verify_contact_calibration.py` | Phase 0：接触模型标定（静态刚度 + 动态冲击） | `--solref-times --approach-speeds --press-pen` |

完整参数表见 §8。`scripts/usages.md` 还保留了逐脚本的详细参数说明，两者可配合使用。

---

## 6. 文档体系与作用

文档按“**根级（部署/总览） → 包级 plan（设计） → 包级 usages（使用/报告） → 包级
verification（验证）**”分层。

### 6.1 根级 docs/

| 文档 | 作用 |
|---|---|
| `docs/project_overview.md` | **本文档**：项目全面介绍 + 使用手册 |
| `docs/deploy_se3_to_hardware_plan.md` | 实机部署总计划：Write Once Run on Any Arm 的原则与分阶段路线（Phase 0–5） |
| `docs/deploy_se3_gic_to_ur12_plan.md` | 纯 GIC 部署到 UR12e 的细化计划 |

### 6.2 se3_control/docs/plan/（设计计划）

| 文档 | 作用 |
|---|---|
| `force_interaction_experiments_plan.md` | **力交互实验总计划**：正弦扫频 · 方向解耦 · 刚性接触 · 负载突变（四个实验的动机/方法/验收标准/参数）；附录 A.9/A.10 为 Phase 0 接触标定、A.11 为 Phase 1 GIC 被动接触标定 |
| `GIC_plan.md` | 核心抽离计划 + 自适应增益推导 + GIC 控制律公式 |
| `GAC_plan.md` | GAC 双层结构设计、GIC/GAC 对等原则、导纳滤波器数学、接口互换 |
| `interface_plan.md` | RobotHWInterface 设计原则（薄层、机器人无关、上下文管理器、类型安全、容错） |
| `verify_gac_mujoco_plan.md` | verify_gac_mujoco.py 的验证计划 |

### 6.3 se3_control/docs/usages/（使用说明与实验报告）

| 文档 | 作用 |
|---|---|
| `exp2_direction_decoupling_report.md` | **实验二（方向解耦）归档报告**：默认位形调整（末端竖直朝下）、IK 修复、GAC/GIC 实测耦合矩阵、回归阈值 10% |
| `exp3_rigid_contact_report.md` | **实验三（GIC 被动接触）报告**：solref→接触刚度、ω=90/ζ=4 的 2D 摩擦参数 |
| `interface_URtest_usages.md` | 硬件接口在 UR 实机上的测试用法 |
| `robot_model_usages.md` | Pinocchio RobotModel 封装的使用说明 |
| `run_se3_control_usage.md` | run_se3_control.py 的使用说明 |

### 6.4 se3_control/docs/verification/ + scripts/usages.md

| 文档 | 作用 |
|---|---|
| `verification/interface_verification.md` | 硬件接口的验证记录 |
| `scripts/usages.md` | **脚本使用速查**（530 行，最全的逐参数说明，按脚本分节） |

> 阅读建议：新读者先读本文档 → 再按需读对应实验的 plan 与报告；写代码/调参时
> 对照 `scripts/usages.md` 与配置文件。

---

## 7. 全部实验介绍

力交互实验按计划分 **4 个核心实验** + **2 个预备/附属实验**。当前实现状态如下表：

| 编号 | 实验 | 主测 | 状态 |
|---|---|---|---|
| Phase 0 | 接触模型标定（静态刚度 + 动态冲击） | 环境 | ✅ 已实现 |
| ① | 正弦扫频（频率响应 / 带宽） | GAC 主测 / GIC 同测 | ⏳ 仅设计（未实现脚本） |
| ② | 方向解耦（耦合矩阵） | GAC 主测 / GIC 基线 | ✅ 已实现 |
| ③ | 刚性接触（GIC 被动接触全流程） | GIC 主测 | ✅ 已实现 |
| ④ | 负载突变（鲁棒性） | GAC / GIC 同测 | ⏳ 仅设计（未实现脚本） |
| — | GAC 力模式实验（5 种外力） | GAC | ✅ 已实现 |

### 7.1 Phase 0 — 接触模型标定（`verify_contact_calibration.py`）

**目的**：实验三之前先标定 MuJoCo 接触模型本身的数值特性，避免接触失稳被求解器
行为污染。**不接入任何控制器**，纯运动学压入。

- **Part A — 静态压深扫描**：直接设 qpos（IK 求压入位形）+ `mj_forward`，逐压深
  读接触力，拟合线性刚度 `K_env = dF/d(pen)`，报告线性区间与 R²；按 solref 时间
  常数扫描，看 `K_env` 随环境刚度标尺的变化。
- **Part B — 动态冲击（回跳特性）**：用 MuJoCo 内置 position actuator 以恒定
  `approach_speed` 逼近并压入刚体球，测峰值力/稳态力/超调/make-break 断开次数/
  回弹速度/恢复系数 `e`/调节时间；扫 approach_speed 看回跳是否随冲击能量变化。

**关键结论**：

| 机器人 | K_env（静态） | 冲击 F_peak | 恢复系数 e |
|---|---|---|---|
| UR12e | 17.84 kN/m | 104 → 167 N | 0.09–0.10 |
| UR3 | 8.06 kN/m | — | — |

- 腕部补 `dof_armature` 电机转子惯量（修正 URDF 缺失腕部电机惯量导致的近零惯量失稳）。

### 7.2 实验一 — 正弦扫频（仅设计，未实现）

**目的**：扫频是阻抗/导纳控制的标准验收手段——GAC 验证导纳滤波器
`1/(M_d s²+D_d s+K_d)` 的**设计带宽/谐振**是否与理论一致；GIC 辨识**实现出来的
阻抗**（自适应惯量成形后的力→位移特性）。

- 扫频范围：GAC 默认 `M=10, D=100, K=500` → `ω_n=√(K/M)=7.07 rad/s≈1.1 Hz`，
  `ζ=0.707`；取 **0.1–8 Hz** 离散频点覆盖谐振与 −3dB 带宽。
- 验收：逐频点 Bode 实测 vs 理论偏差 < 10%。
- 状态：方案与参数在 `force_interaction_experiments_plan.md` §3 中，脚本未实现
  （对应计划中的 `--experiment sweep`）。

### 7.3 实验二 — 方向解耦（`--experiment decouple`，已实现）

**目的**：在 GAC / GIC 两种控制场景下，依次施加三个轴向恒力（Fx/Fy/Fz）与三个
恒力偶（Mx/My/Mz），检验**每类输入只产生对应轴的位移**，不产生额外位移。核心回归
对象是历史 “Z 振荡” bug（施加 x 向力出现 z 向漂移）。

**方法**：仿真按 **7 个时间块**顺序进行：

```
块 0: 基线（零输入）
块 1–3: +x / +y / +z 恒力（世界系）
块 4–6: 绕 x / y / z 恒力偶（世界系）
每块: settle(2s) 过渡 + measure(1s) 稳态测量 → 6×6 静态耦合矩阵
```

**结果（exp2 报告）**：
- GAC 施加 x 向力时 EE 级 `|Δz|/|Δx| ≈ 7.7%`，处于可接受 <10% 区间
  （该耦合全部来自跟踪层位形相关耦合，滤波器层仍严格解耦，断言 <1e-3）；
- 回归断言按可接受线固化为 **< 10%**。

**相关改动**（随实验二一起完成，见 exp2 报告 §1）：
- 默认位形改为**低位 + 末端竖直朝下**（`home_q` 控制，EE 工具 z 轴 `[0,0,-1]`）；
  竖直朝下通过 `q5=±90°` 实现，而腕部奇异在 `q5=0/±180°`，故该位形非奇异
  （UR12e 下 min_sv=0.295）。
- 修复 `gauss_newton_IK` 姿态误差度量：原 `-0.5·Σ cross(R_i,Rd_i)` 在 180° 朝向差
  时三个叉积抵消为 0，误报收敛；换为旋转向量对数映射。

### 7.4 实验三 — 刚性接触 / GIC 被动接触（`verify_gic_contact.py`，已实现）

**目的**：GIC **被动响应**（`Fe_raw=None`，控制器不读接触力）下的接触全流程验证，
这是**无源性验证**：不加力反馈也不发散、稳定、可摩擦、可抬离。

**四阶段轨迹**（工具尖中心，球心在工具正下方 → 逼近沿球面法向）：

```
1. 逼近:  半径 r 从 r_start 匀速(平滑加速)收缩到 r_des = R_eff − δ_pen
          （R_eff = 球半径+尖半径; 接触时 v ≈ approach_speed）
2. 保持:   r = r_des 静止，接触力落入 ±10% 稳态带
3. 表面摩擦: 极角 θ 在 ±θ_amp 间光滑往复（+ φ 维 → 2D 球面 Lissajous 斑块），
          恒压深来回摩擦
4. 离开:   r 回到 r_start（沿法向抬离），接触干净断开
5. 保持:   离开后静止，验证无振铃、无再次误碰
```

**验收目标**：逼近段无提前抖动、轨迹误差小；接触建立 F_peak 超调 <30%、
make-break=1、调节时间 <1s；摩擦段接触力波动 <10% F_ss、径向不脱离、无极限环；
离开后力立即归零、无振铃。

**关键参数与结论**：

| 项 | 值 | 结论 |
|---|---|---|
| `solref` | `[1.0, 1.0]` | 动态接触刚度约 **36 kN/m**，稳定摩擦 |
| `μ`（摩擦系数） | `μ = sqrt(μ球 × μ尖)`，取 0.15 | 复合摩擦 |
| 摩擦斑 | 2D Lissajous，`θ_amp 0.08 × φ_amp 0.8` | 覆盖面积 ≈ **0.87 cm²** |
| 控制带宽/阻尼 | `ω=90, ζ=4`（--bandwidth 90 --damping 4） | 2D 摩擦稳定所需 |
| F_peak 超调 / 调节 | 26.4% / 0.82 s | 全指标通过 |
| 最大力矩 | 41.7 Nm | UR12e 限幅内 |

**已知限制**：`θ_amp` 有上限（过大接触几何改变大，摩擦不稳定）；`mj_forward`
静态接触力 ≠ `mj_step` 动态接触力（近零压深 ~6 MN/m）；重力预载 63 N。

### 7.5 实验四 — 负载突变（仅设计，未实现）

**目的**：GAC 验证固定 `M_d` 滤波器的鲁棒性；GIC 验证**自适应惯量成形**
（`K_adapt=ω²M̃`）对动力学突变的鲁棒性。
- 方案与验收在 `force_interaction_experiments_plan.md` §6（对应 `--experiment loadstep`），
  脚本未实现。

### 7.6 GAC 力模式实验（`verify_gac_mujoco.py --force-mode`，已实现）

**目的**：在 GAC 控制下，施加不同类型的合成外力（体坐标系），观察末端的柔顺响应。
用于验证导纳滤波器在 5 种典型受力下的行为：

| 模式 | 含义 | 关键参数 |
|---|---|---|
| `zero` | 无外力（基线） | — |
| `constant` | 恒力/恒力偶（6 维向量） | `--force-amplitude [fx fy fz tx ty tz]` |
| `pulse` | 时间窗内的脉冲力 | `--force-start --force-duration` |
| `spring` | 虚拟弹簧接触（末端接近表面时产生法向力） | `--force-amplitude` |
| `tangent` | 沿圆周切向力 + 径向虚拟弹簧约束 | `--tangent-*` |

> 注意：GAC 感知力目前是 `R_curᵀ F_world`——**零延迟、完美测量**的理想化。
> 对扫频/解耦可接受，但对接触稳定性是系统性乐观（实机 FT 有延迟，失稳边界随
> 延迟急剧收缩），实机前需建模延迟。

---

## 8. 实验启动命令与参数

> 统一前置：`conda activate roboarm && cd se3_control`

### 8.1 常规任务（GIC 主入口 `run_se3_control.py`）

```bash
python scripts/run_se3_control.py                       # 默认: UR12e + regulation + 可视化
python scripts/run_se3_control.py --robot ur3 --task circle
python scripts/run_se3_control.py --task line --no-viewer          # 无头
python scripts/run_se3_control.py --task circle --save-plot circle.png
python scripts/run_se3_control.py --cross-validate                 # 仅模型交叉验证
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `--robot` | `ur12e` | `ur12e` / `ur3` |
| `--task` | `regulation` | `regulation` / `circle` / `line` |
| `--max-time` | `5.0` | 仿真时长 (s) |
| `--no-viewer` | `False` | 无头模式（SSH/服务器） |
| `--save-plot` | `None` | 保存跟踪图到指定文件 |
| `--cross-validate` | `False` | 仅做 URDF/Pinocchio/MuJoCo 模型交叉验证 |
| `--no-stop` | `False` | 结束后不暂停 |
| `--no-loop` | `False` | 关闭 viewer 循环 |
| `--bandwidth` | `30.0` | 闭环带宽 ω (rad/s) |
| `--damping` | `1.0` | 阻尼比 ζ |

### 8.2 GIC 方向解耦实验（`verify_gic_mujoco.py`）

```bash
python scripts/verify_gic_mujoco.py --experiment decouple --no-viewer
python scripts/verify_gic_mujoco.py --experiment decouple --decouple-force 30 --decouple-moment 2
python scripts/verify_gic_mujoco.py --experiment decouple --decouple-loop --no-viewer   # 可视化循环
```

参数同 §8.1 的公共参数，外加：

| 参数 | 默认 | 说明 |
|---|---|---|
| `--experiment` | `none` | `none` / `decouple`（方向解耦，7 块） |
| `--decouple-force` | `10.0` | 轴向力幅值 (N) |
| `--decouple-moment` | `1.0` | 力偶幅值 (Nm) |
| `--decouple-settle` | `2.0` | 每块过渡时间 (s) |
| `--decouple-measure` | `1.0` | 每块稳态测量时间 (s) |
| `--decouple-loop` | `False` | 可视化循环模式（幅值/时长取 `experiments.decouple_loop`） |

### 8.3 GAC 实验（`verify_gac_mujoco.py`）

```bash
# 恒力: 沿 x 施加 10 N（默认）
python scripts/verify_gac_mujoco.py --force-mode constant --no-viewer
# 自定义 6 维恒力
python scripts/verify_gac_mujoco.py --force-mode constant --force-amplitude 5 0 0 0 0 0 --no-viewer
# 脉冲
python scripts/verify_gac_mujoco.py --force-mode pulse --force-start 0.5 --force-duration 0.2 --no-viewer
# 切向力绕圆（含径向虚拟弹簧）
python scripts/verify_gac_mujoco.py --force-mode tangent --tangent-amplitude 10 --no-viewer
# 方向解耦（GAC 场景）
python scripts/verify_gac_mujoco.py --experiment decouple --no-viewer
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `--force-mode` | `zero` | `zero`/`constant`/`pulse`/`spring`/`tangent` |
| `--force-amplitude` | `[10,0,0,0,0,0]` | 6 维幅值 `[fx fy fz tx ty tz]` |
| `--force-start` | `1.0` | pulse 起始时间 (s) |
| `--force-duration` | `0.5` | pulse 持续时间 (s) |
| `--tangent-circle-center` | `[0.5,0,0.125]` | 切向力圆周中心 |
| `--tangent-radius` | `0.2` | 圆周半径 (m) |
| `--tangent-amplitude` | `10.0` | 切向力幅值 (N) |
| `--tangent-radial-stiffness` | `500.0` | 径向虚拟弹簧刚度 (N/m)；0=无径向约束 |
| `--init-pos` | `None` | 初始位置 `[x y z]` |
| `--M-d` / `--D-d` / `--K-d` | 见 §9 | 导纳虚拟质量/阻尼/刚度（6 维） |
| `--bandwidth` / `--damping` | `30.0` / `1.0` | 内环跟踪参数（同 GIC） |
| `--experiment` / `--decouple-*` | 同 §8.2 | 方向解耦实验 |

### 8.4 实验三：GIC 被动接触（`verify_gic_contact.py`）

```bash
# 默认全流程（UR12e）
python scripts/verify_gic_contact.py
# UR3
python scripts/verify_gic_contact.py --robot ur3
# 更快逼近 + 更浅压入 + 3 个来回
python scripts/verify_gic_contact.py --approach-speed 0.08 --delta-pen 0.004 --rub-cycles 3 --no-viewer
# 扩大表面摩擦面积: 增大 θ/φ 幅值 (2D 球面 Lissajous)
python scripts/verify_gic_contact.py --theta-amp 0.10 --phi-amp 1.5708 --no-viewer
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `--ball-radius` | `0.12` | 刚体球半径 (m) |
| `--ball-pos` | `None` | 球心位置 `[x y z]` |
| `--tool-length` | `0.10` | 工具尖长度 (m) |
| `--tool-radius` | `0.01` | 工具尖半径 (m) |
| `--tool-mass` | `0.05` | 工具尖质量 (kg) |
| `--wrist-armature` | `0.1` | 腕部电机转子惯量补偿 (dof_armature) |
| `--ball-friction` / `--tool-friction` | `0.15` | 接触摩擦系数（复合 `μ=sqrt(μ₁μ₂)`） |
| `--ball-solref` | `[1.0,1.0]` | MuJoCo 接触求解器时间常数 |
| `--delta-pen` | `0.008` | 最大压深 (m) |
| `--approach-speed` | `0.006` | 逼近速度 (m/s) |
| `--settle-time` | `1.2` | 接触建立保持时间 (s) |
| `--theta-amp` | `0.08` | 极角 θ 摆动幅值 (rad) |
| `--phi-amp` | `0.8` | 方位角 φ 摆动幅值 (rad) |
| `--rub-cycles` | `2` | 摩擦来回次数 |
| `--phi-cycles` | `3` | φ 维振荡周期数 |
| `--rub-mode` | `lissajous` | 摩擦模式 |
| `--rub-duration` | `16.0` | 摩擦段时长 (s) |
| `--depart-speed` | `0.05` | 抬离速度 (m/s) |
| `--bandwidth` | `90.0` | GIC 带宽 ω |
| `--damping` | `4.0` | GIC 阻尼比 ζ |
| `--save-dir` | `figures/contact/` | 输出图目录 |

产物：控制台指标报告 + `figures/contact/` 下的 `gic_contact.png`（四阶段时间序列）、
`gic_contact_rub.png`（摩擦斑俯视）、`gic_contact_surface.png`（球面接触轨迹 3D）。

### 8.5 Phase 0：接触标定（`verify_contact_calibration.py`）

```bash
python scripts/verify_contact_calibration.py                     # UR12e
python scripts/verify_contact_calibration.py --robot ur3
python scripts/verify_contact_calibration.py --approach-speeds 0.05 0.1 0.2
python scripts/verify_contact_calibration.py --solref-times 0.02 0.005 0.002
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `--robot` | `ur12e` | `ur12e` / `ur3` |
| `--ball-radius` / `--ball-pos` | `0.12` / None | 刚体球几何 |
| `--tool-length` / `--tool-radius` / `--tool-mass` | `0.10`/`0.01`/`0.05` | 工具尖几何/质量 |
| `--gap` | `0.01` | 初始间隙 (m) |
| `--max-pen` | `0.010` | 最大压深 (m) |
| `--solref-times` | `[0.02,0.01,0.005,0.002]` | solref 时间常数扫描列表 |
| `--press-pen` | `0.005` | 压入压深 (m) |
| `--wrist-armature` | `0.1` | 腕部转子惯量补偿 |
| `--act-kp` / `--act-kv` | `None` | position actuator 刚度/阻尼（6 维，None=自动） |
| `--approach-speeds` | `None` | 冲击速度扫描列表 |
| `--save-dir` | `figures/contact/` | 输出目录 |

产物：控制台标定报告 + `calibration_partA.png`（F vs 压深）、`calibration_partB.png`
（冲击时间序列）。

---

## 9. 配置文件详解

### 9.1 `config/task_config.py`

所有长度单位 m，角度单位 rad。可改默认值实现“改配置即改实验”。

| 节 | 参数 | 默认（UR12e） | 说明 |
|---|---|---|---|
| `circle` | `center` | `[0.35, 0, 0.3]` | 圆心（世界系） |
| | `radius` | `0.08` | 半径 |
| | `speed` | `0.8` | 角速度 (rad/s) |
| | `orientation` | `[0,1,0;1,0,0;0,0,-1]` | 期望姿态（3×3 按行展开） |
| `line` | `center` | `[0.50, 0, 0.05]` | 线段中点 |
| | `amplitude` | `0.125` | 振荡幅度（总长=2×） |
| | `direction` | `[0,1,0]` | 振荡方向（自动归一化） |
| | `frequency` | `0.5` | 角频率 (rad/s) |
| | `orientation` | 同上 | 期望姿态 |
| `regulation` | `target` | `[0.50, 0, 0.50]` | 期望位置（文档性参考，实际以 FK(home_q) 为期望） |
| | `orientation` | 同上 | 期望姿态 |
| `gains` | `regulation` / `tracking` | — | **死代码**（固定增益，已被自适应增益取代，仅作参考） |
| `controller` | `bandwidth` | `30.0` | 闭环带宽 ω (rad/s)，ω≈5Hz 量级 |
| | `damping` | `1.0` | 阻尼比 ζ（1.0=临界阻尼） |
| `trail` | `interval` / `max_points` / `sphere_size` / `color` | `8`/`1200`/`0.006`/红 | viewer 轨迹点样式 |
| `simulation` | `dt` | `0.001` | 仿真步长 (s) |
| | `max_time` | `5.0` | 默认时长 (s) |
| `experiments.decouple` | `force` / `moment` / `settle` / `measure` | `10`/`1`/`2`/`1` | 解耦实验单轮参数 |
| `experiments.decouple_loop` | `force`/`moment`/`settle`/`measure`/`cycles` | `30`/`2`/`2`/`3`/`2` | 可视化循环模式参数 |

### 9.2 `config/robot_configs.py`

| 项 | UR12e | UR3 |
|---|---|---|
| 默认 IP | `192.168.1.100` | `192.168.1.101` |
| 安全力矩限幅 (N·m) | `[165,165,75,27,27,27]` | `[28,28,14,6,6,6]` |
| 满额力矩限幅 | `[330,330,150,54,54,54]` | `[56,56,28,12,12,12]` |
| `home_q` | `[-0.356,-1.498,1.81,1.259,1.571,-0.124]` | `[-0.327,-1.42,1.236,-1.386,-1.571,2.738]` |
| home 处 EE 位姿 | `[0.50, 0, 0.50]`，工具 z 轴 `[0,0,-1]` | `[0.35, 0, 0.35]`，工具 z 轴 `[0,0,-1]` |
| URDF | 项目内 urdf/ | 项目内 urdf/ |

Franka（预留）：硬编码在脚本中，9 关节，`home_q=[0,-0.3,0,-2.5,0,2.5,0,0.02,0.02]`，
EE link `panda_hand_tcp`。仅在 UR12e/UR3 全量通过后接入。

### 9.3 GAC 导纳参数（`--M-d/--D-d/--K-d` 默认）

```
--M-d  默认 [10,10,10,1,1,1]   （平动质量 m×3 + 转动惯量 I×3）
--D-d  默认 None                （None = 按 K_d/M_d 自动临界阻尼；也可显式给 [dx,dy,dz,drot...]）
--K-d  默认 [500,500,500,50,50,50]  （平动刚度 + 转动刚度）
```

- 对角形式 `M_d ≈ diag(m,m,m,Ixx,Iyy,Izz)`，`D_d = 2·sqrt(K_d·M_d)`（对角元）即临界阻尼。
- 导纳稳态位移 ≈ `F/K_d`（平移 10 N / 500 = 2 cm），稳态转动 ≈ `τ/K_rot`。

---

## 10. 运行环境与测试

### 10.1 环境

- **conda 环境**：`roboarm`（Python 3.10），所有依赖都在其中。
- 核心依赖：`pinocchio`、`mujoco`、`numpy`、`sympy`（轨迹符号微分）、`scipy`、
  `matplotlib`（绘图）；实机另需 `ur-rtde`。

### 10.2 测试

```bash
conda activate roboarm
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest tests/ -q
```

- **48 个测试全部通过**（需设 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`，否则第三方插件干扰）。
- 覆盖：`test_experiment_analysis`、`test_gac_controller`、`test_gac_decouple_regression`
  （方向解耦回归，阈值 <10%）、`test_gravity_comp`、`test_joint_states`、
  `test_regulation`、`test_ur_hw_mock`、`test_ur12e_hw_mock`。
- 实机相关测试用 mock 驱动（`ur_rtde` 不可用时不报错）。

---

## 11. 已知问题与后续计划

### 11.1 已知问题

| 问题 | 位置 | 说明 |
|---|---|---|
| `adjoint_g_ed_deriv` 的 dVd* 前馈 bug | `core/se3_math.py` | 调用方把期望速度传入当前速度槽位；正确公式需当前 `Vb`。当前仿真不触发明显问题，实机部署前应修复 |
| 固定增益 `task_config.gains` 为死代码 | `config/task_config.py` | 仅作文档参考，验证脚本已不再使用 |
| 根 `readme.md` 内容过时 | `readme.md` | 建议以本文档为准或更新根 README |
| GAC 力感知为理想化零延迟 | `verify_gac_mujoco.py` | 实机 FT 有延迟，接触稳定性边界会收缩，实机前需建模 |

### 11.2 实验状态与后续

| 项 | 状态 | 下一步 |
|---|---|---|
| 实验一（正弦扫频） | 仅设计 | 实现 `--experiment sweep`（`ForceProfile.sine_sweep`） |
| 实验四（负载突变） | 仅设计 | 实现 `--experiment loadstep` |
| GAC 力感知延迟 | 理想化 | 建 FT 延迟模型，重验接触稳定边界 |
| 实机部署 | 规划中 | 按 `docs/deploy_se3_gic_to_ur12_plan.md` Phase 3 落地 UR12e GIC |
| Franka | 预留 | UR12e/UR3 全量验证后接入（`RobotHWInterface` 加 libfranka 实现） |

---

*本文档由项目代码与既有文档综合整理。如有与代码不一致之处，以代码为准；也请
顺手修正本文档。*
