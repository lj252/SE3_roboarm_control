# SE3 RoboArm Control

基于 **SE(3) 几何控制**的机械臂柔顺控制（导纳 / 阻抗）项目。

用 SE(3) 李群 / 李代数描述机械臂末端的位姿与速度，用 **Pinocchio** 做运动学与
动力学计算，在 **MuJoCo** 中完成物理仿真验证，目标是在真实机械臂（**UR12e**、
**UR3**，未来 **Franka Panda**）上部署柔顺控制。

> 📖 **全面介绍与使用文档（推荐阅读）**：[docs/project_overview.md](docs/project_overview.md)
> 涵盖项目简介、构建思路、架构、全部实验、启动命令与可调参数。

---

## 功能特性

- **SE(3) 几何控制**：用李代数表示 6 维位姿误差与体速度，全局无奇异，同时处理
  平动 + 转动；
- **三个对等可互换的控制器模块**（互不依赖，接口互换即可切换）：

  | 模块 | 含义 | 状态 |
  |---|---|---|
  | GIC | 几何阻抗控制（力→位移被动响应） | ✅ 已实现 + 仿真验证 |
  | GAC | 几何导纳控制（力→位移主动修正） | ✅ 已实现 + 仿真验证 |
  | GUFIC | 混合力-阻抗控制 | ⏳ 预留（Phase 3） |

- **自适应操作空间惯性增益**：`K_adapt = ω²·M̃(q)`、`D_adapt = 2ζω·M̃(q)`，
  刚度随位形变化的操作空间惯性 `M̃(q) = (Jb·M⁻¹·Jbᵀ)⁻¹` 缩放，闭环响应处处一致；
- **核心模块抽离**：控制器 / 轨迹 / SE(3) 数学全部位于 `core/`，项目完全不依赖
  外部仓库，可独立运行；
- **硬件抽象层**：`RobotHWInterface` 统一 MuJoCo / UR / Franka 接口
  （Write Once, Run on Any Arm）；
- **力交互实验**：方向解耦、GIC 被动接触全流程、GAC 五种外力模式等，
  见 [docs/project_overview.md](docs/project_overview.md#7-全部实验介绍)。

---

## 快速开始

所有依赖在 conda 环境 `roboarm` 中。

```bash
conda activate roboarm
cd se3_control

# 默认: UR12e + 调节任务 + 可视化
python scripts/run_se3_control.py

# 指定机器人 / 任务 / 无头模式（SSH/服务器）
python scripts/run_se3_control.py --robot ur3 --task circle --no-viewer

# 保存结果图
python scripts/run_se3_control.py --task circle --save-plot circle.png

# 方向解耦实验（GIC 场景）
python scripts/verify_gic_mujoco.py --experiment decouple --no-viewer

# GAC 恒力实验
python scripts/verify_gac_mujoco.py --force-mode constant --no-viewer

# 实验三: GIC 被动接触全流程
python scripts/verify_gic_contact.py
```

### 运行测试

```bash
conda activate roboarm
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest tests/ -q
```

48 个测试全部通过。

---

## 项目结构

```
SE3_roboarm_control/
├── docs/                  # 根级文档（总览 + 部署计划）
│   └── project_overview.md    ← 项目全面介绍与使用文档
├── se3_control/
│   ├── core/              # 核心层: se3_math / trajectory / gic / gac / 实验分析
│   ├── config/            # 任务与机器人配置（task_config.py / robot_configs.py）
│   ├── robot_model/       # Pinocchio 封装（FK / IK / 雅可比 / 动力学）
│   ├── hardware/          # 硬件抽象（interface.py / ur_hw.py / ur12e_hw.py / ur3_hw.py）
│   ├── scripts/           # 5 个仿真脚本 + usages.md
│   ├── docs/              # 设计计划 / 使用报告 / 验证记录
│   ├── urdf/              # 机器人 URDF
│   └── figures/           # 实验输出图
├── tests/                 # 48 个 pytest 测试
└── README/                # 数学前置知识参考（李群 / SE(3) / 能量油箱）
```

---

## 文档索引

| 文档 | 作用 |
|---|---|
| [docs/project_overview.md](docs/project_overview.md) | **项目全面介绍与使用文档**（首选入口） |
| [docs/deploy_se3_to_hardware_plan.md](docs/deploy_se3_to_hardware_plan.md) | 实机部署总计划（Write Once, Run on Any Arm） |
| [docs/deploy_se3_gic_to_ur12_plan.md](docs/deploy_se3_gic_to_ur12_plan.md) | 纯 GIC 部署到 UR12e 的细化计划 |
| [se3_control/docs/plan/](se3_control/docs/plan/) | 设计计划（GIC / GAC / 硬件接口 / 力交互实验） |
| [se3_control/docs/usages/](se3_control/docs/usages/) | 使用说明与实验报告（实验二/三报告等） |
| [se3_control/scripts/usages.md](se3_control/scripts/usages.md) | 脚本使用速查（最全逐参数说明） |
| [README/代码中的前置知识.md](README/代码中的前置知识.md) | SE(3) / 李代数 / 能量油箱数学背景 |

---

## 当前状态

- ✅ GIC / GAC 已实现并通过仿真验证（48 测试 + MuJoCo 仿真）
- ✅ 方向解耦实验、GIC 被动接触实验、Phase 0 接触标定已完成
- ⏳ 正弦扫频（实验一）、负载突变（实验四）：仅完成设计，脚本未实现
- ⏳ 实机部署：按 `docs/deploy_se3_gic_to_ur12_plan.md` 推进 UR12e 纯 GIC
- ⏳ Franka：UR12e / UR3 全量验证后接入

---

## 许可证

见根目录 LICENSE（如有）。
