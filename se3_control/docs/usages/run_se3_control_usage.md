# run_se3_control.py — SE(3) GIC 控制仿真入口

> 关联文档: [GIC_plan.md](../plan/GIC_plan.md) | [deploy_se3_to_hardware_plan.md](../../../docs/deploy_se3_to_hardware_plan.md)
> 依赖核心模块: `core/se3_math.py`, `core/trajectory.py`, `core/gic_controller.py`

---

## 1. 概述

`run_se3_control.py` 是整个 SE(3) 控制项目的总入口，在 MuJoCo 仿真环境中运行 **GIC (Geometric Impedance Controller)** 控制律验证。

### 架构定位

```
run_se3_control.py  (仿真入口 — URDF→XML, MuJoCo 步进, 可视化, 记录)
       ↓ 使用
core/trajectory.py       — 轨迹生成 (build_trajectory)
core/gic_controller.py   — GIC 控制律 (GICController)
core/se3_math.py         — SE(3) 数学 (hat_map, vee_map, rotmat_slerp, ...)
       ↓ 使用
robot_model/robot_model.py  (Pinocchio 封装)
```

---

## 2. 基础用法

### 2.1 运行环境

```bash
conda activate roboarm
cd /media/lj252/Data/catkin_ws/roboarm_test/SE3_roboarm_control
```

### 2.2 默认运行（调节任务）

```bash
python3 se3_control/scripts/run_se3_control.py
```

默认参数:
- 机器人: UR12e
- 任务: regulation（位置保持）
- 仿真时长: 5 秒
- 可视化: 开启

### 2.3 选择任务

```bash
# 调节任务 — 保持末端在当前位置
python3 se3_control/scripts/run_se3_control.py --task regulation

# 圆轨迹跟踪
python3 se3_control/scripts/run_se3_control.py --task circle

# 线轨迹跟踪
python3 se3_control/scripts/run_se3_control.py --task line
```

### 2.4 选择机器人

```bash
# UR12e（默认）
python3 se3_control/scripts/run_se3_control.py --robot ur12e

# UR3
python3 se3_control/scripts/run_se3_control.py --robot ur3

# Franka Panda（预留）
python3 se3_control/scripts/run_se3_control.py --robot franka
```

### 2.5 组合使用

```bash
# UR3 + 圆轨迹
python3 se3_control/scripts/run_se3_control.py --robot ur3 --task circle

# UR12e + 线轨迹 + 更长仿真
python3 se3_control/scripts/run_se3_control.py --robot ur12e --task line --max-time 10
```

---

## 3. 所有命令行参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `--robot` | str | `ur12e` | 机器人类型: `ur12e` / `ur3` / `franka` |
| `--task` | str | `regulation` | 任务类型: `regulation` / `circle` / `line` |
| `--max-time` | float | `5.0` | 仿真时长（秒） |
| `--no-viewer` | flag | — | 关闭 MuJoCo 可视化（无头模式） |
| `--save-plot` | str | None | 保存结果图到文件（如 `circle.png`） |
| `--cross-validate` | flag | — | 仅运行模型交叉验证（不仿真） |
| `--no-stop` | flag | — | 仿真结束后不暂停 viewer |
| `--no-loop` | flag | — | 关闭连续循环模式 |
| `--bandwidth` | float | 来自 task_config | GIC 带宽 ω_des (rad/s)，覆盖配置文件 |
| `--damping` | float | 来自 task_config | GIC 阻尼比 ζ，覆盖配置文件 |

---

## 4. 典型场景

### 4.1 无头模式（SSH / 服务器）

```bash
python3 se3_control/scripts/run_se3_control.py --task circle --max-time 10 --no-viewer
```

输出示例:
```
[GIC] bandwidth=30.0, damping=1.0
  t=0.000s | pos_err=0.000000 | rot_err=0.000000 | tau_norm=38.83
  t=2.000s | pos_err=0.004583 | rot_err=0.000000 | tau_norm=36.97

Simulation Summary:
  Mean pos_err: 0.004583 m
  Max  pos_err: 0.007382 m
  Mean |tau|: 41.22 Nm
```

### 4.2 保存结果图

```bash
python3 se3_control/scripts/run_se3_control.py --task circle --save-plot results/circle_test.png --no-viewer
```

生成包含 6 个子图的 PNG:
1. 末端位置跟踪
2. 朝向误差（角度）
3. 位置误差范数
4. 旋转误差范数
5. 关节力矩
6. 3D 轨迹

### 4.3 连续循环模式

仅在有 viewer 时生效，仿真结束后继续运行，末端持续跟踪轨迹。关闭 viewer 窗口停止。

```bash
# circle 默认开启连续循环（看到末端持续画圆）
python3 se3_control/scripts/run_se3_control.py --task circle

# 关闭连续循环
python3 se3_control/scripts/run_se3_control.py --task circle --no-loop
```

### 4.4 自定义 GIC 参数

覆盖 `task_config.py` 中的控制器参数:

```bash
# 降低带宽（适合小机械臂 UR3）
python3 se3_control/scripts/run_se3_control.py --robot ur3 --task regulation --bandwidth 20 --damping 1.0

# 高带宽 + 低阻尼（更激进）
python3 se3_control/scripts/run_se3_control.py --task circle --bandwidth 40 --damping 0.7
```

### 4.5 模型交叉验证

对比 Pinocchio RobotModel 与 MuJoCo 的运动学/雅可比一致性:

```bash
# UR12e
python3 se3_control/scripts/run_se3_control.py --robot ur12e --cross-validate

# UR3
python3 se3_control/scripts/run_se3_control.py --robot ur3 --cross-validate
```

预期输出（位置和雅可比差异均为 ~10⁻¹¹，即机器精度）:

```
Test config 1: q=[-0.251  0.901  0.464 ...]
    pos_diff = 3.892897e-11
    jac_diff = 2.251614e-11 (rel)
```

---

## 5. 参数配置

控制器参数从 `config/task_config.py` 读取:

```python
# 控制器
controller = {
    'bandwidth': 30.0,   # ω_des (rad/s)
    'damping':   1.0,    # ζ, 1.0 = 临界阻尼
}

# 轨迹可视化
trail = {
    'interval':   8,         # 每隔 N 步添加轨迹点
    'max_points': 1200,      # 轨迹点滑动窗口
    'sphere_size': 0.006,    # 点半径
    'color': [1.0, 0.2, 0.2, 0.85],  # RGBA
}

# 仿真参数
simulation = {
    'dt':        0.001,    # 仿真步长 (s)
    'max_time':  5.0,      # 默认时长 (s)
}
```

机器人参数从 `config/robot_configs.py` 加载（URDF 路径、力矩限幅、网格映射等）。

任务参数（圆心、半径、速度等）从 `config/task_config.py` 中的 `circle` / `line` / `regulation` 节读取。

---

## 6. 结果解读

### tracking 误差参考

| 任务 | 平均位置误差 | 说明 |
|---|---|---|
| regulation | < 0.001 mm | 理想情况为零（起始 = 目标） |
| circle | ~5 mm | 动态跟踪，受带宽和半径影响 |
| line | ~1.5 mm | 线轨迹动态较小 |

### 朝向渐进混合

动态任务（circle / line）的前 0.4 秒会从 home 朝向 SLERP 混合到轨迹朝向，避免初始 0.87 rad 朝向误差导致力矩饱和。

### 控制频率

无 viewer 时可达 ~4000 Hz（1000 步仿真 0.24 秒），有 viewer 时受渲染帧率限制。

---

## 7. 故障排查

| 现象 | 可能原因 | 解决方法 |
|---|---|---|
| MuJoCo 加载 XML 失败 | URDF 路径或网格文件缺失 | 确认 `urdf/` 目录完整 |
| 机器人不动 | 力矩限幅过低 | 检查 `full_torque_limits` |
| 跟踪发散 | 带宽过高或工作空间外 | 降低带宽或调整任务参数 |
| Viewer 启动失败 | 无显示器 | 加 `--no-viewer` |
| 交叉验证失败 | 网格路径错误 | 确认 `mesh_subdir` 配置 |

---

## 8. 相关文件

| 文件 | 说明 |
|---|---|
| [run_se3_control.py](../../scripts/run_se3_control.py) | 主程序 |
| [core/se3_math.py](../../core/se3_math.py) | SE(3) 数学工具 |
| [core/trajectory.py](../../core/trajectory.py) | 轨迹生成 |
| [core/gic_controller.py](../../core/gic_controller.py) | GIC 控制律 |
| [config/task_config.py](../../config/task_config.py) | 任务参数 |
| [config/robot_configs.py](../../config/robot_configs.py) | 机器人参数 |
| [GIC_plan.md](../plan/GIC_plan.md) | 核心移植计划 |

---

*文档创建日期: 2026-07-27*
