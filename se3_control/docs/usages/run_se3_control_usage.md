# run_se3_control.py — SE(3) GIC 控制实机入口

> 关联文档: [GIC_plan.md](../plan/GIC_plan.md) | [interface_URtest_usages.md](interface_URtest_usages.md)
> 依赖核心模块: `core/se3_math.py`, `core/trajectory.py`, `core/gic_controller.py`, `robot_model/robot_model.py`, `hardware/ur3_hw.py`

---

## 1. 概述

`run_se3_control.py` 是 SE(3) 几何控制在**真实机械臂**（UR3 / UR12e）上的统一执行入口。
它不做仿真——所有**仿真**实验统一走 `verify_gic_mujoco.py` / `verify_gac_mujoco.py`（MuJoCo 物理推演）；
本脚本直接连接硬件，以 250 Hz 闭环运行**完整 GICController**（自适应操作空间惯性整形 + 重力补偿）。

> ⚠️ **2026-08-09 起本脚本已从 MuJoCo 仿真入口重写为实机入口**，旧的 `--max-time / --no-viewer / --cross-validate / --no-loop` 等仿真参数全部移除。仿真入口见 `verify_gic_mujoco.py` / `verify_gac_mujoco.py`。

### 架构定位

```
run_se3_control.py   (实机编排: 连接 / 安全检查 / 相位状态机 / 轨迹求值 / 记录 / 停机)
       │  仅做编排, 不含任何控制律 / 运动学数学
       ▼
core/gic_controller.py    — GIC 控制律 (GICController.compute, 自适应 M̃)
core/trajectory.py        — 轨迹生成 (build_trajectory) + 体速度求值 (eval_body_twist)
core/se3_math.py          — SE(3) 数学 (rotmat_slerp / vee_map / hat_map)
robot_model/robot_model.py — Pinocchio FK / Jb / M / 重力补偿 / 高斯-牛顿 IK
       ▼
hardware/ur3_hw.py (UR3HW, ur_rtde) — q/dq 读取 + directTorque 发力矩
```

### 任务与相位状态机

| 相位 | 内容 | 说明 |
|---|---|---|
| Phase 0 | 低带宽保持自检（默认 2s, ω=8 rad/s） | 验证力矩闭环; 若异常臂会**缓慢**偏位, 非急动。可 `--skip-phase0` 跳过（调试用） |
| Phase 2 | 主阶段: regulation 位置保持 / circle·line 轨迹跟踪 | 完整 GICController（默认 ω=20 rad/s） |
| Phase 3 | 释放 + 停机 | 按 Enter 后发零力矩并断开; 异常/急停自动 `emergency_stop` |

---

## 2. 环境与依赖

```bash
conda activate roboarm          # 依赖: ur-rtde / pinocchio / numpy / sympy
cd /media/lj252/Data/catkin_ws/roboarm_test/SE3_roboarm_control
```

实机运行前提（每次必查）:
1. 机械臂已开机, 教示器处于 **远程控制 (Remote Control)** 模式
2. **无需在教示器手动启动程序** — ur_rtde 的 `RTDEControlInterface` 构造器
   默认自动上传并运行内置 RTDE 控制脚本（`rtde_control.script`, 经 30002 端口下发,
   `FLAGS_DEFAULT = FLAG_UPLOAD_SCRIPT`）, 所有 RTDE 指令
   （`moveL`/`servoJ`/`directTorque`）都在该脚本内执行。
   ⚠️ `directTorque` 依赖控制器软件 **PolyScope ≥ 5.23**（e-Series）内置的
   `direct_torque` URScript 函数; CB3 版 UR3（PolyScope 3.x）不支持, 调用会被静默忽略。
3. 电脑网口 IP 与机械臂控制箱在同一网段
4. 臂周围无人/障碍物, 急停按钮可触及

---

## 3. 实机操作流程（按顺序）

### 3.0 干跑 — 连接 + 运动学自检（不发任何力矩）

```bash
# UR3 (默认) — 默认 IP 192.168.1.11
python se3_control/scripts/run_se3_control.py --robot ur3 --dry-run

# UR12e — 默认 IP 192.168.1.100
python se3_control/scripts/run_se3_control.py --robot ur12e --dry-run
```

干跑会连接 RTDE（默认 IP 按机器人配置: ur3→192.168.1.11, ur12e→192.168.1.100; 可 `--ip` 覆盖）、读取关节状态、
做一次 FK 自检并检查错误状态，**不发力矩**。通过后链路与模型正常，可进入实机运行。

> 连不上时 ~5s 内快速失败并提示确认开机 / IP / Remote Control 模式
> （RTDE 端口 30004 TCP 预检，见 §7 修改记录）。

### 3.1 位置保持（验证完整闭环）

```bash
# UR3 (默认) 位置保持
python se3_control/scripts/run_se3_control.py --robot ur3

# UR12e: 大臂力矩限幅是 UR3 的约 6 倍, 首跑建议先 --torque-scale 0.3 降矩
python se3_control/scripts/run_se3_control.py --robot ur12e --torque-scale 0.3
```

流程:
1. 打印安全提醒 → 按 Enter
2. Phase 0 低带宽自检 2s（当前位姿保持）
3. Phase 2 主保持 15s（默认 `--duration 15`），每周期打印 `||ep|| / rot / ||tau||`
4. 打印运行摘要与通过结论
5. 按 Enter 释放并停机

> 若 Phase 0 刚启动即报 **保护性停止**（安全模式 `safety_mode` = 3）:
> **不是缺运行程序**（ur_rtde 已自动上传 RTDE 控制脚本）——最可能是**首发力矩触发
> 安全限位**（URDF 重力补偿/惯量不准或符号错误）。先 `--torque-scale 0.3` 降矩重试;
> 若臂猛动后停止, 先校准重力补偿再全力矩。
>
> ⚠️ 若报 **急停** 但 `safety_mode` 显示 **1 (正常)** — 说明是检测误判, 不是机械臂真急停:
> 旧版按 `safety_status_bits` 位测试（ur_rtde e-Series 位布局）在 classic CB3 上可能
> 误报 bit7=急停; 现已改为以 `safety_mode` 为主判据。可先 `--dry-run` 查看
> `安全模式` 与 `安全状态位` 原始值, 再用 `--skip-phase0` 快速验证主阶段能否跑通。

期望: 稳态位置误差 ±1cm 内（理想 ±mm 量级）。可加 `--save-log hold.npz` 记录。

### 3.2 画圆轨迹跟踪（从小半径起步）

任务几何参数**按 `--robot` 自动匹配**（`task_config.ROBOT_TASK_CONFIGS`）:

| `--robot` | 圆心 | 半径 | 最大线速度 |
|---|---|---|---|
| `ur3`（默认） | `[-0.38,0,0.224]`（基座校准后实机坐标） | `0.06` | 0.048 m/s |
| `ur12e` | `[0.50,0,0.50]`（高位安全） | `0.05` | 0.04 m/s |

```bash
# UR3 画圆（radius=0.06, speed=0.8, 中心 [-0.38,0,0.224]）
python se3_control/scripts/run_se3_control.py --robot ur3 --task circle \
    --duration 16 --bandwidth 20

# UR12e 安全小圆（radius=0.05, 高位; 大臂建议 --torque-scale 0.3）
python se3_control/scripts/run_se3_control.py --robot ur12e --task circle \
    --duration 16 --bandwidth 20 --torque-scale 0.3
```

流程（轨迹任务多两步）:
1. 连接后构建轨迹, 打印轨迹起点
2. **IK 可达性预检**: 采样 24 点高斯-牛顿 IK, 检查收敛/限位/奇异; 未通过自动中止
3. 按 Enter → Phase 0 自检 → Phase 2 跟踪
4. **起步混合**（默认 0.5s）: 从当前位姿平滑过渡到轨迹起点（位置 lerp + 朝向 slerp）
5. 摘要 → 按 Enter 释放停机

### 3.3 线轨迹跟踪

```bash
# UR3 线轨迹（amplitude=0.08, frequency=0.4, 中心 [-0.38,0,0.224], 最大线速度 0.032 m/s）
python se3_control/scripts/run_se3_control.py --robot ur3 --task line \
    --duration 16 --bandwidth 20

# UR12e 安全线轨迹（amplitude=0.05, 中心 [0.50,0,0.50] 高位, 最大线速度 0.02 m/s）
python se3_control/scripts/run_se3_control.py --robot ur12e --task line \
    --duration 16 --bandwidth 20 --torque-scale 0.3
```

> 历史踩坑: 旧默认 line 中心 `[0.50, 0, 0.05]`（z=0.05 贴近地面）**过低, 危险**,
> 已抬高并按机器人分开管理; UR12e 现取 `[0.50, 0, 0.50]`, UR3 取 `[-0.38, 0, 0.224]`（基座校准后）。

### 3.4 逐步加大难度

验证稳定后, 按下列顺序增大（改 `se3_control/config/task_config.py`, 见 §5）:
1. 半径: `radius` 0.08 → 0.10 → 0.12
2. 速度: `speed` 0.8 → 1.0 → 1.2（注意 `最大线速度 = radius × speed`）
3. 带宽: `--bandwidth` 20 → 25（力矩限幅小, 不宜过高）

每步都先跑可达性预检 + Phase 0 观察。

---

## 4. 所有命令行参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `--robot` | str | `ur3` | 机器人: `ur3` / `ur12e`（实机入口面向 UR3） |
| `--task` | str | `regulation` | 任务: `regulation` / `circle` / `line` |
| `--ip` | str | 机器人配置 | UR 控制箱 IP（默认 ur3→192.168.1.11, ur12e→192.168.1.100） |
| `--dt` | float | `0.004` | 标称控制周期 (s) = 250 Hz |
| `--duration` | float | `15.0` | Phase 2 主保持/跟踪时长 (s) |
| `--hold-time` | float | `2.0` | Phase 0 低增益自检时长 (s) |
| `--hold-bandwidth` | float | `8.0` | Phase 0 自检带宽 ω (rad/s) |
| `--skip-phase0` | flag | — | 跳过 Phase 0 低带宽自检, 直接进入主阶段（调试用, 请谨慎） |
| `--bandwidth` | float | `20.0` | 主控制带宽 ω (rad/s)（UR3 推荐; task_config 的 30 是仿真值） |
| `--damping` | float | `1.0` | 阻尼比 ζ（1.0 临界阻尼） |
| `--blend-time` | float | `0.5` | 轨迹起步混合时长 (s), 从当前位姿过渡到轨迹起点 |
| `--no-feasibility` | flag | — | 跳过轨迹 IK 可达性预检 |
| `--feasibility-samples` | int | `24` | 可达性预检采样点数 |
| `--torque-scale` | float | `1.0` | 力矩限幅缩放系数（安全限幅 × scale, 可再降） |
| `--dry-run` | flag | — | 干跑: 连接 + FK 自检, 不发力矩 |
| `--save-log` | str | None | 记录数据保存为 npz（t/p/q/tau/pos_err/rot_err） |
| `--preview` | flag | — | **MuJoCo 闭环预览**: 不连接真机, 同一参数跑仿真 + 自动碰撞判定 |
| `--preview-start-q` | 6 floats | None | 从真实起步位形开始预览 (先 `--dry-run` 读当前 q 传入) |
| `--no-viewer` | flag | — | 预览不开 viewer (headless, 只出碰撞结论) |
| `--preview-speed` | float | `1.0` | 预览实时倍速 |
| `--log-dir` | str | None | 每控制周期写**全分辨率 CSV**（实机/仿真同格式, 见 §4.1 诊断） |

### 4.1 诊断: 记录 + 对照（"仿真正常、真机乱动"排查）

实机跑任务时用 `--log-dir` 记录每控制周期全分辨率数据, 同时开
`tests/monitor/monitor_rtde.py`（只读录 RTDE 原始数据）, 事后用
`tests/monitor/analyze_arm_log.py` 出图 + 自动判定, 与仿真
（`--preview --log-dir`）逐列对照。完整工作流与信号解读见
`se3_control/docs/analysis/real_vs_sim_diagnostics.md`。

```bash
# 实机: 终端 1 跑任务, 终端 2 同步录 RTDE
python se3_control/scripts/run_se3_control.py --robot ur3 --control-mode servoJ \
    --task circle --center -0.40 0.0 0.224 --radius 0.05 --duration 16 --bandwidth 10 \
    --log-dir logs/run_01
python monitor_rtde.py --robot ur3 --rate 500 --out logs/run_01/rtde.csv

# 仿真对照 (同参数, 同格式 CSV)
python se3_control/scripts/run_se3_control.py --robot ur3 --control-mode servoJ \
    --task circle --center -0.40 0.0 0.224 --radius 0.05 --duration 16 --bandwidth 10 \
    --preview --log-dir logs/sim_01

# 分析: 实机 vs 仿真 叠图 + 判定
python analyze_arm_log.py --log 'logs/run_01/Phase2_*.csv' --label 实机 \
    --log 'logs/sim_01/sim_*.csv' --label 仿真 --rtde logs/run_01/rtde.csv
```

---

## 5. 可修改参数（配置文件）

### 5.1 轨迹参数 — `se3_control/config/task_config.py`

任务几何参数**按机器人分开**存放在 `ROBOT_TASK_CONFIGS`, 由 `get_task_config(robot)`
按 `--robot` 自动匹配（模块级 `circle/line/regulation` 保留为 UR3 默认, 供向后兼容）:

| 节 | 参数 | UR3（默认） | UR12e（高位安全） | 说明 |
|---|---|---|---|---|
| `circle` | `center` | `[-0.38,0,0.224]` | `[0.50,0,0.50]` | 圆心世界坐标 (m); UR3 已按基座校准换算为实机坐标 |
| | `radius` | `0.06` | `0.05` | 圆半径 (m); UR12e 缩小降低活动范围 |
| | `speed` | `0.8` | `0.8` | 角速度 ω (rad/s); 最大线速度 = radius×ω |
| | `orientation` | 同右 | 同右 | `[0,-1,0, -1,0,0, 0,0,-1]`, 末端 z 轴朝下 (恒定; 基座 yaw180 后更新) |
| `line` | `center` | `[-0.38,0,0.224]` | `[0.50,0,0.50]` | 线段中点 (m); 历史 z=0.05 过低已抬 |
| | `amplitude` | `0.08` | `0.05` | 半幅 (m), 总长 = 2×amplitude |
| | `direction` | `[0,1,0]` | `[0,1,0]` | 振荡方向单位向量 (自动归一化) |
| | `frequency` | `0.4` | `0.4` | 角频率 ω (rad/s); 最大线速度 = amp×ω |
| `regulation` | `target` | `[-0.35,0,0.224]` | `[0.50,0,0.50]` | 仅文档参考; 实机以当前位姿为期望 |
| `gains` | `regulation`/`tracking` | 共享 | 共享 | 固定增益（GIC 用自适应 K=ω²M̃, 主要调带宽） |
| `controller` | `bandwidth`/`damping` | 共享 `30.0`/`1.0` | 共享 | **仿真**默认; 实机带宽用 `--bandwidth 20` |

> 要自定义某臂参数: 编辑 `task_config.py` → `ROBOT_TASK_CONFIGS['ur3'|'ur12e']` 对应字典。

### 5.2 机器人参数 — `se3_control/config/robot_configs.py`（`ROBOT_CONFIGS`）

| 参数 | UR3 值 | UR12e 值 | 说明 |
|---|---|---|---|
| `default_ip` | `192.168.1.11` | `192.168.1.100` | 控制箱 IP |
| `home_q` | `[-0.327,-1.42,1.236,-1.386,-1.571,2.738]` | `[-0.356,-1.498,1.81,1.259,1.571,-0.124]` | 可达性预检种子; EE≈[-0.35,0,0.224] / [0.50,0,0.50] |
| `torque_limits` | `[28,28,14,6,6,6]` | `[165,165,75,27,27,27]` | **URDF effort 的 50%** 安全限幅 (Nm) |
| `full_torque_limits` | 原始 | 原始 | 完整力矩限幅（参考, 不用） |
| `ee_frame` | `tool0` | `tool0` | 末端 frame |

---

## 6. 结果解读

### 摘要指标

- `平均/最大位置误差` — 对照**真实轨迹参考**（非混合参考）; 起步混合瞬态单独标注
- `稳态位置误差` — 排除起步混合后的误差, 用于通过判定
- `平均/最大旋转误差` (mrad)
- `关节力矩均值/标准差` — 检查是否长期顶限幅

### 通过判定（稳态最大误差）

| 任务 | ✅ 通过 | ⚠️ 基本通过 | ❌ 偏差过大 |
|---|---|---|---|
| regulation | < 10 mm | < 50 mm | ≥ 50 mm |
| circle/line | < 20 mm | < 60 mm | ≥ 60 mm |

### 轨迹任务参考量级

- circle（r=0.08, speed=0.8）: 稳态跟踪误差约 2~5 mm
- 旋转误差: circle/line 朝向恒定, 应 < 20 mrad

### npz 记录字段（`--save-log`）

`t`(时间, 真实经过) · `p`(末端位置) · `q`(关节) · `tau`(力矩) · `pos_err`(位置误差) · `rot_err`(旋转误差)

---

## 7. 修改记录（2026-08-09, Steps 1–3）

### Step 1 — `core/gic_controller.py` 的 `dVd*` 前馈 bug 修复

`adjoint_g_ed_deriv(g, gd, v, w, vd, wd)` 的 `(v,w)` 槽位是**当前体速度**、`(vd,wd)` 是**期望速度**;
修复前误把期望速度传进当前速度槽位。改为先取 `Vb = robot.get_body_ee_velocity()`, 再算
`dVd_star = adjoint_g_ed_deriv(g, gd, Vb[:3], Vb[3:], vd, wd) @ Vd + adjoint_g_ed(g_ed) @ dVd`。
回归: pytest 48 通过; circle 不变; line 在正确参数下不变（旧的 UR12e line 配置本就发散, 与修复无关）。

### Step 2 — `run_se3_control.py` 重写为实机入口 + `hardware/ur_hw.py` 连接预检

- 移除全部 MuJoCo 机制（URDF→XML、`run_simulation`、`cross_validate_models`、`plot_results` 等），
  改为硬件编排（连接 / 自检 / 相位状态机 / 记录 / 停机）。
- `hardware/ur_hw.py` 修复: 原 `timeout` 参数存而不用, 对不可达 IP 的 RTDE 连接会**无限阻塞**;
  现于 `initialize()` 先对 RTDE 端口 30004 做 socket 预检（用配置的 timeout）,
  机器人不在线时 ~5s 快速失败并给出可操作提示。所有实机入口共同受益。

### Step 3 — 轨迹跟踪（circle/line）接入

- 统一控制环 `run_tracking`（保持/跟踪共用）; **真实时间求值**（`t_real` 由 `wait_next_cycle` 累加）,
  而非仿真名义步长。
- **起步混合**: 实机无法仿真式 IK 摆位, 前 `--blend-time` 秒从当前位姿平滑过渡到轨迹起点
  （位置 lerp + 朝向 slerp + 前馈速度×bf）, 避免起始位姿差力矩跳变。
- **IK 可达性预检**: 采样 `--feasibility-samples` 点高斯-牛顿 IK, 检查收敛/关节限位/奇异, 未通过中止。
- `core/trajectory.py` 新增 `eval_body_twist`（轨迹→体坐标系 vd/wd/dvd/dwd, 与 verify_gic 逻辑一致）。
- 新增 `tests/test_trajectory_helpers.py`（7 项）; 全量 pytest **55 通过**。
- 实机流程安全增强: Phase 3 结束**按 Enter 才发零力矩释放**（臂保持最后力矩, 避免突然失去重力补偿）。

---

## 8. 故障排查（实机）

| 现象 | 可能原因 | 解决方法 |
|---|---|---|
| 连接失败 `无法连接 ... :30004 (RTDE): timed out` | IP 错误 / 臂未开机 / 非 Remote Control 模式 | 确认 IP（`--ip`）、开机、教示器切到 Remote Control |
| 错误状态 1 (急停) | 物理急停按下 / 臂被触碰 | 释放急停按钮, 教示器解除停止后重跑 |
| 错误状态 2 (保护性停止, 刚启动就报) | **首发力矩触发安全限位**（URDF 重力补偿/惯量不准或符号错误） | `--torque-scale 0.3` 降矩重试; 校准重力补偿后再全力矩 |
| directTorque 无效但无错误（臂不动） | 控制器软件 **< PolyScope 5.23**（CB3 版 UR3 不支持 `direct_torque` 内置函数） | 教示器"关于"页确认型号/版本; 不支持则改用 `servoJ` 位置伺服 |
| 位置保持漂移 > 1cm | URDF 惯性参数不准 | 校准 URDF / 增大带宽 |
| 轨迹跟踪发散 | 轨迹超出工作空间 / 带宽过高 | 先跑可达性预检; 降带宽或调轨迹参数 |
| 力矩长期顶限幅 | 带宽过高 / 轨迹太快 | 降 `--bandwidth` 或 `speed`; `--torque-scale 0.8` |
| Phase 0 臂明显偏移 | 硬件层问题 | 立即急停, 检查连接与模型 |
| 释放后臂下沉 | 停机发零力矩, 无重力补偿 | 属正常现象; 操作者扶稳后离开 |
| **发散/折叠（仿真正常、真机抬/折叠）** | servoJ 内层追不上高频参考 → 力矩饱和 + 积分漂移 → 外环发散 | **降 `--bandwidth`**（实测 10→6 后 16s 全程稳定）; 详见 [logs/run_02/实验记录_20260811_1056.md](../../../logs/run_02/实验记录_20260811_1056.md) |
| 按 Enter 释放后保护性停止（跟踪全程正常） | `shutdown()` 中 `servoStop()`/断开连接触发 UR 保护性停止 | 属正常现象; 释放前先扶稳机械臂; RTDE 已证跟踪/保持阶段无停止 |
| 实机 TCP 与模型 FK 固定偏差（x 镜像 + z 低 0.126 m） | URDF 基座坐标系与实机不一致（180° yaw + z 平移） | **已校准**（2026-08-11）: ur3.urdf 基座 yaw180° + flange-tool0 偏移 0.126 m, FK 现与 RTDE 一致; 任务坐标见 §8.1 |

---

## 8.1 基座坐标系校准：模型 FK ↔ 实机 TCP（2026-08-11）

run_02 发现同一 home 位形下 `robot_model.get_frame_pose('tool0')` 与 RTDE 实测 `tcp_x/y/z` 不一致：

| 来源 | home 位 TCP |
|---|---|
| 模型 FK(home_q)（校准前） | (+0.350, 0.000, +0.350) |
| RTDE 实测 | (-0.350, 0.000, +0.224) |
| 模型 FK(home_q)（**校准后**） | (-0.350, 0.000, +0.224) ✓ |

→ 实机 = 模型绕 z 转 180°（**x 镜像**）+ tool0/TCP **z 低 0.126 m**（方向+姿态双重确认）。
**已校准**：`ur3.urdf` 的 `shoulder_pan_joint` 基座 yaw180° + `flange-tool0` 沿 tool0 +z
偏移 0.126 m（连杆保持标准高度，仅 tool0/TCP 下移）。校准后：

- 模型 FK == RTDE TCP（home_q → (-0.350, 0.000, 0.224)），任务 `--center` 直接按实机坐标给；
- MuJoCo `--preview` 的 `end_effector` site 同步移到 tool0（`verify_gic_mujoco.py` builder 末端固定链支持）；
- 相关坐标已换算：circle/line 圆心 `[-0.38,0,0.224]`、regulation target `[-0.35,0,0.224]`（`task_config.py`）。

完整记录见 [logs/run_02/实验记录_20260811_1056.md](../../../logs/run_02/实验记录_20260811_1056.md) §5 结论 5。

---

## 9. 相关文件

| 文件 | 说明 |
|---|---|
| [run_se3_control.py](../../scripts/run_se3_control.py) | 实机入口主程序 |
| [core/gic_controller.py](../../core/gic_controller.py) | GIC 控制律 |
| [core/trajectory.py](../../core/trajectory.py) | 轨迹生成 + eval_body_twist |
| [core/se3_math.py](../../core/se3_math.py) | SE(3) 数学 |
| [hardware/ur_hw.py](../../hardware/ur_hw.py) | URHW / UR3HW 硬件接口 |
| [config/task_config.py](../../config/task_config.py) | 任务/增益/控制器参数 |
| [config/robot_configs.py](../../config/robot_configs.py) | 机器人参数（IP/限幅/home_q） |
| [interface_URtest_usages.md](interface_URtest_usages.md) | 硬件接口实测用法 |
| [tests/test_trajectory_helpers.py](../../../tests/test_trajectory_helpers.py) | 轨迹辅助单元测试 |

---

*文档重写日期: 2026-08-09（原文档 2026-07-27 记录的是已移除的 MuJoCo 仿真入口）*
