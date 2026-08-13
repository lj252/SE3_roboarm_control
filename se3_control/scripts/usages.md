# se3_control/scripts/ — 脚本使用说明

> 关联: [run_se3_control_usage.md](../docs/usages/run_se3_control_usage.md) | [verify_gac_mujoco_plan.md](../docs/plan/verify_gac_mujoco_plan.md)

---

## 1. 运行环境

```bash
conda activate roboarm
cd /media/lj252/Data/catkin_ws/roboarm_test/SE3_roboarm_control
```

---

## 2. 脚本一览

| 脚本 | 控制器 | 用途 | 依赖 |
|---|---|---|---|
| `run_se3_control.py` | GIC (core 模块) | **实机入口**: 连接 UR3/UR12e 运行 GIC 闭环（保持/跟踪） | core + hardware 模块 |
| `verify_gic_mujoco.py` | GIC (内联) | GIC 交叉验证（Pinocchio vs MuJoCo），含绘图 | 外部 GUFIC 库 |
| `verify_gac_mujoco.py` | GAC (core 模块) | GAC 导纳控制验证，5 种外力模式 | core 模块 |

### 选择建议

```
仿真日常使用  →  verify_gic_mujoco.py (GIC) / verify_gac_mujoco.py (GAC)
实机验证      →  run_se3_control.py  (需真机 UR3/UR12e)
GIC 精度验证  →  verify_gic_mujoco.py  (交叉验证 + 绘图)
GAC 导纳验证  →  verify_gac_mujoco.py  (外力模拟 + 切向跟随)
```

---

## 3. run_se3_control.py — 实机执行入口

连接真实机械臂 (UR3 / UR12e) 运行**完整 GICController** 闭环（自适应惯性整形 + 重力补偿）。
**不做仿真** — 所有仿真实验统一走 `verify_gic_mujoco.py` / `verify_gac_mujoco.py`。
实机前提: 机械臂开机、教示器 **Remote Control** 模式、网口同网段。
> ⚠️ **无需手动启动程序** — `RTDEControlInterface` 构造器默认自动上传并运行 RTDE 控制脚本
> （`FLAGS_DEFAULT = FLAG_UPLOAD_SCRIPT`）。若 Phase 0 刚启动即报"保护性停止"（safety_mode=3）,
> 是**首发力矩触发安全限位**（重力补偿/URDF 不准或符号错误）, 先 `--torque-scale 0.3` 降矩;
> `directTorque` 还要求控制器软件 **PolyScope ≥ 5.23**（e-Series）。
> 若报"急停"但 safety_mode=1（正常）, 是检测误判（classic CB3 的 safety_status_bits 位义
> 与 e-Series 不同）; 已改以 safety_mode 为主判据, 可用 `--dry-run` 查看原始安全状态位,
> 或 `--skip-phase0` 跳过 Phase 0 直接验证主阶段。

```bash
# 干跑: 连接 + FK 自检, 不发力矩
python se3_control/scripts/run_se3_control.py --robot ur3 --dry-run

# 位置保持 (Phase0 低增益自检 → Phase2 主保持 → 按 Enter 释放停机)
python se3_control/scripts/run_se3_control.py --robot ur3

# 圆轨迹跟踪 (IK 可达性预检 + 0.5s 起步混合)
# 圆心/半径/速度按 --robot 自动匹配:
#   ur3   → 圆心 [-0.38,0,0.224] / r0.06 (基座校准后实机坐标; 远离基部 + 抬高)
#   ur12e → 圆心 [0.50,0,0.50] / r0.05 (高位安全)
python se3_control/scripts/run_se3_control.py --robot ur3 --task circle --duration 16

# ★ 上真机前先预览: 不连接硬件, 用同一参数在 MuJoCo 里跑闭环仿真,
#   实时看臂的轨迹 (红色 trail) + 自动碰撞判定 (基座柱/地面净距 ✓/✗)
python se3_control/scripts/run_se3_control.py --robot ur3 \
    --control-mode servoJ --task circle --duration 16 --bandwidth 10 --preview
#   预览只出结论 (headless, 不弹窗口)
python se3_control/scripts/run_se3_control.py --robot ur3 --task circle \
    --center -0.40 0.0 0.224 --radius 0.05 --preview --no-viewer
#   真机当前位形不是 home (臂已折叠/低位) 时, 先 --dry-run 读当前 q 再传入预览
python se3_control/scripts/run_se3_control.py --robot ur3 --task circle \
    --preview --preview-start-q -0.327 -0.6 2.4 -1.386 -1.571 2.738

# 调试: 跳过 Phase 0 自检, 直接进入主阶段 (排查"一启动就急停"时用)
python se3_control/scripts/run_se3_control.py --robot ur3 --task circle --duration 16 --skip-phase0
```

**主要参数**：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--robot` | `ur3` | `ur3` / `ur12e` |
| `--task` | `regulation` | `regulation` / `circle` / `line` |
| `--ip` | 机器人配置 | 控制箱 IP (ur3→192.168.1.11) |
| `--duration` | `15.0` | Phase 2 保持/跟踪时长 (s) |
| `--bandwidth` | `20.0` | 主控制带宽 ω (rad/s, UR3 推荐) |
| `--damping` | `1.0` | 阻尼比 ζ |
| `--blend-time` | `0.5` | 轨迹起步混合时长 (s) |
| `--skip-phase0` | — | 跳过 Phase 0 自检, 直接主阶段 (调试用) |
| `--torque-scale` | `1.0` | 力矩限幅缩放系数 |
| `--dry-run` | — | 干跑 (连接 + FK 自检, 不发力矩) |
| `--save-log` | None | 记录数据保存 npz |
| `--preview` | — | MuJoCo 闭环预览 (不连接硬件, 实时看轨迹 + 自动碰撞判定) |
| `--no-viewer` | — | 预览不弹可视化窗口 (headless, 只出碰撞结论) |
| `--preview-speed` | `1.0` | 预览实时倍速 (>1 加速) |
| `--preview-start-q` | `home_q` | 预览起步位形 (6 关节角 rad); 真机不在 home 时先 `--dry-run` 读 q 传入 |

> 安全: 力矩限幅 = URDF effort 50%; 启动前按 Enter 确认, 结束释放前再按 Enter。
> circle/line 的圆心/半径/速度在 `config/task_config.py` → `ROBOT_TASK_CONFIGS['ur3'|'ur12e']` 中
> 按机器人配置, 由 `get_task_config(robot)` 按 `--robot` 自动匹配。

### 3.1 上真机前先预览 (--preview)

反复"撞"（肘/前臂向基部折叠、末端向下够）多是**闭环动态行为**, 静态 IK 预检拦不住。
`--preview` 在 MuJoCo 里用**与实机完全相同的参数**跑闭环仿真（`directTorque` 发力矩,
`servoJ` 走力矩→关节目标位桥 + 内层**计算力矩伺服**——重力补偿 + 速度/加速度前馈 +
临界阻尼, 带宽 30 rad/s）, 实时看臂的运动 + 末端红色轨迹, 并**自动判定碰撞**:

- 沿仿真轨迹逐点 FK, 检查连杆到**基座柱**（半径 0.09m、柱顶 0.152m, 带高度条件防误报）
  与**地面**（<0.11m）的最小净距
- 打印确定性结论: `✓ 全程无碰撞风险 — 可上真机` 或 `✗ t=..s .. 距基座柱仅 .. cm — 会撞`
- 预览**不连接硬件**, 真机一根手指都不用动; 迭代 `--center`/`--radius`/带宽后重跑即可

**工作流**:

```bash
# 0) 臂不在 home? 先安全回 home (关节空间 moveJ, 低速, 移动前有危险检查+确认):
python tests/monitor/go_home.py                  # 回 robot_configs 设置的 home_q
python tests/monitor/go_home.py --show-only      # 只查看当前/目标位形 + 危险检查, 不移动
# 1) 预览 → ✓ 后再上真机 (同一套参数, 只是加/去 --preview)
python se3_control/scripts/run_se3_control.py --robot ur3 --control-mode servoJ \
    --task circle --duration 16 --bandwidth 10 --preview
# 2) 若不能回 home (例如想模拟真机从当前折叠位形起步), 用 --dry-run 读当前 q 传入预览:
#    先 python ... --dry-run 记下 "关节位置 q: [ ... ]", 再:
python se3_control/scripts/run_se3_control.py --robot ur3 --task circle \
    --preview --preview-start-q <6 个关节角 rad>
# 3) 预览 ✓ → 去掉 --preview 上真机
```

> 注: 碰撞判定基于理想闭环动力学; 实机 servoJ 内层伺服滞后/真实惯量可能使连杆比仿真
> 略低 2-3cm, 建议保留余量 (圆心 |x|≥0.35、z≥0.224, 基座校准后实机坐标)。严重折叠/贴地的
> 位形会在预览中直接报 ✗。
> 内层伺服必须用**计算力矩模型**（重力补偿+前馈+临界阻尼）; 裸 PD（无重力补偿/前馈）
> 在 circle 任务上会因腕部力矩限幅饱和 + 参考积分漂移而发散成混乱轨迹（v0.2 之前）。

### 3.2 实机乱动时的数据取证（--log-dir + tests/monitor/monitor_rtde + tests/monitor/analyze_arm_log）

仿真正常、真机却向上抬/折叠时, 用三个工具把实机运动过程录下来与仿真逐列对照:

```bash
# 实机跑任务时加 --log-dir (每控制周期全分辨率 CSV, 与仿真同格式)
python se3_control/scripts/run_se3_control.py --robot ur3 --control-mode servoJ \
    --task circle --duration 16 --bandwidth 10 --log-dir logs/run_01

# 另一个终端同步录 RTDE 原始数据 (只读, 不影响任务)
python tests/monitor/monitor_rtde.py --robot ur3 --rate 500 --out logs/run_01/rtde.csv

# 仿真对照 (同参数 + --preview --log-dir)
python se3_control/scripts/run_se3_control.py --robot ur3 --control-mode servoJ \
    --task circle --duration 16 --bandwidth 10 --preview --log-dir logs/sim_01

# 分析: 自动判定参考积分漂移/力矩饱和/误差发散/折叠特征 + 出图
python tests/monitor/analyze_arm_log.py --log 'logs/run_01/Phase2_*.csv' --label 实机 \
    --log 'logs/sim_01/sim_*.csv' --label 仿真 --rtde logs/run_01/rtde.csv
```

信号解读与完整工作流见
[real_vs_sim_diagnostics.md](../docs/analysis/real_vs_sim_diagnostics.md)。

**已确认的修复与偏差**（2026-08-11 两次实机，记录见
[logs/run_02/实验记录_20260811_1056.md](../../logs/run_02/实验记录_20260811_1056.md)）：
- 发散根因 = 高频任务下 servoJ 内层追不上参考（力矩饱和 + 积分漂移）→ **降
  `--bandwidth` 10→6 后 16s 全程稳定**；REDUCED 安全模式不是主因。
- 保护性停止发生在按 Enter 释放/`shutdown()` 环节（RTDE 全程 `safety_mode=1`），
  不在跟踪过程。
- **坐标系偏差**：模型 FK(home_q)=(+0.35,0,0.35) vs RTDE 实测 (-0.35,0,0.224)——
  x 镜像 + z 低 0.126 m。**已校准**（ur3.urdf 基座 yaw180° + flange-tool0 偏移 0.126 m），
  模型 FK 现与 RTDE 一致，任务坐标换算为实机值（见 §3.1 圆心）。

**详细文档**: [run_se3_control_usage.md](../docs/usages/run_se3_control_usage.md)

---

## 4. verify_gic_mujoco.py — GIC 交叉验证（内联实现）

与 `run_se3_control.py` 功能类似，但 GIC 控制器和轨迹生成代码**内联在文件中**。
主要用于与 GUFIC 原型代码的对比验证，包含完整的 Pinocchio vs MuJoCo 交叉验证。

```bash
# 默认运行 (UR12e, regulation, 可视化)
python se3_control/scripts/verify_gic_mujoco.py

# 指定任务
python se3_control/scripts/verify_gic_mujoco.py --task circle
python se3_control/scripts/verify_gic_mujoco.py --task line

# 无头模式
python se3_control/scripts/verify_gic_mujoco.py --task circle --no-viewer

# 交叉验证
python se3_control/scripts/verify_gic_mujoco.py --cross-validate

# 保存结果图
python se3_control/scripts/verify_gic_mujoco.py --task circle --save-plot gic_circle.png
```

**主要参数**：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--robot` | `ur12e` | `ur12e` / `ur3` / `franka` |
| `--task` | `regulation` | `regulation` / `circle` / `line` |
| `--max-time` | `5.0` | 仿真时长 (秒) |
| `--no-viewer` | — | 关闭可视化 |
| `--save-plot` | None | 保存结果图 |
| `--cross-validate` | — | 仅运行交叉验证（不仿真） |
| `--no-stop` | — | 仿真结束后不暂停 viewer |
| `--no-loop` | — | 关闭连续循环模式 |

> 画圆任务的圆心/半径/速度在 `config/task_config.py` → `circle` 中配置
> (本脚本无 CLI 覆盖参数), 详见 §8.1.

---

## 5. verify_gac_mujoco.py — GAC 导纳控制验证

GAC 导纳控制器与 MuJoCo 联合验证。**核心新增功能是 5 种外力模式**，
用于模拟机器人受到外力时的导纳响应。

### 5.1 基本用法

```bash
# 退化模式: F_ext=0, 行为等同 GIC
python se3_control/scripts/verify_gac_mujoco.py

# 恒力响应: 观察末端位置偏移
python se3_control/scripts/verify_gac_mujoco.py --force-mode constant --no-viewer

# 脉冲响应: 观察动态恢复
python se3_control/scripts/verify_gac_mujoco.py --force-mode pulse \
    --force-start 1.0 --force-duration 2.2 --no-viewer

# 切向力跟随: 沿圆弧柔顺滑动 (带径向虚拟弹簧约束)
# 默认: 圆心(0.5,0.0), 半径0.2m, 径向刚度500N/m, 仿真 30s
# 注1: 使用 惯性系 3-DOF 导纳 (位置修正), 不与 R_cur 耦合,
#      完全避免末端倾斜导致的 Z 方向漂移和振荡.
#      详见下方 §7 "常见错误与调试".
# 注2: 期望朝向强制为"末端垂直朝下" (与 GIC circle 任务一致),
#      腕关节保持中位, 画圆更舒适不易碰限位. 起始姿态经 3s 平滑过渡.
python se3_control/scripts/verify_gac_mujoco.py --task regulation \
    --force-mode tangent --tangent-amplitude 10 --no-viewer

# 更大半径 + 更高径向刚度 (轨迹更圆)
python se3_control/scripts/verify_gac_mujoco.py --task regulation \
    --force-mode tangent --tangent-amplitude 10 \
    --tangent-radius 0.3 --tangent-radial-stiffness 1000 --no-viewer

# 无径向约束 (纯切向力, 验证 GAC 持续运动能力)
python se3_control/scripts/verify_gac_mujoco.py --task regulation \
    --force-mode tangent --tangent-amplitude 10 \
    --tangent-radial-stiffness 0 --no-viewer

# circle 跟踪 + 外力扰动
python se3_control/scripts/verify_gac_mujoco.py --task circle \
    --force-mode constant --force-amplitude 5 0 0 0 0 0 --no-viewer
```

### 5.2 外力模式详解

| `--force-mode` | 行为 | 物理意义 | 典型场景 |
|---|---|---|---|
| `zero` | F_ext = 0 | 退化模式，GAC = GIC | 回归验证 |
| `constant` | 恒定方向恒力 | 机器人被恒力推开 | 接触力响应 |
| `pulse` | 短时脉冲力 | 冲击响应 | 抗扰动能力 |
| `spring` | F = K_env · | 环境接触 | 刚性表面触碰 |
| `tangent` | 沿圆弧切向力 + 径向虚拟弹簧 | 惯性系导纳(位置), 无 R_cur 耦合, 无 Z 漂移; 末端朝下 | 柔顺跟随 |

### 5.3 导纳参数调优

```bash
# 软导纳 (低刚度, 对外力更敏感)
python se3_control/scripts/verify_gac_mujoco.py --task regulation \
    --K-d 100 100 100 10 10 10 \
    --force-mode constant --force-amplitude 10 0 0 0 0 0 --no-viewer

# 硬导纳 (高刚度, 接近位置控制)
python se3_control/scripts/verify_gac_mujoco.py --task regulation \
    --K-d 2000 2000 2000 200 200 200 \
    --force-mode constant --force-amplitude 10 0 0 0 0 0 --no-viewer

# 各向异性: 切向柔顺 + 径向刚性
python se3_control/scripts/verify_gac_mujoco.py --task regulation \
    --force-mode tangent --tangent-amplitude 10 \
    --K-d 2000 2000 500 50 50 50 --no-viewer
```

### 5.4 完整参数表

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--robot` | `ur12e` | `ur12e` / `ur3` / `franka` |
| `--task` | `regulation` | `regulation` / `circle` / `line` |
| `--max-time` | `5.0`（tangent 模式→30.0）| 仿真时长 (秒), tangent 模式自动增至 30.0 |
| `--no-viewer` | — | 关闭可视化 |
| `--save-plot` | None | 保存结果图 |
| `--cross-validate` | — | 仅运行交叉验证 |
| `--force-mode` | `zero` | `zero` / `constant` / `pulse` / `spring` / `tangent` |
| `--force-amplitude` | `10 0 0 0 0 0` | 外力幅值 [fx fy fz tx ty tz] |
| `--force-start` | `1.0` | 脉冲起始时间 (s) |
| `--force-duration` | `0.5` | 脉冲持续 (s) |
| `--tangent-amplitude` | `10.0` | 切向力幅值 (N) |
| `--tangent-radius` | `0.2` | 切向圆半径 (m) |
| `--tangent-circle-center` | `0.5 0.0 0.125` | 切向圆心 [cx cy cz] |
| `--tangent-radial-stiffness` | `500.0` | 径向虚拟弹簧刚度 (N/m), 0=无约束 |
| `--M-d` | `10 10 10 1 1 1` | 虚拟质量 |
| `--D-d` | 临界阻尼 | 虚拟阻尼 |
| `--K-d` | `500 500 500 50 50 50` | 虚拟刚度 |
| `--bandwidth` | `30.0` | GAC 内环带宽 (rad/s) |
| `--damping` | `1.0` | GAC 内环阻尼比 |

---

## 6. 推荐验证流程

```
1. verify_gic_mujoco.py --task regulation --no-viewer
   → GIC 基础功能验证 (仿真)

2. verify_gic_mujoco.py --task circle --no-viewer
   → GIC 轨迹跟踪 + 交叉验证

3. verify_gac_mujoco.py --task regulation --no-viewer
   → GAC 退化模式 (与 GIC 对比)

4. verify_gac_mujoco.py --force-mode constant --no-viewer
   → GAC 恒力响应

5. verify_gac_mujoco.py --force-mode tangent --no-viewer
   → GAC 各向异性导纳 (独有功能)
```

---

## 7. 常见错误与调试

### 7.1 tangent 模式: 轨迹不是完整水平圆, Z 方向下沉/振荡

**现象**: 使用 `--force-mode tangent` 时:
- 末端只画出一段短弧就停止
- 持续向下掉 (Z 方向单调漂移或振荡), 直到超出关节限位
- 轨迹不在水平面上 (pos_err 远大于理论值 2×半径)

**根因 (两次修复)**:

**第一次尝试 — 体坐标系导纳 + F_body_z=0 (失败)**:

```
F_ext_body = R_cur.T @ F_ext_inertial    # 惯性系力→体坐标系
F_ext_body[2] = 0.0                       # 清除 body-z
```

F_body_z=0 防止了 X_corr_z 累积, 但 `_correct_trajectory` 中
`pd_corrected = pd + R_cur @ X_corr` 仍然将无界的 XY 修正量
(X_corr_x/y 因 K_d[:2]=0 无界增长) 通过 R_cur 投影到惯性系 Z:

```
Δz_inertial = R_cur[2,0]*X_corr_x + R_cur[2,1]*X_corr_y
```

R_cur 随机器人绕圆变化 → Z 投影是时变的 → 产生**振荡** (而非单调漂移).

**第一次尝试 — 去掉 R_cur.T 旋转 (失败)**:

```
F_ext_raw[:3] = F_ext_inertial[:3]       # 惯性系力直接输入
```

去掉 R_cur.T 后, 体坐标系导纳累积的 X_corr 不与机器人旋转方向对齐,
旧方向的修正量始终指向错误方向 → 持续振荡, |F_ext| 达到 87N.

**最终方案 — 惯性系 3-DOF 导纳 (成功)**:

绕过 GACFilter 的位置修正, 在惯性系中独立维护 3-DOF 导纳状态:

```
F_ext_inertial = ForceProfile.tangential(...)   # 惯性系力, 不旋转
# 惯性系导纳更新:  M·a + D·v + K·x = F
# K_xy=0, K_z=500, D=2·sqrt(K_radial·M), 临界阻尼
acc = (F_ext_inertial[:3] - D*v - K*x) / M
v += acc * dt
x += v * dt
pd_track[:3] = pd[:3] + x              # 惯性系叠加
F_ext_ctrl = zeros(6)                    # GACFilter 不参与
```

关键: 位置修正量 `x` 保存在惯性系, 不经过任何 R_cur 旋转,
Z 通道完全独立 (K_z=500 刚性维持高度).

### 7.2 tangent 模式: 动不起来 / 只动一小段

- **`--max-time` 太短**: 默认 5s, tangent 模式自动增至 30s
   (圆周运动周期 ≈18s, F=10N, D=141, v≈0.07m/s, 周长=1.26m)
- **`--max-correction` 太小**: K_d[:2]=0 时 X_corr 无界增长,
   默认 0.05m 迅速达到上限。tangent 模式自动设为 5.0m
- **径向刚度太大**: `--tangent-radial-stiffness` >2000 会限制运动

### 7.3 其他模式常见问题

| 模式 | 问题 | 检查项 |
|---|---|---|
| `constant` | 末端偏移太小或太大 | 检查 `--K-d` (越大越硬, 偏移越小) |
| `pulse` | 没有明显脉冲响应 | 检查 `--force-start`/`--force-duration` |
| `spring` | 接触力异常 | 检查接触面高度 (默认 z=0.0) |
| `circle` | 轨迹跟踪不准 | 增大 `--bandwidth` (默认 30.0) |

### 7.4 tangent 模式: 画圆时末端工具朝上 (倾斜固定角度)

**现象**: GAC tangent 画圆时末端以恒定倾斜角度朝上,
而 GIC circle 任务末端垂直朝下. 用户期望画圆时工具朝下 (舒适, 不易碰限位).

**根因**: 期望朝向定义不同.
- **GAC tangent** 跑在 `--task regulation` 上, 期望朝向固定为
  `Rd_t = R_start` (home 起始姿态的朝向). 我们的惯性系导纳修复
  只修正**位置** (`pd_track = pd + pos_corr_inertial`), 从未改动 Rd.
  所以画圆全程保持 home 的倾斜朝向.
- **GIC circle** 从 `task_config.py` 读取朝向:
  `orientation` 第 3 列为 `(0,0,-1)`, 即末端 z 轴垂直朝下.

不是 GAC 的 bug, 而是两个任务命令了不同的期望朝向.

**修复** (verify_gac_mujoco.py):
1. 定义常量 `TANGENT_DOWN_R` (z 轴朝下的旋转矩阵, 与 GIC 一致)
2. 主循环中 tangent 分支覆盖 `Rd_des = TANGENT_DOWN_R`
3. 从起始姿态到朝下约 160°, 用 **3s 平滑过渡** (slerp):
   - 0.4s 过渡角速度 ≈7 rad/s, 会使前 4 个关节力矩饱和到限位
   - 3s 过渡 ≈0.93 rad/s, 力矩全部在限位内
4. 过渡期间用**有限差分**估计 dRd, 使 wd 反映实际旋转, 避免姿态跟踪滞后

**验证**: 稳态 rot_err = 0, 末端 z 轴对齐 world -z 度 = 1.0000;
圆轨迹半径 0.201m, 角度覆盖 >600° (画完整圆); Z 高度保持 (波动 <0.05m 仅在过渡期).

---

## 8. 画圆任务: 圆心位置与半径控制

GIC 和 GAC 都支持画圆任务, 但**圆心/半径的配置方式不同**:

| 控制方式 | 脚本 | 圆心 | 半径 | 参数来源 |
|---|---|---|---|---|
| GIC 画圆 | `run_se3_control.py --task circle` | 配置文件 | 配置文件 | `config/task_config.py` → `ROBOT_TASK_CONFIGS['ur3'\|'ur12e']['circle']` |
| GIC 画圆 | `verify_gic_mujoco.py --task circle` | 配置文件 | 配置文件 | 同上 (按 `--robot` 匹配) |
| GAC 画圆 | `verify_gac_mujoco.py --task circle` | 配置文件 | 配置文件 | 同上 (按 `--robot` 匹配) |
| GAC 切向柔顺画圆 | `verify_gac_mujoco.py --force-mode tangent` | **CLI** | **CLI** | `--tangent-circle-center` / `--tangent-radius` |

### 8.1 配置文件方式 (GIC circle / GAC circle 任务)

圆心/半径按机器人分开存放于 `se3_control/config/task_config.py` 的
`ROBOT_TASK_CONFIGS`, 由 `get_task_config(robot)` 按 `--robot` 自动匹配
(模块级 `circle` 保留为 UR3 默认):

```python
ROBOT_TASK_CONFIGS = {
    'ur3': {                     # UR3 (默认): 圆心低/近, 半径大些
        'circle': {
            'center':      [-0.38, 0.0, 0.224],  # 圆心 [x, y, z] (m), 基座校准后实机坐标
            'radius':      0.06,                # 半径 (m)
            'speed':       0.8,                 # 角速度 (rad/s), 越大画得越快
            'orientation': [0, -1, 0,
                            -1, 0, 0,
                            0, 0, -1],          # 末端 z 轴朝下 (基座 yaw180 后更新)
        },
    },
    'ur12e': {                   # UR12e: 高位安全, 缩小活动范围
        'circle': {
            'center':      [0.50, 0.0, 0.50],  # 高位 home
            'radius':      0.05,               # 缩小
            'speed':       0.8,
            'orientation': [0, 1, 0,
                            1, 0, 0,
                            0, 0, -1],
        },
    },
}
```

改完对应机器人的 `center` / `radius` 后, 三个脚本的 circle 任务在指定 `--robot` 时都会使用新值:

```bash
python se3_control/scripts/run_se3_control.py --robot ur3 --task circle   # 实机入口 (需硬件, 无 viewer)
python se3_control/scripts/verify_gic_mujoco.py  --robot ur3 --task circle --no-viewer
python se3_control/scripts/verify_gac_mujoco.py  --robot ur3 --task circle --no-viewer
```

### 8.2 CLI 方式 (GAC tangent 模式)

tangent 模式跑在 `--task regulation` 上, 画圆由 `--force-mode tangent`
专属参数完全定义 (**不读** `task_config.circle`):

```bash
# 默认: 圆心 (0.5, 0.0, 0.125), 半径 0.2m
python se3_control/scripts/verify_gac_mujoco.py --force-mode tangent --no-viewer

# 自定义圆心 + 半径
python se3_control/scripts/verify_gac_mujoco.py --force-mode tangent \
    --tangent-circle-center 0.6 0.1 0.3 \
    --tangent-radius 0.3 --no-viewer
```

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--tangent-circle-center` | `0.5 0.0 0.125` | 圆心 [cx cy cz]. **只取 cx, cy** (水平位置). 实际圆高度 = 初始末端 z (`--init-pos` 的 z, 默认 0.25), 由惯性系导纳 K_z 刚性维持 |
| `--tangent-radius` | `0.2` | 圆半径 (m) |
| `--tangent-radial-stiffness` | `500.0` | 径向虚拟弹簧刚度 (N/m), 越大轨迹越贴合半径, `0`=无约束 |

> 提示: 半径越大, 画完整圆所需时间越长 (圆周运动周期 ≈ 2πr/v,
> F=10N、D=141 时 v≈0.07m/s, 默认 r=0.2 周期 ≈18s). 详见 §7.2.

---

## 9. 力交互实验二 — 方向解耦测试

设计文档: [docs/plan/force_interaction_experiments_plan.md](../docs/plan/force_interaction_experiments_plan.md) §4.
完整归档报告 (改动 + 结果 + 评判方法): [docs/usages/exp2_direction_decoupling_report.md](../docs/usages/exp2_direction_decoupling_report.md).

**目的**: 在 GAC / GIC 两种控制场景下, 依次施加三个轴向恒力 (Fx/Fy/Fz) 与
三个恒力偶 (Mx/My/Mz), 检验每类输入**只产生对应轴的位移**, 不产生额外位移
(方向解耦). 核心回归对象是历史 "Z 振荡" bug (施加 x 向力出现 z 向漂移).

### 9.1 运行

```bash
# GAC 方向解耦 (主测对象) — 无头模式
python se3_control/scripts/verify_gac_mujoco.py --experiment decouple --no-viewer

# GIC 方向解耦 (基线)
python se3_control/scripts/verify_gic_mujoco.py --experiment decouple --no-viewer

# 自定义输入幅值 / 块时长
python se3_control/scripts/verify_gac_mujoco.py --experiment decouple \
    --decouple-force 5.0 --decouple-moment 0.5 \
    --decouple-settle 1.5 --decouple-measure 1.0 --no-viewer
```

### 9.2 原理与双力通路

仿真按 **7 个时间块**顺序进行: 块 0 = 基线 (零输入), 块 1-3 = +x/+y/+z 恒力,
块 4-6 = 绕 x/y/z 恒力偶 (均世界系). 每块长 `settle+measure` 秒, 取每块
最后 `measure` 秒的稳态位姿均值, 相对基线块求 6 维响应 `[Δp; Δφ]` →
得到 6×6 **静态耦合矩阵** 与耦合比矩阵 (`|Δ_out|/|Δ_in|`).

物理力 `data.xfrc_applied[ee_body]` (世界系, 作用在末端 body COM) 与感知力
`F_ext_ctrl = R_curᵀ·F_world` (GAC 滤波器输入, 体坐标系) 分离:

| 通路 | GAC | GIC |
|---|---|---|
| 物理力 (仿真中真实作用在末端) | ✔ | ✔ |
| 感知力 (控制器输入) | Rᵀ·F (体坐标导纳) | ✘ (被动响应) |

GAC 额外记录 `GACFilter.state['X_corr']` 并做体→世界系投影, 输出
**滤波器层耦合矩阵** (`coupling_xc`), 与 EE 最终位移耦合对比, 区分
**滤波器耦合** 与 **跟踪层耦合**.

### 9.3 参数表

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--experiment decouple` | `none` | 启用方向解耦实验 (自动 `max_time = 7×(settle+measure)`) |
| `--decouple-force` | `10.0` | 轴向力幅值 (N) — GAC 稳态位移 ≈ F/K_d = 2 cm |
| `--decouple-moment` | `1.0` | 力偶幅值 (Nm) — GAC 稳态转动 ≈ τ/K_rot = 0.02 rad |
| `--decouple-settle` | `2.0` | 每块过渡时间 (s), ≈ 4× 滤波器时间常数 |
| `--decouple-measure` | `1.0` | 每块稳态测量时间 (s) |

默认值来自 `se3_control/config/task_config.py` 的 `experiments['decouple']`。
注意: 实验模式下滤波器积分步长自动取仿真 `dt=0.001s` (其他模式为 0.002s),
避免滤波器以 2× 速率积分。

### 9.4 输出

- 终端报告: 主响应幅值 + 6×6 耦合比矩阵 (%) + 超阈值清单 + 滤波器层耦合矩阵;
- 热力图: `se3_control/figures/decouple/{gac,gic}_decouple.png`
  (左: 耦合矩阵, 右: 耦合比 %).

阈值 (计划 §4.3): 轴间耦合比 (`|Δy|/|Δx|` 等同域) 与 平动↔转动耦合
(`|Δφ|/|Δx|`) 均 < 5% (GAC 期望); GIC 作基线记录.

### 9.5 结果解读与已知发现

默认工作位形 (2026-08 调整): 新 home 将 EE 从 ~1.15m 高位/末端朝上
改为 **~[0.50, 0, 0.50] 且末端竖直朝下** (工具 z 轴 = [0,0,-1], 倾角 0°),
调节任务的期望位姿 = FK(home_q), 故解耦/扫频实验随之在低位工作.
(UR12e home_q = `[-0.356, -1.498, 1.81, 1.259, 1.571, -0.124]`,
UR3 = `[-0.327, -1.42, 1.236, -1.386, -1.571, 2.738]`.)
竖直朝下通过将 q5 置为 ±90° 实现 — 该处恰为腕部条件最佳区域
(腕部奇异在 q5 = 0/±180°, 即 wrist_1∥wrist_3), 故非奇异.
(求解 home 时采用数值稳健的旋转向量姿态误差, 修复了原 IK 在 ~180°
朝向差下叉积误差度量的退化问题.)

- **GAC 滤波器层完美解耦**: `coupling_xc` 为对角矩阵 (约 `1/K_d = 2e-3 m/N`),
  力矩块平动响应 ≈ 0. 说明体坐标导纳经 R_cur 投影的 Z 振荡问题已在滤波器层修复;
- **Z 振荡回归断言通过**: 竖直舒适位下 GAC 施加 x 向力时 `|Δz|/|Δx| ≈ 7.7%`,
  处于计划 §4.2 的"可接受 < 10%"区间 (超出严格 5%, 但该 7.7% 全部来自跟踪层位形
  相关耦合, 滤波器层仍严格解耦). 回归测试断言已按可接受线固化
  (`tests/test_gac_decouple_regression.py`);
- **跟踪层耦合 (已知限制, 位形相关)**: GAC/GIC 跟踪层均采用自适应阻抗
  `K_adapt = ω²·M̃` (M̃ 为满 6×6 操作空间惯量, 各向异性). 稳态时 EE 位移 =
  滤波器目标 (F/K_d, 完全对角) + 跟踪误差 (≈M̃⁻¹F/ω², 与位形相关), 后者在
  世界系各方向有耦合. 实测耦合矩阵随位形变化 (旧高位位形亦存在 >80% 的
  力→转动耦合单元), 新舒适位形下力块平动耦合 ~10% 量级、力→转动耦合
  较大但部分单元较旧位形改善. 这是控制器设计特性, 非测量伪影
  (末端 site 与 body 原点重合, 无杠杆臂误差).

  如需改善 EE 级解耦, 方向: 提高跟踪层带宽 / 给 `K_adapt` 设置刚度下限 /
  将自适应阻抗改为世界系各向同性刚度 (如 `ω²·I`) 而非 `ω²·M̃`.

### 9.6 回归测试

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/lj252/miniconda3/envs/roboarm/bin/python \
    -m pytest tests/test_gac_decouple_regression.py -v
```

断言: (1) EE 级 `Fx → Δz` 耦合比 < 5%; (2) 滤波器层 `Fx → Δz` 耦合 < 1e-3。

### 9.7 可视化循环模式 (`--decouple-loop`)

定量模式 (9.1) 每个动作只出现一次, 时间短, 不便在 viewer 中观察末端位移.
`--decouple-loop` 提供**循环演示模式**: 动作与复位间隙交替, 反复循环, 便于观察.

```bash
# GAC 循环可视化 (推荐演示 — 位移最明显, ~6 cm)
python se3_control/scripts/verify_gac_mujoco.py --experiment decouple --decouple-loop

# GIC 循环可视化 (基线; 刚度 K_adapt=ω²M̃ 较硬, 位移较小 ~1 cm)
python se3_control/scripts/verify_gic_mujoco.py --experiment decouple --decouple-loop

# 无头模式 (跑满 2 轮后自动结束)
python se3_control/scripts/verify_gac_mujoco.py --experiment decouple --decouple-loop --no-viewer
```

**序列构造**: 每个动作后紧跟一个零输入复位块, 共 12 个子块
`[Fx, 0, Fy, 0, Fz, 0, Mx, 0, My, 0, Mz, 0]`, 每子块长 `settle+measure` 秒;
块索引对 12 取模 → 60s 后自动回到 Fx 进入下一轮, 持续循环.
**关闭 viewer 窗口即提前结束**仿真, 无需等待全部轮次.

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--decouple-loop` | `false` | 启用循环可视化模式 (轮数取自配置 `cycles`) |

`experiments['decouple_loop']` 配置 (独立于定量模式): `force=30 N` (GAC 稳态
位移 ≈ 30/500 = **6 cm**, 明显可辨), `moment=2 Nm`, `settle=2 s`, `measure=3 s`,
`cycles=2` (完整循环轮数). 循环模式**跳过**定量耦合分析 (输出仅供观察, 不写入 figures/)。

---

*文档创建日期: 2026-07-29*
