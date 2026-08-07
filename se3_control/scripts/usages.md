# se3_control/scripts/ — 仿真验证脚本使用说明

> 关联: [run_se3_control_usage.md](../docs/run_se3_control_usage.md) | [verify_gac_mujoco_plan.md](../docs/verify_gac_mujoco_plan.md)

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
| `run_se3_control.py` | GIC (core 模块) | GIC 主验证入口，MuJoCo 仿真 | core 模块 |
| `verify_gic_mujoco.py` | GIC (内联) | GIC 交叉验证（Pinocchio vs MuJoCo），含绘图 | 外部 GUFIC 库 |
| `verify_gac_mujoco.py` | GAC (core 模块) | GAC 导纳控制验证，5 种外力模式 | core 模块 |

### 选择建议

```
新用户 / 日常使用  →  run_se3_control.py  (精简, core 模块导入)
GIC 精度验证      →  verify_gic_mujoco.py  (交叉验证 + 绘图)
GAC 导纳验证      →  verify_gac_mujoco.py  (外力模拟 + 切向跟随)
```

---

## 3. run_se3_control.py — GIC 仿真主入口

基于已抽离的 `core/` 模块（`se3_math` / `trajectory` / `gic_controller`）运行 MuJoCo 物理仿真。
适合日常调参和快速验证。

```bash
# 默认运行 (UR12e, regulation, 可视化)
python se3_control/scripts/run_se3_control.py

# 不同机器人 + 轨迹
python se3_control/scripts/run_se3_control.py --robot ur3   --task circle
python se3_control/scripts/run_se3_control.py --robot ur12e --task line

# 无头模式 (SSH/服务器)
python se3_control/scripts/run_se3_control.py --task circle --no-viewer

# 自定义控制器参数
python se3_control/scripts/run_se3_control.py --robot ur3 --bandwidth 20 --damping 1.0
```

**主要参数**：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--robot` | `ur12e` | `ur12e` / `ur3` / `franka` |
| `--task` | `regulation` | `regulation` / `circle` / `line` |
| `--max-time` | `5.0` | 仿真时长 (秒) |
| `--bandwidth` | 来自 config | GIC 带宽 ω_des (rad/s) |
| `--damping` | 来自 config | GIC 阻尼比 ζ |
| `--no-viewer` | — | 关闭可视化（无头模式） |
| `--save-plot` | None | 保存结果图到文件 |
| `--cross-validate` | — | 仅运行模型交叉验证 |
| `--no-loop` | — | 关闭连续循环模式 |

**详细文档**：[run_se3_control_usage.md](../docs/run_se3_control_usage.md)

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
# 注: 使用 惯性系 3-DOF 导纳 (位置修正), 不与 R_cur 耦合,
#     完全避免末端倾斜导致的 Z 方向漂移和振荡.
#     详见下方 §7 "常见错误与调试".
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
| `tangent` | 沿圆弧切向力 + 径向虚拟弹簧 | 惯性系导纳(位置), 无 R_cur 耦合, 无 Z 漂移 | 柔顺跟随 |

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
1. run_se3_control.py --task regulation --no-viewer
   → GIC 基础功能验证

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

---

*文档创建日期: 2026-07-29*
