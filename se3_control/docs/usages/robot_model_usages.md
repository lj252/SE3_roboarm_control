# SE(3) Roboarm Control

Pinocchio 驱动的 SE(3) 几何阻抗控制（GIC）框架，支持 MuJoCo 仿真验证与实机部署。

## 项目结构

```
SE3_roboarm_control/
├── README.md                          # 本文件 — 使用说明
├── README/                            # 前置知识文档
│   └── 代码中的前置知识.md
├── docs/                              # 部署方案文档
│   ├── deploy_se3_gic_to_ur12_plan.md
│   └── deploy_se3_to_hardware_plan.md
├── se3_control/
│   ├── robot_model/                   # ★ RobotModel — Pinocchio 运动学/动力学封装
│   │   ├── __init__.py
│   │   └── robot_model.py
│   ├── scripts/
│   │   ├── verify_gic_mujoco.py       # MuJoCo 仿真验证脚本
│   │   └── __pycache__/
│   ├── urdf/                          # 机器人 URDF 模型文件
│   │   ├── ur12e.urdf                 # UR12e 模型（6-DOF）
│   │   └── franka_panda.urdf          # Franka Panda 模型
│   └── hard_ware/                     # (占位) 实机硬件接口
├── tracking_circle.png               # 圆轨迹跟踪结果图
└── tracking_regulation.png           # 调节控制结果图
```

---

## RobotModel 使用说明

### 概述

`RobotModel` 是基于 **Pinocchio 4.0** 的机器人运动学/动力学计算封装。它的设计目标：

1. **替代 MuJoCo 的 RobotState**，让 SE(3) 控制律可以脱离仿真器运行
2. **对齐 GUFIC 代码库的 RobotState 接口**，上层控制代码无需修改即可切换
3. **同时支持仿真验证与实机部署**——仿真中用 MuJoCo 做物理推演，实机中只用 Pinocchio 做正逆运动学计算

### 安装与依赖

```bash
# 1. 创建 conda 环境（Python 3.10）
conda create -n roboarm python=3.10
conda activate roboarm

# 2. 安装 Pinocchio 4.0
conda install -c conda-forge pinocchio=4.0.0

# 3. 安装其他依赖
pip install numpy scipy matplotlib

# 4. （可选）MuJoCo 仿真验证
pip install mujoco
```

### 基本用法

#### 1. 加载模型

```python
from robot_model import RobotModel

# 加载 URDF，指定末端执行器 frame 名称
model = RobotModel(
    urdf_path="se3_control/urdf/ur12e.urdf",
    ee_frame_name="tool0",         # 对应 URDF 中的 link 或 frame 名
    robot_name="UR12e",
    verbose=True
)
```

#### 2. 正运动学（Forward Kinematics）

```python
import numpy as np

# 设定关节角度（rad）— 默认舒适位形 (EE≈[0.50,0,0.50], 末端竖直朝下)
q = np.array([-0.356, -1.498, 1.81, 1.259, 1.571, -0.124])
dq = np.zeros(6)

# 更新计算
model.update(q, dq)

# 获取末端位姿
p, R = model.get_pose()
print(f"位置: {p}")        # (3,) — 世界坐标系
print(f"朝向 (SO(3)):\n{R}")  # (3,3) — 旋转矩阵
```

#### 3. 雅可比矩阵

```python
# 空间雅可比 Js (6×nv)：将关节速度映射到世界系末端速度
Js = model.get_jacobian()

# 体雅可比 Jb (6×nv)：将关节速度映射到体坐标系末端速度
# Jb = diag(Rᵀ, Rᵀ) @ Js
Jb = model.get_body_jacobian()

# 体速度（末端自身坐标系下的 twist）
Vb = model.get_body_ee_velocity()   # (6,1): [vx, vy, vz, ωx, ωy, ωz]ᵀ

# 空间速度（世界坐标系下的 twist）
Vs = model.get_spatial_ee_velocity()  # (6,1)
```

#### 4. 动力学

```python
# 关节空间惯性矩阵 M(q) ∈ ℝⁿᵛˣⁿᵛ（CRBA 算法）
M = model.get_full_inertia()

# 偏置力矩（重力 + 科氏力/离心力）
# 相当于 RNEA(q, dq, 0)
bias = model.get_bias_torque()  # (nv,)

# 也可分别获取
g = model.get_gravity()                  # 重力力矩 g(q)
C = model.get_coriolis_matrix()          # 科氏力矩阵 C(q, dq)
```

#### 5. 逆运动学（Inverse Kinematics）

```python
# 期望位姿
pd = np.array([0.4, 0.0, 0.3])
Rd = np.eye(3)

# 初始关节猜测 (默认舒适位形)
q_init = np.array([-0.356, -1.498, 1.81, 1.259, 1.571, -0.124])

# 高斯-牛顿法（Levenberg-Marquardt）求解
q_solution = model.gauss_newton_IK(
    pd, Rd, q_init,
    step_size=0.5,    # 迭代步长
    tol=1e-3,         # 收敛容差
    max_cnt=200       # 最大迭代次数
)

# 验证解
model.update(q_solution)
p_ik, R_ik = model.get_pose()
pos_err = np.linalg.norm(p_ik - pd)
print(f"IK 位置误差: {pos_err:.6f} m")
```

### 完整示例

```python
from robot_model import RobotModel
import numpy as np

# ---------- 加载 ----------
model = RobotModel("se3_control/urdf/ur12e.urdf", ee_frame_name="tool0")

# ---------- 随机初始状态 ----------
np.random.seed(42)
q0 = np.random.rand(model.nq) * 0.5 - 0.25
dq0 = np.random.rand(model.nv) * 0.1 - 0.05

model.update(q0, dq0)

# ---------- 正运动学 ----------
p, R = model.get_pose()
print(f"位置: {p}")
print(f"朝向:\n{R}")

# ---------- 雅可比 ----------
Js = model.get_jacobian()
Jb = model.get_body_jacobian()
print(f"空间雅可比: {Js.shape}")
print(f"体雅可比:   {Jb.shape}")

# ---------- 动力学 ----------
M = model.get_full_inertia()
bias = model.get_bias_torque()
print(f"惯性矩阵:\n{M}")
print(f"偏置力矩: {bias}")

# ---------- 逆运动学 ----------
pd_des = p + np.array([0.05, 0.03, -0.02])
Rd_des = R.copy()
q_ik = model.gauss_newton_IK(pd_des, Rd_des, q0)

model.update(q_ik)
p_ik, R_ik = model.get_pose()
print(f"IK 位置误差: {np.linalg.norm(p_ik - pd_des):.6f}")
```

### 自检模式

直接运行 `robot_model.py` 可执行快速自检：

```bash
# 使用 UR12e 模型
python se3_control/robot_model/robot_model.py se3_control/urdf/ur12e.urdf tool0

# 使用 Franka Panda 模型
python se3_control/robot_model/robot_model.py se3_control/urdf/franka_panda.urdf panda_hand_tcp
```

### API 参考

#### 构造方法

| 方法 | 说明 |
|------|------|
| `RobotModel(urdf_path, ee_frame_name, robot_name, verbose)` | 加载 URDF 模型，初始化 Pinocchio 数据 |

**参数**：

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `urdf_path` | str | — | URDF 文件路径（相对/绝对均可） |
| `ee_frame_name` | str | `"end_effector"` | 末端执行器 frame 名称 |
| `robot_name` | str | `"generic"` | 机器人名（仅日志） |
| `verbose` | bool | `True` | 是否打印加载信息 |

**特性**：
- 支持名称模糊匹配：如果精确名称找不到，会自动搜索包含该名称的 frame
- 支持回退：如果找不到指定 frame，默认选择最后一个 OP_FRAME
- 尝试加载几何模型（碰撞/可视化），失败时回退纯运动学加载

#### 核心方法

| 方法 | 返回 | 对标 MuJoCo | 说明 |
|------|------|-------------|------|
| `update(q, dq)` | `self` | `mj_step1` + `mj_rnePostConstraint` | 更新正运动学与动力学计算 |
| `get_pose()` | `(p, R)` | `site_xpos` + `site_xmat` | 末端位置 (3,) 与朝向 (3,3) |
| `get_jacobian()` | `(6, nv)` | `mj_jacSite` | 几何雅可比（世界坐标系） |
| `get_body_jacobian()` | `(6, nv)` | `get_body_jacobian()` | 体雅可比（末端坐标系） |
| `get_body_ee_velocity()` | `(6, 1)` | `get_body_ee_velocity()` | 体 twist = Jb @ dq |
| `get_spatial_ee_velocity()` | `(6, 1)` | `get_spatial_ee_velocity()` | 空间 twist = Js @ dq |
| `get_full_inertia()` | `(nv, nv)` | `mj_fullM` | CRBA 惯性矩阵 M(q) |
| `get_bias_torque()` | `(nv,)` | `qfrc_bias` | 偏置力矩（重力+科氏力） |
| `get_gravity()` | `(nv,)` | `qfrc_grav` | 重力力矩 g(q) |
| `get_coriolis_matrix()` | `(nv, nv)` | — | 科氏力矩阵 C(q, dq) |
| `gauss_newton_IK(pd, Rd, init_q, ...)` | `(nv,)` | 功能等价 | LM 法逆运动学求解 |
| `get_joint_pose()` | `(nv,)` | `data.qpos` | 当前关节角度 |
| `get_joint_velocity()` | `(nv,)` | `data.qvel` | 当前关节速度 |
| `get_num_joints()` | int | — | 自由度数 |
| `get_timestep()` | float | `model.opt.timestep` | 控制周期（默认 2ms） |

#### 辅助方法

| 方法 | 说明 |
|------|------|
| `print_frame_list()` | 打印模型所有可用 frame 名称及索引（调试用） |
| `set_ee_force(fe)` | 由硬件接口注入力传感器读数 |
| `get_ee_force()` | 获取末端力/力矩（占位，实机时由硬件接口覆盖） |

---

## 仿真验证（MuJoCo）

`verify_gic_mujoco.py` 使用 MuJoCo 做物理推演 + Pinocchio 做控制计算，验证 GIC 控制律的可行性。

### 运行

```bash
# 确保在项目根目录
cd SE3_roboarm_control

# 默认运行：可视化 + UR12e + 调节任务 (regulation)
python se3_control/scripts/verify_gic_mujoco.py

# 无头模式（SSH / 服务器环境，不弹出可视化窗口）
python se3_control/scripts/verify_gic_mujoco.py --no-viewer

# 画圆任务 (默认 viewer 中连续循环, 可随时关闭窗口暂停)
python se3_control/scripts/verify_gic_mujoco.py --task circle

# 关闭连续循环, 结束后停在最终位姿
python se3_control/scripts/verify_gic_mujoco.py --task circle --no-loop

# 圆轨迹跟踪 + 保存结果图 (无头模式)
python se3_control/scripts/verify_gic_mujoco.py --no-viewer --no-loop --task circle --save-plot circle_result.png

# Franka Panda 线轨迹
python se3_control/scripts/verify_gic_mujoco.py --robot franka --task line

# 仅做模型交叉验证（不运行控制仿真）
python se3_control/scripts/verify_gic_mujoco.py --cross-validate
```

### 可视化界面

可视化默认打开，使用 MuJoCo 的被动模式（`mujoco.viewer.launch_passive`）实时显示机器人的运动：

- **交互操作**：在可视化窗口中拖拽视角（鼠标左键旋转、滚轮缩放、右键平移）
- **无头模式**：在无 GUI 的服务器上运行时，添加 `--no-viewer` 关闭可视化
- **性能影响**：可视化会略微降低仿真速度（每 5 步同步一次画面，影响很小）

仿真结束后，默认行为取决于任务类型：

- **Regulation 任务**：停在最终位姿，方便检查。添加 `--no-stop` 可立即退出。
- **Circle/Line 任务**：打开 viewer 时自动**连续循环运行**，机械臂持续画圆/直线，方便观察。关闭窗口或按 MuJoCo viewer 的 pause 按钮可停止。添加 `--no-loop` 可关闭循环。

### 命令行参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--robot` | `str` | `ur12e` | 机器人模型 (`ur12e` / `franka`) |
| `--task` | `str` | `regulation` | 轨迹任务 (`regulation` / `circle` / `line`) |
| `--max-time` | `float` | `5.0` | 仿真时长（秒） |
| `--no-viewer` | `flag` | `False` | 关闭可视化（默认开启） |
| `--no-stop` | `flag` | `False` | 仿真结束后不暂停（默认暂停在最终位姿） |
| `--no-loop` | `flag` | `False` | 关闭连续循环（默认 circle/line 任务开启 viewer 时持续运行） |
| `--save-plot` | `str` | `None` | 保存结果图到文件 |
| `--cross-validate` | `flag` | `False` | 仅做模型交叉验证，不运行控制 |

### 验证内容

| 项目 | 指标 | 状态 |
|------|------|------|
| 正运动学交叉验证 | 4e-11 m 位置匹配 | ✅ |
| 雅可比矩阵交叉验证 | 2e-11 数值匹配 | ✅ |
| 动力学交叉验证 | 1e-8 相对误差 | ✅ |
| 调节任务（Regulation） | 零稳态误差 | ✅ |
| 圆轨迹跟踪 | 7.2 mm 均值 / 10.4 mm 最大误差 | ✅ |

### 输出

- `tracking_regulation.png` — 调节任务结果图
- `tracking_circle.png` — 圆轨迹跟踪结果图

---

## 关键技术说明

### 雅可比约定

| 坐标系 | 方法 | 表达式 | 用途 |
|--------|------|--------|------|
| 世界系 | `get_jacobian()` | Js: [ṗ; ω] = Js @ dq | 可视化、IK |
| 体坐标系 | `get_body_jacobian()` | Jb = diag(Rᵀ, Rᵀ) @ Js | 控制律 |

Pinocchio 4.0 的 `getFrameJacobian(..., pin.WORLD)` 返回空间雅可比（线速度分量关于世界坐标系原点而非末端点），需要通过伴随变换 `[[I, -p̂], [0, I]]` 转换为几何雅可比。

### 自适应增益

GIC 控制器使用自适应刚度/阻尼：

```python
M_tilde = (Jb @ M⁻¹ @ Jbᵀ)⁻¹     # 操作空间惯性矩阵
K_adapt = ω² · M_tilde              # 刚度：高惯性→高刚度
D_adapt = 2ζω · M_tilde            # 阻尼：随惯性自适应缩放
```

这样在末端执行器的 6 个自由度上获得一致的闭环动力学特性，解决了腕部关节（M≈0.0003 kg·m²）与平移自由度（M≈15-100 kg）之间 10⁵ 倍的惯性差异问题。

### URDF → MJCF 转换

MuJoCo 的 body 坐标系定义与 URDF 不同。`verify_gic_mujoco.py` 中的 `urdf_joints_to_mujoco_xml()` 函数处理了以下差异：

| 差异 | 处理方式 |
|------|---------|
| 惯性帧旋转 | 将惯性张量旋转到 body 系: `R @ I_inertial @ Rᵀ` |
| 基座固定变换 | 追踪从 world 到 root_body 的累计固定变换 |
| joint 层级 | joint 作为 body 的子元素（非兄弟） |
| armature/damping | 移除默认值以避免干扰 Pinocchio 对标 |

---

## 索引

- [代码中的前置知识](../../../README/代码中的前置知识.md) — 李群、SE(3)、能量油箱等数学背景
- [部署方案 — GIC-only](../../../docs/deploy_se3_gic_to_ur12_plan.md) — 纯阻抗控制实机部署计划
- [部署方案 — GUFIC](../../../docs/deploy_se3_to_hardware_plan.md) — 统一力-阻抗控制实机部署计划
