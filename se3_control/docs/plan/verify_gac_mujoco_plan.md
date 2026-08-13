# verify_gac_mujoco.py — MuJoCo + Pinocchio 联合验证 GAC 导纳控制计划

> 关联: [GAC_plan.md](./GAC_plan.md) | [run_se3_control_usage.md](../usages/run_se3_control_usage.md)
> 下游: `se3_control/scripts/verify_gac_mujoco.py`

---

## 0. 背景

现有 `verify_gic_mujoco.py`（1474 行）已完成 GIC 阻抗控制器的 MuJoCo 仿真验证。
现在需要为 GAC 导纳控制器编写对等的验证脚本。

### 与 verify_gic_mujoco.py 的核心差异

| 维度 | GIC 版本 | GAC 版本（本计划） |
|---|---|---|
| 控制器 | `GICController` | `GACController` |
| 外力输入 | 无（阻抗被动） | **可施加 F_ext**（导纳主动） |
| 验证重点 | 轨迹跟踪精度 | 外力响应 + 退化模式 + 轨迹跟踪 |
| 测试模式 | regulation / circle / line | 同上 + **外力扰动模式** |
| 依赖模块 | `core/gic_controller.py` | `core/gac_controller.py` |
| 控制器构造 | 不需要导纳参数 | 需要 M_d, D_d, K_d, dt |

### 共享基础设施

两者共用 90% 的代码：
- URDF → MuJoCo XML 转换
- RobotModel (Pinocchio) 运动学/动力学
- MuJoCo 物理推演、可视化、轨迹记录
- 绘图与结果分析
- 交叉验证（Pinocchio vs MuJoCo）

目标：**在复用基础设施的同时，保持 GIC 和 GAC 验证脚本相互独立**（不互相 import）。

---

## 1. 架构方案

```
┌──────────────────────────────────────────────────────────┐
│                    verify_gac_mujoco.py                   │
│                                                          │
│  ┌──────────────┐  ┌──────────────────┐  ┌────────────┐ │
│  │ URDF→MuJoCo  │  │  RobotModel      │  │ 绘图/记录  │ │
│  │ XML 转换      │  │  (Pinocchio)     │  │           │ │
│  └──────────────┘  └──────────────────┘  └────────────┘ │
│         ↓                    ↓                            │
│  ┌──────────────────────────────────────────────────────┐│
│  │              MuJoCo 物理引擎                           ││
│  │  (前向动力学: τ → q, dq, p, R, Vb)                  ││
│  └──────────────────────────────────────────────────────┘│
│         ↑                    ↑                            │
│  ┌──────────────────────────────────────────────────────┐│
│  │              GACController                            ││
│  │  导纳滤波 → 轨迹修正 → M_tilde 位置跟踪 → τ         ││
│  │                                                        ││
│  │  F_ext 模拟方式:                                      ││
│  │    a) 零外力 (退化模式, 与 GIC 对比)                  ││
│  │    b) 体坐标系恒定外力 (模拟环境接触)                   ││
│  │    c) 脉冲外力 (模拟冲击响应)                          ││
│  │    d) 弹簧环境 (F_ext = K_env · Δx, 模拟刚性接触)    ││
│  └──────────────────────────────────────────────────────┘│
└──────────────────────────────────────────────────────────┘
```

### 独立原则

```
❌ verify_gac_mujoco.py 不 import verify_gic_mujoco.py
✅ 两者共享 URDF→XML 函数 → 提取到 core/utils.py 或各自保留独立副本
✅ GACController 不依赖 GICController（已确认零耦合）
```

### 代码复用策略

`verify_gic_mujoco.py` 中有大量可复用的函数。为保持两个验证脚本独立，采用**独立副本**策略：

| 函数 | GIC 版本 | GAC 版本 |
|---|---|---|
| `urdf_joints_to_mujoco_xml()` | 内联 | **复制副本**（修改为从 `core/` 导入 `se3_math`） |
| `build_trajectory_from_config()` | 内联 | **直接复用** `core.trajectory.build_trajectory()` |
| `_rotmat_slerp()` | 内联 | **直接复用** `core.se3_math.rotmat_slerp()` |
| `run_verification()` | 含 GIC 控制 | **重写为 GAC 控制版本** |
| `plot_results()` | 内联 | **复制副本**（内容相同，标题改为 GAC） |
| `cross_validate_models()` | 内联 | **复制副本**（内容完全相同） |

保留独立副本的理由：
- 两个脚本各自 .py 文件完整自包含，调试时无需交叉引用
- 未来升级互不影响
- 共享代码在 ~100 行左右，重复代价小

---

## 2. GAC 专属功能：外力模拟

### 2.1 外力模式

```python
class ForceProfile:
    """外力配置文件 — 定义 F_ext(t) 的时间序列."""

    @staticmethod
    def zero(t):
        """零外力 — 用于退化模式验证."""
        return np.zeros(6)

    @staticmethod
    def constant(t, force=[10.0, 0, 0, 0, 0, 0]):
        """恒定外力 — 验证稳态响应."""
        return np.array(force, dtype=float)

    @staticmethod
    def pulse(t, start=0.5, duration=0.2, amplitude=[20.0, 0, 0, 0, 0, 0]):
        """脉冲外力 — 验证动态响应和恢复."""
        if start <= t <= start + duration:
            return np.array(amplitude, dtype=float)
        return np.zeros(6)

    @staticmethod
    def spring_contact(t, p_ee, p_surface, stiffness=1000.0):
        """模拟刚性表面接触: F = K_env · penetration.

        :param p_ee: 当前末端位置 (3,)
        :param p_surface: 接触表面位置 (3,)
        :param stiffness: 环境刚度 (N/m)
        :returns: 体坐标系接触力 (6,)
        """
        penetration = p_surface - p_ee
        if penetration[2] > 0:  # 只在 z 方向接触
            f_ext = np.zeros(6)
            f_ext[2] = stiffness * penetration[2]
            return f_ext
        return np.zeros(6)

    @staticmethod
    def tangential(t, p_ee, center=[0.5, 0.0, 0.125],
                   radius=0.1, speed=1.0, amplitude=10.0):
        """沿圆弧切线方向的恒力 — 验证各向异性导纳.

        计算末端当前位置相对于圆心的角度 θ,
        在切向施加恒力 F_tangent, 径向不施力.
        预期: 机器人沿圆弧柔顺滑动, 径向无偏移.

        数学:
          θ = atan2(p_ee.y - center.y, p_ee.x - center.x)
          tangent  = [-sin(θ),  cos(θ), 0]   × amplitude
          normal   = [ cos(θ),  sin(θ), 0]   (不施力)

        :param p_ee: 当前末端位置 (3,)
        :param center: 圆心 [cx, cy, cz]
        :param radius: 圆半径 (m)
        :param speed: 轨迹速度 (rad/s)
        :param amplitude: 切向力幅值 (N)
        :returns: 惯性系外力 (6,)
        """
        c = np.array(center, dtype=float)
        dx = p_ee[0] - c[0]
        dy = p_ee[1] - c[1]
        theta = np.arctan2(dy, dx)
        # 切向: [-sin(θ), cos(θ), 0]
        f_dir = np.array([-np.sin(theta), np.cos(theta), 0.0])
        f_ext = np.zeros(6)
        f_ext[:3] = f_dir * amplitude
        return f_ext
```

### 2.2 CLI 参数

```python
# 新增参数
parser.add_argument('--force-mode', type=str, default='zero',
                    choices=['zero', 'constant', 'pulse', 'spring',
                             'tangent'],
                    help='External force mode for admittance verification')
parser.add_argument('--force-amplitude', type=float, nargs=6,
                    default=[10.0, 0, 0, 0, 0, 0],
                    help='Force amplitude [fx, fy, fz, tx, ty, tz]')
parser.add_argument('--force-start', type=float, default=1.0,
                    help='Force start time (s)')
parser.add_argument('--force-duration', type=float, default=0.5,
                    help='Force duration (s) for pulse mode')

# 切向力模式参数 (force_mode=tangent)
parser.add_argument('--tangent-circle-center', type=float, nargs=3,
                    default=[0.5, 0.0, 0.125],
                    help='Circle center for tangent force [cx, cy, cz]')
parser.add_argument('--tangent-radius', type=float, default=0.1,
                    help='Circle radius for tangent force (m)')
parser.add_argument('--tangent-amplitude', type=float, default=10.0,
                    help='Tangential force amplitude (N)')

# 导纳参数
parser.add_argument('--M-d', type=float, nargs=6,
                    default=[10.0, 10.0, 10.0, 1.0, 1.0, 1.0],
                    help='Virtual mass [m, m, m, I, I, I]')
parser.add_argument('--D-d', type=float, nargs=6,
                    default=None,  # 默认根据 K_d 和 M_d 计算临界阻尼
                    help='Virtual damping')
parser.add_argument('--K-d', type=float, nargs=6,
                    default=[500.0, 500.0, 500.0, 50.0, 50.0, 50.0],
                    help='Virtual stiffness')
```

### 2.3 控制循环中的外力注入

```python
# 在控制循环中:
if args.force_mode == 'zero':
    F_ext = ForceProfile.zero(t)
elif args.force_mode == 'constant':
    F_ext = ForceProfile.constant(t, force=args.force_amplitude)
elif args.force_mode == 'pulse':
    F_ext = ForceProfile.pulse(t, start=args.force_start,
                                duration=args.force_duration,
                                amplitude=args.force_amplitude)
elif args.force_mode == 'spring':
    # 需要知道接触面位置
    F_ext = ForceProfile.spring_contact(t, p_ee, p_surface)
elif args.force_mode == 'tangent':
    # 沿圆弧切向施力 (位置相关, 需传入当前 p_ee)
    F_ext = ForceProfile.tangential(t, p_ee,
                center=args.tangent_circle_center,
                radius=args.tangent_radius,
                amplitude=args.tangent_amplitude)

# 注意: tangent 模式下 F_ext 在惯性系中计算,
# 而 GACController 期望 F_ext 在体坐标系.
# 需要将惯性系力旋转到体坐标系:
# F_ext_body = Rᵀ @ F_ext[:3]   (拼接到 6 维)
if args.force_mode == 'tangent':
    F_ext_body = np.zeros(6)
    F_ext_body[:3] = R_cur.T @ F_ext[:3]
else:
    F_ext_body = F_ext

# 传给 GACController
tau_cmd = controller.compute(q, dq, pd, Rd, vd, wd, dvd, dwd, F_ext=F_ext)
```

---

## 3. run_verification() 接口设计

```python
def run_verification(robot_urdf, task='regulation',
                     show_viewer=True, max_time=5.0,
                     home_q=None, ee_frame='tool0',
                     link_to_mesh=None, mesh_subdir='',
                     torque_limits=None,
                     # ── GAC 专属参数 ──
                     M_d=None, D_d=None, K_d=None, dt_filter=0.002,
                     force_mode='zero',
                     force_amplitude=None,
                     force_start=1.0, force_duration=0.5,
                     bandwith=30.0, damping=1.0,
                     verbose=True, stop_at_end=True, loop=False):
    """GAC 控制验证主循环.

    步骤与 GIC 版本相同，区别:
      1. 控制器使用 GACController 而非 GICController
      2. 控制循环中注入 F_ext（根据 force_mode）
      3. F_ext=zero 时退化为位置跟踪，应与 GIC 行为一致
    """
```

---

## 4. 验证场景

### 4.1 退化模式验证 (force_mode=zero)

**目的**：确认 F_ext=0 时 GAC 行为等价于 GIC。

```bash
# 与 GIC 版本使用相同参数, 结果应一致
python se3_control/scripts/verify_gic_mujoco.py --robot ur12e --task circle --no-viewer
python se3_control/scripts/verify_gac_mujoco.py --robot ur12e --task circle --no-viewer
```

### 4.2 恒力响应 (force_mode=constant)

**目的**：观察外力下末端位置偏移。

```bash
# 10N 恒力, 期望位置被推开约 2cm (F/K_d = 10/500 = 0.02m)
python se3_control/scripts/verify_gac_mujoco.py --task regulation \
    --force-mode constant --force-amplitude 10 0 0 0 0 0 --no-viewer
```

### 4.3 脉冲响应 (force_mode=pulse)

**目的**：观察动态响应和恢复。

```bash
# 1 秒时施加 0.2 秒 20N 脉冲
python se3_control/scripts/verify_gac_mujoco.py --task regulation \
    --force-mode pulse --force-amplitude 20 0 0 0 0 0 \
    --force-start 1.0 --force-duration 0.2 --no-viewer
```

### 4.4 轨迹跟踪 + 外力扰动

**目的**：跟踪 circle 同时施加外力扰动，观察稳定性。

```bash
python se3_control/scripts/verify_gac_mujoco.py --task circle \
    --force-mode constant --force-amplitude 5 0 0 0 0 0 --no-viewer
```

### 4.5 自定义导纳参数

**目的**：验证不同 M_d/D_d/K_d 下的响应差异。

```bash
# 软导纳 (低刚度)
python se3_control/scripts/verify_gac_mujoco.py --task regulation \
    --K-d 100 100 100 10 10 10 \
    --force-mode constant --force-amplitude 10 0 0 0 0 0 --no-viewer

# 硬导纳 (高刚度, 接近位置控制)
python se3_control/scripts/verify_gac_mujoco.py --task regulation \
    --K-d 2000 2000 2000 200 200 200 \
    --force-mode constant --force-amplitude 10 0 0 0 0 0 --no-viewer
```

### 4.6 切向力柔顺跟随 (force_mode=tangent)

**目的**：验证 GAC 的**各向异性导纳**——末端沿圆弧切向柔顺滑动，径向保持刚度。

**原理**：
```
    F_tangent
    ────→
    ↑ ·  ────→  末端沿切向柔顺跟随
    │ 圆心    径向(高刚度): 保持距离不变
    └───→
```

在 X-Y 平面上定义一个圆心 `(cx, cy, cz)` 和半径 `r`。
在 regulation 模式下，末端初始位置停在圆弧上某一点，然后施加沿切线的恒力。
切向方向力使末端沿圆弧滑动，径向方向不施力（利用 K_d 刚度保持）。

```bash
# 基本切向跟随: 10N 切向力, 验证沿圆弧滑动
python se3_control/scripts/verify_gac_mujoco.py --task regulation \
    --force-mode tangent --tangent-amplitude 10 \
    --tangent-circle-center 0.5 0.0 0.125 --tangent-radius 0.1 --no-viewer
```

**预期行为**：
- 末端沿圆弧缓慢滑动（切向柔顺）
- 径向偏移 < 5mm（径向刚度保持）
- 末端轨迹落在以圆心为基准的圆弧上

**各向异性导纳参数建议**：

```bash
# 切向柔顺 (低切向刚度) + 径向刚性 (高径向刚度)
python se3_control/scripts/verify_gac_mujoco.py --task regulation \
    --force-mode tangent --tangent-amplitude 10 \
    --K-d 2000 2000 500 50 50 50 \
    --D-d 283 283 141 14 14 14 \
    --no-viewer
```

> 注意: K_d[:2] 设高（径向/切向通用刚度）但通过 M_d/D_d 调节切向响应速度。
> 实际各向异性导纳需要 GACController 支持方向依赖的 K_d 矩阵。
> 当前实现使用对角阵 K_d，各向异性通过 M_d/D_d 的组合实现。

**与 GIC 的对比**（展示导纳的独特价值）：

```bash
# GIC 无法实现 — 阻抗控制没有外力输入, 不会对切向力产生跟随
# GAC 独有功能
```

---

## 5. 文件清单

| 文件 | 操作 | 内容 |
|---|---|---|
| `se3_control/scripts/verify_gac_mujoco.py` | **新建** | GAC 版本验证入口，~800 行 |

相较于 GIC 版本（~1474 行），GAC 版本更短的原因是：
- 轨迹生成用 `core.trajectory.build_trajectory()`（无需内联 `build_trajectory_from_config`）
- SE(3) 数学用 `core.se3_math`（无需内联 `hat_map`, `vee_map`, `adjoint_g_ed`）
- 控制器用 `core.gac_controller.GACController`（无需内联 GICController 类）
- SLERP 用 `core.se3_math.rotmat_slerp`（无需内联 `_rotmat_slerp`）

---

## 6. 实施步骤

| 步骤 | 内容 | 预期 |
|---|---|---|
| 1 | 创建 `verify_gac_mujoco.py` 框架 | 导入 core 模块，解析参数，复制 XML 转换和绘图代码 |
| 2 | 实现 ForceProfile | 4 种外力模式可切换 |
| 3 | 实现 GAC 版 run_verification() | 控制循环中注入 F_ext |
| 4 | 测试退化模式 | `F_ext=0` 时结果与 GIC 一致 |
| 5 | 测试恒力模式 | 末端偏移量 ≈ F/K_d |
| 6 | 测试脉冲模式 | 动态响应无震荡 |
| 7 | 交叉验证 | Pinocchio vs MuJoCo 一致性 |

---

## 7. 验证标准

| 测试场景 | 条件 | 预期 |
|---|---|---|
| 退化 regulation | F_ext=0, regulation | 与 GIC 偏差 < 1e-8 |
| 退化 circle | F_ext=0, circle | 与 GIC 跟踪误差一致 |
| 恒力 regulation | 10N 恒力 | 稳态偏移 ≈ 0.02m (F/K) |
| 脉冲 regulation | 20N · 0.2s | 位置偏移后恢复 |
| 不同导纳参数 | K_d 增大 10× | 偏移量减小 ≈ 10× |
| spring 接触 | 末端接触刚性面 | 稳定接触力，无力震荡 |
| **切向跟随** | 10N 切线恒力, regulation | **末端沿圆弧滑动**, 径向误差 < 5mm |
| 各向异性导纳 | 切向低刚度 + 径向高刚度 | 沿切线滑动, 径向无偏移 |

---

*文档创建日期: 2026-07-29*
