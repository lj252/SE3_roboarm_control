# GAC 导纳控制核心实现计划 — Phase 2.5

> 在 GIC 阻抗控制的基础上，实现 SE(3) 导纳控制（Admittance Control），完成阻抗-导纳对偶闭环
> 关联文档: [GIC_plan.md](./GIC_plan.md) | [deploy_se3_to_hardware_plan.md](../../../docs/deploy_se3_to_hardware_plan.md)

---

## 0. 当前状态分析

### 现状：有纯阻抗（GIC），没有对偶的导纳

```
core/
├── se3_math.py            # SE(3) 数学工具          ✅ 已实现
├── trajectory.py           # 轨迹生成                ✅ 已实现
├── gic_controller.py      # GIC 阻抗控制            ✅ 已实现
└── gufic_controller.py    # GUFIC 力-阻抗控制        ❌ 未实现（预留）
```

**缺失**: 阻抗的对偶——**导纳控制**。Hogan 原则要求环境和控制器不能同时为阻抗或同时为导纳。

### GAC 要解决的问题

| 场景 | 纯 GIC | GAC |
|---|---|---|
| 自由空间轨迹跟踪 | ✅ 优秀 | ✅ 优秀（F_ext=0 退化） |
| 人推机器人拖动 | ❌ 僵硬 | ✅ 力→位置修正，主动跟随 |
| 精密装配（轴孔） | ⚠️ 被动柔顺不确定性 | ✅ 力测量→主动修正轨迹 |
| 刚性表面接触 | ❌ 两者皆阻抗 → 震荡/弹开 | ✅ 导纳响应外力的位置修正 |
| 变刚度需求 | ❌ 需停机调参 | ✅ 在线调整 M_d/D_d/K_d |

---

## 1. 核心设计原则：GIC 与 GAC 是对等的独立模块

### 架构定位

```
┌─────────────────────────────────────────┐
│           应用层 (任务/轨迹)              │  ← 机器人无关
├─────────────────────────────────────────┤
│  SE(3) 控制核心 (GIC / GAC / GUFIC)     │  ← 机器人无关
│  ┌──────────┐   ┌──────────┐   ┌──────┐ │
│  │   GIC    │   │   GAC    │   │ GUFIC│ │  ← 对等的独立模块
│  │  阻抗    │   │  导纳    │   │统一  │ │
│  └──────────┘   └──────────┘   └──────┘ │
│        ↓共享          ↓共享              │
│  ┌──────────────────────────────────────┐│
│  │     se3_math + robot_model           ││  ← 共用基础库
│  └──────────────────────────────────────┘│
├─────────────────────────────────────────┤
│     运动学/动力学抽象层 (Pinocchio)      │
├─────────────────────────────────────────┤
│          硬件接口抽象层 (RobotHW)         │
└─────────────────────────────────────────┘
```

### 关键约束

| # | 原则 | 含义 |
|---|---|---|
| 1 | **同级独立** | GAC 和 GIC 是 `core/` 下的平级模块，不在内部互相 import |
| 2 | **接口兼容** | `GACController.compute()` 和 `GICController.compute()` 参数签名相同（GAC 多一个可选 `F_ext`） |
| 3 | **可互换** | 不改变上层控制循环结构的前提下，仅替换实例化语句即可切换 GIC ↔ GAC |
| 4 | **自包含** | GAC 内部实现完整的力→位置修正→力矩输出流程，不委托给 GIC |
| 5 | **无副作用** | GIC 的修改不影响 GAC，GAC 不存在不影响 GIC |

### 为什么 GAC 不能复用 GIC 作为内环

我上一版的方案是多层结构（GAC 接受一个 `inner_controller=GICController(...)`），这个方案被否决的理由：

```
❌ 问题: GAC → GIC 的单向依赖
GACController
  └── inner: GICController    ← GAC 依赖 GIC，删了 GIC 就报错
      
✅ 正确: 平级独立
GICController  ← 只依赖 se3_math + robot_model
GACController  ← 只依赖 se3_math + robot_model (不 import GIC)
```

如果 GAC 包装了 GIC，那么：
- 想删除 GIC 时 GAC 无法工作
- GAC 的 compute() 在调用栈上穿透了 GIC 的 compute()，调试时需要同时理解两个类
- 与控制核心"GIC/GAC/GUFIC 同级"的架构图矛盾

---

## 2. 架构方案：GAC 的双层结构（自包含实现）

```
┌────────────────────────────────────────────────────┐
│  GACController                                     │
│                                                    │
│  输入: q, dq, pd, Rd, vd, wd, dvd, dwd, F_ext     │
│                                                    │
│  ┌──────────────────────────────────────────────┐  │
│  │  层1: 导纳滤波器 (AdmittanceFilter)           │  │
│  │  M_d·dV + D_d·V + K_d·X = F_ext              │  │
│  │  F_ext=0 → X_corr ≈ 0 (滤波器稳态)           │  │
│  │  ↓ 输出: X_corr, V_corr, dV_corr              │  │
│  └──────────────────────────────────────────────┘  │
│                        ↓                           │
│  ┌──────────────────────────────────────────────┐  │
│  │  层2: 轨迹修正                                │  │
│  │  pd_corrected = pd + R @ Δp_body             │  │
│  │  Rd_corrected = Rd @ exp(hat(Δφ_body))       │  │
│  │  vd_corrected  = vd + ... (体→惯性系)         │  │
│  │  wd_corrected  = wd + dV_corr[3:]            │  │
│  │  dvd/dwd_corrected = ...                      │  │
│  └──────────────────────────────────────────────┘  │
│                        ↓                           │
│  ┌──────────────────────────────────────────────┐  │
│  │  层3: SE(3) 位置跟踪 (内嵌, 不调用 GIC)      │  │
│  │  - 正运动学 (robot_model)                      │  │
│  │  - 体坐标系误差 e_op, ev                       │  │
│  │  - M_tilde 自适应刚度/阻尼                    │  │
│  │  - τ = Jbᵀ(M̃·dVd* - D·ev - K·e_op) + bias  │  │
│  └──────────────────────────────────────────────┘  │
│                        ↓                           │
│  输出: τ_cmd (关节力矩)                            │
└────────────────────────────────────────────────────┘
```

**关键点**：层3 是 GAC 自己实现的 SE(3) 位置跟踪逻辑，数学上与 GIC 控制律**完全相同**，但代码是独立副本。两份独立的代码基于同一套数学公式，只是恰好长得像。

### 可接受的代码重复

| 重复部分 | GIC 行数 | GAC 行数 | 说明 |
|---|---|---|---|
| 正运动学/雅可比调用 | 5 行 | 5 行 | 必写，无法抽象 |
| 体坐标系误差计算 | 8 行 | 8 行 | 同一公式，独立副本 |
| M_tilde SVD 惯性整形 | 12 行 | 12 行 | 同一公式，独立副本 |
| 阻尼/刚度自适应 | 3 行 | 3 行 | 同一公式，独立副本 |
| 力矩限幅 | 4 行 | 4 行 | 同一逻辑，独立副本 |
| **总计重复** | **~32 行** | **~32 行** | **可接受** |

这个重复是**代价最小的方案**——如果未来需要提取共享函数，可以用 `_se3_tracking_core.py`（私有模块），但那是一个纯粹的提取优化，不影响架构独立性。

---

## 3. 对比：GIC vs GAC 接口互换性

### 接口签名

```python
# GIC — 位置误差 → 力矩
class GICController:
    def compute(self, q, dq, pd, Rd, vd, wd, dvd, dwd) -> np.ndarray:
        ...

# GAC — 力→修正轨迹→力矩 (接口完全兼容 GIC)
class GACController:
    def compute(self, q, dq, pd, Rd, vd, wd, dvd, dwd,
                F_ext: np.ndarray = None) -> np.ndarray:
        ...
```

**关键设计**：`F_ext` 是可选参数。当 `F_ext=None` 或全零时，GAC 退化为纯位置跟踪器。

### 互换示例

```python
# ── 场景1: 使用阻抗控制 ─────────────────────
from core.gic_controller import GICController
ctrl = GICController(robot, bandwidth=30.0, damping=1.0)

# 控制循环
tau = ctrl.compute(q, dq, pd, Rd, vd, wd, dvd, dwd)

# ── 场景2: 替换为导纳控制 ───────────────────
# 只需改两行：
from core.gac_controller import GACController          # ← 改 import
ctrl = GACController(robot,                            # ← 改实例化
    M_d=10.0, D_d=100.0, K_d=500.0, dt=0.002,
    bandwidth=30.0, damping=1.0, torque_limits=...)

# 控制循环完全不变：
tau = ctrl.compute(q, dq, pd, Rd, vd, wd, dvd, dwd)   # F_ext 可选

# ── 场景3: 需要导纳时传入 F_ext ──────────────
F_ext = hardware.get_ft_sensor()                       # 多读一个传感器
tau = ctrl.compute(q, dq, pd, Rd, vd, wd, dvd, dwd,
                   F_ext=F_ext)                        # 传外力
```

### 构造函数对比

```python
# GIC:
GICController(robot_model, bandwidth=30.0, damping=1.0, torque_limits=None)

# GAC:
GACController(robot_model,
    M_d=10.0, D_d=100.0, K_d=500.0, dt=0.002,  # 导纳参数 (新)
    bandwidth=30.0, damping=1.0,                  # 内环跟踪参数 (同GIC)
    torque_limits=None)                           # 限幅 (同GIC)
```

两者共享 `bandwidth`, `damping`, `torque_limits` 这三个参数的含义和默认值。
GAC 额外需要 `M_d`, `D_d`, `K_d`, `dt` 四个导纳参数。

---

## 4. 数学公式

### 4.1 SE(3) 体坐标系导纳滤波器

```
M_d · dV_corr + D_d · V_corr + K_d · X_corr = F_ext_body

其中:
  X_corr = [Δp_body;  Δφ_body] ∈ ℝ⁶    — 体坐标系位姿修正量
  V_corr = dX_corr/dt ∈ ℝ⁶              — 体坐标系速度修正量
  dV_corr ∈ ℝ⁶                          — 体坐标系加速度修正量
  M_d, D_d, K_d ∈ ℝ⁶ˣ⁶                 — 用户指定的虚拟质量/阻尼/刚度
  F_ext_body ∈ ℝ⁶                       — 体坐标系外力/力矩
```

### 4.2 离散化（前向欧拉）

```python
# 求解修正加速度
dV_corr = np.linalg.solve(M_d, F_ext_body - D_d @ V_corr - K_d @ X_corr)

# 显式积分
V_corr += dV_corr * dt
X_corr += V_corr * dt
```

数值考虑：
- `M_d` 取可逆对角阵（通常 `[m, m, m, I_xx, I_yy, I_zz]`）
- `D_d = 2·sqrt(K_d·M_d)` 时临界阻尼（对角元计算）
- 泄漏积分防漂移：`X_corr *= (1 - leak_rate * dt)`

### 4.3 轨迹修正叠加

```python
# 位置修正 (体→惯性系)
pd_corrected = pd + R @ Δp_body

# 朝向修正 (SO(3) 右乘)
# 默认: 小角度近似 exp(hat(Δφ)) ≈ I + hat(Δφ)
# 当 ||Δφ|| > 0.05 时: 自动切换 Rodrigues 公式
if norm(Δφ_body) < 0.05:
    ΔR = hat_map(Δφ_body)
    Rd_corrected = Rd + ΔR @ Rd
else:
    Rd_corrected = Rd @ so3_exp(Δφ_body)
# 修正后 SVD 重正化: U,_,Vt = svd(Rd_corrected); Rd_corrected = U@Vt

# 速度修正 (体→惯性系, 含科氏项)
vd_corrected = vd + R @ (Δv_corr[:3] + hat_map(w) @ Δp_body)
wd_corrected = wd + Δv_corr[3:]

# 加速度修正 (简化: 直接叠加)
dvd_corrected = dvd + R @ dV_corr[:3]
dwd_corrected = dwd + dV_corr[3:]
```

### 4.4 内嵌 SE(3) 位置跟踪（与 GIC 同公式，独立代码）

```python
# ── 正运动学 ──
robot.update(q, dq)
p, R = robot.get_pose()
M = robot.get_full_inertia()
Jb = robot.get_body_jacobian()
qfrc_bias = robot.get_bias_torque()

# ── SE(3) 位姿变换 ──
g = eye(4); g[:3,:3] = R; g[:3,3] = p
gd = eye(4); gd[:3,:3] = Rd_corrected; gd[:3,3] = pd_corrected
g_ed = inv(g) @ gd

# ── 期望速度变换 ──
Vd = hstack((vd_corrected, wd_corrected))
dVd = hstack((dvd_corrected, dwd_corrected))
Vd_star = adjoint_g_ed(g_ed) @ Vd
dVd_star = adjoint_g_ed_deriv(...) @ Vd + adjoint_g_ed(g_ed) @ dVd

# ── 体坐标系误差 ──
e_pos = R.T @ (p - pd_corrected)
e_rot = vee_map(Rd_corrected.T @ R - R.T @ Rd_corrected)
ev = Vb - Vd_star

# ── M_tilde 自适应 ──
M_tilde_inv = Jb @ inv(M) @ Jb.T
U, s, Vt = svd(M_tilde_inv)
damp_sv = max(1e-6, 0.1 * s[-1])
s_damped = s / (s**2 + damp_sv**2)
M_tilde = (Vt.T * s_damped) @ U.T

# ── 自适应增益 ──
K_adapt = w_des**2 * M_tilde
D_adapt = 2 * zeta_des * w_des * M_tilde

# ── 控制律 ──
tau_tilde = M_tilde @ dVd_star - D_adapt @ ev - K_adapt @ e_op
tau_cmd = Jb.T @ tau_tilde + qfrc_bias
```

### 4.5 退化条件

| 条件 | 行为 |
|---|---|
| `F_ext = None` | 滤波器状态归零，跳过修正，纯位置跟踪 |
| `F_ext = [0]*6` 稳态 | `X_corr → 0`，退化为纯位置跟踪 |
| `F_ext ≠ 0` 但 `K_d → ∞` | 修正量趋近于零，等效高刚度位置控制 |

---

## 5. 接口设计

### 5.1 GACFilter（导纳滤波器内部组件）

```python
class GACFilter:
    """SE(3) 体坐标系导纳滤波器 — 外力 → 轨迹修正量.

    实现虚拟二阶动力学:
      M_d · dV_corr + D_d · V_corr + K_d · X_corr = F_ext_body

    状态量 X_corr, V_corr 在体坐标系中定义。
    泄漏积分防止零漂: X_corr *= (1 - leak * dt)
    """

    def __init__(self,
                 M_d: np.ndarray,    # 虚拟质量 (6,6) 或 (6,) 对角值
                 D_d: np.ndarray,    # 虚拟阻尼 (6,6) 或 (6,) 对角值
                 K_d: np.ndarray,    # 虚拟刚度 (6,6) 或 (6,) 对角值
                 dt: float,          # 控制周期 (s)
                 max_correction: float = 0.05,  # 最大修正量 (m/rad)
                 leak_rate: float = 0.0):       # 泄漏率 (防漂移)
        ...

    def update(self, F_ext_body: np.ndarray
               ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """一步滤波更新.

        :param F_ext_body: 体坐标系外力/力矩 (6,)
        :returns: (X_corr, V_corr, dV_corr) — 修正量、速度、加速度
        """
        ...

    def reset(self):
        """重置滤波器状态为零."""
        ...

    def set_parameters(self, M_d=None, D_d=None, K_d=None):
        """在线更新虚拟阻抗参数."""
        ...
```

### 5.2 GACController

```python
class GACController:
    """SE(3) 导纳控制器 (Geometric Admittance Controller).

    自包含实现，不依赖 GICController。
    三层流程: 导纳滤波 → 轨迹修正 → SE(3) 位置跟踪

    F_ext = None 时退化为纯位置跟踪 (阻抗模式)。

    用法::

        from core.gac_controller import GACController

        ctrl = GACController(robot,
            M_d=[10.0, 10.0, 10.0, 1.0, 1.0, 1.0],
            D_d=[100.0, 100.0, 100.0, 10.0, 10.0, 10.0],
            K_d=[500.0, 500.0, 500.0, 50.0, 50.0, 50.0],
            dt=0.002, bandwidth=30.0, damping=1.0)

        # 控制循环 (与 GIC 完全兼容)
        tau = ctrl.compute(q, dq, pd, Rd, vd, wd, dvd, dwd)
        # 或带力传感器:
        tau = ctrl.compute(q, dq, pd, Rd, vd, wd, dvd, dwd, F_ext=F_ext)
    """

    def __init__(self,
                 robot_model,
                 M_d: np.ndarray,       # 虚拟质量 (6,) 对角值
                 D_d: np.ndarray,       # 虚拟阻尼 (6,) 对角值
                 K_d: np.ndarray,       # 虚拟刚度 (6,) 对角值
                 dt: float,             # 控制周期 (s)
                 bandwidth: float = 30.0,   # 内环带宽 ω_des (同 GIC)
                 damping: float = 1.0,      # 内环阻尼比 ζ (同 GIC)
                 torque_limits: np.ndarray = None,
                 max_correction: float = 0.05,
                 filter_leak_rate: float = 0.0):
        ...

    def compute(self, q: np.ndarray, dq: np.ndarray,
                pd: np.ndarray, Rd: np.ndarray,
                vd: np.ndarray, wd: np.ndarray,
                dvd: np.ndarray, dwd: np.ndarray,
                F_ext: np.ndarray = None) -> np.ndarray:
        """GAC 单步计算.

        :param q:   关节位置 (nv,)
        :param dq:  关节速度 (nv,)
        :param pd:  期望位置 (3,)
        :param Rd:  期望朝向 (3,3)
        :param vd:  期望线速度 (3,)
        :param wd:  期望角速度 (3,)
        :param dvd: 期望线加速度 (3,)
        :param dwd: 期望角加速度 (3,)
        :param F_ext: 体坐标系外力 (6,), None=纯位置跟踪
        :returns: 关节力矩指令 (nv,)
        """
        # 1. 如果 F_ext=None, 跳过滤波 (滤波器状态维持, 视作纯跟踪)
        if F_ext is not None:
            X_corr, V_corr, dV_corr = self._filter.update(F_ext)
            # 2. 轨迹修正
            (pd, Rd, vd, wd, dvd, dwd) = self._correct_trajectory(
                pd, Rd, vd, wd, dvd, dwd, X_corr, V_corr, dV_corr)
        # 3. SE(3) 位置跟踪 (内嵌, 与 GIC 同公式但独立代码)
        tau_cmd = self._compute_tracking(q, dq, pd, Rd, vd, wd, dvd, dwd)
        return tau_cmd

    def reset(self):
        """重置滤波器状态."""
        self._filter.reset()

    def _correct_trajectory(self, pd, Rd, vd, wd, dvd, dwd,
                             X_corr, V_corr, dV_corr):
        """修正期望轨迹 (内部方法)."""
        ...

    def _compute_tracking(self, q, dq, pd, Rd, vd, wd, dvd, dwd):
        """SE(3) 位置跟踪 (内部方法, 与 GIC 同公式)."""
        ...
```

### 5.3 命名约定

| 项 | 命名 |
|---|---|
| 文件名 | `core/gac_controller.py` |
| 主类 | `GACController` |
| 内部滤波器类 | `GACFilter` |
| __init__.py 导出 | `GACController` |

---

## 6. 文件清单与实现顺序

### Phase 2.5a: 核心实现

| 步骤 | 文件 | 内容 | 行数估计 |
|---|---|---|---|
| 1 | `core/gac_controller.py` | GACFilter + GACController（自包含） | ~160 行 |
| 2 | `core/__init__.py` | 添加 `GACController` 导出 | ~3 行 |

### Phase 2.5b: 验证

| 步骤 | 文件 | 内容 |
|---|---|---|
| 3 | `tests/test_gac_controller.py` | 滤波器单元测试 + 完整控制律验证 |
| 4 | 修改 `verify_gic_mujoco.py` | 添加 `--control gac` 模式，验证 GAC 可互换 |
| 5 | 自检运行 | 确认 GIC 不受影响，GAC 功能正常 |

### Phase 3（后续）

| 步骤 | 文件 | 内容 |
|---|---|---|
| 6 | `core/gufic_controller.py` | GUFIC — 可继承 GIC 或独立实现 |
| 7 | 更新 `verify_gic_mujoco.py` | 支持 `--control gufic` |

---

## 7. 验证标准

### 7.1 单元测试 (test_gac_controller.py)

| # | 测试名 | 条件 | 预期 |
|---|---|---|---|
| 1 | `test_filter_zero_force` | F_ext=0 持续 100 步 | X_corr ≈ 0 |
| 2 | `test_filter_constant_force` | F_ext=[10,0,0,0,0,0] | 稳态 X_corr[:3] ≈ K_d⁻¹ · F_ext[:3] |
| 3 | `test_filter_step_response` | 阶跃力, 临界阻尼设置 | 无超调, 稳态误差 < 1% |
| 4 | `test_filter_reset` | reset() 后 | 状态全零 |
| 5 | `test_gac_zero_force_equals_gic` | F_ext=0, 同输入 | GAC 输出 ≈ GIC 输出 (相对误差 < 1e-10) |
| 6 | `test_gac_force_deviates` | 施加 10N 恒力 | 位置缓慢偏移动, 平衡时 X_corr ≈ K_d⁻¹·F_ext |
| 7 | `test_gac_tracking_with_force` | circle + 脉冲力 | 有修正但轨迹不发散 |

### 7.2 仿真验证 (verify_gic_mujoco.py --control gac)

| 测试 | 条件 | 预期 |
|---|---|---|
| 自由空间 regulation | F_ext=0 | 与 GIC regulation 误差 < 1e-8 |
| 自由空间 circle 跟踪 | F_ext=0 | 与 GIC circle 跟踪误差一致 |
| 恒力扰动 | F_ext 脉冲 5N·s | 位置偏移 < 2cm 后恢复 |
| F_ext 阶跃 | 10N 阶跃 | 导纳响应平滑 |

### 7.3 回归测试

```bash
# GIC 不受影响
python se3_control/scripts/verify_gic_mujoco.py --robot ur12e --task regulation --no-viewer
python se3_control/scripts/verify_gic_mujoco.py --robot ur3   --task circle --no-viewer

# GAC 新模式
python se3_control/scripts/verify_gic_mujoco.py --control gac --robot ur12e --task regulation --no-viewer

# 硬件 mock 回归
python se3_control/scripts/test_ur_hw_mock.py --robot ur12e
python se3_control/scripts/test_ur_hw_mock.py --robot ur3
```

---

## 8. 运行时架构

```
┌──────────────────────────────────────────────────────────┐
│  控制循环 (run_se3_control.py / 实机)                    │
│                                                          │
│  # 实例化 (选其一)                                        │
│  ctrl = GICController(robot, ...)   # 阻抗               │
│  ctrl = GACController(robot, ...)   # 导纳 (可互换)      │
│                                                          │
│  # 控制循环 (完全不变)                                    │
│  while running:                                          │
│      q, dq = hw.get_joint_states()                       │
│      pd, Rd, ... = traj(t)                               │
│                                                          │
│      # GIC 调用方式:                                     │
│      tau = ctrl.compute(q, dq, pd, Rd, vd, wd, dvd, dwd) │
│                                                          │
│      # GAC 调用方式 (完全兼容, 多一个可选参数):           │
│      F_ext = hw.get_ft_sensor() if has_ft else None      │
│      tau = ctrl.compute(q, dq, pd, Rd, vd, wd, dvd, dwd, │
│                          F_ext=F_ext)                    │
│                                                          │
│      hw.set_joint_torques(tau)                           │
└──────────────────────────────────────────────────────────┘
          ↓ 择一使用                ↓ 共享
┌───────────────────┐   ┌───────────────────┐
│ GICController     │   │ GACController     │
│ (gic_controller)  │   │ (gac_controller)  │
│                   │   │ ┌───────────────┐ │
│ 位置误差→力矩      │   │ │ GACFilter    │ │
│                   │   │ │ 力→修正       │ │
│                   │   │ ├───────────────┤ │
│                   │   │ │ _compute_     │ │
│                   │   │ │ tracking()    │ │
│                   │   │ │ 位置跟踪      │ │
│                   │   │ └───────────────┘ │
└────────┬──────────┘   └────────┬──────────┘
         ↓ 都依赖                ↓ 都依赖
┌─────────────────────────────────────────────┐
│          se3_math + robot_model              │
│  (共用基础库, 不是控制器, 没有"依赖方向")     │
└─────────────────────────────────────────────┘
```

### 互换性验证

```
                 GICController  │  GACController
                                │
compute(q, dq, pd, Rd,         │  compute(q, dq, pd, Rd,
        vd, wd, dvd, dwd)      │          vd, wd, dvd, dwd)
                                │          [, F_ext=None])
────────────────────────────────┼───────────────────────────
返回值类型     np.ndarray       │  np.ndarray
力矩单位       Nm               │  Nm
Bandwidth      默认 30          │  默认 30 (位置跟踪部分)
Damping        默认 1.0         │  默认 1.0
Torque Limits  支持             │  支持
依赖           se3_math +      │  se3_math + robot_model
               robot_model     │  (不导入 GIC)
F_ext          不支持           │  可选, None=纯跟踪
新增参数       —                │  M_d, D_d, K_d, dt
```

---

## 9. 与 GUFIC 的关系（未来工作）

GUFIC 是导纳和阻抗的**统一框架**，不是简单的串联：

```
GUFIC = GIC项 + 力跟踪项 + 能量油箱
         ↑        ↑         ↑
       阻抗     导纳式    安全切换
       (误差→力) (力→修正) (无源化)
```

GAC 实现了 GUFIC 的"力跟踪"部分 + 独立的阻抗部分（位置跟踪），但不是统一的：

| | GAC（本计划） | GUFIC（Phase 3） |
|---|---|---|
| 阻抗项 | 隐式包含在位置跟踪中 | 显式阻抗项 |
| 力跟踪 | 导纳滤波 (二阶) | PI 力控制器 |
| 能量油箱 | ❌ 无 | ✅ 有 |
| 切换逻辑 | 手动 F_ext=None | 自动无源切换 |
| 实现复杂度 | ~160 行 | ~250 行 |
| 依赖 | 独立 ← 本计划新增 | 可继承 GIC 或独立实现 |

**实施路径**：GAC 完成后，GUFIC 在 GAC 基础上增加力跟踪 PI 控制器和能量油箱模块，或直接从 GIC 扩展力控制项。

---

## 10. 风险与缓解

| 风险 | 影响 | 缓解措施 |
|---|---|---|
| 力传感器噪声被滤波器放大 | 力矩抖动 | 滤波器输入加低通 (~10Hz)；M_d 避免过小 |
| F_ext 零漂导致位置漂移 | 稳态位置偏移 | leaky integrator (X_corr × 0.999)；定期 reset |
| K_d 过小 + 持续外力 | 位置偏移超出工作空间 | 修正量限幅 ±5cm/±10° |
| 内环带宽与外环不匹配 | 跟踪震荡 | 外环响应频率 < 内环带宽 / 5 |
| dt 变化（非实时系统） | 滤波器积分精度下降 | 固定 dt 步进，实际 dt 变化时用插值 |
| 两套位置跟踪代码不一致 | GAC 与 GIC 行为差异 | 验证标准 #5 保证 F_ext=0 时输出一致 |

---

*文档创建日期: 2026-07-29*
*关联: deploy_se3_to_hardware_plan.md 的 Phase 5 预备工作*
