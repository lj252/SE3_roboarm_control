# GIC 控制核心移植计划 — Phase 2

> 从 `verify_gic_mujoco.py` 中抽离 GIC 控制核心，形成机器人无关的独立库
> 关联文档: [deploy_se3_to_hardware_plan.md](./deploy_se3_to_hardware_plan.md)

---

## 0. 当前状态分析

### 现状：全部耦合在 `verify_gic_mujoco.py` (1473 行)

```
verify_gic_mujoco.py
├── SE(3) 数学函数          ← 从 GUFIC_mujoco-main 导入（外部依赖）
│   ├── vee_map, hat_map
│   ├── adjoint_g_ed, adjoint_g_ed_deriv
│   └── (内部自含) _rotmat_slerp
├── 轨迹生成 (build_trajectory_from_config)
│   ├── sympy 符号微分
│   ├── lambdify 转 numpy 函数
│   └── 支持 regulation / circle / line / sphere
├── GICController 类
│   ├── 自适应 M_tilde (带宽 ω_des + 阻尼 ζ)
│   ├── SE(3) 误差 (e_pos, e_rot, ev)
│   ├── 操作空间惯性矩阵
│   └── 力矩限幅
├── 增益加载 (load_gains_from_config)
├── URDF → MuJoCo XML 转换  ← 仿真专用
└── 主验证循环 (run_verification)  ← 仿真专用
```

**问题**: GIC 控制核心与 MuJoCo 仿真、XML 生成、可视化等耦合在一起，无法独立用于实机。

### 目标：独立的核心库

```
se3_control/core/
├── se3_math.py            # SE(3) 数学 — 纯 numpy, 0 依赖
├── trajectory.py           # 轨迹生成 — 依赖 sympy, 可选依赖
├── gic_controller.py      # GIC 控制律 — 只依赖 se3_math + robot_model
└── gufic_controller.py    # GUFIC 控制律 — 依赖 se3_math + robot_model (Phase 3)
```

---

## 1. se3_math.py — SE(3) 数学工具

### 定位

纯 numpy 实现，零外部依赖。从 `GUFIC_mujoco-main/gufic_env/utils/misc_func.py` 和 `verify_gic_mujoco.py` 中提取所有 SE(3) 数学函数。

### 接口设计

```python
# 基本操作
hat_map(w: np.ndarray) -> np.ndarray          # ℝ³ → 𝔰𝔬(3) 反对称矩阵
vee_map(R: np.ndarray) -> np.ndarray           # 𝔰𝔬(3) → ℝ³ 逆映射

# SE(3) 变换
adjoint_g_ed(g_ed: np.ndarray) -> np.ndarray            # Ad_{g_ed} 伴随变换
adjoint_g_ed_dual(g_ed: np.ndarray) -> np.ndarray       # Ad_{g_ed}^{-T}
adjoint_g_ed_deriv(g, gd, v, w, vd, wd) -> np.ndarray  # d/dt Ad_{g_ed}

# 旋转矩阵
rotmat_x(th: float) -> np.ndarray              # Rx 基本旋转矩阵
rotmat_slerp(R1, R2, alpha) -> np.ndarray      # SO(3) 球面线性插值
rpy_to_rotmat(rpy) -> np.ndarray               # URDF RPY → 旋转矩阵
rotmat_to_xyz_euler(R) -> np.ndarray           # 旋转矩阵 → XYZ 欧拉角
```

### 来源映射

| 函数 | 来源文件 |
|---|---|
| `hat_map`, `vee_map` | `GUFIC/misc_func.py` (原样复制) |
| `adjoint_g_ed`, `adjoint_g_ed_dual`, `adjoint_g_ed_deriv` | `GUFIC/misc_func.py` (原样复制) |
| `rotmat_x` | `GUFIC/misc_func.py` (原样复制) |
| `rotmat_slerp` | `verify_gic_mujoco.py:_rotmat_slerp` (重命名, 去掉私有前缀) |
| `rpy_to_rotmat`, `rotmat_to_xyz_euler` | `verify_gic_mujoco.py` (原样复制) |

---

## 2. trajectory.py — 轨迹生成

### 定位

从 `verify_gic_mujoco.py` 的 `build_trajectory_from_config()` 中提取，对接 `config/task_config.py`。

### 接口设计

```python
def build_trajectory(task: str, cfg=None) -> TrajectoryFuncs:
    """从 task_config 读取参数，构建轨迹函数族。
    
    返回 NamedTuple:
        pd_t(t) -> (3,)      位置
        Rd_t(t) -> (3,3)     朝向
        dpd_t(t) -> (3,)     位置速度
        dRd_t(t) -> (3,3)    朝向速度
        ddpd_t(t) -> (3,)    位置加速度
        ddRd_t(t) -> (3,3)   朝向加速度
    """
```

### 实现说明

- 使用 `sympy` 进行符号微分 → `lambdify` 转 numpy
- 支持任务类型: `regulation`, `circle`, `line`, `sphere`
- 参数从 `task_config.py` 模块读取（不硬编码默认值到代码中）

### 可选改进

- 添加缓存机制：对相同 task 参数避免重复符号微分
- 添加 `TrajectoryFuncs` NamedTuple 类型签名

---

## 3. gic_controller.py — GIC 控制律

### 定位

从 `verify_gic_mujoco.py:GICController` 提取。核心控制律，只依赖 `se3_math` + `RobotModel`。

### 接口设计

```python
class GICController:
    """Geometric Impedance Controller — 自适应惯性整形"""

    def __init__(self, robot_model: RobotModel,
                 bandwidth: float = 30.0,    # ω_des (rad/s)
                 damping: float = 1.0,        # ζ_des
                 torque_limits: np.ndarray = None):
        ...

    def compute(self, q: np.ndarray, dq: np.ndarray,
                pd: np.ndarray, Rd: np.ndarray,
                vd: np.ndarray, wd: np.ndarray,
                dvd: np.ndarray, dwd: np.ndarray,
                Fe_raw: np.ndarray = None) -> np.ndarray:
        """GIC 控制律单步计算.
        
        Args:
            q:   关节位置 (nv,)
            dq:  关节速度 (nv,)
            pd:  期望位置 (3,)
            Rd:  期望朝向 (3,3)
            vd:  期望线速度 (3,)
            wd:  期望角速度 (3,)
            dvd: 期望线加速度 (3,)
            dwd: 期望角加速度 (3,)
            Fe_raw: 外力矩传感器读数（可选，GUFIC 用）
        
        Returns:
            tau_cmd: 关节力矩指令 (nv,)
        """
        ...
```

### 控制律公式

```
输入: q, dq, pd, Rd, vd, wd, dvd, dwd

1. 正运动学: p, R = fk(q); Jb = body_jacobian(q)
2. SE(3) 误差:
   g_ed = inv(g) @ gd
   Vd* = Ad_{g_ed} @ Vd
   dVd* = d/dt(Ad_{g_ed}) @ Vd + Ad_{g_ed} @ dVd
   e_pos = R^T @ (p - pd)           (体坐标系)
   e_rot = vee(Rd^T @ R - R^T @ Rd) (体坐标系)
   ev = Vb - Vd*
3. 操作空间惯性: M̃ = (Jb @ M^{-1} @ Jb^T)^{-1}
4. 自适应增益: K_adapt = ω² · M̃, D_adapt = 2ζω · M̃
5. 控制律: τ̃ = M̃·dVd* - D·ev - K·e_op
6. 力矩: τ_cmd = Jb^T @ τ̃ + b(q,dq)
7. 限幅输出
```

### 与原始实现的关键差异

| 项 | 原始 (verify_gic_mujoco.py) | 新实现 |
|---|---|---|
| 增益来源 | `load_gains_from_config()` + 自适应 | **仅自适应** (带宽+阻尼) |
| `Kp/KR/Kd` 参数 | 从 config 读取对角矩阵 | **移除** — 改用自适应 M̃ |
| `compute()` 参数 | `q, dq, pd, Rd, vd, wd, dvd, dwd` | 相同 |
| 力矩限幅 | 构造时传入 | 相同 |
| 外力传感 | 未使用 (Fe_raw 保留接口) | 保留 Fe_raw 参数 |

---

## 4. gufic_controller.py — GUFIC 控制律（预留）

### 状态

Phase 3 实施，当前仅创建占位文件。

### 接口设计（预览）

```python
class GUFICController(GICController):
    """GUFIC 力-阻抗控制 — GIC + 力跟踪 + 能量油箱"""

    def __init__(self, robot_model, bandwidth=30.0, damping=1.0,
                 torque_limits=None,
                 kp_force=1.0, kd_force=0.5, ki_force=4.0,
                 tank_capacity=100.0):
        ...

    def compute(self, q, dq, pd, Rd, vd, wd, dvd, dwd,
                Fd=None, Fe_raw=None):
        """GUFIC 控制律."""
        ...
```

---

## 5. 文件清单与实现顺序

### Phase 2a: 核心文件 (本次实现)

| 步骤 | 文件 | 内容 | 行数估计 |
|---|---|---|---|
| 1 | `core/se3_math.py` | SE(3) 数学函数（纯 numpy） | ~120 行 |
| 2 | `core/trajectory.py` | 轨迹生成（依赖 sympy） | ~130 行 |
| 3 | `core/gic_controller.py` | GIC 控制律（依赖 se3_math + RobotModel） | ~100 行 |
| 4 | 更新 `core/__init__.py` | 导出新模块 | ~20 行 |

### Phase 2b: 验证与集成

| 步骤 | 文件 | 内容 |
|---|---|---|
| 5 | 新建 `tests/test_gic_controller.py` | 单元测试：se3_math 正确性、GIC 控制律数值 |
| 6 | 修改 `verify_gic_mujoco.py` | 用 `core/gic_controller.py` 替换内联 GICController |
| 7 | 运行 mock 测试 | 确认不引入回归 |

### Phase 3（后续）

| 步骤 | 文件 | 内容 |
|---|---|---|
| 8 | `core/gufic_controller.py` | GUFIC 力-阻抗控制 |
| 9 | 更新 `verify_gic_mujoco.py` | 支持 GUFIC 控制模式 |

---

## 6. 验证标准

1. **单元测试**: `test_gic_controller.py`
   - `hat_map` ∘ `vee_map` 为恒等映射
   - `adjoint_g_ed` 乘法性质: Ad(g1) @ Ad(g2) = Ad(g1 @ g2)
   - `rotmat_slerp` 端点正确: α=0 → R1, α=1 → R2
   - GICController.compute 返回正确形状 (nv,)
   - 零误差时输出 ≈ 偏置力矩

2. **集成测试**: 修改后的 `verify_gic_mujoco.py`
   - 用新 GICController 替换内联版本
   - regulation 任务稳态误差 < 1cm
   - circle 任务跟踪误差 < 0.05m

3. **回归测试**: mock 测试
   - `test_ur_hw_mock.py --robot ur12e` 34/34 通过
   - `test_ur_hw_mock.py --robot ur3` 34/34 通过

---

## 7. 移植后的架构

```
┌─────────────────────────────────────────────────────┐
│                  verify_gic_mujoco.py                │
│  (仿真专用: URDF→XML, MuJoCo 步进, 可视化, 记录)     │
│  ┌─────────────────────────────────────────────────┐ │
│  │              run_verification()                 │ │
│  │  调用 → GICController.compute(q, dq, ...)        │ │
│  │         Trajectory.build_trajectory(task)        │ │
│  └─────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────┘
                        ↓ 依赖
┌─────────────────────────────────────────────────────┐
│                   core/  (机器人无关)                 │
│  ┌──────────┐ ┌────────────┐ ┌──────────────────┐  │
│  │se3_math  │ │trajectory  │ │gic_controller    │  │
│  │(numpy)   │ │(sympy)     │ │(se3_math+RobotMo)│  │
│  └──────────┘ └────────────┘ └──────────────────┘  │
└─────────────────────────────────────────────────────┘
                        ↓ 依赖
┌─────────────────────────────────────────────────────┐
│              robot_model/robot_model.py              │
│  (Pinocchio 封装, URDF 驱动, 机器人无关)             │
└─────────────────────────────────────────────────────┘
                        ↓ 依赖
┌─────────────────────────────────────────────────────┐
│  实机控制循环 (run_se3_control.py)                   │
│  调用: RobotHW.get_joint_states()                    │
│        GICController.compute(q, dq, ...)              │
│        RobotHW.set_joint_torques(tau)                 │
└─────────────────────────────────────────────────────┘
```

---

*文档创建日期: 2026-07-27*
*关联: deploy_se3_to_hardware_plan.md 的 Phase 2*
