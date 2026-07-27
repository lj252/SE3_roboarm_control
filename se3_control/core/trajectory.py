"""
轨迹生成 — SymPy 符号微分 → NumPy 函数
=========================================

从 ``config/task_config.py`` 读取参数，使用 SymPy 构建轨迹函数族并微分，
生成可直接调用的 NumPy 函数。支持 regulation / circle / line / sphere 四种任务。

用法::

    from core.trajectory import build_trajectory

    # 从 task_config 读取参数构建 circle 轨迹
    pd_t, Rd_t, dpd_t, dRd_t, ddpd_t, ddRd_t = build_trajectory('circle')

    # 在控制循环中调用
    for t in np.arange(0, 5.0, 0.001):
        pd = pd_t(t).ravel()
        Rd = Rd_t(t).reshape(3, 3)
        ...
"""

import sys
from typing import Callable, NamedTuple, Tuple

import numpy as np


# ====================================================================
# 类型定义
# ====================================================================

class TrajectoryFuncs(NamedTuple):
    """轨迹函数族 — 6 个函数的命名元组.

    所有函数接受时间 t (float)，返回对应的轨迹值。
    """
    pd_t:   Callable[[float], np.ndarray]   # 期望位置 (3,)
    Rd_t:   Callable[[float], np.ndarray]   # 期望朝向 (3,3)
    dpd_t:  Callable[[float], np.ndarray]   # 期望线速度 (3,)
    dRd_t:  Callable[[float], np.ndarray]   # 期望朝向速度 (3,3)
    ddpd_t: Callable[[float], np.ndarray]   # 期望线加速度 (3,)
    ddRd_t: Callable[[float], np.ndarray]   # 期望朝向加速度 (3,3)


# ====================================================================
# 轨迹构建
# ====================================================================

def build_trajectory(task: str, cfg=None) -> TrajectoryFuncs:
    """从 task_config 读取参数, 用 SymPy 构建轨迹函数族.

    :param task: 任务类型 — 'regulation', 'circle', 'line', 'sphere'
    :param cfg:  task_config 模块 (或类似接口的对象).
                 None 时自动导入 ``config.task_config``.
    :returns: TrajectoryFuncs — 包含 6 个时间函数的命名元组
    """
    import sympy as sp

    if cfg is None:
        try:
            from config import task_config as cfg
        except ModuleNotFoundError:
            # 当直接从 se3_control/ 外运行时，添加路径重试
            import os
            _script_dir = os.path.dirname(os.path.abspath(__file__))
            _project_dir = os.path.dirname(_script_dir)  # se3_control/
            sys.path.insert(0, _project_dir)
            from config import task_config as cfg

    t = sp.symbols('t')

    # 读取任务参数
    task_cfg = getattr(cfg, task, {})

    # 默认朝向 (3x3 旋转矩阵, 行优先展开的 9 个数字)
    flat_R = task_cfg.get(
        'orientation',
        [0, 1, 0, 1, 0, 0, 0, 0, -1],
    )
    Rd_default = np.array(flat_R, dtype=float).reshape(3, 3)

    # ── 按任务类型构建符号轨迹 ──────────────────────────────────

    if task == 'regulation':
        center = task_cfg.get('target', [0.5, 0.0, 0.125])
        pd_default = np.array(center, dtype=float)
        pd_t_sim = sp.Matrix(pd_default)
        Rd_t_sim = sp.Matrix(Rd_default)

    elif task == 'circle':
        center = task_cfg.get('center', [0.5, 0.0, 0.125])
        radius = task_cfg.get('radius', 0.1)
        speed = task_cfg.get('speed', 1.0)
        pd_default = np.array(center, dtype=float)
        pd_t_sim = (
            sp.Matrix(pd_default)
            + sp.Matrix([radius * sp.cos(speed * t),
                         radius * sp.sin(speed * t),
                         0])
        )
        Rd_t_sim = sp.Matrix(Rd_default)

    elif task == 'line':
        center = task_cfg.get('center', [0.5, 0.0, 0.125])
        amplitude = task_cfg.get('amplitude', 0.1)
        direction = task_cfg.get('direction', [0, 1, 0])
        freq = task_cfg.get('frequency', 0.5)
        pd_default = np.array(center, dtype=float)
        d = np.array(direction, dtype=float)
        d_norm = d / np.linalg.norm(d) if np.linalg.norm(d) > 0 else np.array([0, 1, 0])
        offset = amplitude * d_norm
        pd_t_sim = (
            sp.Matrix(pd_default)
            + sp.Matrix(offset.tolist()) * sp.sin(freq * t)
        )
        Rd_t_sim = sp.Matrix(Rd_default)

    elif task == 'sphere':
        center = task_cfg.get('center', [0.40, 0.0, 0.0])
        max_time_val = task_cfg.get('max_time', 10.0)
        total_radian = 0.5 * np.pi
        omega_value = total_radian / max_time_val
        theta_y = omega_value * t - total_radian * 0.5
        r_sphere = task_cfg.get('radius', 0.304)
        pd_t_sim = (
            sp.Matrix(center)
            + sp.Matrix([0,
                         r_sphere * sp.sin(theta_y),
                         -0.10 + r_sphere * sp.cos(theta_y)])
        )
        rotmat_y = sp.Matrix([
            [sp.cos(-theta_y), 0, sp.sin(-theta_y)],
            [0, 1, 0],
            [-sp.sin(-theta_y), 0, sp.cos(-theta_y)],
        ])
        Rd_t_sim = sp.Matrix(Rd_default) @ rotmat_y

    else:
        raise ValueError(f"未知任务类型: '{task}'. "
                         f"可选: regulation, circle, line, sphere")

    # ── 符号微分 → NumPy 函数 ──────────────────────────────────

    dpd_t_sim = sp.diff(pd_t_sim, t)
    dRd_t_sim = sp.diff(Rd_t_sim, t)
    ddpd_t_sim = sp.diff(dpd_t_sim, t)
    ddRd_t_sim = sp.diff(dRd_t_sim, t)

    return TrajectoryFuncs(
        pd_t   = sp.lambdify(t, pd_t_sim,   "numpy"),
        Rd_t   = sp.lambdify(t, Rd_t_sim,   "numpy"),
        dpd_t  = sp.lambdify(t, dpd_t_sim,  "numpy"),
        dRd_t  = sp.lambdify(t, dRd_t_sim,  "numpy"),
        ddpd_t = sp.lambdify(t, ddpd_t_sim, "numpy"),
        ddRd_t = sp.lambdify(t, ddRd_t_sim, "numpy"),
    )


# ====================================================================
# 自检
# ====================================================================

if __name__ == '__main__':
    # 确保能从 se3_control/ 上层导入
    import os
    _script_dir = os.path.dirname(os.path.abspath(__file__))
    _project_dir = os.path.dirname(_script_dir)
    if _project_dir not in sys.path:
        sys.path.insert(0, _project_dir)

    print("=" * 60)
    print("轨迹生成自检")
    print("=" * 60)

    for task in ['regulation', 'circle', 'line']:
        print(f"\n[{task}]")
        funcs = build_trajectory(task)
        t_test = 1.23
        pd = funcs.pd_t(t_test).ravel()
        Rd = funcs.Rd_t(t_test).reshape(3, 3)
        dpd = funcs.dpd_t(t_test).ravel()
        dRd = funcs.dRd_t(t_test).reshape(3, 3)
        print(f"  t={t_test}:")
        print(f"    pd  = {pd}")
        print(f"    dpd = {dpd}")
        print(f"    |dRd| = {np.linalg.norm(dRd):.4f}")
        # 验证形状
        assert pd.shape == (3,), f"pd shape: {pd.shape}"
        assert Rd.shape == (3, 3), f"Rd shape: {Rd.shape}"
        # 验证 SO(3) 性质
        det = np.linalg.det(Rd)
        assert abs(det - 1.0) < 1e-6, f"det(Rd) = {det}"
        print(f"    det(Rd) = {det:.4f} ✅")

    print("\n所有自检通过 ✅")
