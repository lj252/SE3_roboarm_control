# -*- coding: utf-8 -*-
"""
se3_control.core — SE(3) Control Core

模块:
  - se3_math.py:          SE(3) 数学工具（hat_map, vee_map, adjoint, slerp ...）
  - trajectory.py:        轨迹生成（SymPy 符号微分 → NumPy 函数）
  - gic_controller.py:    GIC 控制律（自适应操作空间惯性整形）
  - gac_controller.py:    GAC 导纳控制 (Geometric Admittance Controller)
  - gufic_controller.py:  GUFIC 控制律 (Phase 3 — 预留)
"""

from .se3_math import (
    hat_map,
    vee_map,
    adjoint_g_ed,
    adjoint_g_ed_dual,
    adjoint_g_ed_deriv,
    rotmat_x,
    rotmat_slerp,
    rpy_to_rotmat,
    rotmat_to_xyz_euler,
)

from .trajectory import (
    build_trajectory,
    TrajectoryFuncs,
)

from .gic_controller import (
    GICController,
)

from .gac_controller import (
    GACController,
)

__all__ = [
    # se3_math
    'hat_map', 'vee_map',
    'adjoint_g_ed', 'adjoint_g_ed_dual', 'adjoint_g_ed_deriv',
    'rotmat_x', 'rotmat_slerp',
    'rpy_to_rotmat', 'rotmat_to_xyz_euler',
    # trajectory
    'build_trajectory', 'TrajectoryFuncs',
    # gic_controller
    'GICController',
    # gac_controller
    'GACController',
]
