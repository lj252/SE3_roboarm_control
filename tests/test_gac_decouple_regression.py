"""实验二 (方向解耦) 回归测试 — 固化 Z 振荡修复.

计划 docs/plan/force_interaction_experiments_plan.md §4.2:
  施加 x 向力时 |Δz|/|Δx| < 5% (严格) / < 10% (可接受).

历史 bug: 体坐标系导纳经 R_cur 投影, 力方向相对期望任务系旋转 → z 向漂移.
修复后: GAC 滤波器输出 X_corr 完美解耦 (各向同性 K_d), EE 级 Fx→Δz 耦合 < 5%.

默认 home 为舒适位形 (EE≈[0.50,0,0.50], 末端竖直朝下) 后, EE 级 Fx→Δz ≈ 7.7%,
超出严格 5% 但处于计划 §4.2 的"可接受 <10%"区间. 该 7.7% 全部来自跟踪层位形相关
耦合 (K_adapt = ω²·M̃ 各向异性, 稳态跟踪误差 ≈ M̃⁻¹F/ω²), 滤波器层 (ratios_xc)
仍严格解耦 (< 1e-3). 故本测试断言: EE 级 < 10% (可接受线), 滤波器层 < 1e-3 (严格).

本测试运行完整 GAC decouple 仿真 (headless), 断言:
  1. EE 级耦合比 ratios[0, 2] = |Δz|/|Δx| < 10%  (计划 §4.2 可接受线; 竖直舒适位下 ≈7.7%)
  2. 滤波器输出 ratios_xc[0, 2] < 1e-3           (滤波器层完美解耦, 严格)
"""
import os
import sys

import pytest

_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_PROJECT_ROOT,
           os.path.join(_PROJECT_ROOT, 'se3_control'),
           os.path.join(_PROJECT_ROOT, 'se3_control', 'scripts')):
    if _p not in sys.path:
        sys.path.insert(0, _p)


@pytest.mark.simulation
def test_gac_decouple_z_oscillation_regression():
    """GAC 方向解耦: X 向力不引起 Z 向漂移 (Z 振荡回归)."""
    import verify_gac_mujoco as V
    from config.robot_configs import get_robot_config

    cfg = get_robot_config('ur12e')
    log, robot = V.run_verification(
        'ur12e.urdf', task='regulation', show_viewer=False,
        link_to_mesh=cfg['link_to_mesh'],
        mesh_subdir=cfg['mesh_subdir'],
        experiment='decouple',
        decouple_force=10.0, decouple_moment=1.0,
        decouple_settle=1.0, decouple_measure=0.5,
        verbose=False)
    res = log.get('decouple')
    assert res is not None, "decouple 分析结果缺失 (仿真/分析失败)"
    # ── 1. EE 级: Fx → Δz 耦合比 < 10% (计划 §4.2 可接受线; 竖直舒适位下 ≈7.7%) ──
    assert res['ratios'][0, 2] < 0.10, \
        f"Fx → Δz 耦合比 {res['ratios'][0, 2]*100:.2f}% ≥ 10% (Z 振荡回归失败)"
    # ── 2. 滤波器输出: Fx → Δz 耦合 < 0.1% (滤波器层应完美解耦) ──
    assert res['ratios_xc'][0, 2] < 1e-3, \
        f"滤波器输出 Fx → Δz 耦合 {res['ratios_xc'][0, 2]:.2e} ≥ 1e-3"
