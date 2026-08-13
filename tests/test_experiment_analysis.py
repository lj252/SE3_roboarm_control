"""实验分析库 (core/experiment_analysis.py) 单元测试.

覆盖:
  - 实验一 (正弦扫频): fit_sinusoid, filter_transfer, extract_sweep_bode
  - 实验二 (方向解耦): build_decouple_inputs, rotation_vector,
    extract_decouple (完美解耦对角矩阵 / 交叉耦合比)
"""
import os
import sys
import numpy as np
import pytest

_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from se3_control.core.experiment_analysis import (fit_sinusoid,
                                                  filter_transfer,
                                                  extract_sweep_bode,
                                                  build_decouple_inputs,
                                                  build_decouple_loop_inputs,
                                                  rotation_vector,
                                                  extract_decouple)


# ====================================================================
# 实验一: 正弦扫频
# ====================================================================

def test_fit_sinusoid_recovers_amp_phase():
    """合成信号 y = DC + amp·cos(2πft − φ) 应被精确恢复."""
    t = np.linspace(0.0, 3.0, 3001)   # 非整周期窗口 (3s @ 0.7Hz)
    freq = 0.7
    amp_true, ph_true, dc_true = 0.05, 0.7, -0.002
    y = dc_true + amp_true * np.cos(2 * np.pi * freq * t - ph_true)
    amp, ph = fit_sinusoid(t, y, freq)
    assert amp == pytest.approx(amp_true, abs=1e-8)
    assert ph == pytest.approx(ph_true, abs=1e-8)


def test_fit_sinusoid_without_dc_biased_but_with_dc_exact():
    """非整周期窗 + DC: 含 DC 项拟合精确, 无 DC 项会污染相位."""
    t = np.linspace(0.0, 3.0, 3001)
    freq, amp, dc = 0.7, 0.05, 0.01
    y = dc + amp * np.cos(2 * np.pi * freq * t)
    amp_fit, ph_fit = fit_sinusoid(t, y, freq)
    assert amp_fit == pytest.approx(amp, abs=1e-8)
    assert ph_fit == pytest.approx(0.0, abs=1e-8)


def test_filter_transfer_dc_gain():
    """DC 增益应为 1/K."""
    M, D, K = 10.0, 2 * np.sqrt(500 * 10), 500.0
    g, _ = filter_transfer([1e-6], M, D, K)
    assert g[0] == pytest.approx(1.0 / K, rel=1e-6)


def test_filter_transfer_lagging():
    """低频相位应为负 (滞后), 且随频率滞后增大."""
    M, D, K = 10.0, 2 * np.sqrt(500 * 10), 500.0
    _, ph1 = filter_transfer([0.5], M, D, K)
    _, ph2 = filter_transfer([2.0], M, D, K)
    assert ph1[0] < 0
    assert ph2[0] < ph1[0]


def _make_synthetic_log(freqs, settle, measure, force_amp=5.0, axis=0,
                        dt=0.001, M=10.0, D=141.42, K=500.0):
    """构造理想扫频日志: 响应 = H(jω)·F, R 恒等 (体=世界系)."""
    block = settle + measure
    n = int(len(freqs) * block / dt)
    t = np.arange(n) * dt
    p = np.zeros((n, 3))
    pd = np.zeros((n, 3))
    R = np.zeros((n, 3, 3))
    xc = np.zeros((n, 6))
    for i in range(n):
        R[i] = np.eye(3)
        k = int(t[i] // block)
        if k < len(freqs):
            f = freqs[k]
            tl = t[i] - k * block
            g, ph = filter_transfer([f], M, D, K)
            # 线性响应: F=A·cos(θ) 经 H=g·e^{jφ} → x = A·g·cos(θ+φ)
            x = (force_amp * g[0]) * np.cos(2 * np.pi * f * tl + ph[0])
            p[i, axis] = pd[i, axis] + x
            xc[i, axis] = x
    return {'t': t, 'p': p, 'pd': pd, 'R': R, 'x_corr': xc}


def test_extract_sweep_bode_recovers_theory():
    """扫频日志的滤波器增益/相位应等于理论 H(jω)."""
    freqs = [0.5, 1.0, 2.0, 5.0]
    settle, measure = 2.0, 3.0
    log = _make_synthetic_log(freqs, settle, measure)
    res = extract_sweep_bode(log, freqs, settle, measure, 5.0, axis=0,
                             use_xc=True)
    M, D, K = 10.0, 141.42, 500.0
    g_theo, ph_theo = filter_transfer(freqs, M, D, K)
    np.testing.assert_allclose(res['gain_xc'], g_theo, rtol=1e-6)
    np.testing.assert_allclose(res['gain_ee'], g_theo, rtol=1e-6)
    np.testing.assert_allclose(res['phase_xc'], np.degrees(ph_theo),
                               atol=1e-6)
    np.testing.assert_allclose(res['phase_ee'], np.degrees(ph_theo),
                               atol=1e-6)


def test_extract_sweep_bode_world_projection():
    """体→世界系投影: R 非恒等时, 世界系 x 位移来自体轴投影后仍应吻合."""
    freqs = [0.5, 1.0]
    settle, measure = 2.0, 3.0
    log = _make_synthetic_log(freqs, settle, measure, axis=0)
    # 把体系 X_corr 旋进一个任意 R, 验证 extract 内部投影恢复世界系响应
    # (任意正交矩阵: 绕 z 轴 0.6 rad, 再绕新 y 轴 0.4 rad — 手动构造避免 scipy 依赖)
    def _rot_z(a):
        c, s = np.cos(a), np.sin(a)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    def _rot_y(a):
        c, s = np.cos(a), np.sin(a)
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    R0 = _rot_z(0.6) @ _rot_y(0.4)
    n = len(log['t'])
    for i in range(n):
        log['R'][i] = R0
        log['x_corr'][i, :3] = R0.T @ log['p'][i]  # 体系修正量 (逆旋转)
    res = extract_sweep_bode(log, freqs, settle, measure, 5.0, axis=0,
                             use_xc=True)
    M, D, K = 10.0, 141.42, 500.0
    g_theo, ph_theo = filter_transfer(freqs, M, D, K)
    np.testing.assert_allclose(res['gain_xc'], g_theo, rtol=1e-6)
    np.testing.assert_allclose(res['phase_xc'], np.degrees(ph_theo),
                               atol=1e-6)


# ====================================================================
# 实验二: 方向解耦
# ====================================================================

def _exp_so3(v):
    """SO(3) 指数映射 (Rodrigues), 局部 helper 避免 scipy 依赖."""
    v = np.asarray(v, dtype=float)
    th = np.linalg.norm(v)
    if th < 1e-12:
        return np.eye(3)
    K = np.array([[0.0, -v[2], v[1]],
                  [v[2], 0.0, -v[0]],
                  [-v[1], v[0], 0.0]])
    return (np.eye(3) + np.sin(th) / th * K
            + (1.0 - np.cos(th)) / th ** 2 * (K @ K))


def _make_decouple_synthetic_log(settle=1.0, measure=1.0, force=10.0,
                                 moment=1.0, dt=0.001, d_lin=0.020,
                                 d_rot=0.020, fz_coupling=0.0):
    """理想解耦合成日志: 输入 i → 仅第 i 通道响应 (力→平动, 力偶→转动).

    :param fz_coupling: Fx 块附加 z 向耦合 (|Δz|/|Δx|), 默认 0 = 完美解耦
    """
    inputs = build_decouple_inputs(force, moment)
    block = settle + measure
    n = int(len(inputs) * block / dt)
    t = np.arange(n) * dt
    p = np.zeros((n, 3))
    R = np.zeros((n, 3, 3))
    p0 = np.array([0.5, 0.0, 0.6])
    R0 = np.eye(3)
    for i in range(n):
        k = int(t[i] // block)
        p[i] = p0
        R[i] = R0
        if k == 0:
            continue
        if k <= 3:                     # 力块: 平动响应 (沿力方向)
            ch = k - 1                 # 0/1/2
            p[i] = p0 + d_lin * np.eye(3)[ch]
            if ch == 0 and fz_coupling > 0:
                p[i, 2] += fz_coupling * d_lin
        else:                          # 力矩块: 转动响应 (绕力矩轴)
            ch = k - 4                 # 0/1/2 → 转动通道 3/4/5
            R[i] = R0 @ _exp_so3(np.eye(3)[ch] * d_rot)
    return {'t': t, 'p': p, 'R': R}


def test_build_decouple_inputs():
    """输入序列: 基线 0 + 3 力 + 3 力偶, 幅值正确."""
    inp = build_decouple_inputs(force=10.0, moment=1.0)
    assert inp.shape == (7, 6)
    np.testing.assert_allclose(inp[0], np.zeros(6))
    np.testing.assert_allclose(inp[1], [10, 0, 0, 0, 0, 0])
    np.testing.assert_allclose(inp[2], [0, 10, 0, 0, 0, 0])
    np.testing.assert_allclose(inp[3], [0, 0, 10, 0, 0, 0])
    np.testing.assert_allclose(inp[4], [0, 0, 0, 1, 0, 0])
    np.testing.assert_allclose(inp[5], [0, 0, 0, 0, 1, 0])
    np.testing.assert_allclose(inp[6], [0, 0, 0, 0, 0, 1])
    np.testing.assert_allclose(np.linalg.norm(inp[1:], axis=1),
                               [10, 10, 10, 1, 1, 1])


def test_build_decouple_loop_inputs():
    """循环模式序列: 每个动作后跟一个零块 (复位间隙), 共 12 子块."""
    seq = build_decouple_loop_inputs(force=10.0, moment=1.0)
    assert seq.shape == (12, 6)
    # 偶数位是施加动作 (顺序 Fx,Fy,Fz,Mx,My,Mz), 奇数位是复位零块
    np.testing.assert_allclose(seq[1], np.zeros(6))
    np.testing.assert_allclose(seq[0], [10, 0, 0, 0, 0, 0])   # Fx
    np.testing.assert_allclose(seq[2], [0, 10, 0, 0, 0, 0])   # Fy
    np.testing.assert_allclose(seq[6], [0, 0, 0, 1, 0, 0])    # Mx (第 4 个施加块)
    np.testing.assert_allclose(seq[10], [0, 0, 0, 0, 0, 1])   # Mz
    np.testing.assert_allclose(seq[11], np.zeros(6))          # 末尾复位
    # 所有零块 (奇数位) 均为零
    np.testing.assert_allclose(seq[1::2], np.zeros((6, 6)))
    # 所有施加块 (偶数位) 幅值正确
    np.testing.assert_allclose(np.linalg.norm(seq[0::2], axis=1),
                               [10, 10, 10, 1, 1, 1])


def test_rotation_vector_identity():
    """相同旋转 → 零旋转向量."""
    R0 = _exp_so3(np.array([0.0, 0.0, 0.6]))
    assert np.allclose(rotation_vector(R0, R0), 0.0)


def test_rotation_vector_world_frame():
    """Rb = Exp(v)·Ra → 世界系旋转向量 φ = v (Ra 非恒等也成立)."""
    Ra = _exp_so3(np.array([0.0, 0.0, 0.6]))
    v = np.array([0.02, -0.03, 0.05])
    Rb = _exp_so3(v) @ Ra
    phi = rotation_vector(Ra, Rb)
    np.testing.assert_allclose(phi, v, atol=1e-9)


def test_extract_decouple_recovers_diagonal():
    """完美解耦日志 → 耦合矩阵对角 = 响应/输入幅值, 非对角耦合比 ≈ 0."""
    settle, measure = 1.0, 1.0
    log = _make_decouple_synthetic_log(settle, measure)
    res = extract_decouple(log, settle, measure)
    # 力块: 平动主响应 0.020 m / 10 N = 0.002 m/N
    assert res['coupling'][0, 0] == pytest.approx(0.020 / 10.0, abs=1e-9)
    assert res['coupling'][1, 1] == pytest.approx(0.020 / 10.0, abs=1e-9)
    # 力矩块: 转动主响应 0.020 rad / 1 Nm = 0.02 rad/Nm
    assert res['coupling'][3, 3] == pytest.approx(0.020 / 1.0, abs=1e-6)
    # 耦合比: 对角 = 1, 非对角 ≈ 0
    off = np.abs(res['ratios'] - np.eye(6))
    assert np.max(off) < 1e-6


def test_extract_decouple_captures_cross_coupling():
    """Fx 块附加 Δz 耦合 → ratios[0,2] = |Δz|/|Δx| 被正确恢复."""
    settle, measure = 1.0, 1.0
    log = _make_decouple_synthetic_log(settle, measure, fz_coupling=0.02)
    res = extract_decouple(log, settle, measure)
    assert res['ratios'][0, 2] == pytest.approx(0.02, abs=1e-6)
    # 其余轴间耦合仍 ≈ 0
    assert res['ratios'][0, 1] < 1e-6
