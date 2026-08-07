"""
实验分析工具库 — 实验一 (正弦扫频) + 实验二 (方向解耦)
====================================================

纯 numpy 数学函数 (绘图函数为 matplotlib 可选), 与控制器无关, GIC/GAC 脚本共用.
用法示例见 se3_control/docs/plan/force_interaction_experiments_plan.md §3-4.

实验一 (正弦扫频) 约定:
  - 扫频力输入: 世界系单轴正弦力  F = A·cos(2π·f·t_local),
    t_local 以当前频率块起点为 0.
  - fit_sinusoid 用最小二乘把响应拟合到 cos/sin 基上,
    返回 (幅值, 相位), 相位为相对输入 cos 的滞后角:
        y = amp·cos(2πft − phase)
  - Bode 幅值增益: |响应| / A (位移 m/N 或修正量 m/N).

实验二 (方向解耦) 约定:
  - 依次施加 +x/+y/+z 恒力 与 绕 x/y/z 恒力偶 (世界系), 共 7 块
    (索引 0 = 零输入基线). 每块稳态位姿相对基线位姿的差为响应
    [Δp; Δφ], 组成 6×6 静态耦合矩阵与耦合比.
"""

import os

import numpy as np


# ====================================================================
# 实验一: 正弦扫频 (频率响应)
# ====================================================================

def fit_sinusoid(t: np.ndarray, y: np.ndarray, freq: float):
    """稳态周期信号单频最小二乘拟合 (含 DC 项).

    :param t:    时间 (s), 应与输入力同时间基准
    :param y:    响应信号
    :param freq: 激励频率 (Hz)
    :returns: (amplitude, phase_rad)
        amplitude — 响应幅值;
        phase_rad — 响应相对输入 cos(2πft) 的滞后角 (rad).

    模型: y ≈ DC + c·cos(2πft) + s·sin(2πft)
          = DC + amp·cos(2πft − phase), phase = atan2(s, c).

    DC 项必不可少: 稳态窗若跨非整数个周期, DC/谐波会泄漏进 cos/sin
    基, 污染幅值与相位.
    """
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    c = np.cos(2.0 * np.pi * freq * t)
    s = np.sin(2.0 * np.pi * freq * t)
    A = np.column_stack([np.ones_like(t), c, s])
    sol, *_ = np.linalg.lstsq(A, y, rcond=None)
    amplitude = np.hypot(sol[1], sol[2])
    phase = np.arctan2(sol[2], sol[1])
    return amplitude, phase


def sweep_windows(freqs, settle, measure):
    """扫频实验中每个频率的稳态测量窗口 (绝对时间).

    :param freqs:   频率序列 (Hz)
    :param settle:  每频率过渡时间 (s)
    :param measure: 每频率稳态测量时间 (s)
    :returns: [(t_start, t_end, freq), ...] — 测量窗绝对起止时间
    """
    block = settle + measure
    windows = []
    for k, f in enumerate(freqs):
        start = k * block + settle
        windows.append((start, start + measure, float(f)))
    return windows


def extract_sweep_bode(log, freqs, settle, measure, force_amp, axis,
                       use_xc=True):
    """从扫频仿真日志提取逐频幅值/相位 (均在世界系, 与输入力同系).

    :param log:      仿真日志 dict, 需含 't','p','pd'; 若 use_xc 还需
                     'x_corr'(体系) 与 'R'(末端朝向)
    :param freqs:    频率序列 (Hz)
    :param settle:   每频率过渡时间 (s)
    :param measure:  每频率稳态测量时间 (s)
    :param force_amp:扫频力幅值 (N)
    :param axis:     施力轴索引 (0/1/2 = x/y/z, 世界系)
    :param use_xc:   是否提取导纳滤波器输出 X_corr 的传递
    :returns: dict — freqs(Hz), gain_ee/phase_ee (末端位移),
        gain_xc/phase_xc (滤波器修正量, 世界系), 均为 ndarray

    帧一致性: 输入力在世界系. 末端位移 dp = p − pd 天然在世界系.
    滤波器 X_corr 在体坐标系, 需投影到世界系 (x_corr_world = R @ x_corr)
    才能与输入力同一参考系比较 —— 否则 regulation 位姿下体 x 轴与
    世界 x 轴不重合, 相位/幅值全部错位.
    """
    t = log['t']
    dp = log['p'] - log['pd']   # 末端相对期望位移 (世界系)
    if use_xc and 'x_corr' in log and 'R' in log:
        # 体坐标系修正量 → 世界系: x_corr_world(t) = R(t) @ x_corr_lin(t)
        # (仅线性 3 分量; 旋转修正量对平动扫频无贡献)
        xc_world = np.matmul(log['R'], log['x_corr'][:, :3, np.newaxis])[..., 0]
    else:
        use_xc = False
    res = {'freqs': [], 'gain_ee': [], 'phase_ee': [],
           'gain_xc': [], 'phase_xc': []}
    for k, f in enumerate(freqs):
        start = k * (settle + measure) + settle
        end = k * (settle + measure) + (settle + measure)
        m = (t >= start) & (t < end)
        if m.sum() < 20:
            continue
        tm = t[m] - k * (settle + measure)   # 与输入同时间基准
        amp_ee, ph_ee = fit_sinusoid(tm, dp[m, axis], f)
        res['freqs'].append(float(f))
        res['gain_ee'].append(amp_ee / force_amp)
        # fit_sinusoid 返回 φ (滞后角为正), 传输参数 ψ = −φ.
        # 统一用 ∠H(jω) 约定 (滞后为负), 与 filter_transfer 一致.
        res['phase_ee'].append(-np.degrees(ph_ee))
        if use_xc:
            amp_xc, ph_xc = fit_sinusoid(tm, xc_world[m, axis], f)
            res['gain_xc'].append(amp_xc / force_amp)
            res['phase_xc'].append(-np.degrees(ph_xc))
    for key in ('gain_ee', 'phase_ee', 'gain_xc', 'phase_xc'):
        res[key] = np.asarray(res[key], dtype=float)
    res['freqs'] = np.asarray(res['freqs'], dtype=float)
    # 相位解卷: 高阶次滞后超过 ±180° 会产生环绕跳变, 解卷后连续单调递减
    for key in ('phase_ee', 'phase_xc'):
        if len(res[key]) > 1:
            res[key] = np.degrees(np.unwrap(np.deg2rad(res[key])))
    return res


def filter_transfer(freqs, M, D, K):
    """二阶导纳滤波器理论传递函数 H(s) = 1/(M·s² + D·s + K).

    :param freqs: 频率 (Hz)
    :param M:     虚拟质量 (对角标量)
    :param D:     虚拟阻尼
    :param K:     虚拟刚度
    :returns: (gain, phase_rad) — gain 单位与 1/K 相同 (m/N 或 rad/Nm)
    """
    w = 2.0 * np.pi * np.asarray(freqs, dtype=float)
    denom_re = K - M * w ** 2
    denom_im = D * w
    gain = 1.0 / np.hypot(denom_re, denom_im)
    phase = -np.arctan2(denom_im, denom_re)
    return gain, phase


# ====================================================================
# 实验二: 方向解耦 (静态 6×6 耦合矩阵)
# ====================================================================

def build_decouple_inputs(force: float = 10.0, moment: float = 1.0):
    """构造方向解耦实验的世界系输入序列 (7,6).

    索引 0 为零输入基线; 索引 1-3 依次为 +x/+y/+z 恒力;
    索引 4-6 依次为绕 x/y/z 恒力偶 (纯力矩, 无平移分量).

    :param force:  轴向力幅值 (N)
    :param moment: 力偶幅值 (Nm)
    :returns: ndarray (7,6) — 每行 [fx, fy, fz, tx, ty, tz] (世界系)
    """
    zero = np.zeros(6)
    return np.array([
        zero,                                   # 0: 基线
        [force, 0.0, 0.0, 0, 0, 0],             # 1: +Fx
        [0.0, force, 0.0, 0, 0, 0],             # 2: +Fy
        [0.0, 0.0, force, 0, 0, 0],             # 3: +Fz
        [0, 0, 0, moment, 0.0, 0.0],            # 4: +Mx (力偶)
        [0, 0, 0, 0.0, moment, 0.0],            # 5: +My
        [0, 0, 0, 0.0, 0.0, moment],            # 6: +Mz
    ])


def build_decouple_loop_inputs(force: float = 10.0, moment: float = 1.0):
    """构造方向解耦**可视化循环**模式的世界系输入序列 (12,6).

    在每两个施加动作之间插入零块 (复位间隙), 便于观察:
      施加块 → 位移出现; 零块 → 位移回到基线. 序列循环运行
      (块索引对长度取模), 直到关闭 viewer.

    :param force:  轴向力幅值 (N)
    :param moment: 力偶幅值 (Nm)
    :returns: ndarray (12,6) — [Fx,0,Fy,0,Fz,0,Mx,0,My,0,Mz,0]
    """
    units = build_decouple_inputs(force, moment)   # [0, Fx,Fy,Fz, Mx,My,Mz]
    seq = []
    for k in range(1, units.shape[0]):
        seq.append(units[k])          # 施加动作 (1..6)
        seq.append(np.zeros(6))       # 复位间隙
    return np.asarray(seq, dtype=float)


def mean_rotation(Rs: np.ndarray):
    """对一组旋转矩阵取平均 (均值后 SVD 重正交化保证 SO(3)).

    :param Rs: ndarray (N,3,3)
    :returns: (3,3) 平均旋转矩阵
    """
    Rm = np.asarray(Rs, dtype=float).mean(axis=0)
    U, _, Vt = np.linalg.svd(Rm)
    return U @ Vt


def rotation_vector(Ra: np.ndarray, Rb: np.ndarray):
    """Ra → Rb 的世界系旋转向量 (rad), 即 Rb ≈ exp(hat(φ))·Ra.

    :param Ra: 基准旋转矩阵 (3,3)
    :param Rb: 当前旋转矩阵 (3,3)
    :returns: (3,) 世界系旋转向量 φ, ‖φ‖ = 旋转角
    """
    Ra = np.asarray(Ra, dtype=float)
    Rb = np.asarray(Rb, dtype=float)
    R_rel = Ra.T @ Rb                      # 体坐标系相对旋转
    cos_a = np.clip((np.trace(R_rel) - 1.0) / 2.0, -1.0, 1.0)
    theta = np.arccos(cos_a)
    if theta < 1e-10:
        return np.zeros(3)
    sin_t = np.sin(theta)
    if abs(sin_t) < 1e-10:
        # θ ≈ π: 罕见情形, 用矩阵差近似 (幅值 ≈ θ)
        v = np.array([R_rel[2, 1] - R_rel[1, 2],
                      R_rel[0, 2] - R_rel[2, 0],
                      R_rel[1, 0] - R_rel[0, 1]])
        n = max(1e-10, np.linalg.norm(v))
        return Ra @ (v * (0.5 * theta / n))
    # 体坐标系旋转向量: vee(R_rel − R_relᵀ) = 2·sinθ·axis_body
    body_vec = (theta / (2.0 * sin_t)) * np.array([
        R_rel[2, 1] - R_rel[1, 2],
        R_rel[0, 2] - R_rel[2, 0],
        R_rel[1, 0] - R_rel[0, 1],
    ])
    return Ra @ body_vec                   # 旋转到世界系


def extract_decouple(log, settle, measure, inputs=None, use_xc=False):
    """从方向解耦仿真日志提取静态响应 / 6×6 耦合矩阵 / 耦合比.

    块结构: 共 n_blocks 块 (默认 7 = 基线 + 6 输入), 每块长 settle+measure,
    测量窗取每块最后 measure 秒. 响应 = 该窗稳态位姿 − 基线块稳态位姿
    (平动用位置均值差, 转动用世界系旋转向量).

    :param log:     仿真日志 dict, 需含 't'(N,), 'p'(N,3), 'R'(N,3,3);
                    use_xc=True 时还需 'x_corr'(N,6) 与 'R' (GAC 滤波器输出).
    :param settle:  每块过渡时间 (s)
    :param measure: 每块稳态测量时间 (s)
    :param inputs:  (7,6) 世界系输入序列; None = build_decouple_inputs()
    :param use_xc:  是否同时提取 GAC 滤波器输出 X_corr 的耦合 (体→世界系投影)
    :returns: dict:
        'inputs'      (7,6)  输入序列
        'responses'   (7,6)  每块稳态 6 维响应 [Δp; Δφ], 第 0 行 ≈ 0
        'coupling'    (6,6)  静态耦合矩阵: C[k,j] = response_{k+1}[j] / 输入幅值
        'ratios'      (6,6)  耦合比: ratios[i,j] = |resp_{i+1}[j]| / |resp_{i+1}[i]|
        'windows'     (7,2)  每块测量窗 [t_start, t_end)
        'responses_xc' (7,3) 仅 use_xc: 滤波器输出平动响应 (世界系)
        'coupling_xc'  (6,3) 仅 use_xc: 滤波器输出耦合矩阵
        'ratios_xc'    (6,3) 仅 use_xc: 滤波器输出耦合比 (力块有意义)
    """
    t = np.asarray(log['t'], dtype=float)
    if inputs is None:
        inputs = build_decouple_inputs()
    inputs = np.asarray(inputs, dtype=float)
    n_blocks = inputs.shape[0]
    block = settle + measure
    windows = np.array([[k * block + settle, (k + 1) * block]
                        for k in range(n_blocks)])

    def _window(k):
        m = (t >= windows[k, 0]) & (t < windows[k, 1])
        if m.sum() < 20:
            raise ValueError(
                f"Block {k} measurement window too short: {m.sum()} samples "
                f"(settle+measure={block:.2f}s, max_time={t[-1]:.2f}s)")
        return m

    def _block_pose(k):
        m = _window(k)
        pk = np.asarray(log['p'])[m].mean(axis=0)
        Rk = mean_rotation(np.asarray(log['R'])[m])
        return pk, Rk

    p0, R0 = _block_pose(0)
    responses = np.zeros((n_blocks, 6))
    for k in range(1, n_blocks):
        pk, Rk = _block_pose(k)
        responses[k, :3] = pk - p0
        responses[k, 3:] = rotation_vector(R0, Rk)

    # 输入幅值 (力 N / 力矩 Nm), 每行对应一个输入
    norms = np.linalg.norm(inputs[1:], axis=1)          # (6,)
    coupling = responses[1:] / norms[:, None]           # (6,6)
    # 耦合比: 以主通道响应幅值为分母 (力块→平动主通道, 力矩块→转动主通道)
    diag = np.abs(np.diag(responses[1:]))
    diag = np.where(diag < 1e-12, np.nan, diag)
    ratios = np.abs(responses[1:]) / diag[:, None]      # (6,6)

    res = {
        'inputs': inputs,
        'responses': responses,
        'coupling': coupling,
        'ratios': ratios,
        'windows': windows,
    }

    if use_xc and 'x_corr' in log and 'R' in log:
        # 滤波器输出 X_corr (体坐标系) → 世界系 (仅平动分量, 与输入力同系)
        xc_world = np.matmul(np.asarray(log['R']),
                             np.asarray(log['x_corr'])[:, :3, np.newaxis])[..., 0]
        xc0 = xc_world[_window(0)].mean(axis=0)
        resp_xc = np.zeros((n_blocks, 3))
        for k in range(1, n_blocks):
            resp_xc[k] = xc_world[_window(k)].mean(axis=0) - xc0
        coupling_xc = resp_xc[1:] / norms[:, None]      # (6,3)
        # 力块 (rows 0-2) 主通道 = 同轴平动; 力矩块无平动主通道 → NaN
        diag_xc = np.abs(np.diag(resp_xc[1:]))          # (3,) 力块主通道
        main_xc = np.concatenate([diag_xc, np.full(3, np.nan)])
        main_xc = np.where(main_xc < 1e-12, np.nan, main_xc)
        ratios_xc = np.abs(resp_xc[1:]) / main_xc[:, None]  # (6,3)
        res['responses_xc'] = resp_xc
        res['coupling_xc'] = coupling_xc
        res['ratios_xc'] = ratios_xc
    return res


_INPUT_LABELS = ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']
_OUTPUT_LABELS = ['Δx', 'Δy', 'Δz', 'Δφx', 'Δφy', 'Δφz']


def print_decouple_report(res, controller_name, threshold=0.05):
    """打印方向解耦结果: 主响应幅值 + 6×6 耦合比矩阵 + 阈值判定.

    阈值判定 (计划 §4.3): 施加轴向力时 |Δz|/|Δx| 等轴间耦合 < 5%,
    力偶同理; 平动↔转动耦合 < 5% (GAC 期望, GIC 作基线记录).

    :param res: extract_decouple() 的返回 dict
    :param controller_name: 'GAC' / 'GIC'
    :param threshold: 耦合比阈值 (0.05 = 5%)
    """
    responses = res['responses']
    ratios = res['ratios']
    print(f"\n{'='*72}")
    print(f"{controller_name} 方向解耦 — 主响应幅值 (每输入只开一个通道)")
    print(f"{'='*72}")
    for i, lab in enumerate(_INPUT_LABELS):
        unit = 'm' if i < 3 else 'rad'
        print(f"  {lab:>3}: Δ[{_OUTPUT_LABELS[i]:>2}] = {responses[i+1, i]:+.6f} {unit}")

    print(f"\n耦合比矩阵 |Δ_out|/|Δ_in|  (%)  "
          f"[对角=主响应, 非对角 <{threshold*100:.0f}% 期望]")
    print(f"{'输入':>6} | " + '  '.join(f'{lab:>5}' for lab in _OUTPUT_LABELS))
    print('-' * 58)
    fail = []
    for i, lab in enumerate(_INPUT_LABELS):
        row = ratios[i]
        print(f"{lab:>6} | " + '  '.join(f'{v*100:5.1f}' if np.isfinite(v)
                                         else '   --' for v in row))
        # 阈值判定: 轴间耦合 (同域) 与 平动↔转动耦合, 均 < threshold
        for j in range(6):
            if i == j:
                continue
            v = row[j]
            if np.isfinite(v) and v > threshold:
                fail.append((lab, _OUTPUT_LABELS[j], v))
    if fail:
        print(f"\n⚠  超出 {threshold*100:.0f}% 阈值 ({controller_name}):")
        for src, dst, v in fail:
            print(f"    {src} → {dst}: {v*100:.2f}%")
    else:
        print(f"\n✅ 所有耦合比 ≤ {threshold*100:.0f}% ({controller_name})")

    if 'coupling_xc' in res:
        print(f"\n滤波器输出 X_corr 平动耦合矩阵 (m/N, 区分滤波器耦合 vs 跟踪层耦合):")
        print(f"{'输入':>6} | " + '  '.join(f'{lab:>10}' for lab in _OUTPUT_LABELS[:3]))
        for i, lab in enumerate(_INPUT_LABELS):
            row = res['coupling_xc'][i]
            print(f"{lab:>6} | " + '  '.join(f'{v:.4e}' for v in row))


def plot_coupling_matrix(res, controller_name, save_path=None):
    """方向解耦耦合矩阵热力图 (左: 静态耦合矩阵, 右: 耦合比%)."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        # 优先使用 CJK 字体渲染中文标签, 缺字体时降级为方框
        try:
            for _f in ('Noto Sans CJK JP', 'Droid Sans Fallback',
                       'AR PL UMing CN'):
                if any(_f in f.name for f in
                       matplotlib.font_manager.fontManager.ttflist):
                    plt.rcParams['font.sans-serif'] = [_f]
                    plt.rcParams['axes.unicode_minus'] = False
                    break
        except Exception:
            pass
    except ImportError:
        print("[Plot] matplotlib not available, skipping coupling plot")
        return

    C = np.asarray(res['coupling'])
    ratios = np.asarray(res['ratios'])
    fig, axes = plt.subplots(1, 2, figsize=(14, 5.4))
    fig.suptitle(f'{controller_name} 方向解耦 — 6×6 静态耦合矩阵', fontsize=14)

    im0 = axes[0].imshow(C, cmap='RdBu_r', aspect='auto')
    axes[0].set_xticks(range(6)); axes[0].set_xticklabels(_OUTPUT_LABELS)
    axes[0].set_yticks(range(6)); axes[0].set_yticklabels(_INPUT_LABELS)
    axes[0].set_title('耦合矩阵 (m/N 或 rad/Nm, 对角 = 1/K)')
    for i in range(6):
        for j in range(6):
            axes[0].text(j, i, f'{C[i, j]:.4f}', ha='center', va='center',
                         fontsize=7)
    fig.colorbar(im0, ax=axes[0])

    ratio_plot = np.where(np.eye(6, dtype=bool), np.nan, ratios * 100.0)
    im1 = axes[1].imshow(ratio_plot, cmap='viridis', vmin=0, vmax=10,
                         aspect='auto')
    axes[1].set_xticks(range(6)); axes[1].set_xticklabels(_OUTPUT_LABELS)
    axes[1].set_yticks(range(6)); axes[1].set_yticklabels(_INPUT_LABELS)
    axes[1].set_title('耦合比 |Δ_out|/|Δ_in| (%) — 阈值 < 5%')
    for i in range(6):
        for j in range(6):
            if i != j:
                axes[1].text(j, i, f'{ratios[i, j]*100:.1f}', ha='center',
                             va='center', fontsize=7)
    fig.colorbar(im1, ax=axes[1])
    plt.tight_layout()

    if save_path:
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"[Plot] Coupling matrix saved to {save_path}")
