#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_arm_log.py — 分析实机/仿真 CSV 记录, 定位"仿真正常、真机乱动"的差异来源
=============================================================================

输入两种 CSV:
  1. **arm_log CSV** (必需): run_se3_control.py --log-dir 或 --preview --log-dir
     每控制周期一行, 列序见 core/arm_log.py, 已含 q / dq / q_servo / dq_des / tau / tau_lim.
  2. **monitor CSV** (可选): monitor_rtde.py 录的原始 RTDE 数据 (q/dq/电流/TCP/安全事件).

输出:
  * 控制台诊断报告: 参考积分漂移 / 力矩饱和 / 误差发散 / "向上抬折叠"特征 逐项判定
  * PNG 图 (输出到 --out 目录):
      errors.png     位置/旋转误差 vs t (对比参考; 多份日志叠加)
      windup.png     q_servo − q 逐关节 (servoJ 参考积分漂移的直接证据)
      torque.png     逐关节 tau / tau_lim 限幅带 (力矩饱和直接证据)
      cartesian.png  TCP x/y/z vs 参考 + 3D 轨迹
      compare.png    多份日志的 pos_err 与 p_z 叠图 (实机 vs 仿真)
      rtde_q.png     (若有 --rtde) RTDE 实测 q / 速度 / 电流 / momentum / 安全事件

典型用法::

  conda activate roboarm

  # 只分析实机控制回路 CSV
  python analyze_arm_log.py --log logs/run_01/Phase2_*.csv --label 实机

  # 实机 vs 仿真 对照 (都录了 --log-dir 才可比)
  python analyze_arm_log.py \
      --log logs/run_01/Phase2_20260810_*.csv --label 实机 \
      --log logs/sim_01/sim_20260810_*.csv   --label 仿真

  # 同时叠上 RTDE 原始数据 (不同时间轴, 单独出图)
  python analyze_arm_log.py --log logs/run_01/Phase2_*.csv --rtde logs/rtde_01.csv

  # 只看文本报告, 不出图
  python analyze_arm_log.py --log ... --no-plots
"""

import argparse
import glob
import os
import sys

import numpy as np

# 让脚本从任意工作目录都能 import se3_control
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from se3_control.core.arm_log import arm_log_columns  # noqa: E402

DQ_MAX_DEFAULT = 2.0  # ServoJTorqueBridge 默认期望速度限幅 (rad/s)


# ====================================================================
# 加载
# ====================================================================
def load_arm_log(path):
    """读 run_se3_control --log-dir 的 CSV → 带名字的 np 数组 dict."""
    import csv
    with open(path, newline='') as f:
        rows = list(csv.reader(f))
    header, data = rows[0], rows[1:]
    if not data:
        raise ValueError(f"{path}: 空文件 (无数据行)")

    nv = sum(1 for c in header if c.startswith('q') and c[1:].isdigit())
    if nv == 0:
        raise ValueError(f"{path}: 未找到 q* 列, 不是 arm_log 格式?")
    exp = arm_log_columns(nv)
    if header[:len(exp)] != exp:
        raise ValueError(f"{path}: 列头与 arm_log_columns(nv={nv}) 不一致")
    arr = np.array([[float(x) if x else np.nan for x in r[:len(exp)]]
                    for r in data])

    def col(name):  # 按列名取整列
        i = exp.index(name)
        return arr[:, i]

    def block(prefix, n):
        # 数值分块: q0..q5 / dq0.. / ... 按数字后缀
        idxs = [exp.index(f'{prefix}{j}') for j in range(n)]
        return arr[:, idxs]

    def axis3(*names):
        return arr[:, [exp.index(name) for name in names]]

    return {
        'path': path,
        'nv': nv,
        't': col('t'), 'bf': col('bf'),
        'pos_err': col('pos_err'), 'rot_err': col('rot_err'),
        'pd': axis3('pd_x', 'pd_y', 'pd_z'),
        'pd_ref': axis3('pd_ref_x', 'pd_ref_y', 'pd_ref_z'),
        'p': axis3('p_x', 'p_y', 'p_z'),
        'q': block('q', nv), 'dq': block('dq', nv),
        'q_servo': block('q_servo', nv), 'dq_des': block('dq_des', nv),
        'tau': block('tau', nv), 'tau_lim': block('tau_lim', nv),
    }


def load_rtde(path):
    """读 monitor_rtde.py 的 CSV → dict (t_wall, q, dq, momentum, ...)."""
    import csv
    with open(path, newline='') as f:
        rows = list(csv.reader(f))
    header, data = rows[0], rows[1:]
    if not data:
        raise ValueError(f"{path}: 空文件")
    arr = np.array([[float(x) if x else np.nan for x in r] for r in data])
    col = {name: i for i, name in enumerate(header)}

    def get(name, n=1):
        if name not in col:
            return np.full((arr.shape[0], n), np.nan)
        v = arr[:, col[name]]
        if n == 1:
            return v
        return np.column_stack([arr[:, col[f'{name}{j}']]
                                for j in range(n)])

    return {
        'path': path,
        't': arr[:, col['t_wall']],
        'q': get('q', 6), 'dq': get('dq', 6),
        'current': get('current', 6),
        'momentum': get('momentum'),
        'speed_scaling': get('speed_scaling'),
        'safety_mode': get('safety_mode'),
        'runtime_state': get('runtime_state'),
    }


# ====================================================================
# 诊断指标
# ====================================================================
def metrics(log, dq_max=None):
    """从一份 arm_log 计算诊断指标."""
    nv = log['nv']
    t = log['t']
    n = len(t)
    half = max(1, n // 2)
    if dq_max is None:
        dq_max = DQ_MAX_DEFAULT
    if np.ndim(dq_max) == 0:
        dq_max = np.full(nv, dq_max)

    q_servo, q = log['q_servo'], log['q']
    windup = q_servo - q                        # (N,nv) 参考积分漂移
    windup_rms2 = np.sqrt(np.mean(windup[half:] ** 2, axis=0))  # 后半段 RMS
    windup_max = np.max(np.abs(windup), axis=0)

    dq_des = log['dq_des']
    dqsat = np.mean(np.abs(dq_des) >= 0.99 * dq_max, axis=0)  # 期望速度饱和占比

    tau, tau_lim = log['tau'], log['tau_lim']
    with np.errstate(divide='ignore', invalid='ignore'):
        tau_ratio = np.abs(tau) / np.where(tau_lim > 0, tau_lim, 1.0)
    tausat = np.mean(tau_ratio >= 0.95, axis=0)               # 力矩饱和占比

    # 后半段误差斜率 (去掉起步混合段干扰)
    i1 = half
    if n - i1 > 2:
        tc = t[i1:] - t[i1]
        slope_pos = float(np.polyfit(tc, log['pos_err'][i1:], 1)[0])
        slope_rot = float(np.polyfit(tc, log['rot_err'][i1:], 1)[0])
    else:
        slope_pos = slope_rot = 0.0

    pz = log['p'][:, 2]
    pz_ref = log['pd_ref'][:, 2]
    pz_dev = pz - pz_ref
    pz_dev_max = float(np.max(pz_dev[half:]))     # 后半段抬升量
    pz_dev_min = float(np.min(pz_dev[half:]))     # 后半段塌落量

    return {
        't': t, 'n': n, 'nv': nv,
        'windup': windup, 'windup_rms2': windup_rms2, 'windup_max': windup_max,
        'dqsat': dqsat, 'tausat': tausat, 'tau_ratio': tau_ratio,
        'slope_pos': slope_pos, 'slope_rot': slope_rot,
        'pos_err_final': float(log['pos_err'][-1]),
        'pos_err_max': float(np.max(log['pos_err'])),
        'rot_err_final': float(log['rot_err'][-1]),
        'rot_err_max': float(np.max(log['rot_err'])),
        'pz_dev_max': pz_dev_max, 'pz_dev_min': pz_dev_min,
    }


def verdict(m):
    """根据指标给出一段可读结论 + 风险标记列表."""
    nv = m['nv']
    warn = []

    wj = np.argsort(m['windup_rms2'])[::-1]
    if m['windup_rms2'][wj[0]] > 0.05:
        j = wj[0]
        warn.append(f"关节{j+1} 参考积分漂移 RMS={m['windup_rms2'][j]*1000:.0f} mrad "
                    f"(q_servo−q 后半段平均) — servoJ 内层追不上参考, 真机会乱动")

    tj = np.argsort(m['tausat'])[::-1]
    if m['tausat'][tj[0]] > 0.20:
        j = tj[0]
        warn.append(f"关节{j+1} 力矩饱和占比 {m['tausat'][j]*100:.0f}% — 臂达不到期望, 误差累积")

    if m['slope_pos'] > 2e-3 and m['pos_err_final'] > 0.08:
        warn.append(f"位置误差持续增长 (后半段斜率 {m['slope_pos']*1000:.1f} mm/s, "
                    f"终值 {m['pos_err_final']*100:.0f} cm) — 闭环发散")
    elif m['pos_err_final'] > 0.15:
        warn.append(f"位置误差终值 {m['pos_err_final']*100:.0f} cm — 显著超差")

    if m['pz_dev_max'] > 0.05 and m['pz_dev_min'] < -0.05:
        warn.append(f"TCP z 先抬 {m['pz_dev_max']*100:.0f} cm 后塌 {abs(m['pz_dev_min'])*100:.0f} cm "
                    f"(相对参考) — 正是'向上抬然后折叠'的笛卡尔特征")

    if m['rot_err_final'] > 0.3:
        warn.append(f"旋转误差终值 {m['rot_err_final']:.2f} rad — 姿态明显偏了")

    if not warn:
        return "✓ 基本正常: 无积分漂移/力矩饱和/误差发散/折叠特征."
    s = "⚠️ 发现风险:\n"
    s += "\n".join(f"  - {w}" for w in warn)
    return s


# ====================================================================
# 绘图
# ====================================================================
CJK_FONTS = ['Noto Sans CJK SC', 'Noto Sans CJK TC', 'WenQuanYi Zen Hei',
             'AR PL UMing CN', 'Droid Sans Fallback']
CJK_FONT_FILES = [
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
    '/usr/share/fonts/truetype/arphic/uming.ttc',
    '/usr/share/fonts/truetype/arphic/ukai.ttc',
]


def _setup_plt(show):
    import matplotlib
    if not show:
        matplotlib.use('Agg')
    # 优先用系统中文字体 (图内中文标注), 找不到就回退英文 DejaVu
    from matplotlib import font_manager
    for path in CJK_FONT_FILES:
        if os.path.exists(path):
            try:
                font_manager.fontManager.addfont(path)
            except Exception:
                pass
    avail = {f.name for f in font_manager.fontManager.ttflist}
    for fam in CJK_FONTS:
        if fam in avail:
            matplotlib.rcParams['font.sans-serif'] = [fam, 'DejaVu Sans']
            break
    matplotlib.rcParams['axes.unicode_minus'] = False
    import matplotlib.pyplot as plt
    return plt


def plot_all(logs, ms, rtde, out_dir, show):
    plt = _setup_plt(show)
    labels = [l for _, l in logs]
    colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple']

    # ── errors.png ──
    fig, ax = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    for (log, lab), c in zip(logs, colors):
        ax[0].plot(log['t'], log['pos_err'], c, lw=1.2, label=lab)
        ax[1].plot(log['t'], log['rot_err'], c, lw=1.2, label=lab)
    ax[0].set_ylabel('pos_err (m)'); ax[0].legend(); ax[0].grid(alpha=0.3)
    ax[1].set_ylabel('rot_err (rad)'); ax[1].set_xlabel('t (s)')
    ax[1].legend(); ax[1].grid(alpha=0.3)
    fig.suptitle('跟踪误差 vs 时间')
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'errors.png'), dpi=110)
    if show:
        plt.show()
    plt.close(fig)

    # ── windup.png (第一份日志) ──
    log, m = logs[0][0], ms[0]
    fig, ax = plt.subplots(m['nv'], 1, figsize=(10, 2.2 * m['nv']), sharex=True)
    for j in range(m['nv']):
        ax[j].plot(log['t'], m['windup'][:, j] * 1000, lw=1.0)
        ax[j].axhline(0, color='k', lw=0.5)
        ax[j].set_ylabel(f'j{j+1} (mrad)')
        ax[j].grid(alpha=0.3)
    ax[-1].set_xlabel('t (s)')
    fig.suptitle('servoJ 参考积分漂移 q_servo − q (实机单位: 仅 q_servo 有值)')
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'windup.png'), dpi=110)
    if show:
        plt.show()
    plt.close(fig)

    # ── torque.png ──
    fig, ax = plt.subplots(m['nv'], 1, figsize=(10, 2.2 * m['nv']), sharex=True)
    for j in range(m['nv']):
        tl = log['tau_lim'][:, j]
        ax[j].plot(log['t'], log['tau'][:, j], lw=1.0)
        ax[j].plot(log['t'], tl, 'r--', lw=0.8, label='tau_lim')
        ax[j].plot(log['t'], -tl, 'r--', lw=0.8)
        ax[j].set_ylabel(f'j{j+1} (N·m)')
        ax[j].legend(loc='upper right', fontsize=8)
        ax[j].grid(alpha=0.3)
    ax[-1].set_xlabel('t (s)')
    fig.suptitle('GIC 力矩 vs 限幅')
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'torque.png'), dpi=110)
    if show:
        plt.show()
    plt.close(fig)

    # ── cartesian.png ──
    fig = plt.figure(figsize=(12, 6))
    ax = fig.add_subplot(1, 2, 1)
    for (log, lab), c in zip(logs, colors):
        ax.plot(log['t'], log['p'][:, 2], color=c, lw=1.2, label=f'{lab} z')
        ax.plot(log['t'], log['pd_ref'][:, 2], color=c, ls='--', lw=0.9, alpha=0.6)
    ax.set_ylabel('z (m)'); ax.set_xlabel('t (s)'); ax.legend(); ax.grid(alpha=0.3)
    ax2 = fig.add_subplot(1, 2, 2, projection='3d')
    for (log, lab), c in zip(logs, colors):
        ax2.plot(log['pd_ref'][:, 0], log['pd_ref'][:, 1], log['pd_ref'][:, 2],
                 color=c, ls='--', lw=0.8, alpha=0.5)
        ax2.plot(log['p'][:, 0], log['p'][:, 1], log['p'][:, 2], color=c, lw=1.2, label=lab)
    ax2.set_xlabel('x'); ax2.set_ylabel('y'); ax2.set_zlabel('z')
    ax2.legend()
    fig.suptitle('TCP 轨迹: 实线=实际, 虚线=参考')
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'cartesian.png'), dpi=110)
    if show:
        plt.show()
    plt.close(fig)

    # ── compare.png (>=2 份日志) ──
    if len(logs) >= 2:
        fig, ax = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
        for (log, lab), c in zip(logs, colors):
            ax[0].plot(log['t'], log['pos_err'], c, lw=1.2, label=lab)
            ax[1].plot(log['t'], log['p'][:, 2] - log['pd_ref'][:, 2], c, lw=1.2, label=lab)
        ax[0].set_ylabel('pos_err (m)'); ax[0].legend(); ax[0].grid(alpha=0.3)
        ax[1].set_ylabel('p_z − ref (m)'); ax[1].legend(); ax[1].grid(alpha=0.3)
        ax[1].set_xlabel('t (s)')
        fig.suptitle('多份日志对照 (同一控制周期时间轴)')
        fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'compare.png'), dpi=110)
        if show:
            plt.show()
        plt.close(fig)

    # ── rtde_q.png (若有 RTDE) ──
    if rtde is not None:
        r = rtde
        fig, ax = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
        for j in range(6):
            ax[0].plot(r['t'], r['q'][:, j], lw=1.0)
        ax[0].set_ylabel('q (rad)'); ax[0].grid(alpha=0.3)
        for j in range(6):
            ax[1].plot(r['t'], r['dq'][:, j], lw=1.0)
        ax[1].set_ylabel('dq (rad/s)'); ax[1].grid(alpha=0.3)
        ax[2].plot(r['t'], np.nanmean(np.abs(r['current']), axis=1), lw=1.0)
        ax[2].set_ylabel('|current| 均值 (A)'); ax[2].grid(alpha=0.3)
        ax[3].plot(r['t'], r['momentum'], lw=1.0, label='momentum')
        ax[3].plot(r['t'], r['speed_scaling'], lw=1.0, label='speed_scaling')
        ax[3].plot(r['t'], r['safety_mode'], lw=1.0, label='safety_mode')
        ax[3].legend(); ax[3].set_xlabel('t_wall (s)'); ax[3].grid(alpha=0.3)
        fig.suptitle('RTDE 原始数据 (t_wall 时间轴)')
        fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'rtde_q.png'), dpi=110)
        if show:
            plt.show()
        plt.close(fig)


# ====================================================================
# 主流程
# ====================================================================
def main():
    ap = argparse.ArgumentParser(
        description="分析实机/仿真 arm_log CSV + 可选 RTDE CSV, 输出诊断报告与图",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument('--log', action='append', default=[], required=True,
                    help="arm_log CSV (可重复; 支持通配符), 如 logs/run_01/Phase2_*.csv")
    ap.add_argument('--label', action='append', default=[],
                    help="与 --log 一一对应的标签 (默认 日志1/2/...)")
    ap.add_argument('--rtde', type=str, default=None,
                    help="monitor_rtde.py 的 CSV (可选)")
    ap.add_argument('--out', type=str, default=None,
                    help="输出目录 (默认 ./analysis_output/<时间>)")
    ap.add_argument('--dq-max', type=float, default=DQ_MAX_DEFAULT,
                    help="servoJ 期望速度限幅 (rad/s), 用于 dq_des 饱和判定")
    ap.add_argument('--no-plots', action='store_true', help="只出文本报告不出图")
    ap.add_argument('--show', action='store_true', help="交互显示图 (需桌面)")
    args = ap.parse_args()

    logs = []
    for pat in args.log:
        matched = sorted(glob.glob(pat))
        if not matched:
            ap.error(f"--log {pat} 没有匹配到文件")
        for path in matched:
            logs.append(load_arm_log(path))
    if args.label:
        if len(args.label) != len(logs):
            ap.error("--label 数量必须与展开后的 --log 文件数一致")
        labels = args.label
    else:
        labels = [f'日志{i+1}' for i in range(len(logs))]

    rtde = load_rtde(args.rtde) if args.rtde else None

    ms = [metrics(l, dq_max=args.dq_max) for l in logs]

    print("=" * 72)
    print("诊断报告 (每份日志独立判定)")
    print("=" * 72)
    for (log, lab), m in zip(zip(logs, labels), ms):
        print(f"\n── {lab}  ({os.path.basename(log['path'])})  "
              f"t∈[{m['t'][0]:.2f},{m['t'][-1]:.2f}]s  n={m['n']}  nv={m['nv']} ──")
        print(f"  pos_err:  max {m['pos_err_max']*100:5.1f} cm, 终值 {m['pos_err_final']*100:5.1f} cm"
              f", 后半段斜率 {m['slope_pos']*1000:+.1f} mm/s")
        print(f"  rot_err:  max {m['rot_err_max']:.3f} rad, 终值 {m['rot_err_final']:.3f} rad"
              f", 后半段斜率 {m['slope_rot']*1000:+.1f} mrad/s")
        print(f"  p_z 相对参考: 最高抬 {m['pz_dev_max']*100:+.1f} cm, 最低塌 {m['pz_dev_min']*100:+.1f} cm")
        jd = np.argmax(m['windup_rms2'])
        print(f"  积分漂移 RMS(后半): {[f'{v*1000:.0f}' for v in m['windup_rms2']]} mrad"
              f" (最差 关节{jd+1})")
        jt = np.argmax(m['tausat'])
        print(f"  力矩饱和占比: {[f'{v*100:.0f}%' for v in m['tausat']]}"
              f" (最差 关节{jt+1})")
        print(f"  dq_des 饱和占比: {[f'{v*100:.0f}%' for v in m['dqsat']]}")
        print("  " + verdict(m).replace("\n", "\n  "))

    if not args.no_plots:
        os.makedirs(args.out or 'analysis_output', exist_ok=True)
        out_dir = args.out or 'analysis_output'
        plot_all(list(zip(logs, labels)), ms, rtde, out_dir, args.show)
        print(f"\n图已保存 → {os.path.abspath(out_dir)}/errors.png 等")
    if args.show:
        import matplotlib.pyplot as plt
        plt.show()


if __name__ == '__main__':
    main()
