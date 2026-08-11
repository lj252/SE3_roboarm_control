#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
monitor_sim.py — 仿真/实机 GIC 控制回路内部量监控 (倾斜圆诊断专用)
====================================================================

在 monitor_rtde.py 基础上改写（RTDE 实机取样 → 仿真 CSV 逐周期重建）。
monitor_rtde.py 监控**原始 RTDE 数据**（真机）；本脚本监控**控制器内部量**
（仿真与真机日志都能用），专门盯会导致 circle 画圆倾斜的数据。

为什么需要它
------------
circle 画出的圆是倾斜平面（§11.3），但 arm_log CSV 只记录
t/bf/pd/p_ref/p/q/dq/tau 等外部量，看不到控制器内部发生了什么。
本脚本按 GICController.compute() 的公式（core/gic_controller.py 逐行对应）
从 CSV 里的 (q, dq, pd, bf, t) **重建控制器内部量**，逐周期输出：

  * dVd*_z      期望体坐标加速度的 z 分量（应≈0，参考是水平圆）
  * FF_z = M̃·dVd* 的 z 分量 —— **M̃ 非对角耦合把水平向心前馈泄漏成 z 力**（当前主嫌疑）
  * e_op_z / ev_z  位姿/速度误差 z 分量
  * corr_z = −ω²e_op_z − 2ζω·ev_z（反馈项）
  * plant_cmd_z = (M̃inv·τ̃)_z = P@dVd* − 2ζω·P@ev − ω²·P@e_op（闭环期望 z 加速度）
  * Vḃ_z = d/dt(Vb_z)（数值差分）—— 实际体 z 加速度
  * jb_dot_qd_z = Ĵb·q̇（数值差分）—— 体坐标系框架/科氏项
  * resid_z = Vḃ_z − plant_cmd_z − jb_dot_qd_z —— **植物残差**（≈0 说明模型与仿真一致）
  * M̃[2,:] / P[2,:] 行 —— 操作空间惯性耦合 / SVD 阻尼后投影
  * 平面拟合 z = a·x + b·y + c —— 倾斜量直接量化
  * --fit-bandwidth：扫描 ω 使重算 τ 与 CSV 记录 τ 最吻合（directTorque 模式）

输入是 run_se3_control.py --log-dir 写的统一 CSV（仿真 --preview 与实机同格式），
只读不写源文件，可对已有日志事后分析，也可 tail 实时跟随（另一终端同时开）。

用法::

  conda activate roboarm

  # 1) 事后分析已有仿真日志 (directTorque 模式; --fit-bandwidth 自动找有效带宽)
  python monitor_sim.py --robot ur3 --task circle \
      --csv logs/sim_dt_full/sim_20260811_134942.csv --fit-bandwidth

  # 2) 事后分析真机日志 (servoJ: CSV 的 tau 是内层伺服力矩, 不能拟合带宽, 显式给 ω)
  python monitor_sim.py --robot ur3 --task circle --csv logs/run_03 \
      --bandwidth 10 --once

  # 3) 与 run_se3_control --preview 同开: 终端2 tail 实时跟随 (Ctrl+C 结束)
  #    终端1: python se3_control/scripts/run_se3_control.py --robot ur3 \\
  #           --control-mode servoJ --task circle --duration 16 --bandwidth 10 \\
  #           --preview --log-dir logs/mon_run
  #    终端2: python monitor_sim.py --robot ur3 --task circle --csv logs/mon_run

  # 自定义圆心/半径要与运行命令一致 (覆盖后同样按此重建轨迹)
  python monitor_sim.py --robot ur3 --task circle \
      --csv logs/run_x --center -0.40 0.0 0.224 --radius 0.05 --once

输出:
  * 扩展 CSV (默认 logs/monitor_sim_<时间>.csv): 每个控制周期一行内部量
  * 控制台: 实时状态行 + 结束时 z 行强迫分解 / 耦合结构 / 平面拟合报告

注意:
  * --bandwidth/--damping 必须与运行时的控制器参数一致, 否则 report 里的
    corr/cl_cmd/plant_cmd/tau 会偏 (τ 残差会变大提示你). 直接用 --fit-bandwidth
    让脚本自己找 ω 最省事 (仅 directTorque 模式有效).
  * 本脚本重建的是控制器内部量 (GIC 前馈/反馈分解), 不重复 monitor_rtde.py
    的 RTDE 字段 (q/dq/TCP/安全状态等原始数据另看 monitor_rtde.py).
"""

import argparse
import csv
import glob
import os
import sys
import time

import numpy as np

# 让脚本从任意工作目录都能 import se3_control (与 monitor_rtde.py 相同)
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from se3_control.config.robot_configs import get_robot_config, get_urdf_path
from se3_control.config import task_config
from se3_control.robot_model.robot_model import RobotModel
from se3_control.core.se3_math import (
    adjoint_g_ed, adjoint_g_ed_deriv, vee_map, rotmat_slerp,
)
from se3_control.core.trajectory import build_trajectory, eval_body_twist


NV = 6  # UR3 / UR12e 都是 6 自由度

# 体速度 Vb 的线性 z 分量索引: Vb = [vx, vy, vz, wx, wy, wz] → z = 索引 2
Z = 2


# ====================================================================
# 1. 控制器内部量重建 (与 core/gic_controller.py 逐行对应)
# ====================================================================

def compute_gic_internals(robot, q, dq, pd, Rd, vd, wd, dvd, dwd):
    """按 GICController.compute() 的公式重建全部**带宽无关**中间量.

    只计算不依赖 ω/ζ 的量 (FK / 动力学 / M̃ / SE(3) 误差 / 前馈基),
    供监控输出与 --fit-bandwidth 共用。与 core/gic_controller.py 的
    compute() 逐行对应; 若改控制律必须同步这里。

    :param robot: RobotModel 实例 (Pinocchio)
    :param q/dq:  关节位置/速度 (nv,)
    :param pd:    期望位置 (3,)  — **混合后** (CSV 的 pd 列, 即控制器实际收到值)
    :param Rd:    期望朝向 (3,3) — 混合后 (slerp)
    :param vd/wd/dvd/dwd: 体坐标系期望速度/加速度 (3,1), eval_body_twist 输出
    :returns: dict — 带宽无关的中间量 (见各键注释)
    """
    robot.update(q, dq)
    p, R = robot.get_pose()
    M = robot.get_full_inertia()
    nv = M.shape[0]
    bias = robot.get_bias_torque()
    Jb = robot.get_body_jacobian()

    pd = np.asarray(pd, dtype=float).ravel()
    vd = np.asarray(vd, dtype=float).ravel()
    wd = np.asarray(wd, dtype=float).ravel()
    dvd = np.asarray(dvd, dtype=float).ravel()
    dwd = np.asarray(dwd, dtype=float).ravel()

    # ── SE(3) 位姿变换 ──
    g = np.eye(4); g[:3, :3] = R; g[:3, 3] = p
    gd = np.eye(4); gd[:3, :3] = Rd; gd[:3, 3] = pd
    g_ed = np.linalg.inv(g) @ gd

    # ── 期望速度变换到体坐标系 ──
    # 与 gic_controller.py 同步: 用 ravel 拼接保证块序 [v; w].
    # (eval_body_twist 返回 (3,1) 列, np.hstack 会造成交错序错位,
    #  详见 gic_controller.py 修复注释 — 该 bug 曾让画圆平面倾斜 ~11mm.)
    Vd = np.concatenate([np.asarray(vd).ravel(), np.asarray(wd).ravel()]).reshape((-1, 1))
    dVd = np.concatenate([np.asarray(dvd).ravel(), np.asarray(dwd).ravel()]).reshape((-1, 1))
    Vb = robot.get_body_ee_velocity()                      # (6,1) 当前体速度
    Vd_star = adjoint_g_ed(g_ed) @ Vd
    # dVd* 含 d/dt(Ad_{g_ed}) 项 (与 gic_controller.py line 102-103 相同)
    dVd_star = (adjoint_g_ed_deriv(g, gd, Vb[:3], Vb[3:], vd, wd) @ Vd
                + adjoint_g_ed(g_ed) @ dVd)

    # ── SE(3) 误差 (体坐标系) ──
    e_pos = R.T @ (p - pd).reshape((-1, 1))
    e_rot = vee_map(Rd.T @ R - R.T @ Rd)
    e_op = np.vstack((e_pos, e_rot))                       # (6,1)
    ev = Vb - Vd_star                                      # (6,1)

    # ── 操作空间惯性 M̃ (SVD 阻尼伪逆, 与 gic_controller.py line 116-122 相同) ──
    M_inv = np.linalg.solve(M, np.eye(nv))
    M_tilde_inv = Jb @ M_inv @ Jb.T
    U_t, s_t, Vt_t = np.linalg.svd(M_tilde_inv)
    damp_sv = max(1e-6, 0.1 * s_t[-1]) if len(s_t) > 0 else 1e-6
    s_damped = s_t / (s_t**2 + damp_sv**2)
    M_tilde = (Vt_t.T * s_damped) @ U_t.T
    P = M_tilde_inv @ M_tilde                               # SVD 阻尼投影

    # ── 带宽无关的前馈/反馈基向量 ──
    #   τ̃ = M̃·dVd* − D·ev − K·e_op,  D=2ζωM̃, K=ω²M̃
    #   → 只有 (M̃·dVd*), (M̃·ev), (M̃·e_op) 不依赖 ω/ζ, 其余按 ω 在导出时组合.
    Mevd = (M_tilde @ dVd_star).ravel()    # FF = M̃·dVd* 前馈力
    Mev  = (M_tilde @ ev).ravel()
    Meop = (M_tilde @ e_op).ravel()
    P_dvd = (P @ dVd_star).ravel()         # 投影后前馈
    P_ev  = (P @ ev).ravel()
    P_eop = (P @ e_op).ravel()

    return dict(p=p, R=R, Vb=Vb, e_op=e_op, ev=ev, dVd_star=dVd_star,
                M_tilde=M_tilde, P=P, Jb=Jb, bias=bias,
                Mevd=Mevd, Mev=Mev, Meop=Meop,
                P_dvd=P_dvd, P_ev=P_ev, P_eop=P_eop)


# ====================================================================
# 2. 扩展 CSV 列 (顺序必须与 derive_row 完全一致 — 按数组分块, 勿交叉)
# ====================================================================

def monitor_columns():
    cols = ['t', 'bf',
            'p_x', 'p_y', 'p_z', 'pd_x', 'pd_y', 'pd_z', 'ep_z_world']
    for base in ('q', 'dq'):
        cols += [f'{base}{i}' for i in range(NV)]
    cols += ['Vb_x', 'Vb_y', 'Vb_z', 'Vb_wx', 'Vb_wy', 'Vb_wz',
             'Vb_dot_z', 'plant_cmd_z', 'jb_dot_qd_z', 'resid_z',
             'dVd_star_x', 'dVd_star_y', 'dVd_star_z',
             'FF_x', 'FF_y', 'FF_z',
             'e_op_x', 'e_op_y', 'e_op_z',
             'ev_x', 'ev_y', 'ev_z',
             'corr_z', 'cl_cmd_z',
             'proj_dvd_z', 'proj_ev_z', 'proj_eop_z',
             'tau_tilde_z', 'tau_z', 'tau_lim_z']
    cols += [f'Mtilde2_{i}' for i in range(NV)]
    cols += [f'P2_{i}' for i in range(NV)]
    cols += ['Mtilde_zz']
    return cols


def derive_row(s0, s1, s2, w, zeta):
    """由三个相邻样本 (s0, s1, s2) 导出中间样本 s1 的扩展行.

    Vḃ 与 Ĵb·q̇ 需要 s1 前后两帧做中心差分. 各样本 = (t, bf, src_row, internals).

    :param w/zeta: 运行时控制器带宽/阻尼 (ω/ζ) — 决定反馈项与 τ̃
    :returns: list — 与 monitor_columns() 对应的行
    """
    t, bf, src, it = s1
    dt = s2[0] - s0[0]
    if dt <= 0:
        dt = 1e-6

    # ── 数值差分 (带宽无关) ──
    dVb = (s2[3]['Vb'].ravel() - s0[3]['Vb'].ravel()) / dt          # V̇b
    Jb_dot = (s2[3]['Jb'] - s0[3]['Jb']) / dt
    dq1 = np.array([src['dq%d' % i] for i in range(NV)])
    Jb_dot_qd = (Jb_dot @ dq1.reshape((-1, 1))).ravel()             # Ĵb·q̇

    # ── 按 ω 组合反馈/力矩 (基向量带宽无关) ──
    w2 = w * w
    z2w = 2.0 * zeta * w
    dvd_s = it['dVd_star'].ravel()
    Mevd, Mev, Meop = it['Mevd'], it['Mev'], it['Meop']
    P_dvd, P_ev, P_eop = it['P_dvd'], it['P_ev'], it['P_eop']
    eop = it['e_op'].ravel()
    ev = it['ev'].ravel()

    corr = (-z2w * Mev - w2 * Meop).ravel()                          # 反馈项
    cl_cmd = dvd_s + corr                                           # 闭环期望加速度
    plant = (P_dvd - z2w * P_ev - w2 * P_eop).ravel()               # (M̃inv·τ̃)
    tau_tilde = (Mevd - z2w * Mev - w2 * Meop).ravel()              # 操作空间力矩
    tau_cmd = (it['Jb'].T @ tau_tilde.reshape((-1, 1))
               + it['bias'].reshape((-1, 1))).ravel()               # 关节力矩 (未限幅)
    resid = dVb[Z] - plant[Z] - Jb_dot_qd[Z]                        # 植物残差

    Vb1 = it['Vb'].ravel()
    row = [t, bf]
    row += [float(src['p_%s' % c]) for c in 'xyz']
    row += [float(src['pd_%s' % c]) for c in 'xyz']
    row += [float(src['p_z']) - float(src['pd_z'])]                 # ep_z_world
    for i in range(NV):
        row.append(float(src['q%d' % i]))
    for i in range(NV):
        row.append(float(src['dq%d' % i]))
    row += [float(v) for v in Vb1]
    row += [float(dVb[Z]), float(plant[Z]), float(Jb_dot_qd[Z]), float(resid)]
    row += [float(v) for v in dvd_s[:3]]
    row += [float(v) for v in Mevd[:3]]                             # FF = M̃·dVd*
    row += [float(v) for v in eop[:3]]
    row += [float(v) for v in ev[:3]]
    row += [float(corr[Z]), float(cl_cmd[Z])]
    row += [float(P_dvd[Z]), float(P_ev[Z]), float(P_eop[Z])]
    row += [float(tau_tilde[Z]), float(src['tau%d' % Z]),
            float(src['tau_lim%d' % Z])]
    for i in range(NV):
        row.append(float(it['M_tilde'][Z, i]))
    for i in range(NV):
        row.append(float(it['P'][Z, i]))
    row += [float(it['M_tilde'][Z, Z])]
    return row


# ====================================================================
# 3. 带宽拟合 (directTorque 模式: 扫 ω 使重算 τ 最接近 CSV 记录 τ)
# ====================================================================

def fit_bandwidth(samples, zeta, max_n=2000):
    """扫描有效闭环带宽 ω ∈ [0.1, 10], 使 ‖τ_re(ω) − τ_csv‖ 最小.

    τ_re = Jbᵀ·(M̃·dVd* − 2ζω·M̃·ev − ω²·M̃·e_op) + bias
    基向量 (M̃·dVd*, M̃·ev, M̃·e_op, Jb, bias) 都来自 samples 的 internals,
    与 ω 无关 → 只需逐 ω 做线性组合, 很快.

    只用稳态 (bf≥0.999) 且无力矩饱和的样本 (饱和会破坏 τ 与 ω 的线性关系).

    :returns: (best_w, best_resid_N, n_used) — 样本不足返回 (None, None, n)
    """
    X0, X1, X2, Y = [], [], [], []
    n = 0
    for t, bf, src, it in samples:
        if bf < 0.999:
            continue
        tau_c = np.array([src['tau%d' % i] for i in range(NV)])
        tl = np.array([src['tau_lim%d' % i] for i in range(NV)])
        if not np.all(np.abs(tau_c) < 0.99 * tl):
            continue
        Jb = it['Jb']
        X0.append((Jb.T @ it['Mevd'] + it['bias']).ravel())
        X1.append((Jb.T @ it['Mev']).ravel())
        X2.append((Jb.T @ it['Meop']).ravel())
        Y.append(tau_c)
        n += 1
        if n >= max_n:
            break
    if n < 20:
        return None, None, n

    X0 = np.asarray(X0).T   # (6, N)
    X1 = np.asarray(X1).T
    X2 = np.asarray(X2).T
    Y = np.asarray(Y).T

    def resid(ww):
        tre = X0 - 2.0 * zeta * ww * X1 - ww * ww * X2
        return float(np.mean(np.sqrt(np.sum((tre - Y) ** 2, axis=0))))

    best_w, best_r = None, np.inf
    for ww in np.linspace(0.1, 10.0, 190):
        r = resid(ww)
        if r < best_r:
            best_w, best_r = ww, r
    lo, hi = max(0.02, best_w - 0.25), best_w + 0.25
    for ww in np.linspace(lo, hi, 100):
        r = resid(ww)
        if r < best_r:
            best_w, best_r = ww, r
    return best_w, best_r, n


# ====================================================================
# 4. 源 CSV 读取 (tail 实时跟随 / 事后一次性)
# ====================================================================

def source_rows(path, tail, idle_timeout):
    """逐行读取 run_se3_control --log-dir 的统一 CSV.

    :param tail: True 持续等待新行 (与运行同时开); False 读到 EOF 即止.
    :param idle_timeout: tail 下文件无新行的最长等待 (s)
    :yields: dict — 每行按列名映射 (数值已转 float)
    """
    f = open(path, 'r', newline='')
    header = None
    ncol = None
    buf = ''
    last_growth = time.time()
    try:
        while True:
            chunk = f.read(65536)
            if chunk:
                buf += chunk
                last_growth = time.time()
            else:
                if not tail:
                    break
                if time.time() - last_growth > idle_timeout:
                    break
                time.sleep(0.1)
                continue
            lines = buf.split('\n')
            buf = lines[-1]                       # 残行 (文件可能刚写一半) 留到下一块
            for ln in lines[:-1]:
                ln = ln.strip()
                if not ln:
                    continue
                parts = ln.split(',')
                if header is None:
                    # 只接受完整表头 (含 q0 列), 避免半行被当成表头
                    if len(parts) >= 20 and any(p.startswith('q0') for p in parts):
                        header = parts
                        ncol = len(parts)
                    continue
                if len(parts) != ncol:
                    continue                     # 不完整行, 跳过
                row = {}
                ok = True
                for k, v in zip(header, parts):
                    try:
                        row[k] = float(v)
                    except ValueError:
                        ok = False
                        break
                if ok:
                    yield row
    finally:
        f.close()


def resolve_source(path):
    """--csv 可以是文件或目录 (目录则取最新一个 *.csv)."""
    if path is None:
        print("需要 --csv <文件或目录> 指定仿真日志 CSV "
              "(run_se3_control --log-dir 产物, 与 --preview 同用则记仿真)")
        sys.exit(2)
    if os.path.isdir(path):
        csvs = sorted(glob.glob(os.path.join(path, '*.csv')),
                      key=os.path.getmtime)
        if not csvs:
            print(f"目录 {path} 中没有 CSV")
            sys.exit(2)
        return csvs[-1]
    if not os.path.exists(path):
        print(f"找不到 {path}")
        sys.exit(2)
    return path


# ====================================================================
# 5. 报告与状态行
# ====================================================================

def status_line(sample):
    """实时状态行 (仅带宽无关量, 不依赖最终 ω)."""
    t, bf, src, it = sample
    eop = it['e_op'].ravel()
    ev = it['ev'].ravel()
    return (f"t={t:6.2f}s | pz={src['p_z']:6.3f} "
            f"epz_w={(src['p_z'] - src['pd_z']) * 1000:6.1f}mm "
            f"| eop_z={eop[Z] * 1000:6.1f}mm ev_z={ev[Z] * 1000:6.1f}mm/s "
            f"| FF_z={it['Mevd'][Z]:6.3f}N | M̃zz={it['M_tilde'][Z, Z]:5.2f}kg")


def print_report(rows, src_path, mode, w, zeta, fit_info):
    """结束时打印 z 行强迫分解 / 耦合结构 / 平面拟合报告."""
    print("\n" + "=" * 68)
    print("  监控报告 — GIC 内部量 (z 行强迫分解)")
    print("=" * 68)
    if len(rows) < 10:
        print("样本太少, 跳过报告")
        return

    def arr(name):
        return np.array([r[name] for r in rows])

    t_all = arr('t')
    bf = arr('bf')
    steady = bf >= 0.999
    st = [r for r in rows if r['bf'] >= 0.999]

    print(f"  来源: {src_path}")
    print(f"  模式: {mode} "
          f"({'CSV tau=GIC 力矩 (可拟合带宽)' if mode == 'directTorque' else 'CSV tau=内层伺服力矩'})")
    print(f"  GIC 参数: ω={w:.3f} rad/s  ζ={zeta:.1f}")
    print(f"  稳态样本: N={np.sum(steady)} / 总 {len(rows)} "
          f"(t={t_all[0]:.2f}..{t_all[-1]:.2f}s, 起步混合 {np.sum(~steady)} 点)")

    if fit_info is not None:
        fw, fr, fn = fit_info
        if fw is not None:
            print(f"  └─ --fit-bandwidth: ω_eff={fw:.3f} rad/s, "
                  f"拟合残差 ‖τ_re−τ_csv‖={fr * 1000:.1f} mN·m ({fn} 未饱和稳态样本)")

    # ── [1] 世界系高度 (圆是否水平) ──
    p = np.array([[r['p_x'], r['p_y'], r['p_z']] for r in st])
    A = np.column_stack([p[:, 0], p[:, 1], np.ones(len(p))])
    a = np.linalg.lstsq(A, p[:, 2], rcond=None)[0]
    z_fit = A @ a
    pz = arr('p_z')[steady]
    print(f"\n  [1] 世界系高度 (圆是否水平, 稳态 {len(pz)} 点)")
    print(f"      p_z: 均值 {np.mean(pz):.4f} m  极差 {np.ptp(pz) * 1000:.1f} mm")
    print(f"      平面拟合: z = {a[0]:.4f}·x {a[1]:+.4f}·y {a[2]:+.4f}")
    print(f"      平面残差 z-std = {np.std(pz - z_fit) * 1000:.2f} mm  "
          f"({'≈0 → 轨迹精确落在倾斜平面上' if np.std(pz - z_fit) * 1000 < 1.0 else '≠0 → 不只是倾斜平面'})")

    # ── [2] z 行强迫分解 (体坐标系, z=index2) ──
    def rms(name):
        v = arr(name)[steady]
        return float(np.sqrt(np.mean(v ** 2)))

    def mean(name):
        return float(np.mean(arr(name)[steady]))

    print(f"\n  [2] z 行强迫分解 (体坐标系 z=index2, 稳态 RMS)")
    labels = [('dVd*_z', 'dVd_star_z', 'm/s²', '期望 z 加速度 (参考水平 → 应≈0)'),
              ('FF_z', 'FF_z', 'N', 'M̃·dVd* 前馈 z 分量 (M̃ 非对角泄漏 → 应≈0!)'),
              ('e_op_z', 'e_op_z', 'mm', '位姿误差 z (体坐标)'),
              ('ev_z', 'ev_z', 'mm/s', '速度误差 z (体坐标)'),
              ('corr_z', 'corr_z', 'mm/s²', '-ω²e_op_z - 2ζω·ev_z (反馈项)'),
              ('cl_cmd_z', 'cl_cmd_z', 'mm/s²', 'dVd*_z + corr_z (闭环期望加速度)'),
              ('plant_cmd_z', 'plant_cmd_z', 'mm/s²', '(M̃inv·τ̃)_z (经 P 投影)'),
              ('Vḃ_z', 'Vb_dot_z', 'mm/s²', '实际体 z 加速度 (数值差分)'),
              ('Ĵb·q̇_z', 'jb_dot_qd_z', 'mm/s²', '体框架/科氏项'),
              ('resid_z', 'resid_z', 'mm/s²', 'Vḃ_z − plant − Ĵb·q̇ (≈0=模型一致)')]
    for name, key, unit, note in labels:
        scale = 1.0 if unit == 'N' else (1.0 if unit == 'm/s²' else 1000.0)
        if unit in ('mm', 'mm/s', 'mm/s²'):
            v = rms(key) * 1000.0
            m = mean(key) * 1000.0
            print(f"      {name:11s} RMS={v:8.3f} {unit}  均值={m:+8.3f}   {note}")
        else:
            v = rms(key)
            m = mean(key)
            print(f"      {name:11s} RMS={v:8.4f} {unit}  均值={m:+8.4f}   {note}")

    # ── [3] 耦合结构 ──
    Mt_row = np.median([[r[f'Mtilde2_{i}'] for i in range(NV)] for r in st], axis=0)
    P_row = np.median([[r[f'P2_{i}'] for i in range(NV)] for r in st], axis=0)
    Mzz = float(np.median([r['Mtilde_zz'] for r in st]))
    print(f"\n  [3] 耦合结构 (稳态中位数)")
    print(f"      M̃[2,:] = [{', '.join(f'{v:.3f}' for v in Mt_row)}]  "
          f"(M̃[2,2]={Mzz:.3f} kg, z 有效质量)")
    print(f"      P[2,:]  = [{', '.join(f'{v:.4f}' for v in P_row)}]  "
          f"(SVD 阻尼投影, 主对角≈1 → z 行几乎解耦)")

    # ── [4] z 闭环平衡 ──
    clz = arr('cl_cmd_z')[steady]
    vbdz = arr('Vb_dot_z')[steady]
    plz = arr('plant_cmd_z')[steady]
    rez = arr('resid_z')[steady]
    c1 = float(np.corrcoef(clz, vbdz)[0, 1]) if len(clz) > 2 else float('nan')
    c2 = float(np.corrcoef(plz, vbdz)[0, 1]) if len(plz) > 2 else float('nan')
    print(f"\n  [4] z 闭环平衡 (稳态)")
    print(f"      corr(cl_cmd_z, Vḃ_z)    = {c1:+.3f}   (期望 vs 实际 z 加速度)")
    print(f"      corr(plant_cmd_z, Vḃ_z) = {c2:+.3f}   (植物方程一致性)")
    print(f"      resid_z: RMS={np.sqrt(np.mean(rez ** 2)) * 1000:.2f} mm/s²  "
          f"max={np.max(np.abs(rez)) * 1000:.2f} mm/s²")
    print("\n  结论提示: FF_z≠0 且 e_op_z≠0 → 前馈泄漏经闭环产生 z 向稳态误差;"
          "\n  resid_z≈0 说明重建的控制器内部量与 MuJoCo 物理一致 (根因在控制律本身).")
    print("=" * 68)


# ====================================================================
# 6. 主入口
# ====================================================================

def parse_args():
    parser = argparse.ArgumentParser(
        description="监控仿真/实机 GIC 控制回路内部量 (倾斜圆诊断, 由 monitor_rtde.py 改写)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--robot', type=str, default='ur3',
                        choices=['ur3', 'ur12e'], help="机器人类型 (选 URDF/任务配置)")
    parser.add_argument('--task', type=str, default='circle',
                        choices=['circle', 'line', 'regulation'], help="任务类型 (重建轨迹用)")
    parser.add_argument('--csv', type=str, default=None,
                        help="源 CSV (run_se3_control --log-dir 产物); 文件或目录 (目录取最新)")
    parser.add_argument('--out', type=str, default=None,
                        help="扩展 CSV 输出 (默认 logs/monitor_sim_<时间>.csv)")
    parser.add_argument('--bandwidth', type=float, default=20.0,
                        help="运行时 GIC 带宽 ω (rad/s). directTorque 仿真建议用 --fit-bandwidth 自动找")
    parser.add_argument('--damping', type=float, default=1.0,
                        help="运行时阻尼比 ζ (必须与运行一致)")
    parser.add_argument('--fit-bandwidth', action='store_true',
                        help="扫描 ω 使重算 τ 最接近 CSV 记录 τ (仅 directTorque 模式有效)")
    parser.add_argument('--center', type=float, nargs=3, metavar=('X', 'Y', 'Z'),
                        default=None, help="覆盖轨迹圆心/中点 (须与运行命令一致)")
    parser.add_argument('--radius', type=float, default=None,
                        help="覆盖 circle 半径 (须与运行命令一致)")
    parser.add_argument('--once', action='store_true',
                        help="只处理当前已有行即退出 (默认 tail 实时跟随, 无新行 idle_timeout 后退出)")
    parser.add_argument('--idle-timeout', type=float, default=3.0,
                        help="tail 模式下文件无新行的最长等待 (s)")
    parser.add_argument('--report', type=float, default=1.0,
                        help="控制台状态行间隔 (s, 按日志 t 计)")
    return parser.parse_args()


def main():
    args = parse_args()

    src_path = resolve_source(args.csv)
    print(f"源日志: {src_path}")

    cfg = get_robot_config(args.robot)
    robot = RobotModel(get_urdf_path(args.robot), ee_frame_name=cfg['ee_frame'],
                       robot_name=cfg['name'], verbose=False)

    # 重建轨迹 (按 --robot/--task + 可选覆盖, 与 run_se3_control 同一逻辑)
    task_cfg = task_config.get_task_config(args.robot)
    if args.center is not None:
        tdict = getattr(task_cfg, args.task, None)
        if isinstance(tdict, dict) and 'center' in tdict:
            tdict['center'] = list(args.center)
    if args.radius is not None and args.task == 'circle':
        task_cfg.circle['radius'] = args.radius
    traj = build_trajectory(args.task, cfg=task_cfg)
    print(f"轨迹: {args.task} ({cfg['name']})  起点 {np.round(traj.pd_t(0.0).ravel(), 3)} m")

    # ── 主循环: 逐行重建带宽无关内部量 ──
    samples = []            # (t, bf, src_row, internals)
    mode = 'directTorque'
    R_start = None
    t_last = -1e9
    t0_wall = time.time()
    try:
        for src in source_rows(src_path, tail=not args.once,
                               idle_timeout=args.idle_timeout):
            if R_start is None:
                q0 = np.array([src['q%d' % i] for i in range(NV)])
                dq0 = np.array([src['dq%d' % i] for i in range(NV)])
                robot.update(q0, dq0)
                _, R_start = robot.get_pose()      # 起步朝向 (bf=0 时臂在起点)
                dqdes = [src.get('dq_des%d' % i, np.nan) for i in range(NV)]
                mode = ('directTorque'
                        if all(np.isnan(x) for x in dqdes) else 'servoJ')

            t = float(src['t'])
            bf = float(src['bf'])
            pd = np.array([src['pd_%s' % c] for c in 'xyz'])   # 混合后期望 (CSV 原样)
            Rd_ref = traj.Rd_t(t).reshape(3, 3)
            Rd = rotmat_slerp(R_start, Rd_ref, bf)
            vd, wd, dvd, dwd = eval_body_twist(traj, t, Rd, bf)
            q = np.array([src['q%d' % i] for i in range(NV)])
            dq = np.array([src['dq%d' % i] for i in range(NV)])
            it = compute_gic_internals(robot, q, dq, pd, Rd, vd, wd, dvd, dwd)
            samples.append((t, bf, src, it))

            if t - t_last >= args.report:
                t_last = t
                print(f"[{time.time() - t0_wall:6.1f}s] " + status_line(samples[-1]))
    except KeyboardInterrupt:
        print("\nCtrl+C 停止")

    if not samples:
        print("没有读到任何数据行 — 检查 --csv 路径与文件内容")
        return
    print(f"已读取 {len(samples)} 个控制周期 (模式={mode})")

    # ── 带宽拟合 (可选) ──
    fit_info = None
    w = args.bandwidth
    if args.fit_bandwidth:
        if mode == 'servoJ':
            print("⚠️  servoJ 日志的 tau 是内层伺服力矩, 带宽拟合不适用 — 请用 --bandwidth 显式给出")
        else:
            w, wres, n = fit_bandwidth(samples, args.damping)
            if w is None:
                print("⚠️  未饱和稳态样本不足 (<20), 无法拟合带宽 — 用 --bandwidth 显式给出")
            else:
                fit_info = (w, wres, n)
                print(f"带宽拟合: ω_eff={w:.3f} rad/s  (ζ={args.damping}, {n} 样本)")

    # ── 导出扩展行 + 写 CSV ──
    cols = monitor_columns()
    out = args.out or os.path.join(
        'logs', time.strftime('monitor_sim_%Y%m%d_%H%M%S.csv'))
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    rows = []
    for i in range(1, len(samples) - 1):            # 中心差分需前后帧 → 丢首尾各一
        row = derive_row(samples[i - 1], samples[i], samples[i + 1], w, args.damping)
        if len(row) != len(cols):
            print(f"⚠️  行宽不符 (row={len(row)}, cols={len(cols)}) — 列定义与导出不一致!")
            break
        rows.append(dict(zip(cols, row)))
    with open(out, 'w', newline='') as f:
        cw = csv.writer(f)
        cw.writerow(cols)
        cw.writerows([list(r.values()) for r in rows])
    print(f"已写扩展 CSV → {out} ({len(rows)} 行)")

    print_report(rows, src_path, mode, w, args.damping, fit_info)


if __name__ == '__main__':
    main()
