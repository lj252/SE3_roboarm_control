"""
arm_log.py — 实机/仿真控制回路的统一 CSV 记录 (诊断"真机 vs 仿真"差异)
======================================================================

真机跑 run_se3_control.py 时（--log-dir）与 MuJoCo 预览（--log-dir）都写
**同一格式**的每控制周期 CSV，字段完全对齐，这样 analyze_arm_log.py 可以把
实机与仿真逐列对照，定位"仿真正常、真机向上抬/折叠撞"的差异来源。

每行记录 (nv=6):
  t, bf, pos_err, rot_err,                    # 时间 / 混合系数 / 位置误差 / 旋转误差
  pd_x/y/z, pd_ref_x/y/z, p_x/y/z,            # 混合后期望 / 轨迹参考 / 实际末端位置
  q0..5, dq0..5,                              # 实际关节角 / 速度
  q_servo0..5,                                # 下发给 servoJ 的目标位 (directTorque 时=实际 q)
  dq_des0..5,                                 # 桥接器参考速度 (directTorque 时=NaN)
  tau0..5, tau_lim0..5,                       # GIC 力矩 / 该关节力矩限幅

关键诊断信号:
  - q_servo - q 增大 → servoJ 参考积分漂移 (内层伺服追不上 → 真机会乱动)
  - dq_des 顶到 ±dq_max → 参考速度饱和 (级联不稳的征兆)
  - |tau| 顶到 tau_lim → 力矩饱和 (臂达不到期望 → 误差累积)
  - pos_err / rot_err 后半段增长 → 闭环发散
  - p_z 先升后塌 / 偏离 pd_z → "向上抬然后折叠"的笛卡尔表现
"""

import csv
import os

import numpy as np


def arm_log_columns(nv: int):
    """固定列名 (两处写日志共用, 保证格式一致).

    注意顺序必须与 :func:`arm_log_row` 完全一致: row 按数组打包 (先全部 q, 再全部
    dq, ...), 所以这里也按数组分块命名, **不能**按关节交叉排列 (否则同一份 CSV
    头尾对不上, 读出来的 q/dq/tau 全是错的).
    """
    cols = ['t', 'bf', 'pos_err', 'rot_err',
            'pd_x', 'pd_y', 'pd_z',
            'pd_ref_x', 'pd_ref_y', 'pd_ref_z',
            'p_x', 'p_y', 'p_z']
    for base in ('q', 'dq', 'q_servo', 'dq_des', 'tau', 'tau_lim'):
        cols += [f'{base}{i}' for i in range(nv)]
    return cols


def arm_log_row(nv, t, bf, pos_err, rot_err, pd, pd_ref, p,
                q, dq, q_servo, dq_des, tau, tau_lim):
    """按固定列序组装一行 (所有字段转为 float)."""
    row = [float(t), float(bf), float(pos_err), float(rot_err)]
    for arr in (pd, pd_ref, p, q, dq, q_servo, dq_des, tau, tau_lim):
        row += [float(x) for x in np.asarray(arr, dtype=float).ravel()[:nv]]
    return row


class ArmCsvLogger:
    """每控制周期增量写 CSV (每行 flush → 崩溃/断电时已记录数据都在)."""

    def __init__(self, path: str, nv: int):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self.path = path
        self._f = open(path, 'w', newline='')
        self._w = csv.writer(self._f)
        self._w.writerow(arm_log_columns(nv))
        self._f.flush()

    def write(self, row) -> None:
        self._w.writerow(row)
        self._f.flush()   # 每周期 flush, 崩溃时保留数据

    def close(self) -> None:
        try:
            self._f.close()
        except Exception:
            pass
