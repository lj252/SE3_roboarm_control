#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
monitor_rtde.py — 实机 RTDE 原始数据实时记录 (诊断"仿真正常、真机乱动"的取证工具)
=================================================================================

真机跑任务时（尤其复现"向上抬/折叠碰撞"）在**另一个终端**同时开这个脚本，
把机械臂运动过程中的**原始 RTDE 数据**全部落盘 CSV。它是**只读**的（只连
RTDEReceiveInterface，不连控制口、不发任何指令），完全不影响正在跑的任务。

记录了什么 (每次采样一行):
  * 实际关节角 q / 速度 dq; 目标关节角 target_q / 目标速度 target_qd
      → target_q − q 持续增大 = servoJ 内层参考积分漂移 (仿真里不会有)
  * 电机电流 current / target_current / target_moment
      → CB3 没有 getActualJointTorques, 电流是力矩的最接近代理;
        顶到限幅 = 力矩饱和, 与仿真 CSV 的 tau_lim 对照
  * TCP 位姿 / TCP 速度 (平移+旋转)
      → p_z 先升后塌 = "向上抬然后折叠"的笛卡尔表现
  * robot_status / robot_mode / safety_mode / safety_status_bits / runtime_state
      → 保护性停止 (PROTECTIVE_STOP) 或其他安全事件的确凿证据
  * speed_scaling / momentum / joint_temperatures / 供电电压 等
      → 速度缩放会改变 servoJ 内层增益行为; momentum 是碰撞冲击量的证据

列名与仿真/实机 run_se3_control 的 CSV 不同 (这里只有原始 RTDE), 所以能对齐的
只有 q/dq/TCP; 分析时 monitor CSV 与 arm_log CSV 按 q 一致对齐即可.

用法::

  conda activate roboarm

  # 终端 1: 跑任务 (任意 --log-dir 记录控制回路 CSV)
  python se3_control/scripts/run_se3_control.py --robot ur3 --control-mode servoJ \\
      --task circle --center 0.40 0.0 0.35 --radius 0.05 --duration 16 \\
      --log-dir logs/run_01

  # 终端 2: 同时录原始 RTDE (500 Hz, 直到 Ctrl+C)
  python monitor_rtde.py --robot ur3 --rate 500

  # 固定时长 20 s, 指定输出文件
  python monitor_rtde.py --robot ur3 --rate 500 --duration 20 --out logs/rtde_01.csv

  # 其它机器人 / 其它 IP
  python monitor_rtde.py --robot ur12e --ip 192.168.1.100 --rate 125

Ctrl+C 正常结束 (文件已逐行 flush, 崩溃也保留数据). 录制结果用
analyze_arm_log.py 做图对照.
"""

import argparse
import csv
import os
import sys
import time

import numpy as np

# 让脚本从任意工作目录都能 import se3_control
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)


NV = 6  # UR3 / UR12e 都是 6 自由度


def rtde_columns():
    """monitor CSV 列名 (原始 RTDE 字段, 不含前置的 t_wall 墙钟列).

    注意顺序必须与 ``sample()`` 完全一致: sample 按数组打包 (先全部 q, 再全部
    dq, ...), 所以这里也要按数组分块命名, 不能按关节交叉排列.
    """
    cols = ['t_rtde']  # RTDE 控制器时间戳 (s)
    for base in ('q', 'dq', 'target_q', 'target_qd', 'current',
                 'target_current', 'target_moment', 'temp'):
        cols += [f'{base}{i}' for i in range(NV)]
    cols += ['tcp_x', 'tcp_y', 'tcp_z', 'tcp_rx', 'tcp_ry', 'tcp_rz',
             'tcp_vx', 'tcp_vy', 'tcp_vz', 'tcp_wx', 'tcp_wy', 'tcp_wz',
             'tcp_t_vx', 'tcp_t_vy', 'tcp_t_vz',
             'tcp_t_wx', 'tcp_t_wy', 'tcp_t_wz',
             'robot_status', 'robot_mode', 'safety_mode',
             'safety_status_bits', 'runtime_state',
             'speed_scaling', 'speed_scaling_combined', 'momentum',
             'target_speed_fraction', 'payload',
             'ft_wrench0', 'ft_wrench1', 'ft_wrench2', 'ft_wrench3',
             'ft_wrench4', 'ft_wrench5',
             'execution_time', 'joint_voltage0', 'joint_voltage1',
             'joint_voltage2', 'joint_voltage3', 'joint_voltage4',
             'joint_voltage5', 'main_voltage',
             'robot_current', 'robot_voltage',
             'din_bits', 'dout_bits']
    return cols


def safe(fn, *a):
    """调用一个 RTDE getter; 失败/空 → np.nan; 标量 → float; 数组 → list[float]."""
    try:
        v = fn(*a) if a else fn()
        if v is None:
            return np.nan
        v = np.asarray(v)
        if v.ndim == 0:
            return float(v)
        return [float(x) for x in v.ravel()]
    except Exception:
        return np.nan


def _arr_or_nan(v, n):
    """取数组前 n 个; 不是有效数组 → n 个 NaN."""
    if isinstance(v, list) and len(v) >= n:
        return v[:n]
    return [np.nan] * n


def sample(rtde_r):
    """一次采样全部 RTDE 字段, 返回与 rtde_columns() 对应的行. 单个字段失败用 NaN."""
    row = [safe(rtde_r.getTimestamp, )]                       # t_rtde

    q = safe(rtde_r.getActualQ, )
    dq = safe(rtde_r.getActualQd, )
    tq = safe(rtde_r.getTargetQ, )
    tqd = safe(rtde_r.getTargetQd, )
    cur = safe(rtde_r.getActualCurrent, )
    tcur = safe(rtde_r.getTargetCurrent, )
    tmom = safe(rtde_r.getTargetMoment, )
    temp = safe(rtde_r.getJointTemperatures, )
    for arr in (q, dq, tq, tqd, cur, tcur, tmom, temp):        # 8 × 6
        row += _arr_or_nan(arr, NV)

    pose = safe(rtde_r.getActualTCPPose, )
    tspeed = safe(rtde_r.getActualTCPSpeed, )
    tspeed_t = safe(rtde_r.getTargetTCPSpeed, )
    for arr in (pose, tspeed, tspeed_t):                        # 3 × 6
        row += _arr_or_nan(arr, 6)

    # 状态字段 (5) + 标量 (5)
    row += [safe(rtde_r.getRobotStatus, ), safe(rtde_r.getRobotMode, ),
            safe(rtde_r.getSafetyMode, ), safe(rtde_r.getSafetyStatusBits, ),
            safe(rtde_r.getRuntimeState, ),
            safe(rtde_r.getSpeedScaling, ), safe(rtde_r.getSpeedScalingCombined, ),
            safe(rtde_r.getActualMomentum, ),
            safe(rtde_r.getTargetSpeedFraction, ), safe(rtde_r.getPayload, )]

    # FT 力/力矩 (6)
    row += _arr_or_nan(safe(rtde_r.getFtRawWrench, ), 6)

    # 供电 / 温度 / 状态位 (12)
    jvol = safe(rtde_r.getActualJointVoltage, )
    row += [safe(rtde_r.getActualExecutionTime, )]
    row += _arr_or_nan(jvol, NV)                                # joint_voltage0..5
    row += [safe(rtde_r.getActualMainVoltage, ),
            safe(rtde_r.getActualRobotCurrent, ),
            safe(rtde_r.getActualRobotVoltage, ),
            safe(rtde_r.getActualDigitalInputBits, ),
            safe(rtde_r.getActualDigitalOutputBits, )]
    return row


SAFETY_NAMES = {
    -2: 'FAULT', -1: 'VALIDATE_FAULT', 0: 'NORMAL', 1: 'REDUCED',
    2: 'PROTECTIVE_STOP', 3: 'RECOVERY', 4: 'SAFEGUARD_STOP',
    5: 'SYSTEM_EMERGENCY_STOP', 6: 'ROBOT_EMERGENCY_STOP',
    7: 'SYSTEM_RECOVERY', 8: 'ROBOT_RECOVERY', 9: 'SYSTEM_SAFEGUARD_STOP',
    10: 'SYSTEM_FAULT',
}
def main():
    parser = argparse.ArgumentParser(
        description="只读实时记录 UR 机械臂 RTDE 原始数据 (运动过程取证)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--robot', type=str, default='ur3',
                        choices=['ur3', 'ur12e'], help="机器人类型 (选 default_ip)")
    parser.add_argument('--ip', type=str, default=None,
                        help="控制箱 IP (默认按 --robot 取 robot_configs)")
    parser.add_argument('--rate', type=float, default=125.0,
                        help="采样频率 Hz (CB3 一般 RTDE 最高 500)")
    parser.add_argument('--duration', type=float, default=0.0,
                        help="录制时长 s, 0=直到 Ctrl+C")
    parser.add_argument('--out', type=str, default=None,
                        help="输出 CSV 路径 (默认 logs/rtde_<时间>.csv)")
    parser.add_argument('--report', type=float, default=1.0,
                        help="控制台状态打印间隔 s")
    args = parser.parse_args()

    import rtde_receive
    from se3_control.config.robot_configs import get_robot_config

    cfg = get_robot_config(args.robot)
    ip = args.ip if args.ip else cfg['default_ip']

    print(f"连接 {ip} ... (只读 RTDE, 不发任何指令)")
    try:
        rtde_r = rtde_receive.RTDEReceiveInterface(ip)
    except Exception as e:
        print(f"❌ 连接失败: {e}")
        print("  检查网线/IP/教示器是否上电。")
        sys.exit(1)
    print("✅ 已连接. 采样中, Ctrl+C 结束.")

    if args.out is None:
        os.makedirs('logs', exist_ok=True)
        out = os.path.join('logs', time.strftime('rtde_%Y%m%d_%H%M%S.csv'))
    else:
        out = args.out
        os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    print(f"写入 → {out}")

    dt = 1.0 / max(1.0, args.rate)
    nv = NV
    f = open(out, 'w', newline='')
    w = csv.writer(f)
    w.writerow(['t_wall'] + rtde_columns())
    f.flush()

    t0 = time.time()
    n = 0
    t_last_report = 0.0
    last_q = np.zeros(nv)
    last_dq = np.zeros(nv)
    try:
        while True:
            t_wall = time.time() - t0
            if args.duration > 0 and t_wall >= args.duration:
                break
            row = sample(rtde_r)
            w.writerow([f"{t_wall:.6f}"] + row)
            f.flush()
            n += 1

            # row = sample() 与 rtde_columns() 1:1 对齐 (row[0]=t_rtde, row[1..6]=q)
            last_q = row[1:1 + nv]
            last_dq = row[1 + nv:1 + 2 * nv]
            cols = rtde_columns()
            if t_wall - t_last_report >= args.report:
                t_last_report = t_wall
                i_safe = cols.index('safety_mode')
                i_rt = cols.index('runtime_state')
                i_ss = cols.index('speed_scaling')
                i_pz = cols.index('tcp_z')
                i_mom = cols.index('momentum')
                safe = int(row[i_safe]) if isinstance(row[i_safe], (int, float)) and not np.isnan(row[i_safe]) else -99
                rt = int(row[i_rt]) if isinstance(row[i_rt], (int, float)) and not np.isnan(row[i_rt]) else -99
                ss = row[i_ss] if isinstance(row[i_ss], (int, float)) and not np.isnan(row[i_ss]) else float('nan')
                pz = row[i_pz] if isinstance(row[i_pz], (int, float)) and not np.isnan(row[i_pz]) else float('nan')
                mom = row[i_mom] if isinstance(row[i_mom], (int, float)) and not np.isnan(row[i_mom]) else float('nan')
                qsum = sum(abs(float(x)) for x in last_q if isinstance(x, (int, float)) and not np.isnan(x))
                dqmax = max(abs(float(x)) for x in last_dq if isinstance(x, (int, float)) and not np.isnan(x))
                sn = SAFETY_NAMES.get(safe, f'?({safe})')
                print(f"t={t_wall:6.2f}s  |q|={qsum:5.2f} |dq|max={dqmax:5.2f} "
                      f"TCPz={pz:6.3f} SS={ss:4.0f}% mom={mom:7.2f} "
                      f"safety={sn} state={rt}")

            # 限速到 --rate (实际采样耗时若已超过 dt 则立即继续, 不堆积)
            time.sleep(max(0.0, dt - (time.time() - t0 - t_wall)))
        print(f"\n完成: 记录 {n} 行 → {out}")
    except KeyboardInterrupt:
        print(f"\nCtrl+C 停止: 记录 {n} 行 → {out}")
    finally:
        try:
            f.close()
        except Exception:
            pass


if __name__ == '__main__':
    main()
