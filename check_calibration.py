#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_calibration.py — 实机校准验证: 模型 FK(tool0) vs RTDE actual_TCP_pose 交叉核对
=================================================================================

基座校准 (ur3.urdf: shoulder_pan yaw180° + flange-tool0 偏移 0.126 m) 之后,
模型 FK 应与 RTDE 实测 TCP 一致。本脚本**只读**连接 RTDE (只连 RTDEReceiveInterface,
不发任何指令、不动臂), 同时采样 actual_q 与 actual_TCP_pose, 逐点算模型 FK(tool0)
并比对位置 + 朝向。

判定标准 (默认):
  - 位置差 |p_model − p_rtde| < 1 cm  → 位置对齐
  - 姿态误差 angle(R_model^T · R_rtde) < 0.05 rad → 朝向对齐
全部通过 → "✅ 基座校准在实机验证通过"; 否则打印逐点数值供诊断。

RTDE actual_TCP_pose 的 rx/ry/rz 是**旋转向量 (轴角)**, 不是 RPY:
  R = I + sinθ·[k]× + (1−cosθ)·[k]×²,  θ = |rotvec|, k = rotvec/θ
(本脚本按此把 RTDE 朝向换算成旋转矩阵再与模型 FK 比对; 若当 RPY 直接读会错。)

用法::

  conda activate roboarm

  # 臂静止在 home (先 go_home 或用当前位形) — 验证锚点
  python check_calibration.py --robot ur3 --duration 5

  # 更严苛: 另一个终端正在跑任务时, 让本脚本覆盖运动中的位形 (最有说服力)
  python check_calibration.py --robot ur3 --duration 10

  # 其它机器人 / 其它 IP
  python check_calibration.py --robot ur12e --ip 192.168.1.100 --duration 5

臂静止或运动中均可; 本脚本全程只读, 不连控制口, 不影响正在跑的任务。
"""

import argparse
import os
import sys
import time

import numpy as np

# 让脚本从任意工作目录都能 import se3_control
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)


# ====================================================================
# 旋转向量(轴角) → 旋转矩阵 (Rodrigues)
# ====================================================================
def rotvec_to_rotmat(rotvec):
    """旋转向量(轴角) → 旋转矩阵.

    :param rotvec: ndarray (3,) — rx/ry/rz, 轴角表示 (方向=轴, 模长=角度 rad)
    :returns: ndarray (3,3) — SO(3) 旋转矩阵
    """
    r = np.asarray(rotvec, dtype=float).ravel()
    th = float(np.linalg.norm(r))
    if th < 1e-12:
        return np.eye(3)
    k = r / th
    K = np.array([[0.0, -k[2], k[1]],
                  [k[2], 0.0, -k[0]],
                  [-k[1], k[0], 0.0]])
    return np.eye(3) + np.sin(th) * K + (1.0 - np.cos(th)) * (K @ K)


def angle_between(Ra, Rb):
    """两个旋转矩阵间的夹角 (rad)."""
    tr = np.clip((np.trace(Ra.T @ Rb) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.arccos(tr))


# ====================================================================
# 主流程
# ====================================================================
def main():
    parser = argparse.ArgumentParser(
        description="实机校准验证: 模型 FK(tool0) vs RTDE actual_TCP_pose (只读)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--robot', type=str, default='ur3',
                        choices=['ur3', 'ur12e'], help="机器人类型")
    parser.add_argument('--ip', type=str, default=None,
                        help="控制箱 IP (默认按 --robot 取 robot_configs)")
    parser.add_argument('--duration', type=float, default=5.0,
                        help="采样时长 (s)")
    parser.add_argument('--pos-tol', type=float, default=0.010,
                        help="位置差容忍 (m), 默认 1 cm")
    parser.add_argument('--rot-tol', type=float, default=0.05,
                        help="姿态误差容忍 (rad)")
    args = parser.parse_args()

    # ── 延迟 import (需要 roboarm env 里的 rtde / pinocchio) ──
    import rtde_receive
    from se3_control.config.robot_configs import get_robot_config, get_urdf_path
    from se3_control.robot_model.robot_model import RobotModel

    cfg = get_robot_config(args.robot)
    ip = args.ip if args.ip else cfg['default_ip']

    model = RobotModel(get_urdf_path(args.robot), ee_frame_name=cfg['ee_frame'],
                       robot_name=args.robot, verbose=False)

    print(f"连接 {ip} (只读 RTDE, 不发指令/不动臂) ...")
    try:
        recv = rtde_receive.RTDEReceiveInterface(ip)
        if not recv.isConnected():
            raise RuntimeError("RTDEReceiveInterface 未连接")
    except Exception as e:
        print(f"❌ 连接失败: {e}")
        print("  确认臂已开机 / IP 正确 / 教示器处于远程控制 (Remote) 模式。")
        return 1

    print(f"采样 {args.duration:.1f}s ... (臂静止或运动中均可, ~100 Hz)")
    pos_errs, rot_errs = [], []
    first_rtde = None
    n = 0
    t0 = time.time()
    while time.time() - t0 < args.duration:
        try:
            q = np.asarray(recv.getActualQ(), dtype=float)[:6]
            tcp = np.asarray(recv.getActualTCPPose(), dtype=float)
        except Exception:
            continue  # 单点读取失败跳过, 不中断

        p_rtde = tcp[:3]
        R_rtde = rotvec_to_rotmat(tcp[3:6])

        model.update(q)
        p_model, R_model = model.get_frame_pose('tool0')

        pos_errs.append(float(np.linalg.norm(p_model - p_rtde)))
        rot_errs.append(angle_between(R_model, R_rtde))
        if first_rtde is None:
            first_rtde = p_rtde
        n += 1
        time.sleep(0.01)

    recv.disconnect()

    if n == 0:
        print("❌ 未采到任何有效样本 — 检查 RTDE 连接。")
        return 1

    pos = np.array(pos_errs)
    rot = np.array(rot_errs)
    print(f"采样 {n} 点:")
    print(f"  |p_model − p_rtde| : max {pos.max()*1000:6.1f} mm / mean {pos.mean()*1000:6.1f} mm")
    print(f"  姿态误差 (rad)     : max {rot.max():.4f} / mean {rot.mean():.4f}")
    print(f"  (首点 RTDE TCP = {np.round(first_rtde, 4)} — 模型 FK(同 q) 应≈此处)")

    ok = bool(pos.max() < args.pos_tol and rot.max() < args.rot_tol)
    print()
    if ok:
        print("✅ 基座校准在实机验证通过: 模型 FK(tool0) == RTDE actual_TCP_pose (位置+朝向)")
        return 0
    print("❌ 模型 FK 与 RTDE TCP 仍有偏差 — 校准未完全对齐, 见上方数值 (阈值 "
          f"{args.pos_tol*1000:.0f} mm / {args.rot_tol} rad)。")
    return 1


if __name__ == '__main__':
    sys.exit(main())
