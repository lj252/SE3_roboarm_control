#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
go_home.py — 让 UR 机械臂回到"我们所设置的 home 位形"
=======================================================

用途
----
实机起步位形不在 home 时（臂折叠/低位 → 就是我们反复撞的根因之一），
先把臂安全地 moveJ 回 home_q，再跑 run_se3_control --preview / 实机任务。

安全设计
--------
  * 关节空间 moveJ（从任意当前位形插值到 home，不用 moveL —— 线性轨迹可能扫到基座柱）
  * 低速/低加速度（默认 0.4 rad/s、0.6 rad/s²，可用 --speed/--accel 调）
  * 移动前用 FK 检查当前位形是否贴基座柱/贴地，危险则醒目告警（仍可确认继续）
  * 移动前打印当前/目标关节角与目标 TCP，按 Enter 确认（--yes 跳过）

home_q 来源
-----------
默认读取 ``se3_control/config/robot_configs.py`` 中 ``--robot`` 对应的 home_q
（ur3 = [-0.327, -1.42, 1.236, -1.386, -1.571, 2.738]），即"我们设置的 home 点"。

用法::

  conda activate roboarm
  python go_home.py                          # 连接 ur3(.11)，确认后 moveJ 回 home
  python go_home.py --show-only              # 只打印当前/目标位形 + 危险检查，不移动
  python go_home.py --yes --speed 0.5 --accel 0.8   # 免确认 + 稍快
  python go_home.py --robot ur12e --ip 192.168.1.100   # 其它机器人/其它 IP

注意: 移动前确保教示器处于**远程控制**模式（右上角 Remote）。急停用教示器 STOP。
"""

import argparse
import os
import sys
import time

import numpy as np

# 让脚本从任意工作目录都能 import se3_control (脚本在 tests/monitor/, 项目根在上层)
def _find_project_root():
    _d = os.path.dirname(os.path.abspath(__file__))
    while not os.path.isdir(os.path.join(_d, 'se3_control')):
        _parent = os.path.dirname(_d)
        if _parent == _d:
            raise RuntimeError('找不到含 se3_control/ 的项目根目录')
        _d = _parent
    return _d
_SCRIPT_DIR = _find_project_root()
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)


# ====================================================================
# 危险位形检查 (FK, 复用 preview 的碰撞判定阈值)
# ====================================================================
def danger_warning(robot, q, label):
    """打印当前位形是否贴基座柱/贴地 (容易撞的位形)."""
    from se3_control.core.mujoco_preview import check_simulated_collisions
    v = check_simulated_collisions(robot, np.atleast_2d(np.asarray(q, dtype=float)))
    if v['ok']:
        print(f"  ✓ {label}位形安全 (连杆最低 {v['min_z']*100:5.1f} cm, "
              f"最近基座轴 {v['min_base_d']*100:5.1f} cm)")
    else:
        t, name, kind, val = v['first_violation']
        what = (f"{name} 距基座柱仅 {val*100:.1f} cm" if kind == 'base'
                else f"{name} 低至 {val*100:.1f} cm")
        print(f"  ⚠️ 警告: 当前位形{what} — 贴基座柱/地面, 正是容易撞的位形!")
        print(f"     已放慢移动速度; 请盯住急停按钮, 必要时教示器 STOP.")


# ====================================================================
# 等待 moveJ 到位 (RTDE moveJ 是异步下发, 需轮询关节角)
# ====================================================================
def wait_arrival(rtde_r, q_target, init_err, tol, timeout):
    """轮询实际关节角直到全部关节进入 tol 或超时. 返回 (arrived, max_err)."""
    q_target = np.asarray(q_target, dtype=float)
    start = time.time()
    last_pct = -1
    while time.time() - start < timeout:
        q = np.asarray(rtde_r.getActualQ(), dtype=float)
        err = float(np.max(np.abs(q - q_target)))
        if err < tol:
            return True, err
        pct = (1.0 - err / init_err) * 100 if init_err > 0 else 0.0
        pct = max(0.0, min(100.0, pct))
        if int(pct // 10) > last_pct:
            last_pct = int(pct // 10)
            print(f"   移动中... {pct:5.1f}%  (最大关节误差 {err*1000:.0f} mrad)")
        time.sleep(0.1)
    q = np.asarray(rtde_r.getActualQ(), dtype=float)
    return False, float(np.max(np.abs(q - q_target)))


# ====================================================================
# 主流程
# ====================================================================
def main():
    parser = argparse.ArgumentParser(
        description="让 UR 机械臂回到 robot_configs 中设置的 home 位形 (moveJ)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--robot', type=str, default='ur3',
                        choices=['ur3', 'ur12e'], help="机器人类型 (选 home_q)")
    parser.add_argument('--ip', type=str, default=None,
                        help="控制箱 IP (默认按 --robot 取 robot_configs)")
    parser.add_argument('--speed', type=float, default=0.4,
                        help="关节速度 (rad/s)")
    parser.add_argument('--accel', type=float, default=0.6,
                        help="关节加速度 (rad/s²)")
    parser.add_argument('--tol', type=float, default=0.005,
                        help="到位判定误差 (rad)")
    parser.add_argument('--timeout', type=float, default=90.0,
                        help="等待到位超时 (s)")
    parser.add_argument('--show-only', action='store_true',
                        help="只打印当前/目标位形 + 危险检查, 不移动")
    parser.add_argument('--yes', '-y', action='store_true',
                        help="跳过移动前确认")
    parser.add_argument('--home-joints', type=float, nargs=6, default=None,
                        help="手动指定目标关节角 (6 个 rad), 默认读 robot_configs")
    args = parser.parse_args()

    # ── 延迟 import (需要 roboarm env 里的 rtde/pinocchio) ──
    import rtde_control
    import rtde_receive
    from se3_control.config.robot_configs import get_robot_config, get_urdf_path
    from se3_control.robot_model.robot_model import RobotModel

    cfg = get_robot_config(args.robot)
    ip = args.ip if args.ip else cfg['default_ip']
    home_q = (np.asarray(args.home_joints, dtype=float) if args.home_joints is not None
              else np.asarray(cfg['home_q'], dtype=float))

    # ── 1. 连接 ──
    print(f"连接 {ip} ...")
    try:
        rtde_c = rtde_control.RTDEControlInterface(ip)
        rtde_r = rtde_receive.RTDEReceiveInterface(ip)
        print("✅ 机械臂连接成功")
    except Exception as e:
        print(f"❌ 连接失败: {e}")
        print("  检查网线/IP/教示器是否处于远程控制 (Remote) 模式。")
        sys.exit(1)

    # ── 2. 读取当前位形 + 危险检查 ──
    robot = RobotModel(get_urdf_path(args.robot), ee_frame_name=cfg['ee_frame'],
                       robot_name=args.robot, verbose=False)
    q0 = np.asarray(rtde_r.getActualQ(), dtype=float)
    print(f"当前关节角 q = {[round(x, 4) for x in q0]}")
    danger_warning(robot, q0, "当前")

    # ── 3. 打印目标 (home) ──
    robot.update(home_q)
    p_home, R_home = robot.get_pose()
    print(f"目标 home_q = {[round(x, 4) for x in home_q]}")
    print(f"目标 TCP 位置 = {[round(x, 4) for x in p_home]}")

    if args.show_only:
        print("\n(--show-only) 未移动。确认安全后可去掉该参数真正回 home。")
        return

    # ── 4. 已在 home? ──
    err0 = float(np.max(np.abs(q0 - home_q)))
    if err0 < args.tol:
        print(f"  ✓ 已在 home 附近 (最大关节误差 {err0*1000:.1f} mrad), 无需移动。")
        return

    # ── 5. 确认后 moveJ ──
    print(f"\n将从当前位形 moveJ 回 home (最大关节行程 {err0:.3f} rad, "
          f"speed={args.speed} rad/s, accel={args.accel} rad/s²)")
    if not args.yes:
        try:
            input("按 Enter 确认移动 (教示器 STOP 随时急停) ... ")
        except KeyboardInterrupt:
            print("\n已取消。")
            return
        # 确认期间臂可能被动过, 重新读取
        q0 = np.asarray(rtde_r.getActualQ(), dtype=float)
        err0 = float(np.max(np.abs(q0 - home_q)))
        if err0 < args.tol:
            print("  ✓ 已到 home, 无需移动。")
            return

    ok = rtde_c.moveJ(home_q.tolist(), args.speed, args.accel)
    if not ok:
        print("❌ moveJ 下发失败 — 检查教示器是否处于远程控制且无弹窗阻塞。")
        sys.exit(1)

    # ── 6. 等待到位 + 最终校验 ──
    arrived, err_final = wait_arrival(rtde_r, home_q, err0, args.tol, args.timeout)
    if not arrived:
        q_final = np.asarray(rtde_r.getActualQ(), dtype=float)
        print(f"⚠️ 超时未到位 (最大误差 {err_final*1000:.0f} mrad). "
              f"当前 q = {[round(x, 4) for x in q_final]}")
        print("  检查教示器是否有弹窗 (安全边界/连接中断等)。")
    q_final = np.asarray(rtde_r.getActualQ(), dtype=float)
    robot.update(q_final)
    p_f, _ = robot.get_pose()
    print(f"到位后关节角 q = {[round(x, 4) for x in q_final]}")
    print(f"到位后 TCP 位置 = {[round(x, 4) for x in p_f]}")
    print("\n✅ home 到位。现在可以跑 --preview 预览或实机任务了。")


if __name__ == '__main__':
    main()
