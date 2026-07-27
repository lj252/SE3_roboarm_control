# -*- coding: utf-8 -*-
"""
Step 3: 力矩下发验证 + 重力补偿验证
======================================

目标:
  验证 UR 机械臂硬件接口的 set_joint_torques() 能正确下发力矩指令，
  并验证 Pinocchio 计算的重力补偿力矩能使机械臂保持静止。

支持机械臂:
  - UR12e (默认): --robot ur12e
  - UR3:          --robot ur3

测试阶段:
  Phase A — 零力矩下发: 发送 tau=0，臂应在重力作用下缓慢下落
  Phase B — 重力补偿:   发送 tau=tau_bias，臂应保持静止 (< 5 mm/min 漂移)

安全机制:
  - 力矩限幅为 URDF 限位的 50%
  - 任何异常自动触发 emergency_stop
  - 可选的紧急恢复: 按 Ctrl+C 立即停止

用法:
  conda activate roboarm
  python se3_control/scripts/test_gravity_comp.py [--ip 192.168.1.100]
  python se3_control/scripts/test_gravity_comp.py --robot ur3 --ip 192.168.1.101
"""

import argparse
import logging
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from config.robot_configs import get_robot_config, get_urdf_path, get_hw_class, add_robot_arg
from robot_model.robot_model import RobotModel


def parse_args():
    parser = argparse.ArgumentParser(description="UR 机械臂力矩下发 + 重力补偿验证")
    add_robot_arg(parser)
    cfg_default = get_robot_config(parser.get_default('robot'))
    parser.add_argument("--ip", default=cfg_default['default_ip'],
                        help="UR 控制箱 IP 地址")
    parser.add_argument("--urdf", type=str, default=None,
                        help="URDF 文件路径 (默认从配置加载)")
    parser.add_argument("--ee-frame", default=None,
                        help="末端执行器 frame 名称 (默认从配置加载)")
    parser.add_argument("--phase-a-duration", type=float, default=3.0,
                        help="Phase A 测试时长 (零力矩, 秒)")
    parser.add_argument("--phase-b-duration", type=float, default=10.0,
                        help="Phase B 测试时长 (重力补偿, 秒)")
    parser.add_argument("--dt", type=float, default=0.004,
                        help="控制周期 (秒)")
    return parser.parse_args()


def check_tcp_drift(p_initial, p_current, tolerance=0.005):
    """检查 TCP 位置漂移是否在容忍范围内 (m)。"""
    drift = np.linalg.norm(p_current - p_initial)
    return drift, drift < tolerance


def main():
    args = parse_args()
    cfg = get_robot_config(args.robot)
    RobotHW = get_hw_class(args.robot)
    robot_name = cfg['name']

    # 解析 URDF 和 EE frame (支持命令行覆盖)
    urdf_path = args.urdf if args.urdf else get_urdf_path(args.robot)
    ee_frame = args.ee_frame if args.ee_frame else cfg['ee_frame']

    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s.%(msecs)03d] %(message)s",
        datefmt="%H:%M:%S",
    )
    logger = logging.getLogger("test_gravity_comp")

    print("=" * 70)
    print(f"  Step 3: {robot_name} 力矩下发 + 重力补偿验证")
    print("=" * 70)
    print(f"\n   机器人:        {robot_name}")
    print(f"   机器人 IP:      {args.ip}")
    print(f"   URDF:           {urdf_path}")
    print(f"   EE Frame:       {ee_frame}")
    print(f"   Phase A:        {args.phase_a_duration}s (零力矩)")
    print(f"   Phase B:        {args.phase_b_duration}s (重力补偿)")
    print(f"\n   ⚠️  安全警告:")
    print(f"     1. Phase A 中臂会在重力作用下自然下落")
    print(f"     2. 请确保下方无人员/障碍物")
    print(f"     3. 随时准备按急停按钮或 Ctrl+C")
    print(f"     4. 建议手持示教器在旁边")
    print(f"\n   ⚠️  确保:")
    print(f"     1. {robot_name} 处于远程控制模式 (Remote Control)")
    print(f"     2. 急停按钮已释放")
    print(f"     3. 臂处于安全位置, 下方无遮挡\n")

    input("   按 Enter 继续 Phase A (零力矩测试) ...")

    # ── 初始化 ──
    logger.info(f"初始化 {robot_name} 连接和 RobotModel ...")
    robot_model = RobotModel(urdf_path, ee_frame_name=ee_frame,
                              robot_name=robot_name, verbose=True)
    hw = RobotHW(ip=args.ip, dt=args.dt, verbose=True)

    nv = robot_model.nv

    try:
        hw.initialize()

        # 读取初始状态
        q_initial, dq_initial = hw.get_joint_states()
        robot_model.update(q_initial, dq_initial)

        # 记录初始 TCP 位置
        p_initial, R_initial = robot_model.get_pose()
        logger.info(f"初始 TCP 位置: {np.round(p_initial, 4)} m")
        logger.info(f"初始关节位置: {np.round(q_initial, 4)}")

        # ════════════════════════════════════════════════════════
        # Phase A: 零力矩测试
        # ════════════════════════════════════════════════════════
        logger.info(f"\n{'='*50}")
        logger.info(f"Phase A: 下发零力矩 ({args.phase_a_duration} 秒)")
        logger.info(f"{'='*50}")
        logger.info("  → 臂应在重力作用下自然下落 (正常现象)")

        n_a = int(args.phase_a_duration / args.dt)
        p_a_log = np.zeros((n_a, 3))
        q_a_log = np.zeros((n_a, nv))
        tau_a_log = np.zeros((n_a, nv))
        t_a_log = np.zeros(n_a)

        t_start = time.perf_counter()
        for i in range(n_a):
            # 读状态
            q, dq = hw.get_joint_states()
            robot_model.update(q, dq)
            p, R = robot_model.get_pose()

            # 发零力矩
            tau = np.zeros(nv)
            hw.set_joint_torques(tau)

            # 记录
            t_elapsed = time.perf_counter() - t_start
            p_a_log[i] = p
            q_a_log[i] = q
            tau_a_log[i] = tau
            t_a_log[i] = t_elapsed

            hw.wait_next_cycle()

            if (i + 1) % int(1/args.dt) == 0:
                drift, _ = check_tcp_drift(p_initial, p)
                logger.info(f"  t={t_elapsed:.2f}s  位置: {np.round(p, 4)}  "
                           f"漂移: {drift*1000:.1f} mm")

        # Phase A 结束时 TCP 位置
        drift_a, _ = check_tcp_drift(p_initial, p_a_log[-1])
        logger.info(f"\nPhase A 结果:")
        logger.info(f"  TCP 位置变化: {np.round(p_initial, 4)} → "
                    f"{np.round(p_a_log[-1], 4)}")
        logger.info(f"  总漂移: {drift_a*1000:.1f} mm")
        logger.info(f"  → {'✅ 正常下落' if drift_a > 0.005 else '⚠️  无明显下落, 检查补偿状态'}")

        # ════════════════════════════════════════════════════════
        # Phase B: 重力补偿
        # ════════════════════════════════════════════════════════
        input(f"\nPhase A 完成。按 Enter 继续 Phase B (重力补偿测试) ...")

        logger.info(f"\n{'='*50}")
        logger.info(f"Phase B: 重力补偿 ({args.phase_b_duration} 秒)")
        logger.info(f"{'='*50}")
        logger.info("  → 臂应在重力补偿下保持静止")

        n_b = int(args.phase_b_duration / args.dt)
        p_b_log = np.zeros((n_b, 3))
        q_b_log = np.zeros((n_b, nv))
        tau_b_log = np.zeros((n_b, nv))
        t_b_log = np.zeros(n_b)

        # 记录 Phase B 开始时的位置
        q_start_b, dq_start_b = hw.get_joint_states()
        robot_model.update(q_start_b, dq_start_b)
        p_start_b, _ = robot_model.get_pose()
        logger.info(f"Phase B 起始位置: {np.round(p_start_b, 4)} m")

        hw._ctrl.initPeriod()  # 重新同步定时器
        t_start = time.perf_counter()
        for i in range(n_b):
            # 读状态
            q, dq = hw.get_joint_states()
            robot_model.update(q, dq)

            # 计算重力补偿力矩
            tau_bias = robot_model.get_bias_torque()

            # 发重力补偿力矩
            hw.set_joint_torques(tau_bias)

            # 记录
            t_elapsed = time.perf_counter() - t_start
            p, _ = robot_model.get_pose()
            p_b_log[i] = p
            q_b_log[i] = q
            tau_b_log[i] = tau_bias
            t_b_log[i] = t_elapsed

            hw.wait_next_cycle()

            if (i + 1) % int(1/args.dt) == 0:
                drift, ok = check_tcp_drift(p_start_b, p)
                status = "✅" if ok else "⚠️"
                logger.info(f"  t={t_elapsed:.2f}s  位置: {np.round(p, 4)}  "
                           f"漂移: {drift*1000:.1f} mm {status}")

        # ── 统计 ──
        logger.info(f"\n{'='*50}")
        logger.info(f"Phase B 统计结果")

        # TCP 漂移
        final_drift_b, final_ok_b = check_tcp_drift(p_start_b, p_b_log[-1])
        max_drift_b = max(
            np.linalg.norm(p_b_log[i] - p_start_b) for i in range(n_b)
        )

        # 力矩统计
        tau_b_mean = np.mean(tau_b_log, axis=0)
        tau_b_std = np.std(tau_b_log, axis=0)

        logger.info(f"  TCP 初始位置:  {np.round(p_start_b, 4)}")
        logger.info(f"  TCP 最终位置:  {np.round(p_b_log[-1], 4)}")
        logger.info(f"  最终漂移:      {final_drift_b*1000:.2f} mm "
                    f"{'✅' if final_ok_b else '⚠️'}")
        logger.info(f"  最大漂移:      {max_drift_b*1000:.2f} mm")
        logger.info(f"  重力补偿均值:  {np.round(tau_b_mean, 2)} Nm")
        logger.info(f"  重力补偿标准差: {np.round(tau_b_std, 3)} Nm")

        # ── 结论 ──
        logger.info(f"\n{'='*50}")
        if final_ok_b:
            logger.info(f"  ✅ Step 3 ({robot_name}) 全部通过")
            logger.info(f"     力矩下发链路正常")
            logger.info(f"     重力补偿有效 (漂移 < 5 mm)")
        else:
            logger.warning(f"  ⚠️  重力补偿漂移较大")
            logger.warning(f"     请检查 URDF 惯性参数是否准确")
            logger.warning(f"     或尝试调整补偿模式 (UR 内置 vs Pinocchio)")
        logger.info(f"{'='*50}")

    except KeyboardInterrupt:
        logger.warning("\n\n测试被用户终止 — 执行安全停止")
        hw.emergency_stop()
    except Exception as e:
        logger.error(f"测试失败: {e}")
        hw.emergency_stop()
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        hw.shutdown()

    logger.info("测试结束")


if __name__ == "__main__":
    main()
