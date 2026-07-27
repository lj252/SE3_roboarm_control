# -*- coding: utf-8 -*-
"""
Step 2: 关节状态读取验证 (q, dq)
===================================

目标:
  验证 UR 机械臂硬件接口的 get_joint_states() 方法能正确读取关节状态。

支持机械臂:
  - UR12e (默认): --robot ur12e
  - UR3:          --robot ur3

测试内容:
  1. 建立 RTDE 连接
  2. 读取关节位置 q (rad) 和关节速度 dq (rad/s)
  3. 对比 RTDE 读数与教示器显示值
  4. 连续读取 N 秒，观测数据稳定性
  5. 检查通信延迟和丢帧

用法:
  conda activate roboarm
  python se3_control/scripts/test_joint_states.py [--ip 192.168.1.100] [--duration 5]
  python se3_control/scripts/test_joint_states.py --robot ur3 --ip 192.168.1.101
"""

import argparse
import logging
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from config.robot_configs import get_robot_config, get_urdf_path, get_hw_class, add_robot_arg


def parse_args():
    parser = argparse.ArgumentParser(description="UR 机械臂关节状态读取验证")
    add_robot_arg(parser)
    cfg_default = get_robot_config(parser.get_default('robot'))
    parser.add_argument("--ip", default=cfg_default['default_ip'],
                        help="UR 控制箱 IP 地址")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="测试持续时间 (秒)")
    parser.add_argument("--dt", type=float, default=0.004,
                        help="读取周期 (秒), 默认 0.004 (250 Hz)")
    return parser.parse_args()


def main():
    args = parse_args()
    cfg = get_robot_config(args.robot)
    RobotHW = get_hw_class(args.robot)
    robot_name = cfg['name']

    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s.%(msecs)03d] %(message)s",
        datefmt="%H:%M:%S",
    )
    logger = logging.getLogger("test_joint_states")

    print("=" * 70)
    print(f"  Step 2: {robot_name} 关节状态读取验证")
    print("=" * 70)
    print(f"\n   机器人:        {robot_name}")
    print(f"   机器人 IP:      {args.ip}")
    print(f"   测试持续时间:  {args.duration} s")
    print(f"   读取频率:       {1/args.dt:.0f} Hz")
    print(f"\n   ⚠️  请确保:")
    print(f"     1. {robot_name} 已开机且处于远程控制模式 (Remote Control)")
    print(f"     2. 急停按钮已释放")
    print(f"     3. 臂处于安全位置")
    print(f"     4. 按 Ctrl+C 可随时终止\n")

    input("   按 Enter 继续 ...")

    try:
        with RobotHW(ip=args.ip, dt=args.dt, verbose=True) as robot:
            # ── 基本连接验证 ──
            logger.info("基本连接验证 ...")
            assert robot.is_connected(), "连接失败"
            assert robot.is_enabled(), "机器人未使能"
            logger.info(f"  关节名称: {robot.get_joint_names()}")
            logger.info(f"  力矩限幅: {robot.get_torque_limits()}")
            logger.info(f"  连接状态: {robot.is_connected()} ✅")
            logger.info(f"  使能状态: {robot.is_enabled()} ✅")

            # ── 单帧读取 ──
            logger.info("\n单帧状态读取:")
            q, dq = robot.get_joint_states()
            logger.info(f"  关节位置 q:  {np.round(q, 6)}")
            logger.info(f"  关节速度 dq: {np.round(dq, 6)}")

            # 合理性检查
            nv = len(cfg['joint_names'])
            assert len(q) == nv, f"q 维度应为 {nv}, 实际 {len(q)}"
            assert len(dq) == nv, f"dq 维度应为 {nv}, 实际 {len(dq)}"
            assert np.all(np.isfinite(q)), "q 包含 NaN 或 Inf"
            assert np.all(np.isfinite(dq)), "dq 包含 NaN 或 Inf"
            logger.info(f"  数据完整性: ✅ (均为有限值)")

            # ── 连续读取 ──
            logger.info(f"\n连续读取 {args.duration} 秒 ...")
            n_samples = int(args.duration / args.dt)
            q_log = np.zeros((n_samples, nv))
            dq_log = np.zeros((n_samples, nv))
            t_log = np.zeros(n_samples)
            latency_log = np.zeros(n_samples)

            t_start = time.perf_counter()
            for i in range(n_samples):
                loop_start = time.perf_counter()
                q, dq = robot.get_joint_states()
                t_elapsed = time.perf_counter() - t_start

                q_log[i] = q
                dq_log[i] = dq
                t_log[i] = t_elapsed
                latency_log[i] = time.perf_counter() - loop_start

                robot.wait_next_cycle()

                if (i + 1) % 250 == 0:
                    logger.info(f"  [{i+1}/{n_samples}] t={t_elapsed:.3f}s")

            t_total = time.perf_counter() - t_start

            # ── 统计 ──
            logger.info(f"\n=== 统计结果 ===")
            logger.info(f"总采样数:     {n_samples}")
            logger.info(f"实际耗时:     {t_total:.3f} s")
            logger.info(f"平均频率:     {n_samples/t_total:.1f} Hz")

            # 关节位置统计
            q_mean = np.mean(q_log, axis=0)
            q_std = np.std(q_log, axis=0)
            q_range = np.max(q_log, axis=0) - np.min(q_log, axis=0)
            logger.info(f"\n关节位置统计 (rad):")
            logger.info(f"  均值: {np.round(q_mean, 4)}")
            logger.info(f"  标准差: {np.round(q_std, 6)}")
            logger.info(f"  极差: {np.round(q_range, 6)}")

            # 关节速度统计 (静止时 ≈ 0)
            dq_mean = np.mean(dq_log, axis=0)
            dq_std = np.std(dq_log, axis=0)
            dq_max = np.max(np.abs(dq_log), axis=0)
            logger.info(f"\n关节速度统计 (rad/s):")
            logger.info(f"  均值: {np.round(dq_mean, 6)}")
            logger.info(f"  标准差: {np.round(dq_std, 6)}")
            logger.info(f"  最大绝对值: {np.round(dq_max, 6)}")

            # 延迟统计
            lat_mean = np.mean(latency_log) * 1000
            lat_max = np.max(latency_log) * 1000
            logger.info(f"\n读取延迟:")
            logger.info(f"  平均: {lat_mean:.3f} ms")
            logger.info(f"  最大: {lat_max:.3f} ms")

            # ── 结论 ──
            all_finite = np.all(np.isfinite(q_log)) and np.all(np.isfinite(dq_log))
            q_reasonable = np.all(np.abs(q_log) < 2 * np.pi)
            dq_static_ok = np.all(dq_std < 1.0)

            logger.info(f"\n{'='*50}")
            logger.info(f"  数据完整性:  {'✅' if all_finite else '❌'}")
            logger.info(f"  位置范围合理: {'✅' if q_reasonable else '❌'}")
            logger.info(f"  静止速度正常: {'✅' if dq_static_ok else '❌'}")
            logger.info(f"{'='*50}")

            if all_finite and q_reasonable and dq_static_ok:
                logger.info(f"\nStep 2 ({robot_name}) 全部通过 ✅")
            else:
                logger.warning(f"\n部分检查未通过 ⚠️  请检查 {robot_name} 状态")

    except KeyboardInterrupt:
        logger.info("\n测试被用户终止")
    except Exception as e:
        logger.error(f"测试失败: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
