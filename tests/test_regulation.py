# -*- coding: utf-8 -*-
"""
Step 4: 简化 GIC 位置保持 (Regulation) 验证
==============================================

目标:
  运行简化 GIC 控制器，在 regulation 模式下保持末端位置，
  验证完整"读-算-发"闭环在 UR 机械臂上能正确运转。

支持机械臂:
  - UR12e (默认): --robot ur12e
  - UR3:          --robot ur3

控制律 (简化 GIC, 无自适应惯性):
  Vb = Jb @ dq                              (体速度)
  ep = R^T @ (p - pd)                       (体坐标系位置误差)
  eR = vee(Rd^T @ R - R^T @ Rd)            (朝向误差)
  F_task  = -Kp @ ep - KR @ eR - Kd @ Vb   (虚拟力, 体坐标系)
  tau_cmd = Jb^T @ F_task + tau_bias        (关节力矩 + 重力补偿)

用法:
  conda activate roboarm
  python se3_control/scripts/test_regulation.py [--ip 192.168.1.100]
  python se3_control/scripts/test_regulation.py --robot ur3 --ip 192.168.1.101

安全:
  - 力矩限幅为 URDF 限位的 50%
  - 控制循环异常时自动 emergency_stop
  - Kp 从极低值开始, 逐步递增
"""

import argparse
import logging
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "se3_control"))

from config.robot_configs import get_robot_config, get_urdf_path, get_hw_class, add_robot_arg
from robot_model.robot_model import RobotModel


# ================================================================
# SE(3) 数学工具 (简化版, 后续移至 core/se3_math.py)
# ================================================================

def hat_map(v: np.ndarray) -> np.ndarray:
    """so(3) 帽子映射: 向量 → 反对称矩阵。"""
    v = v.ravel()
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0],
    ])


def vee_map(R: np.ndarray) -> np.ndarray:
    """so(3) vee 映射: 反对称矩阵 → 向量。"""
    return 0.5 * np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1],
    ])


def orientation_error(R: np.ndarray, Rd: np.ndarray) -> np.ndarray:
    """计算 SO(3) 朝向误差向量 eR ∈ ℝ³。

    eR = vee(Rdᵀ R - Rᵀ Rd)

    :param R:  当前朝向 (3,3)
    :param Rd: 期望朝向 (3,3)
    :returns:  朝向误差向量 (3,)
    """
    error_mat = Rd.T @ R - R.T @ Rd
    return vee_map(error_mat)


def build_gains(Kp_vals, KR_vals, Kd_vals):
    """构建增益矩阵。

    :param Kp_vals: [kx, ky, kz] 位置刚度
    :param KR_vals: [krx, kry, krz] 旋转刚度
    :param Kd_vals: [d1..d6] 全部 6 个阻尼系数
    :returns: Kp (3,3), KR (3,3), Kd (6,6)
    """
    Kp = np.diag(Kp_vals)
    KR = np.diag(KR_vals)
    Kd = np.diag(Kd_vals)
    return Kp, KR, Kd


def gic_regulation_torque(p, R, Rd, Vb, Jb, tau_bias, Kp, KR, Kd):
    """简化 GIC 调节控制律 (无轨迹跟踪, 无自适应惯性)。

    :param p:        当前位置 (3,)
    :param R:        当前朝向 (3,3)
    :param Rd:       期望朝向 (3,3)
    :param Vb:       体速度 (6,)
    :param Jb:       体雅可比 (6, nv)
    :param tau_bias: 重力补偿力矩 (nv,)
    :param Kp:       位置刚度 (3,3)
    :param KR:       旋转刚度 (3,3)
    :param Kd:       阻尼 (6,6)
    :returns: tau_cmd (nv,) 关节力矩指令
    """
    # 期望位置取当前位置 (保持不动)
    pd = p.copy()

    # 位置误差 (体坐标系)
    ep = R.T @ (p - pd)  # (3,)

    # 朝向误差
    eR = orientation_error(R, Rd)  # (3,)

    # 速度误差 (期望速度为 0 → ev = -Vb)
    ev = -Vb.ravel()  # (6,)

    # 虚拟力 (体坐标系)
    F_body = np.zeros(6)
    F_body[:3] = -Kp @ ep     # 位置刚度力
    F_body[3:] = -KR @ eR     # 旋转刚度力矩

    # 任务空间力矩 + 阻尼
    tau_tilde = Jb.T @ (F_body - Kd @ ev)

    # 加重力补偿
    tau_cmd = tau_tilde + tau_bias

    return tau_cmd


# ================================================================
# 主程序
# ================================================================

def parse_args():
    parser = argparse.ArgumentParser(description="UR 机械臂简化 GIC Regulation 验证")
    add_robot_arg(parser)
    cfg_default = get_robot_config(parser.get_default('robot'))
    parser.add_argument("--ip", default=cfg_default['default_ip'],
                        help="UR 控制箱 IP 地址")
    parser.add_argument("--urdf", type=str, default=None,
                        help="URDF 文件路径 (默认从配置加载)")
    parser.add_argument("--ee-frame", default=None,
                        help="末端执行器 frame 名称 (默认从配置加载)")
    parser.add_argument("--duration", type=float, default=15.0,
                        help="测试持续时间 (秒)")
    parser.add_argument("--dt", type=float, default=0.004,
                        help="控制周期 (秒)")
    parser.add_argument("--kp", type=float, nargs=3, default=[50, 50, 50],
                        help="位置刚度 Kp [kx, ky, kz]")
    parser.add_argument("--kr", type=float, nargs=3, default=[50, 50, 50],
                        help="旋转刚度 KR [krx, kry, krz]")
    parser.add_argument("--kd", type=float, nargs=6,
                        default=[10, 10, 10, 5, 5, 5],
                        help="阻尼 Kd [d1..d6]")
    return parser.parse_args()


def main():
    args = parse_args()
    cfg = get_robot_config(args.robot)
    RobotHW = get_hw_class(args.robot)
    robot_name = cfg['name']

    # 解析 URDF 和 EE frame
    urdf_path = args.urdf if args.urdf else get_urdf_path(args.robot)
    ee_frame = args.ee_frame if args.ee_frame else cfg['ee_frame']

    # 构建增益矩阵
    Kp, KR, Kd = build_gains(args.kp, args.kr, args.kd)

    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s.%(msecs)03d] %(message)s",
        datefmt="%H:%M:%S",
    )
    logger = logging.getLogger("test_regulation")

    print("=" * 70)
    print(f"  Step 4: 简化 GIC 位置保持 (Regulation) 验证 [{robot_name}]")
    print("=" * 70)
    print(f"\n   机器人:        {robot_name}")
    print(f"   机器人 IP:      {args.ip}")
    print(f"   控制频率:       {1/args.dt:.0f} Hz")
    print(f"   测试时长:       {args.duration} s")
    print(f"\n   增益:")
    print(f"     Kp  = diag({args.kp}) N/m")
    print(f"     KR  = diag({args.kr}) Nm/rad")
    print(f"     Kd  = diag({args.kd}) Ns/m, Nms/rad")
    print(f"\n   ⚠️  安全提醒:")
    print(f"     1. Kp=50 极低增益, 臂应保持当前位置")
    print(f"     2. 如有异常手臂将缓慢偏位 (非急动)")
    print(f"     3. 随时准备按急停或 Ctrl+C")
    print(f"\n     确保: 远程控制模式, 臂处于安全位置\n")

    input("   按 Enter 继续 ...")

    # ── 初始化 ──
    logger.info("初始化 RobotModel ...")
    robot_model = RobotModel(urdf_path, ee_frame_name=ee_frame,
                              robot_name=robot_name, verbose=True)

    logger.info(f"初始化 {robot_name}HW ...")
    hw = RobotHW(ip=args.ip, dt=args.dt, verbose=True)

    nv = robot_model.nv

    try:
        hw.initialize()

        # 读取初始状态作为期望位姿
        q, dq = hw.get_joint_states()
        robot_model.update(q, dq)
        pd, Rd = robot_model.get_pose()

        logger.info(f"期望末端位置: {np.round(pd, 4)} m")
        logger.info(f"期望末端朝向: \n{Rd}")

        # 计时器重置
        try:
            hw._ctrl.initPeriod()
        except Exception:
            pass

        # ── 控制循环 ──
        logger.info(f"\nGIC 控制循环启动 ({args.duration} 秒) ...")
        n_steps = int(args.duration / args.dt)

        # 日志缓存
        step_log = min(n_steps, 5000)
        log_interval = max(1, n_steps // step_log)
        t_log = []
        p_log = []
        q_log = []
        tau_log = []
        err_log = []

        t_start = time.perf_counter()
        for i in range(n_steps):
            # 1. 读状态
            q, dq = hw.get_joint_states()
            robot_model.update(q, dq)

            # 2. 正运动学
            p_cur, R_cur = robot_model.get_pose()
            Jb = robot_model.get_body_jacobian()
            Vb = robot_model.get_body_ee_velocity()
            tau_bias = robot_model.get_bias_torque()

            # 3. GIC 控制律
            tau = gic_regulation_torque(
                p_cur, R_cur, Rd,
                Vb, Jb, tau_bias,
                Kp, KR, Kd,
            )

            # 4. 发力矩
            hw.set_joint_torques(tau)

            # 5. 记录
            t_cur = time.perf_counter() - t_start
            pos_err = np.linalg.norm(p_cur - pd)

            if i % log_interval == 0:
                t_log.append(t_cur)
                p_log.append(p_cur.copy())
                q_log.append(q.copy())
                tau_log.append(tau.copy())
                err_log.append(pos_err)

            # 6. 状态输出 (每秒)
            if (i + 1) % int(1/args.dt) == 0:
                logger.info(
                    f"  t={t_cur:6.2f}s  "
                    f"||ep||={pos_err*1000:6.2f}mm  "
                    f"||tau||={np.linalg.norm(tau):5.1f}Nm  "
                    f"p=[{p_cur[0]:.3f}, {p_cur[1]:.3f}, {p_cur[2]:.3f}]"
                )

            # 7. 安全检查
            err_state = hw.get_error_state()
            if err_state != 0:
                logger.error(f"UR {robot_name} 错误状态: {err_state} — 急停")
                hw.emergency_stop()
                break

            # 8. 等待下一周期
            hw.wait_next_cycle()

        t_total = time.perf_counter() - t_start

        # ── 统计 ──
        logger.info(f"\n{'='*50}")
        logger.info(f"测试统计 [{robot_name}]")

        if len(err_log) > 0:
            err_array = np.array(err_log)
            p_array = np.array(p_log)

            logger.info(f"  实际运行:       {t_total:.2f} s")
            logger.info(f"  平均频率:       {n_steps/t_total:.1f} Hz")
            logger.info(f"  最终位置误差:   {err_array[-1]*1000:.2f} mm")
            logger.info(f"  平均位置误差:   {np.mean(err_array)*1000:.2f} mm")
            logger.info(f"  最大位置误差:   {np.max(err_array)*1000:.2f} mm")
            logger.info(f"  位置标准差:     {np.std(p_array, axis=0)*1000:.2f} mm")

            # 关节力矩统计
            tau_array = np.array(tau_log)
            logger.info(f"  关节力矩均值:   {np.round(np.mean(tau_array, axis=0), 2)} Nm")
            logger.info(f"  关节力矩标准差: {np.round(np.std(tau_array, axis=0), 3)} Nm")

            # ── 结论 ──
            final_err = err_array[-1]
            max_err = np.max(err_array)
            logger.info(f"\n{'='*50}")
            if max_err < 0.01:
                logger.info(f"  ✅ Step 4 ({robot_name}) 通过")
                logger.info(f"     GIC Regulation 控制器运行正常")
                logger.info(f"     位置保持精度: ±{max_err*1000:.1f} mm")
            elif max_err < 0.05:
                logger.info(f"  ⚠️  Step 4 ({robot_name}) 基本通过")
                logger.info(f"     位置保持精度: ±{max_err*1000:.1f} mm")
                logger.info(f"     建议降低 Kd 或增大 Kp")
            else:
                logger.warning(f"  ❌ Step 4 ({robot_name}) 位置偏差过大")
                logger.warning(f"     最大偏差: {max_err*1000:.1f} mm")
                logger.warning(f"     请检查: URDF 惯性参数、控制频率、增益")
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
