#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_se3_control.py — SE(3) 几何控制实机执行入口
=================================================

定位
----
本脚本是 SE(3) 控制在**真实机械臂**上的统一入口。
所有**仿真**实验统一走 verify_gic_mujoco.py / verify_gac_mujoco.py（MuJoCo 物理推演）；
本脚本不做仿真，直接连接硬件（UR3 / UR12e），以 250 Hz 闭环运行
**完整 GICController**（自适应操作空间惯性整形 + 重力补偿）。

架构
----
  run_se3_control.py   (实机编排: 连接 / 安全检查 / 相位状态机 / 轨迹求值 / 记录 / 停机)
       │  仅做编排, 不含任何控制律 / 运动学数学
       ▼
  core/gic_controller.py    — GIC 控制律 (GICController.compute, 自适应 M̃)
  core/trajectory.py        — 轨迹生成 (build_trajectory) + 体速度求值 (eval_body_twist)
  core/se3_math.py          — SE(3) 数学 (rotmat_slerp / vee_map / hat_map)
  robot_model/robot_model.py — Pinocchio FK / Jb / M / 重力补偿 / 高斯-牛顿 IK
       ▼
  hardware/ur3_hw.py (UR3HW, ur_rtde) — q/dq 读取 + directTorque 发力矩

任务
----
  regulation — 位置保持 (Phase 0 低增益自检 → Phase 2 主保持)
  circle/line — 轨迹跟踪 (Phase 0 自检 → Phase 2 跟踪; 含 IK 可达性预检 + 起步混合)

轨迹跟踪要点 (实机, 区别于仿真)
--------------------------------
  1. 真实时间求值: 轨迹在 t_real (wait_next_cycle 累加的真实经过时间) 处求值,
     而非仿真名义步长 i·dt.
  2. 起步混合 (--blend-time): 实机无法像仿真那样先 IK 摆位, 前 blend_time 秒从
     当前位姿平滑过渡到轨迹起点 (位置 lerp + 朝向 slerp), 前馈速度按 bf 缩放,
     避免起始位姿差造成力矩跳变.
  3. IK 可达性预检: 运行前采样轨迹点做高斯-牛顿 IK, 检查收敛 / 关节限位 / 奇异.

用法
----
  conda activate roboarm
  cd /media/lj252/Data/catkin_ws/roboarm_test/SE3_roboarm_control

  # 任务几何参数按 --robot 自动匹配 (config/task_config.py ROBOT_TASK_CONFIGS):
  #   --robot ur3   → circle 圆心 [-0.38,0,0.224] / r0.06; line 中点 [-0.38,0,0.224] / amp0.08
  #   --robot ur12e → circle 圆心 [0.50,0,0.50] / r0.05; line 中点 [0.50,0,0.50] / amp0.05 (高位安全)

  # UR3 画圆 (radius=0.06, speed=0.8, 中心 [-0.38,0,0.224]; UR3 默认 IP .11)
  python se3_control/scripts/run_se3_control.py --robot ur3 --task circle \
      --duration 16 --bandwidth 20

  # UR3 线轨迹 (amplitude=0.08, frequency=0.4, 中点 [-0.38,0,0.224])
  python se3_control/scripts/run_se3_control.py --robot ur3 --task line \
      --duration 16 --bandwidth 20

  # 命令行自定义圆心/半径 (仅 circle/line; 覆盖后同样走 IK 可达性预检,
  # 圆心太远/不可达会预检报错中止, 不会先动臂)
  python se3_control/scripts/run_se3_control.py --robot ur3 --task circle \
      --center -0.40 0.0 0.224 --radius 0.05

  # ★ 上真机前先 --preview: 不连接硬件, 用同一参数在 MuJoCo 里跑闭环仿真,
  #   实时看臂的轨迹 (红色 trail) + 自动碰撞判定 (基座柱/地面净距, ✓/✗).
  #   预览通过后再去掉 --preview 上真机 — 反复"撞"的问题先用预览排查.
  python se3_control/scripts/run_se3_control.py --robot ur3 \
      --control-mode servoJ --task circle --duration 16 --bandwidth 10 --preview
  #   自定义圆心/半径预览 (与实机命令一致, 只是加 --preview)
  python se3_control/scripts/run_se3_control.py --robot ur3 --task circle \
      --center -0.40 0.0 0.224 --radius 0.05 --preview --no-viewer
  #   真机当前位形不是 home (臂已折叠/低位) 时, 用 --dry-run 读当前 q 传入,
  #   让预览从真实起步位形开始 (混合路径才真实)
  python se3_control/scripts/run_se3_control.py --robot ur3 --task circle \
      --preview --preview-start-q -0.327 -0.6 2.4 -1.386 -1.571 2.738

  # 干跑: 连接 + FK 自检, 不发任何力矩
  python se3_control/scripts/run_se3_control.py --robot ur3 --dry-run

  # 记录数据到 npz (供实验分析)
  python se3_control/scripts/run_se3_control.py --robot ur3 \
      --task circle --save-log circle_ur3.npz

  # UR12e 位置保持 (UR12e 默认 IP .100; 大臂建议先 --torque-scale 0.3 降矩)
  python se3_control/scripts/run_se3_control.py --robot ur12e --torque-scale 0.3

  # 调试: 跳过 Phase 0 低带宽自检, 直接进入主阶段 (排查"一启动就急停"时用)
  python se3_control/scripts/run_se3_control.py --robot ur3 --task circle \
      --duration 16 --bandwidth 20 --skip-phase0

安全
----
  - 力矩限幅 = URDF effort 的 50% (可再 --torque-scale 降)
  - 相位状态机: Phase 0 低增益自检 → Phase 2 主阶段 → Phase 3 停机
    (--skip-phase0 时跳过 Phase 0, 直接主阶段)
  - 每控制周期检查错误状态, 异常/急停自动 emergency_stop
  - 启动前需按 Enter 确认; 结束停机前需再按 Enter 释放 (避免臂突然失去重力补偿)
  - 急停 / Ctrl+C 随时可用
"""

import argparse
import logging
import os
import sys
import time
from pathlib import Path

import numpy as np

# ─── 路径设置 ──────────────────────────────────────────────────────────
SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_DIR = SCRIPT_DIR.parent              # se3_control/
sys.path.insert(0, str(PROJECT_DIR))

from config.robot_configs import get_robot_config, get_urdf_path, get_hw_class
from robot_model.robot_model import RobotModel
from core.gic_controller import GICController
from core.servo_bridge import ServoJTorqueBridge
from core.trajectory import build_trajectory, eval_body_twist, TrajectoryFuncs
from core.se3_math import rotmat_slerp
from core.arm_log import ArmCsvLogger, arm_log_row

# 速度缩放感知 (方案 A): 每 _SPEED_SCALE_POLL 个控制周期读一次实机 RTDE 组合速度
# 缩放, 传给桥接器 set_speed_fraction() 缩放参考上限. 缩放变化是安全系统的阶梯
# 式调整, 40ms 一次足够及时 (降速时桥内立即生效).
_SPEED_SCALE_POLL = 10


# ====================================================================
# 1. 命令行参数
# ====================================================================

def parse_args():
    parser = argparse.ArgumentParser(
        description="SE(3) GIC 控制实机执行入口 (UR3/UR12e)")
    parser.add_argument('--robot', type=str, default='ur3',
                        choices=['ur12e', 'ur3'],
                        help='机器人类型 (ur3 默认 IP .11; ur12e 默认 IP .100)')
    parser.add_argument('--task', type=str, default='regulation',
                        choices=['regulation', 'circle', 'line'],
                        help='任务类型: regulation 位置保持 / circle / line 轨迹跟踪')
    parser.add_argument('--ip', type=str, default=None,
                        help='UR 控制箱 IP (默认从机器人配置加载)')
    parser.add_argument('--dt', type=float, default=0.004,
                        help='标称控制周期 (s), 默认 0.004 = 250 Hz')
    parser.add_argument('--duration', type=float, default=15.0,
                        help='Phase 2 主保持/跟踪时长 (s)')
    parser.add_argument('--hold-time', type=float, default=2.0,
                        help='Phase 0 低增益自检时长 (s)')
    parser.add_argument('--hold-bandwidth', type=float, default=8.0,
                        help='Phase 0 自检带宽 ω (rad/s), 低增益安全起步')
    parser.add_argument('--skip-phase0', action='store_true',
                        help='跳过 Phase 0 低带宽自检, 直接进入主阶段 (调试用, 请谨慎)')
    parser.add_argument('--bandwidth', type=float, default=20.0,
                        help='主控制带宽 ω (rad/s), 默认 20 (UR3 推荐)')
    parser.add_argument('--damping', type=float, default=1.0,
                        help='阻尼比 ζ, 默认 1.0 临界阻尼')
    parser.add_argument('--blend-time', type=float, default=0.5,
                        help='轨迹起步混合时长 (s), 从当前位姿平滑过渡到轨迹起点')
    parser.add_argument('--no-feasibility', action='store_true',
                        help='跳过轨迹 IK 可达性预检')
    parser.add_argument('--feasibility-samples', type=int, default=24,
                        help='可达性预检采样点数, 默认 24')
    parser.add_argument('--torque-scale', type=float, default=1.0,
                        help='力矩限幅缩放系数 (安全限幅 × scale), 默认 1.0')
    parser.add_argument('--control-mode', type=str, default='directTorque',
                        choices=['directTorque', 'servoJ'],
                        help='控制模式: directTorque (e-Series ≥5.23, 真力矩控制) / '
                             'servoJ (CB3 classic 回退, 无 directTorque 时用; 力矩折算成关节目标位)')
    parser.add_argument('--servo-gain', type=float, default=1000.0,
                        help='servoJ 跟踪增益 (100–2000), 越高跟踪越紧. 默认 1000')
    parser.add_argument('--servo-lookahead', type=float, default=0.1,
                        help='servoJ lookahead_time (0.03–0.2 s), 越小越灵敏. 默认 0.1')
    parser.add_argument('--servo-qdd-max', type=float, default=20.0,
                        help='servoJ 回退: 期望关节加速度限幅 (rad/s²). 默认 20')
    parser.add_argument('--servo-dq-max', type=float, default=2.0,
                        help='servoJ 回退: 期望关节速度限幅 (rad/s). 默认 2.0')
    parser.add_argument('--servo-ref-damp', type=float, default=15.0,
                        help='servoJ 回退: 参考速度阻尼 (1/s). 防参考积分跑赢内层伺服而发散. 默认 15')
    parser.add_argument('--servo-speed-fraction', type=float, default=None,
                        help='servoJ 回退: 实机速度缩放系数 s (0–1). 默认 None = 启动时 '
                             '从 RTDE 自动读取实际速度缩放 (REDUCED/降速时 <1.0, 如 0.24). '
                             '桥接器参考上限 dq_max/qdd_max 按 s 缩放, 防参考积分跑赢被限速的臂 '
                             '而发散 (方案 A, 见 real_vs_sim_diagnostics §8). '
                             '预览模式无 RTDE, 默认 1.0; 想预演降速场景可显式给 0.24')
    parser.add_argument('--servo-bandwidth-cap', type=float, default=10.0,
                        help='servoJ 回退: GIC 有效带宽上限 (rad/s). '
                             '位置伺服级联要求外环带宽低于内层伺服带宽, 默认 10')
    parser.add_argument('--center', type=float, nargs=3, metavar=('X', 'Y', 'Z'),
                        default=None,
                        help='覆盖轨迹圆心/中点 [x, y, z] (m). 仅对 circle/line 有效')
    parser.add_argument('--radius', type=float, default=None,
                        help='覆盖 circle 半径 (m). 默认读 task_config')
    parser.add_argument('--preview', action='store_true',
                        help='MuJoCo 闭环预览: 不连接硬件, 用同一参数在仿真里跑任务, '
                             '看轨迹 + 自动碰撞判定 (✓/✗), 通过后再上真机')
    parser.add_argument('--no-viewer', action='store_true',
                        help='预览不启动 MuJoCo 可视化窗口 (headless, 只出碰撞结论; 测试用)')
    parser.add_argument('--preview-speed', type=float, default=1.0,
                        help='预览实时倍速 (>1 加速, <1 慢放). 默认 1.0 = 与实机同步节奏')
    parser.add_argument('--preview-start-q', type=float, nargs=6, metavar='Q',
                        default=None,
                        help='预览起步位形 (6 个关节角 rad). 默认 home_q. '
                             '若真机当前位形不是 home (臂已折叠/低位), 用 --dry-run '
                             '读出当前 q 传入, 使预览从真实起步位形开始 (混合路径才真实)')
    parser.add_argument('--dry-run', action='store_true',
                        help='干跑: 连接 + FK 自检, 不发任何力矩')
    parser.add_argument('--save-log', type=str, default=None,
                        help='将记录数据保存为 npz 文件路径 (可选)')
    parser.add_argument('--log-dir', type=str, default=None,
                        help='记录每控制周期全分辨率数据到 CSV (写入此目录, 崩溃时也有数据). '
                             '与 --preview 同用则记仿真; 用于分析真机 vs 仿真差异 '
                             '(配合 tests/monitor/monitor_rtde.py / tests/monitor/analyze_arm_log.py)')
    return parser.parse_args()


# ====================================================================
# 2. 静态轨迹 (regulation: 期望位姿恒定, 速度加速度为零)
# ====================================================================

def make_static_traj(pd, Rd) -> TrajectoryFuncs:
    """构建恒定位姿的静态轨迹 (regulation 保持用)."""
    pd_arr = np.asarray(pd, dtype=float).ravel()
    Rd_arr = np.asarray(Rd, dtype=float).reshape(3, 3)
    return TrajectoryFuncs(
        pd_t=lambda t: pd_arr,
        Rd_t=lambda t: Rd_arr,
        dpd_t=lambda t: np.zeros(3),
        dRd_t=lambda t: np.zeros((3, 3)),
        ddpd_t=lambda t: np.zeros(3),
        ddRd_t=lambda t: np.zeros((3, 3)),
    )


# ====================================================================
# 3. 统一控制循环 (保持 / 跟踪共用)
# ====================================================================

def run_tracking(hw, robot_model, controller, traj, duration, dt,
                 phase_name, logger, blend_time=0.0, log_every=1.0,
                 bridge=None, log_dir=None):
    """以轨迹 ``traj`` 运行 GIC 跟踪 / 保持.

    循环每周期: 真实时间求值轨迹 → 起步混合 → GICController.compute →
    发力矩 / servoJ 下发 → 记录误差 → 安全检查 → wait_next_cycle.
    周期计时使用 ``wait_next_cycle`` 返回的真实经过时间, 不假定严格固定步长.

    :param traj:       TrajectoryFuncs — 期望轨迹时间函数族.
                       regulation 传 make_static_traj() 的静态轨迹, blend_time=0.
    :param blend_time: 起步混合时长 (s). 前 blend_time 秒从当前位姿平滑过渡到
                       轨迹参考 (位置 lerp + 朝向 slerp), 前馈速度/加速度按
                       bf = min(1, t/blend_time) 缩放, 避免起始位姿差力矩跳变.
                       0 → 不混合 (期望恒为轨迹值).
    :param log_every:  状态打印间隔 (s)
    :param bridge:     ServoJTorqueBridge — CB3 servoJ 回退时传入.
                       None=directTorque (直接发力矩); 非 None=力矩折算成
                       关节目标位, 走 hw.set_servo_joint_positions().
    :param log_dir:    若给定, 每控制周期写全分辨率 CSV 到该目录
                       (每个相位一个文件, 崩溃时也有数据).
    :returns: dict — t/p/q/tau/err(位置误差)/rerr(旋转误差)/blend_time/
                     t_total/n_steps/stopped
    """
    nv = robot_model.nv

    # 起步位姿 (混合起点; regulation 时即期望位姿)
    q, dq = hw.get_joint_states()
    robot_model.update(q, dq)
    p_start, R_start = robot_model.get_pose()
    if bridge is not None:
        bridge.reset(q, dq)   # 每个相位开始时复位积分器到当前关节状态

    # 日志缓存 (滑动采样, 上限 5000 点)
    n_steps = int(duration / dt)
    step_log = min(n_steps, 5000)
    log_interval = max(1, n_steps // step_log)
    t_log, p_log, q_log, tau_log, err_log, rerr_log = [], [], [], [], [], []

    # 全分辨率 CSV 记录 (每周期写, 崩溃时数据也在)
    csv_log = None
    if log_dir:
        os.makedirs(log_dir, exist_ok=True)
        fname = os.path.join(
            log_dir,
            f"{phase_name.replace(' ', '_')}_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        csv_log = ArmCsvLogger(fname, nv)
        logger.info(f"[{phase_name}] 记录每控制周期数据 → {fname}")

    t_real = 0.0
    n = 0
    stopped = False

    logger.info(f"[{phase_name}] 开始 ({duration:.1f}s, 混合 {blend_time:.1f}s)")

    while t_real < duration - 1e-6:
        # ── 1. 期望轨迹 (真实时间求值) ──
        bf = 1.0 if blend_time <= 0 else min(1.0, t_real / blend_time)
        pd_ref = traj.pd_t(t_real).ravel()
        Rd_ref = traj.Rd_t(t_real).reshape(3, 3)
        # 起步混合: 当前位姿 → 轨迹参考
        pd = (1.0 - bf) * p_start + bf * pd_ref
        Rd = rotmat_slerp(R_start, Rd_ref, bf)
        # 体坐标系期望速度/加速度 (前馈按 bf 缩放, 以混合后 Rd 为体系)
        vd, wd, dvd, dwd = eval_body_twist(traj, t_real, Rd, bf)

        # ── 2/3. GIC 控制力矩 → 发力矩 或 servoJ 关节目标位 ──
        q, dq = hw.get_joint_states()
        if bridge is not None:
            # 速度缩放感知 (方案 A): 周期性读实机实际速度缩放并缩放参考上限,
            # 防 servoJ 参考积分跑赢被限速的臂 (REDUCED/降速时 s<1.0) 而发散
            if n % _SPEED_SCALE_POLL == 0 and hasattr(hw, 'get_speed_scaling') \
                    and hasattr(bridge, 'set_speed_fraction'):
                bridge.set_speed_fraction(hw.get_speed_scaling())
            # CB3 回退: 力矩折算成 servoJ 关节目标位, 由 UR 内层伺服紧密跟踪
            q_servo, tau = bridge.compute(q, dq, pd, Rd, vd, wd, dvd, dwd)
            hw.set_servo_joint_positions(q_servo)
        else:
            tau = controller.compute(q, dq, pd, Rd, vd, wd, dvd, dwd)
            hw.set_joint_torques(tau)

        # ── 4. 记录 (误差对照真实轨迹参考, 非混合参考) ──
        robot_model.update(q, dq)
        p_cur, R_cur = robot_model.get_pose()
        pos_err = float(np.linalg.norm(p_cur - pd_ref))
        R_rel = R_cur.T @ Rd_ref
        cos_theta = np.clip(0.5 * (np.trace(R_rel) - 1.0), -1.0, 1.0)
        rot_err = float(np.arccos(cos_theta))
        # 全分辨率 CSV (每周期): q_servo=下发目标位, dq_des=桥接器参考速度
        if csv_log is not None:
            q_s = q_servo if bridge is not None else q
            dq_d = bridge.dq_target if bridge is not None else [np.nan] * nv
            tl = getattr(controller, '_tau_limits', None)
            tl_row = list(tl) if tl is not None else [np.nan] * nv
            csv_log.write(arm_log_row(
                nv, t_real, bf, pos_err, rot_err, pd, pd_ref, p_cur,
                q, dq, q_s, dq_d, tau, tl_row))
        n += 1
        if n % log_interval == 0:
            t_log.append(t_real)
            p_log.append(p_cur.copy())
            q_log.append(q.copy())
            tau_log.append(tau.copy())
            err_log.append(pos_err)
            rerr_log.append(rot_err)

        # ── 5. 周期状态 (每秒) ──
        if n % max(1, int(log_every / dt)) == 0:
            logger.info(
                f"[{phase_name}] t={t_real:6.2f}s  "
                f"||ep||={pos_err*1000:6.2f}mm  "
                f"rot={rot_err*1000:6.1f}mrad  "
                f"||tau||={np.linalg.norm(tau):5.1f}Nm  "
                f"p=[{p_cur[0]:.3f}, {p_cur[1]:.3f}, {p_cur[2]:.3f}]")

        # ── 6. 安全检查 ──
        err_state = hw.get_error_state()
        if err_state != 0:
            label = {1: '急停', 2: '保护性停止', 3: '安全模式警告', 4: '安全系统故障'}.get(
                err_state, '未知')
            logger.error(f"[{phase_name}] 错误状态 {err_state} ({label})")
            sm = hw.get_safety_mode()
            if sm is not None:
                logger.error(f"  UR 安全模式: {sm[0]} ({sm[1]})")
            raw_bits = hw.get_safety_status_bits()
            if raw_bits is not None:
                logger.error(f"  safety_status_bits: 0b{raw_bits:032b} (0x{raw_bits:08X})")
            if err_state == 2:
                logger.error("  提示: 保护性停止 — 通常是首发力矩触发安全限位(URDF 重力补偿/惯量不准或符号错误);")
                logger.error("        先 --torque-scale 0.3 降矩重试, 并校准重力补偿")
                logger.error("        (directTorque 还需控制器软件 PolyScope ≥ 5.23, e-Series)")
            hw.emergency_stop()
            stopped = True
            break

        # ── 7. 等待下一周期 (返回真实经过时间) ──
        t_real += hw.wait_next_cycle()

    if csv_log is not None:
        csv_log.close()

    summary = {
        't':          np.asarray(t_log),
        'p':          np.asarray(p_log) if p_log else np.zeros((0, 3)),
        'q':          np.asarray(q_log) if q_log else np.zeros((0, nv)),
        'tau':        np.asarray(tau_log) if tau_log else np.zeros((0, nv)),
        'err':        np.asarray(err_log) if err_log else np.zeros(0),
        'rerr':       np.asarray(rerr_log) if rerr_log else np.zeros(0),
        'blend_time': float(blend_time),
        't_total':    t_real,
        'n_steps':    n,
        'stopped':    stopped,
    }
    return summary


# ====================================================================
# 4. 轨迹 IK 可达性预检
# ====================================================================

def check_trajectory_feasibility(robot_model, traj, home_q, duration,
                                 n_samples=24, tol_pos=0.01, tol_rot=0.05,
                                 warn_lim_frac=0.90, warn_cond=1e4):
    """轨迹 IK 可达性预检 (实机运行前).

    在 ``duration`` 内均匀采样 ``n_samples`` 个轨迹点, 逐点高斯-牛顿 IK 求解,
    检查:
      1. 收敛 — 位置/旋转误差是否在容差内 (能否到达)
      2. 限位 — 关节是否接近运动范围边缘
      3. 奇异 — 体雅可比条件数是否过大

    :returns: (ok, report_dict) — ok=False 时调用方应中止运行
    """
    logger = logging.getLogger("run_se3_control")
    ts = np.linspace(0, duration, max(2, n_samples))
    lo = robot_model.model.lowerPositionLimit[:robot_model.nv]
    hi = robot_model.model.upperPositionLimit[:robot_model.nv]
    span = hi - lo
    q_seed = np.asarray(home_q, dtype=float).ravel()

    fails_pos, fails_rot = [], []
    max_lim_frac, max_cond, worst_cond_t = 0.0, 0.0, None

    for t in ts:
        pd = traj.pd_t(t).ravel()
        Rd = traj.Rd_t(t).reshape(3, 3)
        q_ik = robot_model.gauss_newton_IK(pd, Rd, q_seed, step_size=0.5,
                                           tol=1e-6, max_cnt=300, verbose=False)
        p_ik, R_ik = robot_model.get_pose()
        ep = float(np.linalg.norm(p_ik - pd))
        R_rel = R_ik.T @ Rd
        c = np.clip(0.5 * (np.trace(R_rel) - 1.0), -1.0, 1.0)
        er = float(np.arccos(c))
        if ep > tol_pos:
            fails_pos.append((float(t), ep))
        if er > tol_rot:
            fails_rot.append((float(t), er))

        # 限位利用 (距运动范围中心的偏差占比)
        if np.all(span > 0):
            half = 0.5 * span
            frac = float(np.max(np.abs(q_ik - 0.5 * (lo + hi)) / half))
        else:
            frac = 1.0
        max_lim_frac = max(max_lim_frac, frac)

        # 奇异 (体雅可比条件数)
        robot_model.update(q_ik)
        s = np.linalg.svd(robot_model.get_body_jacobian(), compute_uv=False)
        cond = float(s[0] / s[-1]) if s[-1] > 1e-12 else float('inf')
        if cond > max_cond:
            max_cond, worst_cond_t = cond, float(t)

        q_seed = q_ik   # 热启动下一个采样点

    ok = (not fails_pos) and (not fails_rot)
    report = dict(n_samples=len(ts), fails_pos=fails_pos, fails_rot=fails_rot,
                  max_lim_frac=max_lim_frac, max_cond=max_cond,
                  worst_cond_t=worst_cond_t)

    logger.info(f"\n{'='*50}")
    logger.info(f"轨迹可达性预检: {len(ts)} 采样点, "
                f"容差 pos<{tol_pos*1000:.0f}mm rot<{tol_rot*1000:.0f}mrad")
    if fails_pos:
        logger.warning(f"  ✗ 位置不可达 {len(fails_pos)} 点: "
                       f"{[f'{t:.1f}s({e*1000:.0f}mm)' for t, e in fails_pos[:5]]}")
    else:
        logger.info("  ✓ 位置全部可达")
    if fails_rot:
        logger.warning(f"  ✗ 朝向不可达 {len(fails_rot)} 点: "
                       f"{[f'{t:.1f}s({e*1000:.0f}mrad)' for t, e in fails_rot[:5]]}")
    else:
        logger.info("  ✓ 朝向全部可达")
    logger.info(f"  最大限位利用: {max_lim_frac*100:.0f}%  "
                f"({'⚠️ 接近关节限位' if max_lim_frac > warn_lim_frac else '余量充足'})")
    if np.isfinite(max_cond):
        cond_warn = max_cond > warn_cond
        logger.info(f"  最大雅可比条件数: {max_cond:.1e} @ t={worst_cond_t:.1f}s  "
                    f"({'⚠️ 接近奇异' if cond_warn else '远离奇异'})")
    else:
        logger.warning("  ✗ 存在奇异点 (雅可比奇异)")
        cond_warn = True
    logger.info(f"{'='*50}")

    if not ok:
        logger.error("预检未通过 — 请调整轨迹参数 (se3_control/config/task_config.py) "
                     "或更换起始位形")
    return ok, report


# ====================================================================
# 5. 摘要与结论
# ====================================================================

def print_summary(summary, robot_name, save_log=None, task='regulation'):
    """打印运行摘要统计并给出通过结论."""
    logger = logging.getLogger("run_se3_control")
    err = summary['err']
    rerr = summary['rerr']
    tt = summary['t']
    blend_time = summary.get('blend_time', 0.0)

    logger.info(f"\n{'='*50}")
    logger.info(f"运行摘要 [{robot_name}]  任务={task}")
    logger.info(f"  实际运行:     {summary['t_total']:.2f} s")
    logger.info(f"  控制周期数:   {summary['n_steps']}")
    if summary['n_steps'] > 0:
        logger.info(f"  平均频率:     {summary['n_steps']/summary['t_total']:.1f} Hz")

    if err.size > 0:
        tau = summary['tau']
        p = summary['p']
        # 稳态 (起步混合之后) 的误差, 排除起步瞬态
        mask = tt > blend_time + 1e-6
        err_ss = err[mask] if np.any(mask) else err
        rerr_ss = rerr[mask] if np.any(mask) else rerr

        logger.info(f"  最终位置误差:   {err[-1]*1000:7.2f} mm")
        logger.info(f"  平均位置误差:   {np.mean(err)*1000:7.2f} mm")
        logger.info(f"  最大位置误差:   {np.max(err)*1000:7.2f} mm")
        if np.any(mask):
            logger.info(f"  └─ 稳态位置误差: 均值 {np.mean(err_ss)*1000:6.2f} mm, "
                        f"最大 {np.max(err_ss)*1000:6.2f} mm")
        logger.info(f"  平均旋转误差:   {np.mean(rerr)*1000:7.2f} mrad")
        logger.info(f"  最大旋转误差:   {np.max(rerr)*1000:7.2f} mrad")
        logger.info(f"  位置标准差:     {np.std(p, axis=0)*1000:.2f} mm")
        logger.info(f"  关节力矩均值:   {np.round(np.mean(tau, axis=0), 2)} Nm")
        logger.info(f"  关节力矩标准差: {np.round(np.std(tau, axis=0), 3)} Nm")

        # 结论 (基于稳态误差)
        pass_t, warn_t = (0.01, 0.05) if task == 'regulation' else (0.02, 0.06)
        max_ss = float(np.max(err_ss))
        logger.info(f"\n{'='*50}")
        if max_ss < pass_t:
            logger.info(f"  ✅ 保持/跟踪通过 (稳态最大误差 ±{max_ss*1000:.1f} mm)")
        elif max_ss < warn_t:
            logger.info(f"  ⚠️  基本通过 (稳态最大误差 ±{max_ss*1000:.1f} mm)")
            logger.info(f"     建议增大带宽或检查 URDF 惯性参数")
        else:
            logger.warning(f"  ❌ 偏差过大 (稳态最大误差 {max_ss*1000:.1f} mm)")
            logger.warning(f"     请检查: URDF 惯性参数 / 控制频率 / 带宽 / 轨迹参数")
        logger.info(f"{'='*50}")

    if save_log and summary['t'].size > 0:
        np.savez(save_log,
                 t=summary['t'], p=summary['p'], q=summary['q'],
                 tau=summary['tau'], pos_err=summary['err'],
                 rot_err=summary['rerr'])
        logger.info(f"  已保存记录: {save_log}")


# ====================================================================
# 6. 干跑 (连接 + FK 自检, 不发力矩)
# ====================================================================

def run_dry_run(hw, robot_model):
    """连接 + 运动学自检, 不发任何力矩."""
    logger = logging.getLogger("run_se3_control")
    logger.info("干跑模式: 仅连接 + 运动学自检, 不发任何力矩")
    q, dq = hw.get_joint_states()
    robot_model.update(q, dq)
    p, _ = robot_model.get_pose()
    logger.info(f"  关节位置 q:   {np.round(q, 4)}")
    logger.info(f"  关节速度 dq:  {np.round(dq, 6)}")
    logger.info(f"  末端位置 p:   {np.round(p, 4)} m")
    err_state = hw.get_error_state()
    logger.info(f"  错误状态:     {err_state} ({'正常' if err_state == 0 else '异常!'})")
    sm = hw.get_safety_mode()
    if sm is not None:
        logger.info(f"  安全模式:     {sm[0]} ({sm[1]})")
    raw_bits = hw.get_safety_status_bits()
    if raw_bits is not None:
        logger.info(f"  安全状态位:   0b{raw_bits:032b} (0x{raw_bits:08X})")
    if err_state != 0:
        raise RuntimeError(f"机器人错误状态 {err_state}, 请检查教示器")
    logger.info("干跑完成 ✅ — 链路与模型正常, 可进入实机运行")


# ====================================================================
# 6.5 预览模式辅助 (MuJoCo 闭环仿真 + 碰撞判定, 不连接硬件)
# ====================================================================

def resolve_main_bandwidth(args, logger, speed_fraction=1.0):
    """servoJ 模式应用带宽上限 (级联稳定性), 返回主阶段有效带宽.

    :param float speed_fraction: 实机速度缩放 s (0–1). 降速时内层 UR 伺服有效带宽
        也按 s 缩小 → 外环 GIC 带宽上限按 s 缩放, 否则外环跑赢降速内环而发散
        (run_05_forced 实测: 24% 下 ω=6 发散, λ=+0.31/s, 振荡 1.44 rad/s).
        这是方案 A 完整版的一部分, 见 real_vs_sim_diagnostics §8.4-A.
    """
    main_bandwidth = args.bandwidth
    if args.control_mode == 'servoJ' and args.bandwidth > args.servo_bandwidth_cap:
        main_bandwidth = args.servo_bandwidth_cap
        logger.warning(
            f"servoJ 模式: --bandwidth {args.bandwidth} 超过位置伺服级联上限 "
            f"{args.servo_bandwidth_cap}, 主阶段有效带宽降至 {main_bandwidth} rad/s "
            f"(内层 UR 伺服带宽有限, 参考积分会跑赢伺服而发散; 用 --servo-bandwidth-cap 调整)")
    if args.control_mode == 'servoJ' and speed_fraction < 0.99:
        omega_eff = main_bandwidth * speed_fraction
        if omega_eff < main_bandwidth - 1e-9:
            logger.warning(
                f"⚠️ 实机速度缩放 {speed_fraction:.2f}: 内层伺服有效带宽按 s 缩小, "
                f"外环有效带宽降至 {omega_eff:.2f} rad/s "
                f"(级联稳定, 见 real_vs_sim_diagnostics §8.4-A)")
            main_bandwidth = omega_eff
    return main_bandwidth


def build_task_trajectory(args, robot_model, robot_cfg, logger):
    """按 --task 构建轨迹, 应用 --center/--radius 覆盖, 并做 IK 可达性预检.

    对 regulation 返回 (None, None). 预检未通过时 traj_task=None.

    :returns: (traj_task, task_cfg) — task_cfg 为按 --robot 匹配的任务参数
              namespace (已应用命令行覆盖, 供预览/实机共用)
    """
    if args.task == 'regulation':
        return None, None
    from config import task_config
    task_cfg = task_config.get_task_config(args.robot)
    # 命令行覆盖圆心/半径 (仅 circle/line; 改的是 get_task_config 返回的
    # namespace 内部分拷贝, 不影响 task_config 模块级全局配置)
    if args.center is not None:
        tdict = getattr(task_cfg, args.task, None)
        if isinstance(tdict, dict) and 'center' in tdict:
            tdict['center'] = list(args.center)
            logger.info(f"[轨迹] --center 覆盖 {args.task} 中心 → {list(args.center)} m")
        else:
            logger.warning(f"--center 对任务 '{args.task}' 无效 (仅 circle/line)")
    if args.radius is not None and args.task == 'circle':
        task_cfg.circle['radius'] = args.radius
        logger.info(f"[轨迹] --radius 覆盖 circle 半径 → {args.radius} m")
    traj_task = build_trajectory(args.task, cfg=task_cfg)
    logger.info(f"[轨迹] {args.task}: 中心 {np.round(traj_task.pd_t(0.0).ravel(), 3)} m "
                f"(起点 t=0)")
    if not args.no_feasibility:
        ok, _ = check_trajectory_feasibility(
            robot_model, traj_task, robot_cfg['home_q'], args.duration,
            n_samples=args.feasibility_samples)
        if not ok:
            logger.error("可达性预检未通过 — 中止运行")
            return None, task_cfg
    return traj_task, task_cfg


def run_preview_cli(args, cfg, robot_model, torque_limits, logger):
    """--preview 入口: 在 MuJoCo 里跑与实机 Phase2 相同的闭环任务.

    不连接硬件. 同一套 CLI 参数 (轨迹/带宽/阻尼/力矩限幅/混合), 复用
    core.mujoco_preview.run_preview (directTorque 发力矩, servoJ 走桥+内层伺服),
    末端轨迹实时可视化 + 自动碰撞判定.

    :returns: bool — True=无碰撞风险 (可去掉 --preview 上真机)
    """
    from config import task_config
    from core.mujoco_preview import run_preview

    # 构建轨迹 (应用 --center/--radius 覆盖 + 参考级 IK 预检)
    if args.task == 'regulation':
        traj_task, task_cfg = None, task_config.get_task_config(args.robot)
    else:
        traj_task, task_cfg = build_task_trajectory(args, robot_model, cfg, logger)
        if traj_task is None:
            return False

    if traj_task is None:
        # regulation: 预览假设臂从 home 起步, 保持 home 位姿
        robot_model.update(cfg['home_q'])
        pd, Rd = robot_model.get_pose()
        traj = make_static_traj(pd, Rd)
    else:
        traj = traj_task

    # 预览无 RTDE, 速度缩放只能来自 --servo-speed-fraction (None → 满速 1.0);
    # 想预演"实机被限速"的参考行为可显式传 0.24 (仿真臂本身无速度上限, 只对齐参考侧)
    servo_speed_fraction = args.servo_speed_fraction \
        if args.servo_speed_fraction is not None else 1.0
    main_bandwidth = resolve_main_bandwidth(args, logger,
                                            speed_fraction=servo_speed_fraction)

    logger.info(f"\n{'='*70}")
    logger.info(f"  MuJoCo 闭环预览 [{cfg['name']}]  任务={args.task}  模式={args.control_mode}")
    logger.info(f"  带宽 {main_bandwidth} rad/s | 时长 {args.duration}s | 混合 {args.blend_time}s")
    if args.preview_start_q is not None:
        logger.info(f"  起步位形: --preview-start-q = "
                    f"{np.round(args.preview_start_q, 3)} (真机当前位形)")
    else:
        logger.info(f"  起点假设: home_q (真机从当前位姿起步; 臂不在 home 时 "
                    f"用 --dry-run 读当前 q 传 --preview-start-q)")
    logger.info(f"  {'可视化窗口 (关闭窗口可提前结束)' if not args.no_viewer else 'headless — 仅碰撞结论'}")
    logger.info(f"{'='*70}")

    start_q = args.preview_start_q if args.preview_start_q is not None \
        else cfg['home_q']
    res = run_preview(
        args.robot, get_urdf_path(args.robot), cfg['ee_frame'], cfg['home_q'],
        traj, task_cfg=task_cfg, bandwidth=main_bandwidth, damping=args.damping,
        torque_limits=torque_limits, duration=args.duration, ctrl_dt=args.dt,
        blend_time=args.blend_time, control_mode=args.control_mode,
        servo_speed_fraction=servo_speed_fraction,
        show_viewer=not args.no_viewer, speed=args.preview_speed,
        link_to_mesh=cfg['link_to_mesh'], mesh_subdir=cfg['mesh_subdir'],
        start_q=start_q, logger=logger, log_dir=args.log_dir,
    )

    if res['verdict']['ok']:
        logger.info(f"\n  预览通过 — 无碰撞风险. 去掉 --preview 即可用相同参数上真机.")
        return True
    logger.warning(f"\n  预览发现碰撞风险 — 请调整 --center/--radius/带宽后重跑预览, 勿直接上真机.")
    return False


# ====================================================================
# 7. 主入口
# ====================================================================

def main():
    args = parse_args()
    cfg = get_robot_config(args.robot)
    RobotHW = get_hw_class(args.robot)
    robot_name = cfg['name']
    torque_limits = cfg['torque_limits'] * args.torque_scale

    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s.%(msecs)03d] %(message)s",
        datefmt="%H:%M:%S",
    )
    logger = logging.getLogger("run_se3_control")

    print("=" * 70)
    print(f"  SE(3) GIC 控制 — 实机执行 [{robot_name}]")
    print("=" * 70)
    print(f"\n   机器人:        {robot_name}")
    print(f"   任务:          {args.task}")
    print(f"   控制频率:      {1/args.dt:.0f} Hz")
    print(f"   控制模式:      {args.control_mode} "
          f"({'CB3 servoJ 回退' if args.control_mode == 'servoJ' else 'directTorque 力矩'})")
    print(f"   主带宽:        {args.bandwidth} rad/s")
    if args.skip_phase0:
        print(f"   Phase0:        已跳过 (--skip-phase0)")
    else:
        print(f"   Phase0 带宽:   {args.hold_bandwidth} rad/s")
    print(f"   时长:          {'主阶段' if args.skip_phase0 else f'Phase0 {args.hold_time}s → 主'} {args.duration}s")
    if args.task != 'regulation':
        print(f"   起步混合:      {args.blend_time}s")
        print(f"   可达性预检:    {'跳过' if args.no_feasibility else f'{args.feasibility_samples} 采样点'}")
    print(f"   力矩限幅:      {np.round(torque_limits, 1)} Nm")
    if args.preview:
        print(f"\n   🔎 预览模式: 不连接硬件, 在 MuJoCo 仿真里跑同一任务")
        print(f"     实时看轨迹 + 自动碰撞判定 (✓/✗), 通过后再去掉 --preview 上真机")
    print(f"\n   ⚠️  安全提醒:")
    print(f"     1. 教示器处于 远程控制(Remote Control) 模式")
    print(f"     2. 臂周围无人/障碍物, 急停按钮可触及")
    if args.skip_phase0:
        print(f"     3. 已跳过 Phase 0 自检, 主阶段直接起步 (留意首发力矩)")
    else:
        print(f"     3. Phase 0 为低带宽自检, 若异常臂会缓慢偏位 (非急动)")
    print(f"     4. 轨迹任务会先做可达性预检; 起步有 {args.blend_time:.1f}s 混合过渡")
    print(f"     5. 随时可按急停或 Ctrl+C\n")

    # ── 初始化模型与硬件 ──
    urdf_path = get_urdf_path(args.robot)
    ip = args.ip if args.ip else cfg['default_ip']

    logger.info("初始化 RobotModel ...")
    robot_model = RobotModel(urdf_path, ee_frame_name=cfg['ee_frame'],
                             robot_name=robot_name, verbose=True)

    # ── 预览模式: 不连接硬件, 在 MuJoCo 里跑同一任务闭环仿真 + 碰撞判定 ──
    if args.preview:
        if args.dry_run:
            logger.error("--preview 与 --dry-run 互斥 (预览不连接硬件)")
            sys.exit(2)
        ok = run_preview_cli(args, cfg, robot_model, torque_limits, logger)
        sys.exit(0 if ok else 1)

    logger.info(f"初始化 {robot_name}HW @ {ip} ...")
    hw = RobotHW(ip=ip, dt=args.dt, verbose=True)

    try:
        hw.initialize()
        # 在硬件层也套用 (可能被缩放的) 限幅, 双保险
        hw.set_torque_limits(torque_limits)

        if args.control_mode == 'servoJ':
            hw.set_servo_mode(True)
            logger.info(
                f"servoJ 回退已启用: gain={args.servo_gain} "
                f"lookahead={args.servo_lookahead}s "
                f"qdd_max={args.servo_qdd_max}rad/s² dq_max={args.servo_dq_max}rad/s")

        # ── 速度缩放感知 (方案 A): 启动读实机实际速度缩放, 桥接器参考上限按 s 缩放 ──
        # 实机在 REDUCED/降速时 UR 伺服实际最大关节速度 = s×额定; 参考积分器按满速
        # (dq_max=2 rad/s) 前进会跑赢被限速的臂 → 参考积分漂移 → 力矩饱和 → 发散.
        # 参考上限 (dq_max/qdd_max) 乘 s 后, 参考速度上限 ≤ 实际能力, 任意降速下稳定.
        # 运行中 run_tracking 还会每周期继续读速度缩放更新 (安全降速时立即生效).
        servo_speed_fraction = args.servo_speed_fraction
        if args.control_mode == 'servoJ' and servo_speed_fraction is None:
            try:
                servo_speed_fraction = hw.get_speed_scaling()
            except Exception as e:
                logger.warning(f"读取速度缩放失败, 按满速 1.0 处理: {e}")
                servo_speed_fraction = 1.0
        if servo_speed_fraction is None:
            servo_speed_fraction = 1.0
        if args.control_mode == 'servoJ':
            if servo_speed_fraction < 0.9:
                logger.warning(
                    f"⚠️ 实机速度缩放 = {servo_speed_fraction:.2f} (<1.0, REDUCED/降速). "
                    f"桥接器参考上限已按 s×dq_max 缩放, 防参考积分跑赢被限速的臂而发散; "
                    f"任务将按该速度下稳定运行 (详见 real_vs_sim_diagnostics §8).")
            else:
                logger.info(f"实机速度缩放 = {servo_speed_fraction:.2f} "
                            f"({'满速' if servo_speed_fraction >= 0.99 else '降速'})")

        if args.dry_run:
            run_dry_run(hw, robot_model)
            return

        # 当前位姿 (Phase0 保持期望 + 轨迹混合起点)
        q, dq = hw.get_joint_states()
        robot_model.update(q, dq)
        pd, Rd = robot_model.get_pose()
        logger.info(f"当前末端位置: {np.round(pd, 4)} m")

        # ── 轨迹任务: 构建轨迹 + IK 可达性预检 (与预览共用同一逻辑) ──
        traj_task, _ = build_task_trajectory(args, robot_model, cfg, logger)
        if args.task != 'regulation' and traj_task is None:
            return

        # 重置周期定时器 (连接后可能已过较长时间)
        try:
            hw._ctrl.initPeriod()
        except Exception:
            pass

        if args.skip_phase0:
            input("\n   按 Enter 开始主阶段 (已跳过 Phase 0 自检) ...")
        else:
            input("\n   按 Enter 开始 Phase 0 自检 ...")

        # ── Phase 0: 低带宽保持自检 (当前位姿) ──
        # --skip-phase0 时跳过 (调试/快速验证用); 主阶段同样每周期做安全检查,
        # 若主阶段仍触发急停, 说明是检测或首发力矩问题, 而非 Phase 0 本身.
        # servoJ 级联要求外环带宽 < 内层伺服带宽; 超过上限时限制并告警;
        # 速度缩放感知 (方案A): 降速时内层伺服带宽按 s 缩小 → 外环带宽也按 s 缩放
        main_bandwidth = resolve_main_bandwidth(args, logger,
                                                speed_fraction=servo_speed_fraction)

        def make_bridge(ctrl):
            """servoJ 模式构建力矩→关节目标位桥接器; directTorque 模式返回 None."""
            if args.control_mode != 'servoJ':
                return None
            return ServoJTorqueBridge(
                robot_model, ctrl, args.dt,
                qdd_max=np.full(robot_model.nv, args.servo_qdd_max),
                dq_max=np.full(robot_model.nv, args.servo_dq_max),
                ref_damp=args.servo_ref_damp,
                speed_fraction=servo_speed_fraction)

        if not args.skip_phase0:
            phase0_bandwidth = min(args.hold_bandwidth, main_bandwidth)
            ctrl0 = GICController(robot_model, bandwidth=phase0_bandwidth,
                                  damping=args.damping, torque_limits=torque_limits)
            traj0 = make_static_traj(pd, Rd)
            s0 = run_tracking(hw, robot_model, ctrl0, traj0, args.hold_time,
                              args.dt, "Phase0", logger, blend_time=0.0,
                              bridge=make_bridge(ctrl0), log_dir=args.log_dir)
            if s0['stopped']:
                logger.error("Phase 0 自检因错误状态中止 — 不进入主阶段")
                return

        # ── Phase 2: 主阶段 ──
        ctrl = GICController(robot_model, bandwidth=main_bandwidth,
                             damping=args.damping, torque_limits=torque_limits)
        bridge2 = make_bridge(ctrl)
        if args.task == 'regulation':
            # 重新读当前位姿 (Phase0 后可能微漂) 作为期望
            q, dq = hw.get_joint_states()
            robot_model.update(q, dq)
            pd, Rd = robot_model.get_pose()
            traj2 = make_static_traj(pd, Rd)
            s = run_tracking(hw, robot_model, ctrl, traj2, args.duration,
                             args.dt, "Phase2", logger, blend_time=0.0,
                             bridge=bridge2, log_dir=args.log_dir)
        else:
            s = run_tracking(hw, robot_model, ctrl, traj_task, args.duration,
                             args.dt, "Phase2", logger,
                             blend_time=args.blend_time, bridge=bridge2,
                             log_dir=args.log_dir)

        print_summary(s, robot_name, save_log=args.save_log, task=args.task)

        # ── Phase 3: 释放确认 (正常结束时) ──
        # 停转后 shutdown 会发零力矩, 臂失去重力补偿可能下沉;
        # 在 Enter 前臂保持最后一次力矩指令, 由操作者掌控释放时机.
        if not s['stopped']:
            input("\n   按 Enter 释放机械臂并停机 ...")

    except KeyboardInterrupt:
        logger.warning("\n\n用户终止 — 执行急停")
        hw.emergency_stop()
    except Exception as e:
        logger.error(f"运行失败: {e}")
        hw.emergency_stop()
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        hw.shutdown()

    logger.info("运行结束 — 已安全停机")


if __name__ == '__main__':
    main()
