"""
URHW — 通用 UR 机械臂硬件接口
===================================

基于 ur_rtde（RTDE 协议）实现 RobotHWInterface，
适用于所有 Universal Robots 系列机械臂 (UR3/UR5/UR10/UR12e 等)。

硬件接口:
  - 关节状态: RTDE 协议, 500 Hz
  - 力矩指令: directTorque, 500 Hz
  - 力传感器: 无内置 FT, 返回零向量 (需外部传感器)

用法:
  from hardware.ur_hw import URHW

  with URHW(ip="192.168.1.100",
            torque_limits=np.array([165.0]*6),
            joint_names=["shoulder_pan_joint", ...],
            robot_name="UR12e") as robot:
      q, dq = robot.get_joint_states()
      robot.set_joint_torques(np.zeros(6))
"""

from __future__ import annotations

import datetime
import logging
import socket
import time
from typing import List, Optional, Tuple

import numpy as np

from .interface import (
    RobotHWInterface,
    HardwareConnectionError,
    HardwareSafetyError,
    HardwareTimeoutError,
    clip_torques,
)

# ================================================================
# 尝试导入 ur_rtde，失败时给出清晰的安装提示
# ================================================================

try:
    import rtde_receive
    import rtde_control

    _UR_RTDE_AVAILABLE = True
except ImportError:
    _UR_RTDE_AVAILABLE = False
    _UR_RTDE_IMPORT_ERROR = (
        "ur_rtde 未安装。请执行以下命令安装:\n"
        "  conda activate roboarm\n"
        "  pip install ur-rtde\n"
        "\n"
        "或在 robostack 中:\n"
        "  conda install -c conda-forge ur-rtde"
    )

# ================================================================
# 默认参数
# ================================================================

# 标称控制周期 (秒) — 250 Hz
_DEFAULT_DT = 0.004

# 默认连接超时 (秒)
_DEFAULT_TIMEOUT = 5.0

# RTDE safety_mode → 中文标签 (UR 官方枚举)
_SAFETY_MODE_LABELS = {
    1: '正常', 2: '降级', 3: '保护性停止', 4: '恢复中',
    5: '安全防护停止', 6: '系统急停', 7: '机器人急停', 8: '急停',
    9: '剧烈碰撞', 10: '故障', 11: '参数校验', 12: '无电源',
    13: '无安全控制器', 14: '反向驱动',
}


# ================================================================
# URHW
# ================================================================

class URHW(RobotHWInterface):
    """通用 UR 机械臂硬件接口。

    :param str ip:            UR 控制箱 IP 地址
    :param np.ndarray torque_limits: 关节力矩安全限幅 (6,), Nm
    :param list[str] joint_names: 关节名称列表 (与 URDF 对齐)
    :param str robot_name:    机械臂名称 (用于日志, 如 "UR12e")
    :param float dt:          标称控制周期 (秒), 默认 0.004 (250 Hz)
    :param float timeout:     连接超时 (秒), 默认 5.0
    :param bool verbose:      是否打印详细日志
    :param logger:            外部 logger 实例 (None 则自动创建)
    """

    def __init__(
        self,
        ip: str,
        torque_limits: np.ndarray,
        joint_names: List[str],
        robot_name: str = "UR",
        dt: float = _DEFAULT_DT,
        timeout: float = _DEFAULT_TIMEOUT,
        verbose: bool = True,
        logger: Optional[logging.Logger] = None,
        servo_lookahead: float = 0.1,
        servo_gain: float = 1000.0,
    ):
        if not _UR_RTDE_AVAILABLE:
            raise ImportError(_UR_RTDE_IMPORT_ERROR)

        super().__init__(name=f"{robot_name}@{ip}", logger=logger)

        self._ip = ip
        self._dt = dt
        self._timeout = timeout
        self._verbose = verbose
        self._robot_name = robot_name
        self._joint_names = list(joint_names)

        # RTDE 接口实例
        self._recv: Optional[rtde_receive.RTDEReceiveInterface] = None
        self._ctrl: Optional[rtde_control.RTDEControlInterface] = None

        # CB3 servoJ 回退模式 (默认关; set_servo_mode() 切换)
        self._servo_mode = False
        self._servo_lookahead = float(servo_lookahead)
        self._servo_gain = float(servo_gain)
        self._servo_speed = 0.0
        self._servo_accel = 0.0
        self._last_cycle_mono: Optional[float] = None

        # 初始力矩限幅
        self.set_torque_limits(np.asarray(torque_limits, dtype=float).copy())

        # 缓存
        n_joints = len(self._joint_names)
        self._last_q = np.zeros(n_joints)
        self._last_dq = np.zeros(n_joints)
        self._last_timestamp = 0.0

    # ── 日志简写 ────────────────────────────────────────────────

    def _log(self, msg: str, level: int = logging.INFO):
        if self._verbose or level >= logging.WARNING:
            self._logger.log(level, msg)

    # ── 生命周期 ──────────────────────────────────────────────

    def initialize(self) -> None:
        """建立与 UR 机械臂的 RTDE 连接。

        步骤:
          1. 创建 RTDEReceiveInterface (读关节状态)
          2. 创建 RTDEControlInterface (发力矩指令)
          3. 验证连接状态
          4. 读取初始关节状态
        """
        if self._connected:
            self._logger.warning("已经连接，跳过")
            return

        self._logger.info(f"正在连接 {self._robot_name} @ {self._ip} ...")

        # TCP 预检: RTDEReceiveInterface 对不可达 IP 会无限阻塞,
        # 先用 self._timeout 限定 socket 连接, 使"机器人不在线"快速失败而非挂死.
        try:
            probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            probe.settimeout(self._timeout)
            probe.connect((self._ip, 30004))   # RTDE 端口
            probe.close()
        except OSError as e:
            self._connected = False
            raise HardwareConnectionError(
                f"无法连接 {self._robot_name} @ {self._ip}:30004 (RTDE): {e}\n"
                f"  请确认: 机械臂已开机 / IP 正确 / 教示器处于 Remote Control 模式"
            ) from e

        try:
            # 先建立接收接口 (超时短, 快速失败)
            self._recv = rtde_receive.RTDEReceiveInterface(self._ip)
            if not self._recv.isConnected():
                raise HardwareConnectionError(
                    f"RTDEReceiveInterface 连接失败: {self._ip}"
                )

            # 再建立控制接口
            self._ctrl = rtde_control.RTDEControlInterface(self._ip)
            if not self._ctrl.isConnected():
                raise HardwareConnectionError(
                    f"RTDEControlInterface 连接失败: {self._ip}"
                )

        except Exception as e:
            self._connected = False
            raise HardwareConnectionError(
                f"无法连接到 {self._robot_name} @ {self._ip}: {e}"
            ) from e

        self._connected = True
        self._enabled = True

        # 读取初始关节状态
        try:
            n_joints = len(self._joint_names)
            q_raw = self._recv.getActualQ()
            dq_raw = self._recv.getActualQd()
            self._last_q = np.array(q_raw, dtype=float)[:n_joints]
            self._last_dq = np.array(dq_raw, dtype=float)[:n_joints]
            self._last_timestamp = self._recv.getTimestamp()
        except Exception as e:
            self._logger.warning(f"读取初始状态失败: {e}")

        # 初始化控制周期定时器
        self._ctrl.initPeriod()

        if self._verbose:
            self._logger.info(f"{self._robot_name} @ {self._ip} 连接成功")
            self._logger.info(f"  初始 q:  {np.round(self._last_q, 4)}")
            self._logger.info(f"  初始 dq: {np.round(self._last_dq, 4)}")
            self._logger.info(f"  控制周期: {self._dt * 1000:.1f} ms ({1/self._dt:.0f} Hz)")
            self._logger.info(f"  力矩限幅: {self._torque_limits}")

    def shutdown(self) -> None:
        """断开与 UR 机械臂的 RTDE 连接。

        步骤:
          1. 将所有关节力矩置零
          2. 断开控制接口
          3. 断开接收接口
        """
        if not self._connected:
            return

        self._logger.info(f"正在断开 {self._robot_name} 连接 ...")

        # 先停伺服 / 发零力矩 (安全)
        n_joints = len(self._joint_names)
        try:
            if self._ctrl is not None and self._ctrl.isConnected():
                if self._servo_mode:
                    self._ctrl.servoStop()
                else:
                    self._ctrl.directTorque([0.0] * n_joints)
        except Exception as e:
            self._logger.warning(f"置零失败: {e}")

        # 断开控制接口
        try:
            if self._ctrl is not None:
                self._ctrl.disconnect()
        except Exception as e:
            self._logger.warning(f"断开控制接口失败: {e}")

        # 断开接收接口
        try:
            if self._recv is not None:
                self._recv.disconnect()
        except Exception as e:
            self._logger.warning(f"断开接收接口失败: {e}")

        self._ctrl = None
        self._recv = None
        self._connected = False
        self._enabled = False

        self._logger.info(f"{self._robot_name} 连接已断开")

    # ── 状态读取 ──────────────────────────────────────────────

    def get_joint_states(self) -> Tuple[np.ndarray, np.ndarray]:
        """读取 UR 机械臂当前关节状态。

        使用 RTDE 协议读取关节位置和速度。
        通信失败时返回上一帧缓存 + 日志警告（而非崩溃）。
        """
        if not self._connected:
            raise HardwareConnectionError(f"未连接 {self._robot_name}")

        try:
            q_raw = self._recv.getActualQ()
            dq_raw = self._recv.getActualQd()

            n_joints = len(self._joint_names)
            self._last_q = np.array(q_raw, dtype=float)[:n_joints]
            self._last_dq = np.array(dq_raw, dtype=float)[:n_joints]
            self._last_timestamp = self._recv.getTimestamp()

        except Exception as e:
            self._logger.warning(f"读取关节状态失败 (返回缓存): {e}")
            # 返回缓存值，避免控制循环崩溃

        return self._last_q.copy(), self._last_dq.copy()

    def get_speed_scaling(self) -> float:
        """读取 RTDE 组合速度缩放 — 实机实际生效的关节速度上限比例 (0–1).

        优先 ``getSpeedScalingCombined`` (含安全/降级限制; REDUCED 或降速时 < 1.0,
        如 run_04 恒为 0.24), 失败回退 ``getSpeedScaling``; 未连接返回 1.0.
        上层 ServoJTorqueBridge 据此缩放参考上限 (dq_max×s), 防参考积分跑赢
        被限速的臂而发散 (详见 se3_control/docs/analysis/real_vs_sim_diagnostics §8).
        """
        if not self._connected or self._recv is None:
            return 1.0
        for getter in (self._recv.getSpeedScalingCombined,
                       self._recv.getSpeedScaling):
            try:
                s = getter()
                if s is not None:
                    return float(s)
            except Exception:
                continue
        return 1.0

    def get_ft_sensor(self) -> np.ndarray:
        """获取末端力/力矩传感器读数。

        UR 系列无内置 FT 传感器，返回零向量 (6,)。
        若安装了外部 FT 传感器，需重写此方法或修改此处代码。

        返回: [fx, fy, fz, tx, ty, tz] — 单位 N/Nm
        """
        return np.zeros(6)

    # ── 执行 ──────────────────────────────────────────────────

    def set_joint_torques(self, tau: np.ndarray) -> None:
        """下发 UR 机械臂关节力矩指令。

        使用 ur_rtde 的 directTorque 力矩前馈模式。
        UR 内部位置环作为安全兜底，GIC 力矩作为前馈叠加。

        :param tau: ndarray (nv,) — 期望关节力矩 (Nm)

        安全约束:
          1. 急停检查: emergency_stopped 时禁止发力矩
          2. 力矩限幅: 每个关节不超过 torque_limits
        """
        if self._emergency_stopped:
            raise HardwareSafetyError("急停已触发 — 禁止发力矩")

        if self._servo_mode:
            raise HardwareSafetyError(
                "当前为 servoJ 回退模式 (CB3, 无 directTorque), "
                "请用 set_servo_joint_positions() 下发关节目标位")

        if not self._connected or self._ctrl is None:
            raise HardwareConnectionError(f"未连接 {self._robot_name}")

        # 限幅
        tau_safe = clip_torques(
            np.asarray(tau, dtype=float).ravel(), self._torque_limits
        )

        try:
            self._ctrl.directTorque(tau_safe.tolist())
        except Exception as e:
            self._logger.error(f"发力矩失败: {e}")
            raise HardwareTimeoutError(
                f"RTDE 力矩指令发送失败: {e}"
            ) from e

    def set_servo_mode(self, servo: bool) -> None:
        """切换 CB3 servoJ 回退模式.

        CB3 classic (固件 < 5.23) 无 directTorque, ur_rtde 会静默移除该命令.
        servo=True 时:
          - set_joint_torques() 抛 HardwareSafetyError (防误发死的 directTorque)
          - 控制节奏由 servoJ 的 ``time`` 参数 (阻塞 dt) 提供, wait_next_cycle()
            改为按真实经过时间补足 dt.
        """
        self._servo_mode = bool(servo)
        self._last_cycle_mono = None
        self._logger.info(f"控制模式: {'servoJ (CB3 回退)' if self._servo_mode else 'directTorque'}")

    def set_servo_joint_positions(self, q_target: np.ndarray,
                                  speed: Optional[float] = None,
                                  accel: Optional[float] = None,
                                  lookahead: Optional[float] = None,
                                  gain: Optional[float] = None) -> None:
        """CB3 回退: 以 servoJ 关节位置伺服下发关节目标位.

        上层 ServoJTorqueBridge 把 GIC 力矩折算成关节目标位, 这里只做下发.
        servoJ 全版本支持 (无版本门控), 是 CB3 上驱动机械臂的标准原语.

        :param q_target: ndarray (nv,) — 关节目标位 (rad)
        :param speed:    关节速度上限 (rad/s); None=不用 (ur_rtde 当前版本忽略)
        :param accel:    关节加速度上限 (rad/s²); None=不用 (ur_rtde 当前版本忽略)
        :param lookahead: 0.03–0.2 s, 越小越灵敏; None=用 __init__ 默认 0.1
        :param gain:      100–2000, 越高跟踪越紧; None=用 __init__ 默认 1000

        servoJ 的 ``time`` 参数取 ``self._dt``, 调用会阻塞 ``self._dt`` 秒
        (提供控制循环节奏).
        """
        if self._emergency_stopped:
            raise HardwareSafetyError("急停已触发 — 禁止伺服")

        if not self._connected or self._ctrl is None:
            raise HardwareConnectionError(f"未连接 {self._robot_name}")

        q = np.asarray(q_target, dtype=float).ravel()
        n_joints = len(self._joint_names)
        if q.shape[0] != n_joints:
            raise ValueError(f"关节目标位长度 {q.shape[0]} ≠ {n_joints}")
        if not np.all(np.isfinite(q)):
            raise ValueError("关节目标位含 NaN/Inf")

        spd = self._servo_speed
        acc = self._servo_accel
        lk = self._servo_lookahead if lookahead is None else float(lookahead)
        gn = self._servo_gain if gain is None else float(gain)

        if not (0.03 <= lk <= 0.2):
            self._logger.warning(f"lookahead_time {lk} 超出 [0.03,0.2], 已限幅")
            lk = np.clip(lk, 0.03, 0.2)
        if not (100 <= gn <= 2000):
            self._logger.warning(f"gain {gn} 超出 [100,2000], 已限幅")
            gn = np.clip(gn, 100, 2000)

        try:
            ok = self._ctrl.servoJ(q.tolist(), spd, acc, self._dt, lk, gn)
            if not ok:
                raise HardwareTimeoutError(
                    "servoJ 返回失败 — 检查教示器是否处于远程控制且程序运行中")
        except HardwareTimeoutError:
            raise
        except Exception as e:
            self._logger.error(f"servoJ 下发失败: {e}")
            raise HardwareTimeoutError(f"RTDE servoJ 指令发送失败: {e}") from e

    # ── 定时 ──────────────────────────────────────────────────

    def get_timestep(self) -> float:
        """获取标称控制周期。

        :returns: 默认 0.004 s (250 Hz) — 可通过 __init__ 的 dt 参数修改
        """
        return self._dt

    def wait_next_cycle(self) -> float:
        """等待下一个控制周期。

        使用 ur_rtde 的 waitPeriod() 同步控制循环。
        注: waitPeriod() 需要 datetime.timedelta 参数。

        :returns: 自上次 initPeriod/waitPeriod 以来实际经过的时间 (秒)
        """
        if self._servo_mode:
            # servoJ 的 time 参数已阻塞 dt; 这里按真实经过时间补足, 保证周期 ≥ dt
            now = time.monotonic()
            if self._last_cycle_mono is None:
                self._last_cycle_mono = now
                return self._dt
            elapsed = now - self._last_cycle_mono
            self._last_cycle_mono = now
            if elapsed < self._dt:
                time.sleep(self._dt - elapsed)
            return max(elapsed, self._dt)

        if self._ctrl is None:
            # 无连接时模拟定时
            time.sleep(self._dt)
            return self._dt

        try:
            actual_dt = self._ctrl.waitPeriod(datetime.timedelta(seconds=self._dt))
            return float(actual_dt) if actual_dt is not None else self._dt
        except Exception as e:
            self._logger.warning(f"waitPeriod 失败 (使用 sleep 回退): {e}")
            time.sleep(self._dt)
            return self._dt

    # ── 安全 ──────────────────────────────────────────────────

    def emergency_stop(self) -> None:
        """UR 机械臂急停。

        立即将关节力矩置零并断开控制接口。
        物理急停按钮触发后还需按 Teach Pendant 的"解除停止"按钮。
        """
        super().emergency_stop()  # 设置标志位

        # 立即停伺服 / 发力矩为零
        n_joints = len(self._joint_names)
        try:
            if self._ctrl is not None and self._ctrl.isConnected():
                if self._servo_mode:
                    self._ctrl.servoStop()
                    self._logger.warning(f"[EMERGENCY STOP] {self._robot_name} servoJ 已停止")
                else:
                    self._ctrl.directTorque([0.0] * n_joints)
                    self._logger.warning(f"[EMERGENCY STOP] {self._robot_name} 力矩已置零")
        except Exception as e:
            self._logger.error(f"[EMERGENCY STOP] 置零失败: {e}")

    # ── 状态查询 ──────────────────────────────────────────────

    def is_connected(self) -> bool:
        """检查 RTDE 连接状态。"""
        if not self._connected:
            return False
        recv_ok = self._recv is not None and self._recv.isConnected()
        ctrl_ok = self._ctrl is not None and self._ctrl.isConnected()
        return recv_ok and ctrl_ok

    def is_enabled(self) -> bool:
        """检查 UR 机械臂是否处于可控制状态。

        远程控制模式 + 程序运行中 + 无急停。
        """
        if not self._connected or self._emergency_stopped:
            return False
        try:
            return self._ctrl is not None and self._ctrl.isProgramRunning()
        except Exception:
            return False

    def get_error_state(self) -> int:
        """获取 UR 机械臂安全状态。

        以 RTDE ``safety_mode`` 字段为主判据 — 跨 CB3 classic / e-Series 固件
        语义最一致的权威字段:
          1 正常 / 2 降级 / 4 恢复中 / 11-14  → 0 (可继续控制)
          3 保护性停止 / 9 剧烈碰撞           → 2
          5 安全防护停止                      → 3
          6/7/8 各级急停                      → 1
          10 故障                             → 4

        之前用 ``isEmergencyStopped()/isProtectiveStopped()`` (按 ur_rtde 的
        SafetyStatus 位布局测试 safety_status_bits: bit7=急停, bit2=保护性停止),
        在 classic CB3 上 safety_status_bits 位义可能与 e-Series 不同, 出现过
        "safety_mode=1 正常 但误判急停" 的情况; 故 safety_status_bits 仅作兜底.

        :returns:
            0 = 正常
            1 = 急停触发
            2 = 保护性停止
            3 = 安全模式警告
            4 = 安全模式错误
        """
        if not self._connected:
            return 1  # 未连接视为错误

        # ── 主判据: safety_mode ──
        try:
            mode = int(self._recv.getSafetyMode())
        except Exception:
            mode = None

        if mode is not None:
            if mode in (6, 7, 8):   # 系统急停 / 机器人急停 / 急停
                return 1
            if mode in (3, 9):      # 保护性停止 / 剧烈碰撞
                return 2
            if mode == 5:           # 安全防护停止
                return 3
            if mode == 10:          # 故障
                return 4
            return 0                # 1 正常 / 2 降级 / 4 恢复中 / 11-14 → 可继续

        # ── 兜底: safety_status_bits (仅当 safety_mode 读取失败时) ──
        # 位序参考 ur_rtde SafetyStatus 枚举 (e-Series 布局).
        try:
            safety_bits = self._recv.getSafetyStatusBits()
            if safety_bits & (1 << 7):   # 急停
                return 1
            if safety_bits & (1 << 2):   # 保护性停止
                return 2
            if safety_bits & (1 << 4):   # 安全防护停止
                return 3
            if safety_bits & (1 << 9):   # 故障
                return 4
        except Exception:
            pass

        return 0

    def get_safety_status_bits(self) -> Optional[int]:
        """读取 RTDE ``safety_status_bits`` 原始值 (诊断用).

        :returns: 原始位值 (uint32); 未连接或读取失败返回 None
        """
        if not self._connected or self._recv is None:
            return None
        try:
            return int(self._recv.getSafetyStatusBits())
        except Exception:
            return None

    def get_safety_mode(self) -> Optional[Tuple[int, str]]:
        """读取 UR 安全模式 (RTDE `safety_mode`) — 比 safety bits 更精确。

        典型值: 3=保护性停止, 5=安全防护停止, 9=剧烈碰撞, 6/7/8=各级急停,
        10=故障。用于精确定位"刚发力矩就停"的原因。

        :returns: (mode, 中文标签); 读取失败或未连接返回 None
        """
        if not self._connected or self._recv is None:
            return None
        try:
            mode = int(self._recv.getSafetyMode())
            return mode, _SAFETY_MODE_LABELS.get(mode, '未知')
        except Exception:
            return None

    # ── 配置 ──────────────────────────────────────────────────

    def set_torque_limits(self, limits: np.ndarray) -> None:
        """设置关节力矩限幅。"""
        self._torque_limits = np.asarray(limits, dtype=float).ravel()

    def get_joint_names(self) -> List[str]:
        """返回 UR 机械臂关节名称列表（与 URDF 对齐）。"""
        return self._joint_names.copy()


# ================================================================
# 自检
# ================================================================

if __name__ == "__main__":
    import sys

    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s.%(msecs)03d] %(message)s",
        datefmt="%H:%M:%S",
    )

    ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.100"
    robot_name = sys.argv[2] if len(sys.argv) > 2 else "UR"

    torque_limits = np.array([100.0] * 6)
    joint_names = [
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
    ]

    print("=" * 60)
    print(f"URHW 自检 — 连接 {robot_name} @ {ip}")
    print("=" * 60)
    print(f"\n⚠️  请确保 UR 已开机且教示器处于远程控制模式 (Remote Control)")
    print(f"   IP: {ip}")
    print(f"   Robot: {robot_name}")

    try:
        with URHW(ip=ip, torque_limits=torque_limits,
                   joint_names=joint_names, robot_name=robot_name) as robot:
            q, dq = robot.get_joint_states()
            print(f"\n  关节位置: {np.round(q, 4)}")
            print(f"  关节速度: {np.round(dq, 4)}")
            print(f"  连接状态: {robot.is_connected()}")
            print(f"  使能状态: {robot.is_enabled()}")
            print(f"  错误状态: {robot.get_error_state()}")
            print(f"\n  发送零力矩 ...")
            robot.set_joint_torques(np.zeros(len(joint_names)))
            print(f"  零力矩发送成功 ✅")

    except KeyboardInterrupt:
        print("\n\n  用户终止")
    except Exception as e:
        print(f"\n  错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

    print("\n自检完成")
