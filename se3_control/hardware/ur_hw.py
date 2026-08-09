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

        # 先发零力矩 (安全)
        n_joints = len(self._joint_names)
        try:
            if self._ctrl is not None and self._ctrl.isConnected():
                self._ctrl.directTorque([0.0] * n_joints)
        except Exception as e:
            self._logger.warning(f"力矩置零失败: {e}")

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

        # 立即发力矩为零
        n_joints = len(self._joint_names)
        try:
            if self._ctrl is not None and self._ctrl.isConnected():
                self._ctrl.directTorque([0.0] * n_joints)
                self._logger.warning(f"[EMERGENCY STOP] {self._robot_name} 力矩已置零")
        except Exception as e:
            self._logger.error(f"[EMERGENCY STOP] 力矩置零失败: {e}")

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

        :returns:
            0 = 正常
            1 = 急停触发
            2 = 保护性停止
            3 = 安全模式警告
            4 = 安全模式错误
        """
        if not self._connected:
            return 1  # 未连接视为错误

        try:
            if self._recv is not None and self._recv.isEmergencyStopped():
                return 1
        except Exception:
            pass

        try:
            if self._recv is not None and self._recv.isProtectiveStopped():
                return 2
        except Exception:
            pass

        try:
            if self._recv is not None:
                safety_bits = self._recv.getSafetyStatusBits()
                if safety_bits & 0b11:   # 位0/1: 安全模式
                    return 3
                if safety_bits & 0b1100: # 位2/3: 严重安全错误
                    return 4
        except Exception:
            pass

        return 0

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
