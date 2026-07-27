# -*- coding: utf-8 -*-
"""
RobotHWInterface — Hardware Abstraction Layer (HAL) for SE(3) Control
=====================================================================

定位:
  硬件接口抽象基类。所有真实机械臂的驱动都继承此类，
  上层 SE(3) 控制律（GIC/GUFIC）通过此接口与控制循环交互，
  完全不感知底层用的是 ur_rtde、libfranka 还是其他驱动。

设计原则:
  - 薄层原则: 具体实现（UR12eHW / FrankaHW）控制在 100–200 行
  - 机器人无关: 上层代码禁止引用任何具体驱动库
  - 生命周期安全: 支持 with 语句，异常退出时自动断开
  - 类型安全: numpy ndarray + typing 类型标注
  - 容错: 通信超时抛出 HardwareTimeoutError

对标 MuJoCo:
  仿真中通过 mj_step1/step2 推进物理、data.qpos/qvel 读状态、
  data.ctrl 发力矩。此接口提供相同语义的实机版本。

用法示例:
  ```python
  # 方式一: 上下文管理器（推荐）
  with UR12eHW(ip="192.168.1.100") as robot:
      q, dq = robot.get_joint_states()
      robot.set_joint_torques(np.zeros(6))

  # 方式二: 手动管理生命周期
  robot = UR12eHW(ip="192.168.1.100")
  robot.initialize()
  try:
      q, dq = robot.get_joint_states()
  finally:
      robot.shutdown()
  ```

参考文档: docs/plan.md
首个部署目标: UR12e（详见 ur12e_hw.py）
"""

from __future__ import annotations

import abc
import logging
from typing import List, Optional, Tuple

import numpy as np


# ================================================================
# 自定义异常
# ================================================================

class HardwareError(Exception):
    """硬件接口基类异常。"""
    pass


class HardwareConnectionError(HardwareError):
    """连接失败或断开。"""
    pass


class HardwareTimeoutError(HardwareError):
    """通信超时。"""
    pass


class HardwareSafetyError(HardwareError):
    """安全相关（限位、急停等）。"""
    pass


# ================================================================
# RobotHWInterface — 抽象基类
# ================================================================

class RobotHWInterface(abc.ABC):
    """硬件接口抽象基类。

    所有真实机械臂的驱动都继承此类，上层 SE(3) 控制律通过此接口
    与控制循环交互，不感知具体机器人。

    子类必须实现全部 @abc.abstractmethod 方法。
    子类应调用 super().__init__() 以初始化日志和安全状态。
    """

    def __init__(self, name: str = "generic_hardware", logger: Optional[logging.Logger] = None):
        """

        :param name:   机器人名称（用于日志标识）
        :param logger: 外部 logger 实例，None 则自动创建
        """
        self._name = name
        self._logger = logger or logging.getLogger(f"RobotHW.{name}")
        self._connected = False
        self._enabled = False
        self._emergency_stopped = False
        self._torque_limits: Optional[np.ndarray] = None
        self._error_state: int = 0

    # ──────────────────────────────────────────────────────────
    # 属性
    # ──────────────────────────────────────────────────────────

    @property
    def name(self) -> str:
        """机器人名称。"""
        return self._name

    @property
    def logger(self) -> logging.Logger:
        """本接口使用的 logger 实例。"""
        return self._logger

    # ──────────────────────────────────────────────────────────
    # 生命周期
    # ──────────────────────────────────────────────────────────

    @abc.abstractmethod
    def initialize(self) -> None:
        """初始化硬件连接。

        职责:
          1. 建立与机器人的通信连接（TCP/RTDE/FCI）
          2. 配置控制模式（力矩前馈/纯力矩）
          3. 读取初始关节状态（自检）
          4. 使能控制模式

        幂等: 可多次调用，第二次调用时自动先 shutdown() 再重连。

        :raises HardwareConnectionError: 连接失败
        """
        ...

    @abc.abstractmethod
    def shutdown(self) -> None:
        """安全断开硬件连接。

        职责:
          1. 将力矩指令置零
          2. 退出控制模式
          3. 释放关节制动（如适用）
          4. 关闭通信连接

        幂等: 可多次调用，重复调用不报错。
        """
        ...

    # ──────────────────────────────────────────────────────────
    # 状态读取
    # ──────────────────────────────────────────────────────────

    @abc.abstractmethod
    def get_joint_states(self) -> Tuple[np.ndarray, np.ndarray]:
        """获取当前关节状态。

        此方法会被控制循环以 250–1000 Hz 高频调用，实现必须轻量:
          - 不分配大对象
          - 不写日志
          - 推荐缓存上一帧结果，通信失败时返回缓存

        对标 MuJoCo: ``data.qpos``, ``data.qvel``

        :returns:
            q:  ndarray (nv,) — 关节位置 (rad)
            dq: ndarray (nv,) — 关节速度 (rad/s)

        :raises HardwareTimeoutError:   通信超时
        :raises HardwareConnectionError: 连接断开
        """
        ...

    @abc.abstractmethod
    def get_ft_sensor(self) -> np.ndarray:
        """获取末端力/力矩传感器读数。

        :returns: ndarray (6,) — [fx, fy, fz, tx, ty, tz] (N, Nm)

        注意:
          - GIC-only 模式下返回零向量（不需要传感器）
          - GUFIC 模式下需要真实传感器数据
          - Franka 可基于关节扭矩估计末端力
          - UR12e 需要外部 FT 传感器，无传感器时返回零向量
        """
        ...

    # ──────────────────────────────────────────────────────────
    # 执行
    # ──────────────────────────────────────────────────────────

    @abc.abstractmethod
    def set_joint_torques(self, tau: np.ndarray) -> None:
        """下发关节力矩指令。

        对标 MuJoCo: ``data.ctrl``

        :param tau: ndarray (nv,) — 期望关节力矩 (Nm)

        安全约束（实现层必须执行）:
          1. 力矩限幅: 每个关节力矩不得超过 ``torque_limits``
          2. 急停检查: 如果 ``emergency_stopped`` 为 True，禁止发力矩
          3. 通信检查: 连接断开时抛出 HardwareConnectionError

        :raises HardwareSafetyError:     急停状态下尝试发力矩
        :raises HardwareConnectionError: 连接断开
        :raises HardwareTimeoutError:    通信超时
        """
        ...

    # ──────────────────────────────────────────────────────────
    # 定时
    # ──────────────────────────────────────────────────────────

    @abc.abstractmethod
    def get_timestep(self) -> float:
        """获取标称控制周期。

        :returns: 控制周期 (秒)

        举例:
          UR12e (ur_rtde RTDE):  250–500 Hz → 0.002–0.004 s
          Franka (libfranka):    1000 Hz     → 0.001 s
        """
        ...

    @abc.abstractmethod
    def wait_next_cycle(self) -> float:
        """等待下一个控制周期。

        控制循环通过此方法获取实际经过的时间，不假定严格固定步长。

        :returns: 自上次调用以来实际经过的时间 (秒)
        """
        ...

    # ──────────────────────────────────────────────────────────
    # 安全
    # ──────────────────────────────────────────────────────────

    def emergency_stop(self) -> None:
        """触发急停。

        行为:
          1. 立即将所有关节力矩置零
          2. 退出控制模式
          3. 设置内部急停标志位（此后 ``set_joint_torques`` 不生效）
          4. 记录日志

        恢复: 调用 ``reset_emergency_stop()`` 清除标志位。
        """
        self._emergency_stopped = True
        self._logger.warning("[EMERGENCY STOP] 触发急停 — 力矩已置零")

    def reset_emergency_stop(self) -> None:
        """重置急停状态，允许继续发力矩。

        注意: 调用此方法前务必确认物理急停按钮已释放，
        且机器人处于安全状态。
        """
        self._emergency_stopped = False
        self._logger.info("[EMERGENCY STOP] 已重置 — 可继续发力矩")

    @property
    def is_emergency_stopped(self) -> bool:
        """检查急停是否被触发。"""
        return self._emergency_stopped

    # ──────────────────────────────────────────────────────────
    # 状态查询
    # ──────────────────────────────────────────────────────────

    def is_connected(self) -> bool:
        """检查与机器人的通信连接是否正常。"""
        return self._connected

    def is_enabled(self) -> bool:
        """检查控制模式是否激活（力矩模式已使能）。"""
        return self._enabled

    def get_error_state(self) -> int:
        """获取机器人错误状态码。

        :returns:
            0 = 无错误
            >0 = 错误码（具体值由子类定义）
        """
        return self._error_state

    # ──────────────────────────────────────────────────────────
    # 配置
    # ──────────────────────────────────────────────────────────

    @abc.abstractmethod
    def set_torque_limits(self, limits: np.ndarray) -> None:
        """设置关节力矩限幅值。

        ``set_joint_torques`` 中会执行::
            tau_clipped = np.clip(tau, -limits, limits)

        :param limits: ndarray (nv,) — 每个关节的最大力矩 (Nm)
        """
        ...

    def get_torque_limits(self) -> Optional[np.ndarray]:
        """获取当前关节力矩限幅值。

        :returns: ndarray (nv,) 或 None（未设置限幅）
        """
        return self._torque_limits

    @abc.abstractmethod
    def get_joint_names(self) -> List[str]:
        """获取关节名称列表。

        :returns: [str, ...] — 长度为 nv 的关节名列表，顺序与 URDF 对齐

        用途:
          验证 URDF 中的关节顺序与硬件接口的关节顺序一致。
        """
        ...

    # ──────────────────────────────────────────────────────────
    # 上下文管理器（自动生命周期管理）
    # ──────────────────────────────────────────────────────────

    def __enter__(self) -> "RobotHWInterface":
        """进入上下文时自动初始化。"""
        self.initialize()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """退出上下文时自动断开。"""
        if self._connected:
            self.shutdown()


# ================================================================
# 辅助工具函数
# ================================================================

def clip_torques(tau: np.ndarray, limits: Optional[np.ndarray]) -> np.ndarray:
    """对关节力矩执行对称限幅。

    :param tau:    ndarray (nv,) — 原始力矩指令
    :param limits: ndarray (nv,) — 每关节最大绝对值，None 时不限幅
    :returns:      限幅后的 ndarray (nv,)
    """
    if limits is None:
        return tau
    return np.clip(tau, -limits, limits)


# ================================================================
# 快速验证
# ================================================================

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    print("=" * 60)
    print("RobotHWInterface 接口验证")
    print("=" * 60)

    # 验证抽象类不能直接实例化
    try:
        _ = RobotHWInterface("test")
        print("❌ 抽象类不应能直接实例化")
    except TypeError as e:
        print(f"✅ 抽象类不能实例化: {e}")

    # 验证子类必须实现所有抽象方法
    class IncompleteHW(RobotHWInterface):
        pass

    try:
        _ = IncompleteHW("incomplete")
        print("❌ 未实现抽象方法的子类不应能实例化")
    except TypeError as e:
        print(f"✅ 未实现全部抽象方法的子类不能实例化: {e}")

    print("\n[RobotHWInterface] 接口定义验证通过 ✅")
