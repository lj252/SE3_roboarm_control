"""
UR3HW — UR3 具体硬件实现
==========================

基于通用 URHW 类的 UR3 专用封装。

用法:
  from se3_control.hardware.ur3_hw import UR3HW

  with UR3HW(ip="192.168.1.101") as robot:
      q, dq = robot.get_joint_states()
"""

from __future__ import annotations

import logging
import time
from typing import Optional

import numpy as np

from .ur_hw import URHW

# ================================================================
# UR3 默认参数
# ================================================================

# 关节力矩安全限幅 (Nm) — URDF 中 limit effort 的 50%
_UR3_TORQUE_LIMITS = np.array([
    28.0,    # shoulder_pan: 56 * 0.5
    28.0,    # shoulder_lift: 56 * 0.5
    14.0,    # elbow:        28 * 0.5
    6.0,     # wrist_1:      12 * 0.5
    6.0,     # wrist_2:      12 * 0.5
    6.0,     # wrist_3:      12 * 0.5
])

# 关节名称（与 URDF 对齐，与 UR12e 一致）
_UR3_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# 标称控制周期 (秒) — 250 Hz
_DEFAULT_DT = 0.004

# 默认连接超时 (秒)
_DEFAULT_TIMEOUT = 5.0


# ================================================================
# UR3HW
# ================================================================

class UR3HW(URHW):
    """UR3 硬件接口 (URHW 的 UR3 封装)。

    所有 RTDE 逻辑继承自 URHW 类。
    此处仅指定 UR3 特有的力矩限幅和关节名称。

    :param str ip:            UR3 控制箱 IP 地址
    :param float dt:          标称控制周期 (秒), 默认 0.004 (250 Hz)
    :param float timeout:     连接超时 (秒), 默认 5.0
    :param bool verbose:      是否打印详细日志
    :param logger:            外部 logger 实例 (None 则自动创建)
    """

    def __init__(
        self,
        ip: str = "192.168.1.101",
        dt: float = _DEFAULT_DT,
        timeout: float = _DEFAULT_TIMEOUT,
        verbose: bool = True,
        logger: Optional[logging.Logger] = None,
    ):
        super().__init__(
            ip=ip,
            torque_limits=_UR3_TORQUE_LIMITS.copy(),
            joint_names=_UR3_JOINT_NAMES.copy(),
            robot_name="UR3",
            dt=dt,
            timeout=timeout,
            verbose=verbose,
            logger=logger,
        )


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

    ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.101"

    print("=" * 60)
    print(f"UR3HW 自检 — 连接 {ip}")
    print("=" * 60)
    print(f"\n⚠️  请确保 UR3 已开机且教示器处于远程控制模式 (Remote Control)")
    print(f"   IP: {ip}")
    print(f"   按 Ctrl+C 终止\n")

    try:
        with UR3HW(ip=ip, verbose=True) as robot:
            q, dq = robot.get_joint_states()
            print(f"\n  关节位置: {np.round(q, 4)}")
            print(f"  关节速度: {np.round(dq, 4)}")
            print(f"  连接状态: {robot.is_connected()}")
            print(f"  使能状态: {robot.is_enabled()}")
            print(f"  错误状态: {robot.get_error_state()}")

            print(f"\n  发送零力矩 ...")
            robot.set_joint_torques(np.zeros(6))
            print(f"  零力矩发送成功 ✅")

            limits = robot.get_torque_limits()
            print(f"\n  力矩限幅: {limits}")

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n\n  用户终止")
    except Exception as e:
        print(f"\n  错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

    print("\n自检完成")
