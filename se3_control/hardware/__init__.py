# -*- coding: utf-8 -*-
"""
se3_control.hardware — Hardware Abstraction Layer (HAL)

包含:
  - RobotHWInterface: 抽象基类（所有机械臂驱动的公共接口）
  - UR12eHW:          UR12e 具体实现（基于 URHW 通用类）
  - UR3HW:            UR3 具体实现（基于 URHW 通用类）
  - URHW:             通用 UR 机械臂基类（直接使用参数配置）

使用方式:
  from se3_control.hardware.interface import RobotHWInterface
  from se3_control.hardware.ur12e_hw import UR12eHW
  from se3_control.hardware.ur3_hw import UR3HW
"""

from .interface import RobotHWInterface
from .ur12e_hw import UR12eHW
from .ur3_hw import UR3HW
from .ur_hw import URHW

__all__ = ["RobotHWInterface", "UR12eHW", "UR3HW", "URHW"]
