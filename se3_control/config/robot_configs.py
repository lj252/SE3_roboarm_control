"""
Robot-specific configuration for SE(3) control.
================================================

所有机械臂相关的参数集中在此文件管理。
脚本通过 ``--robot`` 参数自动加载对应配置。

用法:
  from config.robot_configs import get_robot_config, get_urdf_path, get_hw_class

  cfg = get_robot_config('ur12e')
  print(cfg['default_ip'])
"""

import os
import importlib
import numpy as np
from typing import Any, Dict, List, Optional

# ================================================================
# 机械臂参数配置
# ================================================================

ROBOT_CONFIGS: Dict[str, Dict[str, Any]] = {

    # ────────────────────────────────────────────────────────────
    # UR12e
    # ────────────────────────────────────────────────────────────
    'ur12e': {
        'name':                'UR12e',

        # 硬件类 (用于 get_hw_class 动态导入)
        'hw_class':            'UR12eHW',
        'hw_module':           'hardware.ur12e_hw',

        # URDF
        'urdf':                'ur12e.urdf',
        'ee_frame':            'tool0',

        # 网络
        'default_ip':          '192.168.1.100',

        # 关节力矩安全限幅 (Nm) — URDF effort 的 50%
        'torque_limits':       np.array([
            165.0,   # shoulder_pan: 330 * 0.5
            165.0,   # shoulder_lift: 330 * 0.5
            75.0,    # elbow:        150 * 0.5
            27.0,    # wrist_1:       54 * 0.5
            27.0,    # wrist_2:       54 * 0.5
            27.0,    # wrist_3:       54 * 0.5
        ]),

        # 完整 URDF effort 上限 (用于 GICController 安全限幅)
        'full_torque_limits':  np.array([
            330.0,   # shoulder_pan
            330.0,   # shoulder_lift
            150.0,   # elbow
            54.0,    # wrist_1
            54.0,    # wrist_2
            54.0,    # wrist_3
        ]),

        # 关节名称 (与 URDF 对齐)
        'joint_names': [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ],

        # MuJoCo 仿真 — 舒适默认位: EE 在 [0.50, 0, 0.50] (m), 末端竖直朝下
        # (工具 z 轴 = [0,0,-1], 倾角 0°). q5 = +90° 处于腕部条件最佳区域
        # (腕部奇异在 q5 = 0/±180°, 即 wrist_1∥wrist_3).
        # 调节任务期望位姿 = FK(home_q), 故解耦/扫频实验的工作位姿也随之降低.
        'home_q':              np.array([-0.356, -1.498, 1.81, 1.259, 1.571, -0.124]),

        # 网格可视化
        'mesh_subdir':         'UR12e/',   # urdf/meshes/UR12e/
        'link_to_mesh': {
            'base_link_inertia': 'base_vis',
            'shoulder_link':     'shoulder_vis',
            'upper_arm_link':    'upperarm_vis',
            'forearm_link':      'forearm_vis',
            'wrist_1_link':      'wrist1_vis',
            'wrist_2_link':      'wrist2_vis',
            'wrist_3_link':      'wrist3_vis',
        },
    },

    # ────────────────────────────────────────────────────────────
    # UR3
    # ────────────────────────────────────────────────────────────
    'ur3': {
        'name':                'UR3',

        'hw_class':            'UR3HW',
        'hw_module':           'hardware.ur3_hw',

        'urdf':                'ur3.urdf',
        'ee_frame':            'tool0',

        'default_ip':          '192.168.1.101',   # 默认可通过 --ip 覆盖

        # 关节力矩安全限幅 (Nm) — URDF effort 的 50%
        'torque_limits':       np.array([
            28.0,    # shoulder_pan: 56 * 0.5
            28.0,    # shoulder_lift: 56 * 0.5
            14.0,    # elbow:        28 * 0.5
            6.0,     # wrist_1:      12 * 0.5
            6.0,     # wrist_2:      12 * 0.5
            6.0,     # wrist_3:      12 * 0.5
        ]),

        # 完整 URDF effort 上限
        'full_torque_limits':  np.array([
            56.0,    # shoulder_pan
            56.0,    # shoulder_lift
            28.0,    # elbow
            12.0,    # wrist_1
            12.0,    # wrist_2
            12.0,    # wrist_3
        ]),

        # 关节名称 (与 UR12e 一致)
        'joint_names': [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ],

        # 舒适默认位: EE 在 [0.35, 0, 0.35] (m), 末端竖直朝下 (倾角 0°).
        # q5 = -90° 处于腕部条件最佳区域 (奇异在 q5 = 0/±180°) (UR3 工作空间小).
        'home_q':              np.array([-0.327, -1.42, 1.236, -1.386, -1.571, 2.738]),

        'mesh_subdir':         'UR3/',     # urdf/meshes/UR3/
        'link_to_mesh': {
            'base_link_inertia': 'base',       # 碰撞网格, 无 _vis 后缀
            'shoulder_link':     'shoulder',
            'upper_arm_link':    'upperarm',
            'forearm_link':      'forearm',
            'wrist_1_link':      'wrist1',
            'wrist_2_link':      'wrist2',
            'wrist_3_link':      'wrist3',
        },

        # 任务参数注释: UR3 工作空间小, 运行 circle/line 等任务时
        # 建议修改 task_config.py 中的圆心/半径/增益等参数。
        # 参考 task_config.py 中各 section 的"小机械臂(UR3)推荐"注释。
        # 'task_params':  (已移除, 统一在 task_config.py 中管理)
    },
}


# ================================================================
# 路径工具
# ================================================================

# 项目根目录 (se3_control/)
_PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF_DIR = os.path.join(_PROJECT_DIR, 'urdf')
MESHES_DIR = os.path.join(URDF_DIR, 'meshes')


def get_robot_config(name: str) -> Dict[str, Any]:
    """返回机器人配置字典的副本。

    :param name: 机器人类型 ('ur12e', 'ur3')
    :raises KeyError: 未知机器人类型
    """
    if name not in ROBOT_CONFIGS:
        raise KeyError(
            f"未知机器人类型 '{name}'。可用选项: {list(ROBOT_CONFIGS.keys())}"
        )
    return dict(ROBOT_CONFIGS[name])  # 返回副本防止意外修改


def get_urdf_path(name: str) -> str:
    """返回机器人 URDF 文件的绝对路径。

    :param name: 机器人类型
    :returns: URDF 文件的绝对路径
    """
    cfg = get_robot_config(name)
    return os.path.join(URDF_DIR, cfg['urdf'])


def get_mesh_dir(name: str) -> str:
    """返回机器人网格文件目录的绝对路径。

    :param name: 机器人类型
    :returns: 网格目录的绝对路径 (含尾部斜杠)
    """
    cfg = get_robot_config(name)
    subdir = cfg.get('mesh_subdir', '')
    return os.path.normpath(os.path.join(MESHES_DIR, subdir)) + '/'


def get_hw_class(name: str):
    """动态导入并返回机器人硬件类。

    使用 ``importlib.import_module`` 延迟加载，
    避免循环导入 (config → hardware → config)。

    :param name: 机器人类型
    :returns: 硬件类 (UR12eHW / UR3HW)
    """
    cfg = get_robot_config(name)
    mod = importlib.import_module(cfg['hw_module'])
    return getattr(mod, cfg['hw_class'])


# ================================================================
# argparse 工具
# ================================================================


def add_robot_arg(parser, default: str = 'ur12e'):
    """向 argparse 解析器添加 ``--robot`` 参数。

    :param parser: argparse.ArgumentParser 实例
    :param default: 默认机器人类型
    """
    parser.add_argument(
        '--robot', type=str, default=default,
        choices=list(ROBOT_CONFIGS.keys()),
        help=f"机器人类型 (默认: {default})",
    )


# ================================================================
# 自检
# ================================================================

if __name__ == '__main__':
    for name in ROBOT_CONFIGS:
        cfg = get_robot_config(name)
        print(f"[{name}]")
        print(f"  URDF:   {get_urdf_path(name)}")
        print(f"  Mesh:   {get_mesh_dir(name)}")
        print(f"  Home q: {cfg['home_q']}")
        print(f"  Limits: {cfg['torque_limits']}")
        print(f"  HW:     {cfg['hw_module']}.{cfg['hw_class']}")
        print()
