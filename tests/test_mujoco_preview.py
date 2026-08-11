#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tests/test_mujoco_preview.py — MuJoCo 闭环预览与碰撞判定测试
=============================================================

覆盖范围
--------
  - check_simulated_collisions (纯 FK, 无 MuJoCo):
      * home_q 恒定 → 判定安全 (不误报; home 肘部中点距基座轴仅 1.8cm,
        但 z=27cm 高于柱顶, 高度条件正确放行)
      * 臂向基部折叠 → 判定基座碰撞风险
      * 臂向下够近地 → 判定地面碰撞风险 (wrist z < 0.11m)
  - run_preview (完整 MuJoCo 闭环, 标 @pytest.mark.simulation):
      * directTorque / servoJ 两种控制模式 headless 跑 circle → 返回 verdict
      * 低圆心 (z=0.08) → 地面碰撞判定触发

运行:
    conda activate roboarm
    cd /media/lj252/Data/catkin_ws/roboarm_test/SE3_roboarm_control
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest tests/test_mujoco_preview.py -v
"""

import os
import sys
import unittest

import numpy as np
import pytest

# 添加项目根 (tests/ 的上一级)
_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from se3_control.config.robot_configs import get_robot_config, get_urdf_path
from se3_control.core.mujoco_preview import (check_simulated_collisions,
                                             run_preview)
from se3_control.core.trajectory import build_trajectory
from se3_control.robot_model.robot_model import RobotModel


# ====================================================================
# 共享 UR3 模型 / 配置 (模块级缓存)
# ====================================================================

_ROBOT = None
_CFG = None
_TASK_CFG = None


def _robot():
    global _ROBOT
    if _ROBOT is None:
        cfg = _config()
        _ROBOT = RobotModel(get_urdf_path('ur3'), ee_frame_name=cfg['ee_frame'],
                            robot_name='ur3', verbose=False)
    return _ROBOT


def _config():
    global _CFG
    if _CFG is None:
        _CFG = get_robot_config('ur3')
    return _CFG


def _task_config():
    global _TASK_CFG
    if _TASK_CFG is None:
        from se3_control.config import task_config
        _TASK_CFG = task_config.get_task_config('ur3')
    return _TASK_CFG


# 碰撞单测用的静态位形 (已用 FK 验证)
_Q_HOME = None
_Q_FOLD = None      # 臂向基部折叠: wrist3 距基座柱 12cm, z 低
_Q_REACH_DOWN = None  # 臂向下够: tool0/wrist3 z ≈ 0.065 m (低于地面阈值 0.11)


def _poses():
    global _Q_HOME, _Q_FOLD, _Q_REACH_DOWN
    if _Q_HOME is None:
        cfg = _config()
        _Q_HOME = np.asarray(cfg['home_q'], dtype=float)
        _Q_FOLD = _Q_HOME.copy(); _Q_FOLD[1] = -0.6; _Q_FOLD[2] = 2.4
        _Q_REACH_DOWN = np.array([-0.327, 0.3, -0.5, 0.5, -1.571, 2.738],
                                 dtype=float)
    return _Q_HOME, _Q_FOLD, _Q_REACH_DOWN


# ====================================================================
# 1. 碰撞判定 (纯 FK, 无 MuJoCo)
# ====================================================================

class TestCollisionCheck(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = _robot()

    def test_home_q_safe(self):
        """home_q 恒定 → 判定安全 (高度条件: 肘部虽距基座轴近但高于柱顶)."""
        q_home, _, _ = _poses()
        qt = np.tile(q_home, (20, 1))
        v = check_simulated_collisions(self.robot, qt)
        self.assertTrue(v['ok'], "home_q 不应误报碰撞")
        self.assertGreater(v['min_base_d'], 0.0)
        self.assertGreater(v['min_z'], 0.15)

    def test_folded_pose_flags_base(self):
        """臂向基部折叠 → 基座碰撞风险 (抓用户遇到的肘部撞基座场景)."""
        _, q_fold, _ = _poses()
        qt = np.tile(q_fold, (10, 1))
        v = check_simulated_collisions(self.robot, qt)
        self.assertFalse(v['ok'])
        self.assertEqual(v['first_violation'][2], 'base')
        self.assertLess(v['min_base_d'], 0.13)

    def test_reach_down_flags_floor(self):
        """臂向下够近地 (wrist z≈0.065) → 地面碰撞风险."""
        _, _, q_down = _poses()
        qt = np.tile(q_down, (10, 1))
        v = check_simulated_collisions(self.robot, qt)
        self.assertFalse(v['ok'])
        self.assertLess(v['min_z'], 0.11)

    def test_calibrated_fk_matches_rtde(self):
        """基座校准回归: FK(tool0, home_q) == RTDE 实测 TCP (-0.350, 0.000, 0.224) (±5mm).

        校准 = shoulder_pan 基座 yaw180° + flange-tool0 沿 tool0 +z 偏移 0.126 m
        (见 ur3.urdf 注释); 模型坐标系现在与实机一致.
        """
        q_home, _, _ = _poses()
        self.robot.update(q_home, np.zeros(6))
        p, _ = self.robot.get_frame_pose('tool0')
        np.testing.assert_allclose(p, [-0.350, 0.000, 0.224], atol=0.005)

    @pytest.mark.simulation
    def test_calibrated_mujoco_site_matches_fk(self):
        """MuJoCo builder 的 end_effector site == Pinocchio tool0 (末端固定偏移 0.126 生效)."""
        import mujoco
        import tempfile
        from scripts.verify_gic_mujoco import urdf_joints_to_mujoco_xml

        cfg = _config()
        q_home, _, _ = _poses()
        self.robot.update(q_home, np.zeros(6))
        p_pin, R_pin = self.robot.get_frame_pose('tool0')

        xml = urdf_joints_to_mujoco_xml(get_urdf_path('ur3'), cfg['ee_frame'],
                                        link_to_mesh=cfg['link_to_mesh'],
                                        mesh_subdir=cfg['mesh_subdir'], debug=False)
        tmp = tempfile.NamedTemporaryFile(suffix='.xml', mode='w', delete=False)
        try:
            tmp.write(xml)
            tmp.close()
            m = mujoco.MjModel.from_xml_path(tmp.name)
            d = mujoco.MjData(m)
            d.qpos[:m.nv] = q_home
            mujoco.mj_forward(m, d)
        finally:
            os.unlink(tmp.name)

        p_site = d.site_xpos[0].copy()
        R_site = d.site_xmat[0].copy().reshape(3, 3)
        np.testing.assert_allclose(p_site, p_pin, atol=1e-6)
        np.testing.assert_allclose(R_site, R_pin, atol=1e-6)


# ====================================================================
# 2. MuJoCo 闭环预览 (headless, 较慢 → @pytest.mark.simulation)
# ====================================================================

class TestPreviewSim(unittest.TestCase):
    @pytest.mark.simulation
    def test_circle_direct_torque_preview(self):
        """directTorque headless 跑 circle (默认圆心) → 无碰撞."""
        cfg = _config()
        tc = _task_config()
        traj = build_trajectory('circle', cfg=tc)
        res = run_preview('ur3', get_urdf_path('ur3'), cfg['ee_frame'],
                          cfg['home_q'], traj, task_cfg=tc, bandwidth=10.0,
                          damping=1.0, torque_limits=cfg['torque_limits'],
                          duration=1.2, ctrl_dt=0.004, blend_time=0.3,
                          control_mode='directTorque', show_viewer=False,
                          link_to_mesh=cfg['link_to_mesh'],
                          mesh_subdir=cfg['mesh_subdir'],
                          logger=_quiet_logger())
        v = res['verdict']
        self.assertIn('ok', v)
        self.assertTrue(v['ok'], "默认圆心应判定无碰撞")
        self.assertEqual(res['q'].shape[1], cfg['home_q'].size)
        self.assertGreater(res['q'].shape[0], 100)

    @pytest.mark.simulation
    def test_circle_servoj_preview(self):
        """servoJ (桥 + 计算力矩内层伺服) headless 跑 circle → 稳定画圆.

        回归: 内层裸 PD (无重力补偿/前馈) 在 circle 上发散成混乱轨迹
        (末段 pos_err ~0.3m, 圆半径 std ~0.08). 计算力矩伺服应给出
        干净的圆 (末段 pos_err < 0.05m, 半径 std < 0.03).
        """
        cfg = _config()
        tc = _task_config()
        traj = build_trajectory('circle', cfg=tc)
        res = run_preview('ur3', get_urdf_path('ur3'), cfg['ee_frame'],
                          cfg['home_q'], traj, task_cfg=tc, bandwidth=10.0,
                          damping=1.0, torque_limits=cfg['torque_limits'],
                          duration=1.2, ctrl_dt=0.004, blend_time=0.3,
                          control_mode='servoJ', show_viewer=False,
                          link_to_mesh=cfg['link_to_mesh'],
                          mesh_subdir=cfg['mesh_subdir'],
                          logger=_quiet_logger())
        self.assertTrue(np.all(np.isfinite(res['q'])))
        half = res['pos_err'].shape[0] // 2
        # 末段 (混合结束) 位置误差有界 — 拦发散
        self.assertLess(res['pos_err'][half:].max(), 0.05)
        # 末段轨迹应近似圆心 r=0.06 的圆 — 拦混乱轨迹
        c = np.array(tc.circle['center'], dtype=float)
        rad = np.linalg.norm(res['p'][half:] - c, axis=1)
        self.assertLess(rad.std(), 0.03)
        self.assertGreater(rad.mean(), 0.03)
        self.assertLess(rad.mean(), 0.09)

    @pytest.mark.simulation
    def test_low_center_flags_floor(self):
        """低圆心 (z=0.08, 基座校准后坐标系) → 闭环仿真里 tool0 贴地, 地面碰撞判定触发."""
        cfg = _config()
        tc = _task_config()
        tc.circle['center'] = [-0.38, 0.0, 0.08]
        tc.circle['radius'] = 0.06
        traj = build_trajectory('circle', cfg=tc)
        res = run_preview('ur3', get_urdf_path('ur3'), cfg['ee_frame'],
                          cfg['home_q'], traj, task_cfg=tc, bandwidth=10.0,
                          damping=1.0, torque_limits=cfg['torque_limits'],
                          duration=1.5, ctrl_dt=0.004, blend_time=0.3,
                          control_mode='directTorque', show_viewer=False,
                          link_to_mesh=cfg['link_to_mesh'],
                          mesh_subdir=cfg['mesh_subdir'],
                          logger=_quiet_logger())
        v = res['verdict']
        self.assertFalse(v['ok'], "低圆心应判定碰撞风险")
        self.assertLess(v['min_z'], 0.11)

    @pytest.mark.simulation
    def test_folded_start_flags_base(self):
        """从折叠起始位形起步 (--preview-start-q 场景) → 起步即基座碰撞."""
        cfg = _config()
        tc = _task_config()
        traj = build_trajectory('circle', cfg=tc)
        _, q_fold, _ = _poses()
        res = run_preview('ur3', get_urdf_path('ur3'), cfg['ee_frame'],
                          cfg['home_q'], traj, task_cfg=tc, bandwidth=10.0,
                          damping=1.0, torque_limits=cfg['torque_limits'],
                          duration=1.0, ctrl_dt=0.004, blend_time=0.2,
                          control_mode='directTorque', show_viewer=False,
                          link_to_mesh=cfg['link_to_mesh'],
                          mesh_subdir=cfg['mesh_subdir'], start_q=q_fold,
                          logger=_quiet_logger())
        self.assertFalse(res['verdict']['ok'])


def _quiet_logger():
    logger = __import__('logging').getLogger('test_mujoco_preview_quiet')
    logger.disabled = True
    return logger


if __name__ == '__main__':
    unittest.main()
