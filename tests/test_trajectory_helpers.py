#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tests/test_trajectory_helpers.py — 轨迹求值辅助单元测试

覆盖范围:
  - eval_body_twist:  轨迹 → 体坐标系期望速度/加速度 (GIC 控制器输入)
       · circle 前馈速度 = Rdᵀ·ṗ_d (bf=1)
       · Rd 恒定 → wd = 0
       · bf=0 → 前馈全零 (起步混合起点)
       · 静态轨迹 → 全零
  - make_static_traj: 恒定位姿 + 零速度/加速度的 regulation 轨迹

运行:
    conda activate roboarm
    cd /media/lj252/Data/catkin_ws/roboarm_test/SE3_roboarm_control
    python3 -m pytest tests/test_trajectory_helpers.py -v
"""

import os
import sys
import unittest

import numpy as np

# 添加项目路径
_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from se3_control.core.trajectory import (
    build_trajectory,
    eval_body_twist,
    TrajectoryFuncs,
)
from se3_control.core.se3_math import rotmat_x
from se3_control.config import task_config
from se3_control.scripts.run_se3_control import make_static_traj


class TestEvalBodyTwist(unittest.TestCase):
    """eval_body_twist: 轨迹 → 体坐标系期望速度/加速度."""

    def setUp(self):
        self.circle = build_trajectory('circle', cfg=task_config)

    def test_shape(self):
        """返回 4 个 (3,1) 列向量."""
        Rd = self.circle.Rd_t(0.0).reshape(3, 3)
        vd, wd, dvd, dwd = eval_body_twist(self.circle, 0.0, Rd, bf=1.0)
        for x in (vd, wd, dvd, dwd):
            self.assertEqual(x.shape, (3, 1))

    def test_vd_bf1(self):
        """bf=1: vd = Rdᵀ · ṗ_d."""
        t = 0.3
        Rd = self.circle.Rd_t(t).reshape(3, 3)
        vd, wd, dvd, dwd = eval_body_twist(self.circle, t, Rd, bf=1.0)
        dpd = self.circle.dpd_t(t).ravel()
        self.assertTrue(np.allclose(vd.ravel(), Rd.T @ dpd, atol=1e-9))

    def test_wd_zero_for_constant_orientation(self):
        """circle/line 朝向恒定 → 期望角速度 wd = 0."""
        for t in (0.0, 1.0, 2.5):
            Rd = self.circle.Rd_t(t).reshape(3, 3)
            _, wd, _, dwd = eval_body_twist(self.circle, t, Rd, bf=1.0)
            self.assertTrue(np.allclose(wd, 0, atol=1e-9),
                            f"t={t} wd={wd.ravel()}")
            self.assertTrue(np.allclose(dwd, 0, atol=1e-9))

    def test_bf0_zero_feedforward(self):
        """bf=0 (起步混合起点): 期望速度/加速度全零."""
        t = 1.0
        Rd = self.circle.Rd_t(t).reshape(3, 3)
        vd, wd, dvd, dwd = eval_body_twist(self.circle, t, Rd, bf=0.0)
        for x in (vd, wd, dvd, dwd):
            self.assertTrue(np.allclose(x, 0, atol=1e-12))

    def test_static_traj_zero(self):
        """静态轨迹 (恒定位姿) → 全零速度/加速度."""
        pd = np.array([0.3, 0.0, 0.4])
        Rd = rotmat_x(0.3)
        traj = TrajectoryFuncs(
            pd_t=lambda t: pd, Rd_t=lambda t: Rd,
            dpd_t=lambda t: np.zeros(3), dRd_t=lambda t: np.zeros((3, 3)),
            ddpd_t=lambda t: np.zeros(3), ddRd_t=lambda t: np.zeros((3, 3)),
        )
        vd, wd, dvd, dwd = eval_body_twist(traj, 5.0, Rd, bf=1.0)
        for x in (vd, wd, dvd, dwd):
            self.assertTrue(np.allclose(x, 0, atol=1e-12))


class TestMakeStaticTraj(unittest.TestCase):
    """make_static_traj: regulation 保持用静态轨迹."""

    def test_constant_pose_and_zero_vel(self):
        pd = np.array([0.35, 0.0, 0.30])
        Rd = rotmat_x(0.5)
        traj = make_static_traj(pd, Rd)
        for t in (0.0, 1.5, 3.0):
            self.assertTrue(np.allclose(traj.pd_t(t).ravel(), pd))
            self.assertTrue(np.allclose(traj.Rd_t(t).reshape(3, 3), Rd))
            self.assertTrue(np.allclose(traj.dpd_t(t).ravel(), 0))
            self.assertTrue(np.allclose(traj.dRd_t(t).reshape(3, 3), 0))
            self.assertTrue(np.allclose(traj.ddpd_t(t).ravel(), 0))
            self.assertTrue(np.allclose(traj.ddRd_t(t).reshape(3, 3), 0))

    def test_eval_body_twist_of_static(self):
        """静态轨迹经 eval_body_twist → 全零 (regulation 无前馈)."""
        traj = make_static_traj(np.zeros(3), np.eye(3))
        Rd = traj.Rd_t(0.0).reshape(3, 3)
        vd, wd, dvd, dwd = eval_body_twist(traj, 2.0, Rd, bf=1.0)
        for x in (vd, wd, dvd, dwd):
            self.assertTrue(np.allclose(x, 0, atol=1e-12))


if __name__ == '__main__':
    unittest.main()
