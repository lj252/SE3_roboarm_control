# -*- coding: utf-8 -*-
"""
GIC (Geometric Impedance Control) - MuJoCo 仿真实现

======================================================================
核心数学框架 — SE(3) 上的几何阻抗控制
======================================================================

机械臂末端执行器的运动用 SE(3) 李群描述:

    g = [R  p]   ∈ SE(3)
        [0  1]

    - R ∈ SO(3): 旋转矩阵 (3×3), 表示末端朝向
    - p ∈ ℝ³:    位置向量, 表示末端位置
    - g:          齐次变换矩阵 (4×4), 完整描述末端位姿

速度量用 se(3) 李代数(体速度, body twist)表示:

    Vb = [v]   ∈ se(3)
         [w]

    - v ∈ ℝ³: 体坐标系下的线速度
    - w ∈ ℝ³: 体坐标系下的角速度

SE(3) 上的基本运算:
  - hat_map(w):   ℝ³ → so(3), 将角速度向量转为反对称矩阵
  - vee_map(M):  so(3) → ℝ³, 反对称矩阵的逆映射(取出向量)
  - adjoint_g_ed:  SE(3)上的伴随变换, 用于在不同坐标系间转换 twist
  - expm:         se(3) → SE(3), 李代数到李群的指数映射

控制模式:
  - GIC: 纯几何阻抗控制(无主动力控制)
  - 对比 GUFIC: GIC 只有阻抗行为, 没有力跟踪能力

作者: Joohwan Seo (Ph.D. Candidate, UC Berkeley, ME)
"""

# import mujoco_py
import mujoco
import mujoco.viewer
import numpy as np
import sympy as sp

import time, csv, os, copy

import pickle

# import matplotlib.pyplot as plt
from gufic_env.utils.robot_state import RobotState
from gufic_env.utils.mujoco import set_state, set_body_pose_rotm
from gufic_env.utils.misc_func import *

import matplotlib.pyplot as plt

class RobotEnv:
    def __init__(self, robot_name = 'indy7', max_time = 10, show_viewer = False, fz = 10, observables = None,
                 fix_camera = False, task = "regulation", gic_only = False, randomized_start = False, inertia_shaping = False
                 ):

        self.robot_name = robot_name
        self.task = task
        self.gic_only = gic_only
        self.randomized_start = randomized_start
        self.inertia_shaping = inertia_shaping

        if observables is not None:
            self.observables = observables
        else:
            self.observables = ['p', 'pd', 'R', 'Rd']

        self.fz = fz

        self.fix_camera = fix_camera

        print('==============================================')
        print('USING GEOMETRIC IMPEDANCE CONTROL')
        print('==============================================')

        self.p_plate = np.array([0.50, 0.00, 0.11])
        self.R_plate = np.array([[0, 1, 0],
                            [1, 0, 0],
                            [0, 0, -1]])

        if self.task == 'sphere':
            self.p_plate = np.array([0.40, 0.00, 0.0])

        self.z_init_offset = -0.1

        self.contact_count = 0

        self.show_viewer = show_viewer
        self.load_xml()

        self.robot_state = RobotState(self.model, self.data, "end_effector", self.robot_name)

        self.dt = self.model.opt.timestep
        self.max_iter = int(max_time/self.dt)

        self.iter = 0

        # ------------------------------------------------------------------
        # 初始化期望轨迹: 从 task 名称生成轨迹函数
        # pd_t(t):   ℝ → ℝ³    期望位置随时间的函数
        # Rd_t(t):   ℝ → SO(3) 期望朝向随时间的函数
        # dpd_t(t):  期望位置的一阶导数(速度)
        # dRd_t(t):  期望朝向的一阶导数
        # ddpd_t(t): 期望位置的二阶导数(加速度)
        # ddRd_t(t): 期望朝向的二阶导数
        # ------------------------------------------------------------------
        self.pd_t, self.Rd_t, self.dpd_t, self.dRd_t, self.ddpd_t, self.ddRd_t = initialize_trajectory(task = self.task)

        self.Fe = np.zeros((6,1))
        self.reset()

        self.Kp, self.KR, self.Kd = set_gains(controller="GIC", task=self.task)

    def load_xml(self):
        dir = os.getcwd() + '/'
        if self.robot_name == 'ur5e':
            raise NotImplementedError

        elif self.robot_name == 'indy7':
            if self.task == 'sphere':
                model_path = dir + "gufic_env/mujoco_models/Indy7_wiping_sphere.xml"
            else:
                model_path = dir + "gufic_env/mujoco_models/Indy7_wiping.xml"

        elif self.robot_name == 'panda':
            raise NotImplementedError

        else:
            raise NotImplementedError

        self.model = mujoco.MjModel.from_xml_path(model_path)
        # self.sim = mujoco.MjSim(self.model)

        # Need to change self.sim with self.data
        self.data = mujoco.MjData(self.model)
        if self.show_viewer:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        else:
            self.viewer = None

    def reset(self):
        self.iter = 0

        pd = self.pd_t(0)
        Rd = self.Rd_t(0)

        if self.randomized_start:
            rand_xy = 2*(np.random.rand(2,) - 0.5) * 0.05
            rand_rpy = 2*(np.random.rand(3,) - 0.5) * 15 /180 * np.pi
        else:
            rand_xy = np.array([0.05, -0.05])
            rand_rpy = np.array([15, -15, 15]) * np.pi /180

        Rx = np.array([[1, 0, 0], [0, np.cos(rand_rpy[0]), -np.sin(rand_rpy[0])], [0, np.sin(rand_rpy[0]), np.cos(rand_rpy[0])]])
        Ry = np.array([[np.cos(rand_rpy[1]), 0, np.sin(rand_rpy[1])], [0, 1, 0], [-np.sin(rand_rpy[1]), 0, np.cos(rand_rpy[1])]])
        Rz = np.array([[np.cos(rand_rpy[2]), -np.sin(rand_rpy[2]), 0], [np.sin(rand_rpy[2]), np.cos(rand_rpy[2]), 0], [0, 0, 1]])

        # 初始位姿 = 期望位姿 + 随机偏移(用于测试控制的鲁棒性)
        p_init = pd.reshape((-1,1)) + Rd @ np.array([rand_xy[0], rand_xy[1], self.z_init_offset]).reshape(-1,1)
        R_init = Rd @ Rz @ Ry @ Rx

        p_init = p_init.reshape((-1,))

        if self.model.nv == 8:
            q0 = np.array([0, 0, -np.pi/2, 0, -np.pi/2, np.pi/2, 0, 0])
        elif self.model.nv == 6:
            q0 = np.array([0, 0, -np.pi/2, 0, -np.pi/2, np.pi/2])

        # 用高斯-牛顿法求解逆运动学: 由期望位姿(p_init, R_init) → 关节角q
        self.robot_state.gauss_newton_IK(p_init, R_init, q0)

        self.Fe = np.zeros((6,1))

        obs = np.zeros((6,1))

        Rt = np.eye(3)
        self.set_hole_pose(self.p_plate, Rt)

        self.robot_state.update()

        if self.show_viewer:
            self.viewer.sync()

        print('Initialization Complete')
        time.sleep(2)

        return obs

    def run(self):
        p_list = []
        R_list = []
        Fe_list = []
        Fe_raw_list = []
        pd_list = []


        for i in range(self.max_iter):

            pd, Rd, vd, wd, dvd, dwd = self.update_desired_trajectory()

            obs, reward, done, info = self.step()

            # ----------------------------------------------------------
            # 获取当前 SE(3) 位姿
            # p: ℝ³ 位置, R: SO(3) 朝向
            # ----------------------------------------------------------
            p, R = self.robot_state.get_pose()
            Fe = self.get_FT_value()
            Fe_raw = self.get_FT_value_raw()

            p_list.append(p)
            R_list.append(R)
            Fe_list.append(Fe)
            Fe_raw_list.append(Fe_raw)
            pd_list.append(pd)

            # print(reward)

            if self.show_viewer:
                if i % 10 == 0:
                    self.viewer.sync()

            if i % 1000 == 0:
                print(f"Time Step: {i}")

            if done:
                break

        return p_list, R_list, Fe_list, pd_list, Fe_raw_list

    def update_desired_trajectory(self):
        """
        从预定义的轨迹函数中获取当前时刻的期望值。

        SE(3) 相关的返回量:
          - pd: ℝ³, 期望位置
          - Rd: SO(3) (3×3), 期望朝向
          - vd: ℝ³, 体坐标系下的期望线速度, vd = Rdᵀ · dpd
          - wd: ℝ³, 体坐标系下的期望角速度, wd = vee(Rdᵀ · dRd)

        速度量通过将笛卡尔导数变换到体坐标系得到:
          vd = Rdᵀ · dpd    (将世界系速度转到体坐标系)
          wd = vee(Rdᵀ·dRd) (从 so(3) 提取角速度向量)
        """
        # Return pd, Rd, vd, wd, dvd, dwd
        t = self.iter * self.dt
        pd = self.pd_t(t)
        Rd = self.Rd_t(t)

        dpd = self.dpd_t(t)
        dRd = self.dRd_t(t)

        ddpd = self.ddpd_t(t)
        ddRd = self.ddRd_t(t)

        # 将笛卡尔速度变换到体坐标系 (body-fixed frame)
        vd = Rd.T @ dpd.reshape((-1,1))
        wd = vee_map(Rd.T @ dRd)

        # 体坐标系下的期望加速度
        dvd = Rd.T @ ddpd.reshape((-1,1)) - hat_map(wd) @ Rd.T @ dpd.reshape((-1,1))
        dwd = vee_map(Rd.T @ ddRd - hat_map(wd) @ Rd.T @ dRd)

        return pd.reshape((-1,)), Rd, vd.reshape((-1,)), wd.reshape((-1,)), dvd.reshape((-1,)), dwd.reshape((-1,))

    def step(self):
        self.robot_state.update()

        tau_cmd = self.geometric_impedance_control()

        gripper = 0.03

        self.robot_state.set_control_torque(tau_cmd, gripper)

        self.robot_state.update_dynamic()

        if self.show_viewer:
            self.viewer.sync()

        obs = {}
        p, R = self.robot_state.get_pose()
        pd, Rd, vd, wd, dvd, dwd = self.update_desired_trajectory()
        pd = self.pd_t(self.iter * self.dt).reshape((-1,))
        Rd = self.Rd_t(self.iter * self.dt)
        # Put observables in the obs variable
        for obs_name in self.observables:
            if obs_name == 'p':
                obs['p'] = p
            elif obs_name == 'pd':
                obs['pd'] = pd
            elif obs_name == 'R':
                obs['R'] = R
            elif obs_name == 'Rd':
                obs['Rd'] = Rd
            elif obs_name == 'Fe':
                obs['Fe'] = self.get_FT_value()
            elif obs_name == 'Fe_raw':
                obs['Fe_raw'] = self.get_FT_value_raw()
            elif obs_name == 'Psi':
                # SE(3)上的势能函数: Psi = 0.5||p-pd||² + tr(I - RdᵀR)
                # 第一项: 位置误差的二次型
                # 第二项: SO(3)上的势能(在单位元处取最小, 用于朝向跟踪)
                obs['Psi'] = 0.5 * np.linalg.norm(p - pd)**2 + np.trace(np.eye(3) - Rd.T @ R)
            else:
                raise ValueError('Invalid observable name')

        if self.iter == self.max_iter -1:
            done = True
        else:
            done = False

        reward = 0
        info = dict()

        self.iter +=1

        return obs, reward, done, info

    def get_FT_value(self, return_derivative = False):
        """
        获取末端力/力矩传感器值 (Force/Torque)。

        robot_state.get_ee_force() 返回的是环境对末端的力,
        这里取负号得到末端对环境的作用力 Fe。

        Fe ∈ ℝ⁶: [fx, fy, fz, τx, τy, τz]ᵀ
        """
        Fe, dFe = self.robot_state.get_ee_force()
        if return_derivative:
            return -Fe, -dFe
        else:
            return -Fe

    def get_FT_value_raw(self):
        Fe, dFe = self.robot_state.get_ee_force_raw()
        return -Fe

    def get_eg(self, g, gd):
        """
        计算 SE(3) 上的位姿误差。

        参数:
          g:  当前末端位姿 ∈ SE(3)
          gd: 期望位姿 ∈ SE(3)

        返回:
          eg: SE(3)上的误差向量 (6×1)
              eg = [ep; eR]

        其中:
          ep = Rᵀ · (p - pd)  — 体坐标系中的位置误差
          eR = vee(Rdᵀ·R - Rᵀ·Rd) — so(3)上的朝向误差

        这种误差表示是"几何的"(geometric), 与坐标选取无关,
        是SE(3)上控制的基础。
        """
        p = g[:3,3]
        R = g[:3,:3]

        pd = gd[:3,3]
        Rd = gd[:3,:3]

        # 位置误差(转换到体坐标系): ep = Rᵀ(p - pd)
        ep = R.T @ (p - pd)
        # 朝向误差(用 so(3) 表示): eR = vee(RdᵀR - RᵀRd)
        eR = vee_map(Rd.T @ R - R.T @ Rd).reshape((-1,))

        return np.hstack((ep, eR)).reshape((-1,1))

    def get_force_profile(self):

        if self.fz == "time-varying":
            fz = 10 * (np.sin(2 * np.pi / 10 * self.iter * self.dt) + 0.5)

        else:
            fz = self.fz

        Fd = np.array([0, 0, fz, 0, 0, 0])
        return Fd

    def geometric_impedance_control(self):
        """
        ===================================================================
        GIC 控制律的核心实现 (Geometric Impedance Control)

        几何阻抗控制的核心思想: 在SE(3)上定义"虚拟弹簧-阻尼"系统,
        使机械臂末端表现出的阻抗行为具有几何一致性。

        流程概要:
          1. 获取机器人状态 (位姿g, 体速度Vb, 雅可比Jb, 惯性矩阵M)
          2. 构造 SE(3) 期望位姿 gd 和位姿误差 g_ed
          3. 利用伴随变换将笛卡尔速度 Vd 转换到修正参考系 → Vd_star
          4. 计算阻抗弹簧力 fg (位置+朝向的几何弹簧)
          5. 计算控制力矩 tau_cmd = Jᵀ( M_tilde·dVd - Kd·ev - fg ) + τ_bias

        对比 GUFIC:
          - GIC 不做主动力控制 (没有 F_f 力控制项)
          - GIC 没有速度场, 直接用 Vd 和伴随变换
          - GIC 没有能量油箱机制
        ===================================================================
        """
        Jb = self.robot_state.get_body_jacobian()

        # M,C,G = self.robot_state.get_dynamic_matrices()
        qfrc_bias = self.robot_state.get_bias_torque()
        M = self.robot_state.get_full_inertia()

        #0 Get impedance gains
        Kp = self.Kp
        KR = self.KR

        # ----------------------------------------------------------
        # 获取当前末端 SE(3) 位姿 g
        # p = ℝ³ 位置, R = SO(3) 朝向
        # g = [R  p]  ∈ SE(3)
        #     [0  1]
        # ----------------------------------------------------------
        p, R = self.robot_state.get_pose()
        # Update trajectory values
        pd, Rd, vd, wd, dvd, dwd = self.update_desired_trajectory()

        # 当前位姿 g ∈ SE(3)
        g = np.eye(4)
        g[:3,:3] = R
        g[:3,3] = p

        # 期望位姿 gd ∈ SE(3)
        gd = np.eye(4)
        gd[:3,:3] = Rd
        gd[:3,3] = pd

        # ----------------------------------------------------------
        # g_ed = g⁻¹ · gd ∈ SE(3)
        # 这是体坐标系中的位姿误差 (error in body frame)
        # ----------------------------------------------------------
        g_ed = np.linalg.inv(g) @ gd

        # ----------------------------------------------------------
        # 将期望速度 Vd 通过伴随变换(adjoint)转换到修正参考系
        #
        # Vd = [vd; wd] ∈ ℝ⁶ — 体坐标系中的期望速度
        # Vd_star = Ad(g_ed) · Vd
        #
        # 其中 Ad(g_ed) 是 SE(3) 上的伴随变换:
        #   Ad(g) = [R    hat(p)·R ]
        #           [0        R    ]
        #
        # 这个变换将期望速度从 gd 的体坐标系"拉到"当前位置 g 的体坐标系
        # ----------------------------------------------------------
        Vd = np.hstack((vd, wd)).reshape((-1,1)) # shape of (6,1)
        dVd = np.hstack((dvd, dwd)).reshape((-1,1))
        Vd_star = adjoint_g_ed(g_ed) @ Vd

        dVd_star = adjoint_g_ed_deriv(g, gd, vd, wd, dvd, dwd) @ Vd + adjoint_g_ed(g_ed) @ dVd

        # ----------------------------------------------------------
        # Step 3: 计算SE(3)上的几何阻抗弹簧力
        #
        # 位置弹簧力: fp = Rᵀ · Rd · Kp · Rdᵀ · (p - pd)
        #   将位置误差(p - pd)投影到体坐标系, 乘以增益Kp
        #
        # 朝向弹簧力: fR = vee(KR·Rdᵀ·R - Rᵀ·Rd·KR)
        #   这是SO(3)上的"几何"弹簧力矩, 从旋转误差中提取
        #   与传统的"欧拉角弹簧"不同, 这种表示是全局的(无万向锁)
        # ----------------------------------------------------------
        #1 Calculate positional force

        fp = R.T @ Rd @ Kp @ Rd.T @ (p - pd).reshape((-1,1))
        fR = vee_map(KR @ Rd.T @ R - R.T @ Rd @ KR)

        fg = np.vstack((fp,fR))

        Fe, d_Fe = self.get_FT_value(return_derivative=True)
        Fe = Fe.reshape((-1,1))
        d_Fe = d_Fe.reshape((-1,1))

        # Vb: se(3) 体速度 (6×1), [vx, vy, vz, wx, wy, wz]ᵀ
        Vb = self.robot_state.get_body_ee_velocity() #Shape: (6,1)
        Kd = self.Kd

        # ----------------------------------------------------------
        # GIC 控制律
        #
        # 操作空间惯性矩阵:
        #   M_tilde = (J·M⁻¹·Jᵀ)⁻¹
        #
        # 控制律(无惯性整形时):
        #   τ_tilde = M_tilde · dVd_star - Kd · ev - fg
        #
        #   其中:
        #     M_tilde·dVd_star: 前馈项(加速度补偿)
        #     -Kd·ev:           速度阻尼(ev = Vb - Vd_star)
        #     -fg:              阻抗弹簧力(位置+朝向)
        #
        # 最终关节力矩:
        #   τ_cmd = Jᵀ · τ_tilde + τ_bias
        #
        # 注意: 和 GUFIC 相比, 这里没有 F_f (力控制)项
        # ----------------------------------------------------------
        M_tilde_inv = Jb @ np.linalg.pinv(M) @ Jb.T
        M_tilde = np.linalg.pinv(M_tilde_inv)

        M_d = np.eye(6) * 10

        Fe_raw = self.get_FT_value_raw().reshape((-1,1))
        ev = Vb - Vd_star
        if self.inertia_shaping:
            # 惯性整形版本
            tau_tilde = M_tilde @ (dVd_star + np.linalg.inv(M_d) @ (- Kd @ ev - fg + Fe_raw)) - Fe_raw
        else:
            # 标准 GIC: 前馈 + 阻尼 + 弹簧
            tau_tilde = M_tilde @ dVd_star -Kd @ ev - fg
        # tau_tilde = M_tilde @ np.linalg.inv(M_d) @ (- Kd @ ev_mod - fg + F_f_mod + Fe_raw) - Fe_raw


        # 任务空间力矩 → 关节空间力矩
        tau_cmd = Jb.T @ tau_tilde + qfrc_bias.reshape((-1,1))

        return tau_cmd.reshape((-1,))

    def set_hole_pose(self, pos, R):
        set_body_pose_rotm(self.model, 'hole', pos, R)


if __name__ == "__main__":
    robot_name = 'indy7'
    show_viewer = True
    randomized_start = False
    inertia_shaping = False

    task = 'circle'  # 'regulation', 'circle', 'line'

    assert task in ['regulation', 'circle', 'line', 'sphere']

    if task is None:
        max_time = 6
    elif task == 'line':
        max_time = 8
    elif task == 'circle':
        max_time = 10
    elif task == 'sphere':
        max_time = 10
    elif task == 'regulation':
        max_time = 10

    RE = RobotEnv(robot_name, show_viewer = show_viewer, max_time = max_time, fz = 10,
                  fix_camera = False, task = task,randomized_start=randomized_start, inertia_shaping = inertia_shaping)
    RE.run()

    if show_viewer:
        RE.viewer.close()
