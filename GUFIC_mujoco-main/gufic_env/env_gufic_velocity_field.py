# -*- coding: utf-8 -*-
"""
GUFIC (Geometric Unified Force-Impedance Control) - MuJoCo 仿真实现

======================================================================
核心数学框架 — SE(3) 上的几何控制
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
  - hat_map(w):  ℝ³ → so(3), 将角速度向量转为反对称矩阵
  - vee_map(M): so(3) → ℝ³, 反对称矩阵的逆映射(取出向量)
  - adjoint_g:  SE(3)上的伴随变换, 用于在不同坐标系间转换 twist
  - expm:       se(3) → SE(3), 李代数到李群的指数映射(积分速度得到位姿)

控制模式:
  - GUFIC: 统一力-阻抗控制(velocity field 版本), 同时控制力和位置
  - 包含能量油箱(energy tank)机制, 保证无源性和稳定性

作者: Joohwan Seo (Ph.D. Candidate, UC Berkeley, ME)
"""

import mujoco
import mujoco.viewer
import numpy as np
import sympy as sp

from scipy.linalg import expm

import time, csv, os, copy

import pickle

# import matplotlib.pyplot as plt
from gufic_env.utils.robot_state import RobotState
from gufic_env.utils.mujoco import set_state, set_body_pose_rotm
from gufic_env.utils.misc_func import *



import matplotlib.pyplot as plt

class RobotEnv:
    def __init__(self, robot_name = 'indy7', max_time = 10, show_viewer = False, fz = 10, observables = None,
                 fix_camera = False, task = 'regulation', randomized_start = False, inertia_shaping = False
                 ):

        self.robot_name = robot_name
        self.task = task
        self.randomized_start = randomized_start
        self.inertia_shaping = inertia_shaping

        if observables is not None:
            self.observables = observables
        else:
            self.observables = ['p', 'pd', 'R', 'Rd', 'x_tf', 'x_ti', 'Fe', 'Fe_raw', 'Fd', 'rho']

        self.fz = fz
        self.fix_camera = fix_camera

        print('==============================================')
        print('USING GEOMETRIC UNIFED FORCE IMPEDANCE CONTROL')
        print('==============================================')

        self.p_plate = np.array([0.50, 0.00, 0.11])
        self.R_plate = np.array([[0, 1, 0],
                            [1, 0, 0],
                            [0, 0, -1]])

        if self.task == 'sphere':
            self.p_plate = np.array([0.40, 0.00, 0.0])

        self.z_init_offset = -0.1

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

        self.show_viewer = show_viewer
        self.load_xml()

        # robot_state 封装了正/逆运动学、雅可比、动力学等计算
        self.robot_state = RobotState(self.model, self.data, "end_effector", self.robot_name)

        self.dt = self.model.opt.timestep
        self.max_iter = int(max_time/self.dt)

        self.iter = 0

        self.Fe = np.zeros((6,1))
        self.reset()

        self.Kp, self.KR, self.Kd, self.kp_force, self.kd_force, self.ki_force, self.zeta = set_gains(controller = 'GUFIC', task = self.task)

        # print("Gains:", self.Kp, self.KR, self.Kd, self.kp_force, self.kd_force, self.ki_force, self.zeta)
        # print(self.pd_t(0))

        self.int_sat = 50

        ## For the force tracking
        self.e_force_prev = np.zeros((6,1))
        self.int_force_prev = np.zeros((6,1))

        ## For the energy tank
        self.T_f_low = 0.5
        self.T_f_high = 20
        self.delta_f = 1

        self.T_i_low = 0.5
        self.T_i_high = 20
        self.delta_i = 1

        T_i_init = 10
        T_f_init = 10

        if self.task == 'sphere':
            T_i_init = 90
            self.T_i_high = 100

        self.x_tf = np.sqrt(2 * T_f_init)
        self.x_ti = np.sqrt(2 * T_i_init)

        self.T_f = 0.5 * self.x_tf**2
        self.T_i = 0.5 * self.x_ti**2

        self.d_max = 0.03
        self.eR_norm_max = 0.05

        self.use_exponential_gate = True
        self.force_int_gate_sigma = 5.0

        ####### Dummy for the printing
        self.Ff_list = []
        self.Vb_list = []
        self.Ff_activation = []
        self.rho_list = []
        self.Fd_star_list = []
        self.Fi_activation = []

    def load_xml(self):
        # dir = "/home/joohwan/deeprl/research/GIC_Learning_public/"
        dir = os.getcwd() + '/'
        if self.robot_name == 'ur5e':
            raise NotImplementedError

        elif self.robot_name == 'indy7':
            if self.task == "sphere":
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
            if self.fix_camera:
                self.viewer.cam.fixedcamid = 0      # Use a predefined camera from your XML (if available)
                self.viewer.cam.trackbodyid = -1      # Disable tracking any body
                # Alternatively, if you want to set a free camera pose manually:
                self.viewer.cam.lookat = np.array([0.5, 0.0, 0.3])  # Center of the scene
                self.viewer.cam.distance = 1.5                     # Distance from the lookat point
                self.viewer.cam.azimuth = 180                       # Horizontal angle in degrees
                self.viewer.cam.elevation = -20                    # Vertical angle in degrees
        else:
            self.viewer = None

    def reset(self, angle_prefix = None):
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

        p, R = self.robot_state.get_pose()

        # ------------------------------------------------------------------
        # 初始化SE(3)上的积分变量 gd (desired pose on Lie group)
        # gd ∈ SE(3) 用于在速度场中积分跟踪期望轨迹
        #   gd = [R  p]   R: SO(3)旋转, p: ℝ³位置
        #       [0  1]
        # ------------------------------------------------------------------
        self.gd = np.eye(4)
        self.gd[:3,3] = p          # 当前位置
        self.gd[:3,:3] = R         # 当前朝向

        if self.show_viewer:
            self.viewer.sync()

        print('Initialization Complete')
        time.sleep(2)

        return obs

    def run(self):
        p_list = []
        R_list = []
        x_tf_list = []
        x_ti_list = []
        Fe_list = []
        Fd_list = []

        Fe_raw_list = []

        pd_list = []


        for i in range(self.max_iter):

            pd, Rd, vd, wd, dvd, dwd = self.update_desired_trajectory()

            obs, reward, done, info = self.step()

            p, R = self.robot_state.get_pose()
            Fe = self.get_FT_value()
            Fe_raw = self.get_FT_value_raw()

            p_list.append(p)
            R_list.append(R)
            x_tf_list.append(self.x_tf)
            x_ti_list.append(self.x_ti)
            Fe_list.append(Fe)
            Fe_raw_list.append(Fe_raw)
            Fd_list.append(0)
            pd_list.append(pd)

            # print(reward)

            if self.show_viewer:
                if i % 10 == 0:
                    self.viewer.sync()
                if i in [4000]:
                    # print('Stopping here')
                    pass

            if i % 1000 == 0:
                print(f"Time Step: {i}")

            if done:
                break

            # self.iter = i

        return p_list, R_list, x_tf_list, x_ti_list, Fe_list, Fd_list, pd_list, Fe_raw_list

    def update_desired_trajectory(self):
        """
        从预定义的轨迹函数中获取当前时刻的期望值。

        SE(3) 相关的返回量:
          - pd: ℝ³, 期望位置
          - Rd: SO(3) (3×3), 期望朝向
          - vd: ℝ³, 体坐标系下的期望线速度, vd = Rdᵀ · dpd
          - wd: ℝ³, 体坐标系下的期望角速度, wd = vee(Rdᵀ · dRd)

        速度量通过将笛卡尔导数变换到体坐标系得到:
          [vd]  =  Rdᵀ [dpd]
          [wd]       [dRd]   (通过vee_map提取角速度)
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
        # vd = Rdᵀ · dpd, 因为体线速度 = Rᵀ · 世界系速度
        vd = Rd.T @ dpd.reshape((-1,1))
        # wd = vee(Rdᵀ · dRd), 从 so(3) 提取角速度向量
        wd = vee_map(Rd.T @ dRd)

        # 体坐标系下的期望加速度
        dvd = Rd.T @ ddpd.reshape((-1,1)) - hat_map(wd) @ Rd.T @ dpd.reshape((-1,1))
        dwd = vee_map(Rd.T @ ddRd - hat_map(wd) @ Rd.T @ dRd)

        return pd.reshape((-1,)), Rd, vd.reshape((-1,)), wd.reshape((-1,)), dvd.reshape((-1,)), dwd.reshape((-1,))

    def get_velocity_field(self, g, V, t):
        """
        计算速度场(Velocity Field)及其导数。
        这是 GUFIC 的核心 — 在 SE(3) 上构造一个"速度场"引导末端趋向期望轨迹。

        参数:
          g:  SE(3)上的齐次变换矩阵 (4×4), 当前末端位姿
              g = [R  p]  , R ∈ SO(3), p ∈ ℝ³
          V:  se(3)体速度 (6,), V = [v; w], 当前末端速度
          t:  当前时刻

        返回:
          Vd_star:  (6,), 期望体速度(由速度场生成)
          dVd_star: (6,), 期望体速度的导数

        数学推导:
          速度场 Vd_star 在 SE(3) 上几何构造，使得沿该场运动时:
            - 位置误差 (p - pd) 指数收敛
            - 朝向误差 (RdᵀR) 指数收敛
          收敛速度由 zeta 参数控制。
        """
        zeta = self.zeta
        pd = self.pd_t(t).reshape((-1,))
        Rd = self.Rd_t(t)

        dpd = self.dpd_t(t).reshape((-1,))
        dRd = self.dRd_t(t)

        ddpd = self.ddpd_t(t).reshape((-1,))
        ddRd = self.ddRd_t(t)

        # ----------------------------------------------------------
        # 从SE(3)位姿矩阵 g 中提取旋转和平移分量
        # g = [R  p]  ∈ SE(3)
        #     [0  1]
        # ----------------------------------------------------------
        p = g[:3,3]     # ℝ³, 末端位置
        R = g[:3,:3]    # SO(3), 末端朝向

        v = V[:3]        # ℝ³, 体线速度
        w = V[3:]        # ℝ³, 体角速度

        # ----------------------------------------------------------
        # 构造 SE(3) 上的速度场 (Velocity Field)
        #
        # vd_star = Rᵀ·dRd·Rdᵀ·(p - pd) + Rᵀ·dpd - ζ·Rᵀ·(p - pd)
        #           ^-- 旋转耦合项         ^-- 前馈速度  ^-- 位置误差反馈
        #
        # wd_star = vee(Rᵀ·dRd·Rdᵀ·R - ζ·(Rdᵀ·R - Rᵀ·Rd))
        #           ^-- 旋转速度前馈       ^-- 朝向误差反馈(so(3)上)
        #
        # 第一项: 耦合了位置和旋转, 因为旋转运动也会影响体坐标系中的位置误差
        # 第二项: ζ 控制误差收敛速度 (越大收敛越快)
        # 第三项: 朝向误差用 so(3) 上的几何量表示 (RdᵀR - RᵀRd)
        # ----------------------------------------------------------
        Vd_star = np.zeros(6,)
        vd_star = R.T @ dRd @ Rd.T @ (p - pd) + R.T @ dpd - zeta * R.T @ (p - pd)
        wd_star = vee_map(R.T @ dRd @ Rd.T @ R - zeta * (Rd.T @ R - R.T @ Rd)).reshape((-1,))

        Vd_star[:3] = vd_star
        Vd_star[3:] = wd_star

        # ----------------------------------------------------------
        # 速度场的导数 dVd_star (用于控制律中的前馈项)
        # 推导自 Vd_star 对时间的微分，包含李括号项 [w, ·]
        # ----------------------------------------------------------
        dVd_star = np.zeros(6,)
        term1 = -hat_map(w) @ R.T @ dRd @ Rd.T @ R + R.T @ ddRd @ Rd.T @ R + R.T @ dRd @ dRd.T @ R + R.T @ dRd @ Rd.T @ R @ hat_map(w)
        term2 = -hat_map(w) @ R.T @ dRd @ Rd.T @ (p - pd) + R.T @ ddRd @ Rd.T @ (p - pd) + R.T @ dRd @ dRd.T @ (p - pd) \
                + R.T @ dRd @ Rd.T @ (R.T @ v - pd) - hat_map(w) @ R.T @ dpd + R.T @ ddpd
        term3 = dRd.T @ R + Rd.T @ R @ hat_map(w) + hat_map(w) @ R.T @ Rd - R.T @ dRd
        term4 = - hat_map(w) @ R.T @ (p - pd) + v - R.T @ dpd
        dvd_star = term2 - zeta * term4
        dwd_star = vee_map(term1 - zeta * term3).reshape((-1,))

        dVd_star[:3] = dvd_star
        dVd_star[3:] = dwd_star

        return Vd_star, dVd_star


    def step(self):
        self.robot_state.update()

        tau_cmd = self.geometric_unified_force_impedance_control()

        gripper = 0.03

        self.robot_state.set_control_torque(tau_cmd, gripper)

        self.robot_state.update_dynamic()

        if self.show_viewer:
            self.viewer.sync()

        obs = {}
        # Put observables in the obs variable
        p, R = self.robot_state.get_pose()

        pd = self.pd_t(self.iter * self.dt).reshape((-1,))
        Rd = self.Rd_t(self.iter * self.dt)

        for obs_name in self.observables:
            if obs_name == 'p':
                obs['p'] = p
            elif obs_name == 'pd':
                obs['pd'] = pd
            elif obs_name == 'R':
                obs['R'] = R
            elif obs_name == 'Rd':
                obs['Rd'] = Rd
            elif obs_name == 'x_tf':
                obs['x_tf'] = self.x_tf
            elif obs_name == 'x_ti':
                obs['x_ti'] = self.x_ti
            elif obs_name == 'Fe':
                obs['Fe'] = self.get_FT_value()
            elif obs_name == 'Fe_raw':
                obs['Fe_raw'] = self.get_FT_value_raw()
            elif obs_name == 'Fd':
                obs['Fd'] = self.get_force_field(self.gd, self.gd)
            elif obs_name == 'rho':
                obs['rho'] = self.rho
            elif obs_name == 'Psi':
                # SE(3)上的势能函数: Psi = 0.5||p-pd||² + tr(I - RdᵀR)
                # 第一项: 位置误差的二次型
                # 第二项: SO(3)上的势能(在单位元处取最小)
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

    def get_force_field(self,g, gd):
        """
        期望力场 (Force Field).
        当前实现中简化为恒定z方向力:
          Fd = [0, 0, fz, 0, 0, 0]ᵀ
        即期望末端施加 fz(N) 的力, 无力矩。
        """
        fz = self.fz

        Fd = np.array([0, 0, fz, 0, 0, 0])
        return Fd

    def get_force_integral_gate(self, e_force):
        """
        力积分门控函数 (exponential gate).
        当力误差很大时抑制积分作用, 防止积分饱和/windup.

        gate = exp(-(||e_force|| / σ)²)
        """
        Fe_norm = np.linalg.norm(e_force)
        sigma = self.force_int_gate_sigma
        return np.exp(-(Fe_norm / sigma)**2)


    def geometric_unified_force_impedance_control(self):
        """
        ===================================================================
        GUFIC 控制律的核心实现

        这是论文的主要贡献: 在 SE(3) 上几何构造统一力-阻抗控制。

        流程概要:
          1. 获取机器人状态 (位姿g, 体速度Vb, 雅可比Jb, 惯性矩阵M)
          2. 从速度场计算期望速度 Vd_star, dVd_star
          3. 计算阻抗控制项 fg (位置+朝向的弹簧力)
          4. 计算期望力 Fd_star 和力跟踪误差 e_force
          5. 力控制器 PI + 整形函数 ρ (force shaping)
          6. 能量油箱机制 (energy tank) 保证无源性
          7. 修改后的 Vd_star (与力控制协调)
          8. 在SE(3)上积分更新gd (用于阻抗的虚拟参考轨迹)
          9. 计算最终的控制力矩 tau_cmd
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
        # g = [R  p]  ∈ SE(3)
        #     [0  1]
        # ----------------------------------------------------------
        p, R = self.robot_state.get_pose()

        g = np.eye(4)
        g[:3,:3] = R
        g[:3,3] = p

        # Vb: se(3) 体速度 (6×1), [vx, vy, vz, wx, wy, wz]ᵀ
        Vb = self.robot_state.get_body_ee_velocity() # Shape: (6,1)

        # ----------------------------------------------------------
        # Step 1: 从速度场获取期望速度
        # Vd_star ∈ se(3), 由速度场在当前位置生成
        # ----------------------------------------------------------
        # Update trajectory values
        Vd_star, dVd_star = self.get_velocity_field(g, Vb.reshape((-1,)), t = self.iter * self.dt)

        Vd_star = Vd_star.reshape((-1,1))
        dVd_star = dVd_star.reshape((-1,1))

        # ----------------------------------------------------------
        # Step 2: 阻抗控制部分 (ImpedanceControl)
        #
        # gd: 积分得到的SE(3)期望位姿(虚拟参考轨迹)
        #   在 Step 8 中通过积分更新
        #
        # 位置阻抗: fp  = Rᵀ · Rd · Kp · Rdᵀ · (p - pd)
        # 朝向阻抗: fR  = vee(KR·Rdᵀ·R - Rᵀ·Rd·KR)
        #
        # fg = [fp; fR] ∈ ℝ⁶, "几何弹簧力"
        # ----------------------------------------------------------
        #Original GIC Law placeholder
        gd = self.gd
        Rd = gd[:3,:3]
        pd = gd[:3,3]

        # g_ed = g⁻¹·gd ∈ SE(3), 体坐标系中的位姿误差
        g_ed = np.linalg.inv(g) @ gd

        #1 Calculate positional force
        # fp = Rᵀ·Rd·Kp·Rdᵀ·(p - pd): 位置误差在体坐标系中的弹性力
        fp = R.T @ Rd @ Kp @ Rd.T @ (p - pd).reshape((-1,1))
        # fR = vee(KR·Rdᵀ·R - Rᵀ·Rd·KR): so(3)上的朝向弹性力(几何一致)
        fR = vee_map(KR @ Rd.T @ R - R.T @ Rd @ KR)

        fg = np.vstack((fp,fR))

        # ----------------------------------------------------------
        # Step 3: 期望力 Fd_star
        # 基于原始期望轨迹(非积分gd)计算期望力
        # ----------------------------------------------------------
        gd_bar = np.eye(4)
        t = self.iter * self.dt
        gd_bar[:3,:3] = self.Rd_t(t)
        gd_bar[:3,3] = self.pd_t(t).reshape((-1,))
        Fd_star = self.get_force_field(g, gd_bar).reshape((-1,1))

        # ----------------------------------------------------------
        # Step 4: 力跟踪误差
        # e_force = -Fe - Fd_star
        # 注意: Fe = -self.robot_state.get_ee_force()
        # 所以 -Fe 是实际的环境接触力
        # ----------------------------------------------------------
        Fe, d_Fe = self.get_FT_value(return_derivative=True)
        Fe = Fe.reshape((-1,1))
        d_Fe = d_Fe.reshape((-1,1))

        # NOTE(JS) Working is version is that to put e_force = - Fe - Fd, with the Fe = -self.robot_state.get_ee_force()
        # Fd should be positive as well

        e_force = -Fe - Fd_star
        de_force = -d_Fe
        if self.use_exponential_gate:
            force_int_gate = self.get_force_integral_gate(e_force)
        else:
            force_int_gate = 1.0
        int_force = self.int_force_prev + e_force * force_int_gate * self.dt


        int_force = np.clip(int_force, -self.int_sat, self.int_sat)

        # ----------------------------------------------------------
        # Step 5: 力控制器
        #   F_f = -kp_force·e_force - kd_force·de_force - ki_force·int_force + Fd_star
        # 即 PI·D 控制器 + 前馈
        # ----------------------------------------------------------
        if self.fz == "time-varying": # Regular PID Control
            F_f = - self.kp_force * e_force - self.kd_force * de_force - self.ki_force * int_force + Fd_star
        else: # Integral action with minor loop
            F_f = - self.kp_force * (-Fe) - self.ki_force * int_force - self.kd_force * de_force + Fd_star

        # F_f = - self.kp_force * e_force - self.kd_force * de_force - self.ki_force * int_force + Fd_star


        # ----------------------------------------------------------
        # Step 5.5: 力整形函数 ρ (Force Shaping / Activation Function)
        #
        # 当力和阻抗目标方向不一致时(rho < 1)减弱力控制作用，
        # 避免"推拉冲突" (force fighting against impedance).
        #
        # ρ 基于 SE(3) 误差 e = [ep; eR] 和期望力 Fd 的内积
        # 如果 ep 和 f_d 方向相反(ep·f_d > 0)，不需要整形
        # 如果方向相同(ep·f_d <= 0)，则可能存在冲突，需要衰减
        # ----------------------------------------------------------
        #2.5 Apply shaping function to the force control input
        f_d = Fd_star[:3].reshape((-1,))
        m_d = Fd_star[3:].reshape((-1,))

        t = self.iter * self.dt
        gd_t = np.eye(4)
        gd_t[:3,:3] = self.Rd_t(t)
        gd_t[:3,3] = self.pd_t(t).reshape((-1,))
        eg = self.get_eg(g, gd_t)

        ep = eg[:3,0]
        eR = eg[3:,0]

        rho_p = np.zeros((3,))
        rho_R = np.zeros((3,))

        # 位置方向: 如果误差和力方向相反则保持(ρ=1), 否则衰减
        if ep @ f_d <= 0:
            rho_p[:3] = 1
        elif ep @ f_d > 0:
            for i in range(3):
                if np.abs(ep[i]) <= self.d_max:
                    rho_p[i] = 0.5 * (1 + np.cos(np.pi * ep[i] / self.d_max))
                elif np.abs(f_d[i]) <= 0.05:
                    rho_p[i] = 0
        else:
            rho_p[:3] = 0

        # 朝向方向: 类似逻辑
        eR_norm = np.linalg.norm(eR)
        if eR @ m_d <= 0:
            rho_R[:3] = 1
        elif eR @ m_d > 0:
            if eR_norm >= self.eR_norm_max:
                rho_R[:3] = 0.5 * (1 + np.cos(np.pi * eR_norm / self.eR_norm_max))
        else:
            rho_R[:3] = 0

        rho = np.block([rho_p, rho_R]).reshape((-1,1))
        self.rho = rho

        # ensure element-wise multiplication
        F_f = F_f * rho

        self.e_force_prev = e_force
        self.int_force_prev = int_force

        # ----------------------------------------------------------
        # Step 6: 力控制能量油箱 (Energy Tank for Force Control)
        #
        # 原理:  将力控制作用通过一个"能量油箱"来调节,
        #       保证系统的无源性(passivity)。
        #
        # 油箱状态: x_tf, 能量: T_f = 0.5·x_tf²
        # 如果油箱能量过低(低于T_f_low), 力控制作用被抑制(alpha_f=0)
        # 从而确保力控制不会注入过多能量破坏稳定性。
        # ----------------------------------------------------------
        # get a scalar value of the inner product of Vb and F_f without any numpy array
        inner_product_f = (Vb.T @ F_f).reshape((-1,))[0]

        self.T_f = 0.5 * self.x_tf**2

        if inner_product_f < 0:
            gamma_f = 1
        else:
            gamma_f = 0

        if self.T_f <= self.T_f_high:
            beta_f = 1
        else:
            beta_f = 0

        if self.T_f >= self.T_f_low + self.delta_f:
            alpha_f = 1
        elif self.T_f <= self.T_f_low + self.delta_f and self.T_f >= self.T_f_low:
            alpha_f = 0.5 * (1 - np.cos(np.pi * (self.T_f - self.T_f_low) / self.delta_f))
        elif self.T_f < self.T_f_low:
            alpha_f = 0

        dx_tf = - (beta_f / self.x_tf) * gamma_f * inner_product_f + (alpha_f / self.x_tf) * (gamma_f -1) * inner_product_f
        self.x_tf = self.x_tf + dx_tf * self.dt
        self.T_f = 0.5 * self.x_tf**2

        activation_force = gamma_f + alpha_f * (1 - gamma_f)
        F_f_mod = activation_force * F_f

        # ----------------------------------------------------------
        # Step 7: 阻抗能量油箱与修正后的期望速度
        #
        # 类似地, 对阻抗控制也施加能量油箱。
        # Vd_star_mod = gamma_i * Vd_star, 当能量低时减弱
        # ----------------------------------------------------------
        #4. Modified Impedance Control
        inner_product_i = (Vd_star.T @ (F_f_mod + Fe)).reshape((-1,))[0]

        self.T_i = 0.5 * self.x_ti**2

        if inner_product_i > 0:
            gamma_i = 1
        else:
            gamma_i = 0

        if self.T_i <= self.T_i_high:
            beta_i = 1
        elif self.T_i > self.T_i_high:
            beta_i = 0

        if self.T_i >= self.T_i_low + self.delta_i:
            alpha_i = 1
        elif self.T_i <= self.T_i_low + self.delta_i and self.T_i >= self.T_i_low:
            alpha_i = 0.5 * (1 - np.cos(np.pi * (self.T_i - self.T_i_low) / self.delta_i))
        else:
            alpha_i = 0

        activation_impedance = gamma_i + alpha_i * (1 - gamma_i)
        Vd_star_mod = activation_impedance * Vd_star
        dVd_star_mod = activation_impedance * dVd_star
        ev_mod = Vb - Vd_star_mod

        # ----------------------------------------------------------
        # Step 8: 在 SE(3) 上积分更新 gd
        #
        # gd 是阻抗控制的"虚拟参考轨迹"。
        # 通过将修正后的期望速度 Vd_star_mod 积分到SE(3)上得到:
        #
        #   gd ← gd · expm(Vd_mod · dt)
        #
        # 其中 Vd_mod = Ad(g_ed⁻¹) · Vd_star_mod
        # 将体坐标系下的速度转换回世界系(通过伴随变换)
        #
        # expm: 从 se(3) → SE(3) 的指数映射
        # 将 twist 转换为 SE(3) 上的增量位移
        # ----------------------------------------------------------
        # calculate next_step gd
        Vd_mod = adjoint_g_ed(np.linalg.inv(g_ed)) @ Vd_star_mod
        Vd_mod_hat = np.zeros((4,4))
        Vd_mod_hat[:3,:3] = hat_map(Vd_mod[3:,0])
        Vd_mod_hat[:3,3] = Vd_mod[:3,0]
        # 指数映射: se(3) → SE(3), 然后左乘到gd上
        self.gd = gd @ expm(Vd_mod_hat * self.dt)

        Kd = self.Kd

        energy_dissipation = (ev_mod.T @ Kd @ ev_mod)[0,0]
        if energy_dissipation > 10:
            energy_dissipation = 0.1

        if self.iter % 100 == 0: #NOTE(JS) For the Debugging

            # print(f"Sign of impedance inner product:{np.sign(inner_product_i)}, acitvation_impedance: {activation_impedance}")
            # print(f"energy_dissipation:{energy_dissipation}" )
            pass


        dx_ti = (beta_i / self.x_ti) * (gamma_i * inner_product_i + energy_dissipation) \
                + (alpha_i / self.x_ti) * (1 - gamma_i) * inner_product_i

        self.x_ti = self.x_ti + dx_ti * self.dt

        # ----------------------------------------------------------
        # Step 9: GUFIC 控制律 — 计算关节力矩
        #
        # 在任务空间(操作空间)中:
        #   M_tilde = (J·M⁻¹·Jᵀ)⁻¹   — 操作空间惯性矩阵
        #
        # 控制律(无惯性整形时):
        #   τ_tilde = M_tilde·dVd_star - Kd·e_v - fg + F_f
        #
        #   其中:
        #     M_tilde·dVd_star: 前馈项(克服惯性)
        #     -Kd·e_v:          速度阻尼
        #     -fg:              阻抗弹簧力(位置+朝向)
        #     +F_f:             力控制项(经过整形和油箱)
        #
        # 最终关节力矩:
        #   τ_cmd = Jᵀ·τ_tilde + τ_bias
        #
        # 即: 任务空间控制力 → 通过雅可比转置 → 关节空间力矩
        # ----------------------------------------------------------
        M_tilde_inv = Jb @ np.linalg.pinv(M) @ Jb.T
        M_tilde = np.linalg.pinv(M_tilde_inv)

        M_d = np.eye(6) * 10

        Fe_raw = self.get_FT_value_raw().reshape((-1,1))
        if self.inertia_shaping:
            # 惯性整形版本(改变末端惯量):
            # tau_tilde = M_tilde · (dVd_star + M_d⁻¹(-Kd·ev - fg + F_f + Fe_raw)) - Fe_raw
            tau_tilde = M_tilde @ (dVd_star_mod + np.linalg.inv(M_d) @ (- Kd @ ev_mod - fg + F_f_mod + Fe_raw)) - Fe_raw
        else:
            # 标准 GUFIC:
            # tau_tilde = M_tilde · dVd_start - Kd·ev - fg + F_f
            tau_tilde = M_tilde @ dVd_star_mod -Kd @ ev_mod - fg + F_f_mod

        # 任务空间力矩 → 关节空间力矩
        tau_cmd = Jb.T @ tau_tilde + qfrc_bias.reshape((-1,1))

        ####### Save all the dummy variables
        self.Fd_star_list.append(Fd_star)
        self.Ff_list.append(F_f)
        self.Vb_list.append(Vb)
        self.Ff_activation.append(activation_force)
        self.Fi_activation.append(activation_impedance)
        self.rho_list.append(rho)

        return tau_cmd.reshape((-1,))

    def set_hole_pose(self, pos, R):
        set_body_pose_rotm(self.model, 'hole', pos, R)


if __name__ == "__main__":
    robot_name = 'indy7'
    show_viewer = True
    randomized_start = False
    inertia_shaping = False

    task = 'line'  # "regulation", 'circle', 'line'

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
                  fix_camera = True, task = task, randomized_start=randomized_start, inertia_shaping = inertia_shaping)
    RE.run()

    if show_viewer:
        RE.viewer.close()
