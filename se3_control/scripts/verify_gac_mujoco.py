#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SE(3) 几何导纳控制的 MuJoCo + Pinocchio 联合验证
==================================================

验证目标:
  1. GAC 导纳控制器与 RobotModel 集成的正确性
  2. 外力响应（恒力、脉冲、切向力、弹簧接触）的物理合理性
  3. F_ext=0 退化模式下行为与 GIC 一致
  4. MuJoCo 物理仿真中的控制稳定性

方法:
  - 读取 URDF, 生成对应的 MuJoCo 模型
  - RobotModel (Pinocchio) 提供控制律所需的运动学/动力学计算
  - MuJoCo 负责物理推演 (前向动力学)
  - GACController 提供导纳控制律

用法:
  conda activate roboarm
  cd /media/lj252/Data/catkin_ws/roboarm_test/SE3_roboarm_control

  # 默认: 退化模式 (F_ext=0, 可视化 + UR12e + 调节任务)
  python se3_control/scripts/verify_gac_mujoco.py

  # 恒力响应
  python se3_control/scripts/verify_gac_mujoco.py --force-mode constant --no-viewer

  # 切向力柔顺跟随
  python se3_control/scripts/verify_gac_mujoco.py --task regulation \
      --force-mode tangent --tangent-amplitude 10 --no-viewer

  # 脉冲响应
  python se3_control/scripts/verify_gac_mujoco.py --force-mode pulse \
      --force-start 1.0 --force-duration 0.2 --no-viewer

  # circle 跟踪 + 外力扰动
  python se3_control/scripts/verify_gac_mujoco.py --task circle \
      --force-mode constant --force-amplitude 5 0 0 0 0 0 --no-viewer

关联: docs/plan/verify_gac_mujoco_plan.md | docs/plan/GAC_plan.md
"""

import os
import sys
import time
import argparse
import xml.etree.ElementTree as ET
import tempfile
import numpy as np

# ─── 路径设置 ──────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)   # se3_control/
sys.path.insert(0, PROJECT_DIR)

# 导入 Pinocchio 封装
from robot_model.robot_model import RobotModel

# 导入 core 模块
from core.gac_controller import GACController
from core.se3_math import vee_map, hat_map, rotmat_slerp
from core.trajectory import build_trajectory
# 力交互实验分析库 (实验二: 方向解耦)
from core.experiment_analysis import (build_decouple_inputs,
                                      build_decouple_loop_inputs,
                                      extract_decouple,
                                      print_decouple_report,
                                      plot_coupling_matrix)

# 导入配置
from config import task_config
from config.robot_configs import get_robot_config

URDF_DIR = os.path.join(PROJECT_DIR, 'urdf')

# tangent 模式期望朝向: 末端 z 轴垂直朝下 (与 GIC circle 任务一致).
# 列优先: 第 0 列 = EE x 轴 (world +y), 第 1 列 = EE y 轴 (world +x),
# 第 2 列 = EE z 轴 (world -z). 水平面画圆时工具垂直朝下最舒适,
# 腕关节保持中位, 不易碰限位. 详见 usages.md §7.
TANGENT_DOWN_R = np.array([
    [0.0, 1.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0],
])


# ====================================================================
# 1. URDF → MuJoCo XML 转换 (与 verify_gic_mujoco.py 共享逻辑)
# ====================================================================

def rpy_to_rotmat(rpy):
    """URDF RPY (roll-pitch-yaw) → 3×3 旋转矩阵."""
    roll, pitch, yaw = float(rpy[0]), float(rpy[1]), float(rpy[2])
    cx, sx = np.cos(roll), np.sin(roll)
    cy, sy = np.cos(pitch), np.sin(pitch)
    cz, sz = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def rotmat_to_xyz_euler(R):
    """旋转矩阵 → XYZ 顺序欧拉角 (MuJoCo eulerseq='xyz')."""
    ry = np.arcsin(np.clip(R[0, 2], -1.0, 1.0))
    if abs(R[0, 2]) < 0.999999:
        rx = np.arctan2(-R[1, 2], R[2, 2])
        rz = np.arctan2(-R[0, 1], R[0, 0])
    else:
        rx = np.arctan2(R[2, 1], R[1, 1])
        rz = 0.0
    return np.array([rx, ry, rz])


def urdf_rpy_to_mjcf_euler(rpy):
    """URDF RPY → MJCF euler (eulerseq='xyz') 转换."""
    R = rpy_to_rotmat(rpy)
    return rotmat_to_xyz_euler(R)


def parse_urdf_kinematics(urdf_path, debug=False):
    """解析 URDF, 提取运动学树 (仅主线关节链)."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    links = {link.get('name') for link in root.findall('link')}

    joints = []
    for joint in root.findall('joint'):
        name = joint.get('name')
        jtype = joint.get('type')
        parent = joint.find('parent').get('link')
        child = joint.find('child').get('link')
        origin = joint.find('origin')
        if origin is not None:
            xyz_str = origin.get('xyz', '0 0 0')
            rpy_str = origin.get('rpy', '0 0 0')
            origin_xyz = np.array([float(v) for v in xyz_str.split()])
            origin_rpy = np.array([float(v) for v in rpy_str.split()])
        else:
            origin_xyz = np.zeros(3)
            origin_rpy = np.zeros(3)
        axis_el = joint.find('axis')
        if axis_el is not None:
            axis_str = axis_el.get('xyz', '1 0 0')
            axis_xyz = np.array([float(v) for v in axis_str.split()])
        else:
            axis_xyz = np.array([1, 0, 0])
        limit = joint.find('limit')
        if limit is not None:
            lower = float(limit.get('lower', -3.14))
            upper = float(limit.get('upper', 3.14))
            effort = float(limit.get('effort', 100))
            velocity = float(limit.get('velocity', 3.14))
        else:
            lower, upper = -3.14, 3.14
            effort, velocity = 100, 3.14
        joints.append({
            'name': name, 'type': jtype,
            'parent': parent, 'child': child,
            'origin_xyz': origin_xyz, 'origin_rpy': origin_rpy,
            'axis_xyz': axis_xyz,
            'lower': lower, 'upper': upper,
            'effort': effort, 'velocity': velocity,
        })

    rev_joints = [j for j in joints if j['type'] in ('revolute', 'continuous')]
    all_children = set()
    child_to_parent = {}
    for j in joints:
        all_children.add(j['child'])
        child_to_parent[j['child']] = j['parent']
    all_parents = {j['parent'] for j in joints}
    root_link = next(iter(all_parents - all_children), 'world')

    current = root_link
    rev_parents = {j['parent'] for j in rev_joints}
    while current not in rev_parents:
        found_fixed = False
        for j in joints:
            if j['parent'] == current and j['type'] == 'fixed':
                current = j['child']
                found_fixed = True
                break
        if not found_fixed:
            break

    parent_to_rev_joint = dict((j['parent'], j) for j in rev_joints)
    sorted_joints = []
    first_rev = None
    for j in rev_joints:
        if j['parent'] == current:
            first_rev = j
            break
    if first_rev is not None:
        sorted_joints.append(first_rev)
        current = first_rev['child']
        while current in parent_to_rev_joint:
            j = parent_to_rev_joint[current]
            sorted_joints.append(j)
            current = j['child']

    if debug:
        print(f"[URDF] root: {root_link}")
        print(f"[URDF] revolute chain: {[j['name'] for j in sorted_joints]}")

    ee_link = sorted_joints[-1]['child'] if sorted_joints else None
    return sorted_joints, links, ee_link


def urdf_joints_to_mujoco_xml(urdf_path, ee_frame_name='tool0',
                               timestep=0.001, gravity=np.array([0, 0, -9.81]),
                               link_to_mesh=None, mesh_subdir='',
                               debug=False,
                               # ── 接触实验环境 (实验三刚性接触, 计划附录 A) ──
                               # 默认 None/False → 与旧行为完全一致 (零接触几何)
                               rigid_ball=None,   # (pos(3), radius) 或 (pos, r, friction) → 不添加为 None
                               tool_tip=None,     # dict(length,radius,mass) 或 None → 不添加工具尖
                               force_sensor=False):  # 末端 6 维力/力矩传感器 (需 tool_tip)
    """将 URDF 关节链转换为 MuJoCo XML 字符串.

    接触环境可选参数 (实验三, 计划附录 A §A.2-A.3):
      - rigid_ball: 在 worldbody 加一个固定、重质、可碰撞的刚体球
        (接触刚度经 solref 可扫, 用于标定 K_env / 失稳边界扫描);
      - tool_tip:    在末端连杆加一个带质量的工具尖 body (可碰撞球,
        半径≈1cm 单点接触; 质量模拟 FT 传感器前的工具惯量);
      - force_sensor: 在 tool_tip body 上加 6 维力/力矩传感器.
    """
    joints, links, _ = parse_urdf_kinematics(urdf_path, debug)

    tree = ET.parse(urdf_path)
    root = tree.getroot()
    inertia_data = {}
    for link_el in root.findall('link'):
        name = link_el.get('name')
        inertial = link_el.find('inertial')
        if inertial is not None:
            mass_el = inertial.find('mass')
            mass = float(mass_el.get('value', 0))
            origin = inertial.find('origin')
            if origin is not None:
                com_xyz = origin.get('xyz', '0 0 0')
                com_rpy = origin.get('rpy', '0 0 0')
            else:
                com_xyz = '0 0 0'
                com_rpy = '0 0 0'
            inertia_el = inertial.find('inertia')
            if inertia_el is not None:
                ixx = float(inertia_el.get('ixx', 0))
                iyy = float(inertia_el.get('iyy', 0))
                izz = float(inertia_el.get('izz', 0))
                ixy = float(inertia_el.get('ixy', 0))
                ixz = float(inertia_el.get('ixz', 0))
                iyz = float(inertia_el.get('iyz', 0))
            else:
                ixx = iyy = izz = ixy = ixz = iyz = 0
            inertia_data[name] = {
                'mass': mass, 'com': com_xyz, 'com_rpy': com_rpy,
                'ixx': ixx, 'iyy': iyy, 'izz': izz,
                'ixy': ixy, 'ixz': ixz, 'iyz': iyz,
            }

    mesh_origins = {}
    for link_el in root.findall('link'):
        name = link_el.get('name')
        origin = None
        geom_el = link_el.find('collision')
        if geom_el is not None:
            origin = geom_el.find('origin')
        if origin is None:
            vis = link_el.find('visual')
            if vis is not None:
                origin = vis.find('origin')
        if origin is not None:
            mesh_origins[name] = {
                'xyz': origin.get('xyz', '0 0 0'),
                'rpy': origin.get('rpy', '0 0 0'),
            }

    def rotate_inertia_to_body(rpy, ixx, iyy, izz, ixy, ixz, iyz):
        R = rpy_to_rotmat(rpy)
        I_inertial = np.array([[ixx, ixy, ixz],
                               [ixy, iyy, iyz],
                               [ixz, iyz, izz]])
        I_body = R @ I_inertial @ R.T
        return (I_body[0, 0], I_body[1, 1], I_body[2, 2],
                I_body[0, 1], I_body[0, 2], I_body[1, 2])

    if link_to_mesh is None:
        LINK_TO_MESH = {
            'base_link_inertia': 'base_vis',
            'shoulder_link': 'shoulder_vis',
            'upper_arm_link': 'upperarm_vis',
            'forearm_link': 'forearm_vis',
            'wrist_1_link': 'wrist1_vis',
            'wrist_2_link': 'wrist2_vis',
            'wrist_3_link': 'wrist3_vis',
        }
    else:
        LINK_TO_MESH = link_to_mesh

    mesh_eulers = {}
    for ln, mo in mesh_origins.items():
        rpy_arr = np.array([float(v) for v in mo['rpy'].split()])
        mesh_eulers[ln] = urdf_rpy_to_mjcf_euler(rpy_arr)

    indent = '  '
    lines = []
    lines.append('<?xml version="1.0"?>')
    lines.append('<mujoco model="urdf_converted">')
    mesh_dir = os.path.abspath(os.path.join(
        os.path.dirname(urdf_path), 'meshes', mesh_subdir))
    lines.append(f'{indent}<compiler angle="radian" coordinate="local" '
                 f'meshdir="{mesh_dir}/"/>')
    lines.append(f'{indent}<option timestep="{timestep}" '
                 f'gravity="{gravity[0]} {gravity[1]} {gravity[2]}" '
                 f'impratio="10"/>')
    lines.append(f'{indent}<default>')
    lines.append(f'{indent*2}<geom type="mesh" contype="0" conaffinity="0" '
                 f'rgba="0.737 0.737 0.768 1"/>')
    lines.append(f'{indent}</default>')
    lines.append(f'{indent}<asset>')
    for link_name, mesh_name in LINK_TO_MESH.items():
        lines.append(f'{indent*2}<mesh name="{mesh_name}" '
                     f'file="{mesh_name}.stl"/>')
    lines.append(f'{indent}</asset>')
    lines.append(f'{indent}<worldbody>')
    lines.append(f'{indent*2}<light directional="true" diffuse=".8 .8 .8" '
                 f'pos="0 0 5" dir="1.5 1 -2"/>')
    lines.append(f'{indent*2}<geom name="floor" pos="0 0 -0.5" '
                 f'size="2 2 0.5" type="plane" condim="1"/>')
    if rigid_ball is not None:
        ball_pos, ball_r = rigid_ball[0], rigid_ball[1]
        ball_mu = float(rigid_ball[2]) if len(rigid_ball) > 2 else 0.8
        bp = ' '.join(f'{float(v):.10f}' for v in np.asarray(ball_pos))
        lines.append(
            f'{indent*2}<geom name="rigid_ball" type="sphere" '
            f'pos="{bp}" size="{float(ball_r):.10f}" '
            f'friction="{ball_mu:.3f}" '
            f'condim="3" contype="1" conaffinity="1"/>')

    if joints:
        root_body = joints[0]['parent']
        tree_fixed = ET.parse(urdf_path)
        root_fixed = tree_fixed.getroot()
        all_fixed_joints = {}
        for joint_el in root_fixed.findall('joint'):
            if joint_el.get('type') == 'fixed':
                name = joint_el.get('name')
                parent = joint_el.find('parent').get('link')
                child = joint_el.find('child').get('link')
                origin_el = joint_el.find('origin')
                if origin_el is not None:
                    xyz = np.array([float(v) for v in
                                    origin_el.get('xyz', '0 0 0').split()])
                    rpy = np.array([float(v) for v in
                                    origin_el.get('rpy', '0 0 0').split()])
                else:
                    xyz, rpy = np.zeros(3), np.zeros(3)
                all_fixed_joints[child] = {
                    'parent': parent, 'xyz': xyz, 'rpy': rpy, 'name': name,
                }

        root_pos = np.zeros(3)
        root_rpy = np.zeros(3)
        current_link = root_body
        chain_rev = []
        while current_link in all_fixed_joints:
            j = all_fixed_joints[current_link]
            chain_rev.append(j)
            current_link = j['parent']
        for j in reversed(chain_rev):
            R_j = rpy_to_rotmat(j['rpy'])
            root_pos = R_j @ root_pos + j['xyz']
            R_current = rpy_to_rotmat(root_rpy)
            R_new = R_j @ R_current
            root_rpy = rotmat_to_xyz_euler(R_new)

        root_pos_str = (f'{root_pos[0]:.10f} {root_pos[1]:.10f} '
                        f'{root_pos[2]:.10f}')
        root_rpy_str = (f'{root_rpy[0]:.10f} {root_rpy[1]:.10f} '
                        f'{root_rpy[2]:.10f}')
        lines.append(f'{indent*2}<body name="{root_body}" '
                     f'pos="{root_pos_str}" euler="{root_rpy_str}">')
        if root_body in inertia_data and inertia_data[root_body]['mass'] > 0:
            d = inertia_data[root_body]
            com_rpy = [float(v) for v in d['com_rpy'].split()]
            ixx_b, iyy_b, izz_b, ixy_b, ixz_b, iyz_b = \
                rotate_inertia_to_body(
                    com_rpy, d['ixx'], d['iyy'], d['izz'],
                    d['ixy'], d['ixz'], d['iyz'])
            lines.append(f'{indent*3}<inertial mass="{d["mass"]}" '
                         f'pos="{d["com"]}" '
                         f'fullinertia="{ixx_b} {iyy_b} {izz_b} '
                         f'{ixy_b} {ixz_b} {iyz_b}"/>')

        base_mesh = LINK_TO_MESH.get(root_body)
        if base_mesh and root_body in mesh_origins:
            mo = mesh_origins[root_body]
            me = mesh_eulers.get(root_body)
            if me is not None:
                euler_str = (f'{me[0]:.10f} {me[1]:.10f} {me[2]:.10f}')
                lines.append(f'{indent*3}<geom type="mesh" mesh="{base_mesh}" '
                             f'pos="{mo["xyz"]}" euler="{euler_str}"/>')
            else:
                lines.append(f'{indent*3}<geom type="mesh" mesh="{base_mesh}" '
                             f'pos="{mo["xyz"]}"/>')

        def add_body_chain(parent_name, depth=3):
            nonlocal lines
            for j in joints:
                if j['parent'] == parent_name:
                    child_name = j['child']
                    xyz = j['origin_xyz']
                    rpy = j['origin_rpy']
                    axis = j['axis_xyz']
                    lower = j['lower']
                    upper = j['upper']
                    jname = j['name']

                    pos_str = (f'{xyz[0]:.10f} {xyz[1]:.10f} '
                               f'{xyz[2]:.10f}')
                    mjcf_euler = urdf_rpy_to_mjcf_euler(rpy)
                    euler_str = (f'{mjcf_euler[0]:.10f} '
                                 f'{mjcf_euler[1]:.10f} '
                                 f'{mjcf_euler[2]:.10f}')
                    axis_str = (f'{axis[0]:.10f} {axis[1]:.10f} '
                                f'{axis[2]:.10f}')
                    range_str = f'{lower:.10f} {upper:.10f}'

                    outer_indent = indent * depth
                    inner_indent = indent * (depth + 1)

                    lines.append(f'{outer_indent}<body name="{child_name}" '
                                 f'pos="{pos_str}" euler="{euler_str}">')
                    if (child_name in inertia_data
                            and inertia_data[child_name]['mass'] > 0):
                        d = inertia_data[child_name]
                        com_rpy = [float(v) for v in d['com_rpy'].split()]
                        ixx_b, iyy_b, izz_b, ixy_b, ixz_b, iyz_b = \
                            rotate_inertia_to_body(
                                com_rpy, d['ixx'], d['iyy'], d['izz'],
                                d['ixy'], d['ixz'], d['iyz'])
                        lines.append(
                            f'{inner_indent}<inertial mass="{d["mass"]}" '
                            f'pos="{d["com"]}" '
                            f'fullinertia="{ixx_b} {iyy_b} {izz_b} '
                            f'{ixy_b} {ixz_b} {iyz_b}"/>')
                    lines.append(f'{inner_indent}<joint name="{jname}" '
                                 f'type="hinge" axis="{axis_str}" '
                                 f'range="{range_str}"/>')

                    mesh_name = LINK_TO_MESH.get(child_name)
                    if mesh_name and child_name in mesh_origins:
                        mo = mesh_origins[child_name]
                        me = mesh_eulers.get(child_name)
                        if me is not None:
                            euler_str = (f'{me[0]:.10f} {me[1]:.10f} '
                                         f'{me[2]:.10f}')
                            lines.append(
                                f'{inner_indent}<geom type="mesh" '
                                f'mesh="{mesh_name}" pos="{mo["xyz"]}" '
                                f'euler="{euler_str}"/>')
                        else:
                            lines.append(
                                f'{inner_indent}<geom type="mesh" '
                                f'mesh="{mesh_name}" pos="{mo["xyz"]}"/>')

                    child_has_child = any(
                        j2['parent'] == child_name for j2 in joints)
                    if child_has_child:
                        add_body_chain(child_name, depth + 1)
                    else:
                        lines.append(
                            f'{inner_indent}<site name="end_effector" '
                            f'type="sphere" size="0.005" pos="0 0 0" '
                            f'rgba="1 0 0 1"/>')
                        # 工具尖: 挂在末端连杆下方, 带质量的固定 body
                        # (可碰撞球, 单点接触; 质量模拟 FT 传感器前的工具惯量,
                        #  计划附录 A §A.3)
                        if tool_tip is not None:
                            tt_L = float(tool_tip['length'])
                            tt_r = float(tool_tip['radius'])
                            tt_m = float(tool_tip.get('mass', 0.05))
                            tt_I = 0.4 * tt_m * tt_r * tt_r
                            # 工具尖摩擦 (缺省 None → MuJoCo 默认 1.0; 与球摩擦
                            # 组合为几何平均. 表面摩擦跟随需调低, 见实验阶段 1)
                            tt_mu = tool_tip.get('friction', None)
                            tt_fric = (f' friction="{float(tt_mu):.3f}"'
                                       if tt_mu is not None else '')
                            lines.append(
                                f'{inner_indent}<body name="tool_tip" '
                                f'pos="0 0 {tt_L:.10f}">')
                            # 无 joint 的 body = 与父体刚性连接 (工具固定挂载)
                            lines.append(
                                f'{inner_indent*2}<inertial mass="{tt_m:.6f}" '
                                f'pos="0 0 0" '
                                f'fullinertia="{tt_I:.8f} {tt_I:.8f} '
                                f'{tt_I:.8f} 0 0 0"/>')
                            lines.append(
                                f'{inner_indent*2}<geom name="tool_tip" '
                                f'type="sphere" size="{tt_r:.6f}" '
                                f'pos="0 0 0" contype="1" conaffinity="1" '
                                f'rgba="0.2 0.8 0.2 1"{tt_fric}/>')
                            lines.append(
                                f'{inner_indent*2}<site name="tool_tip_site" '
                                f'type="sphere" size="{tt_r:.6f}" '
                                f'pos="0 0 0"/>')
                            lines.append(f'{inner_indent}</body>')
                    lines.append(f'{outer_indent}</body>')

        add_body_chain(root_body)
        lines.append(f'{indent*2}</body>')

    lines.append(f'{indent}</worldbody>')
    lines.append(f'{indent}<actuator>')
    for j in joints:
        lines.append(f'{indent*2}<motor name="{j["name"]}_actuator" '
                     f'joint="{j["name"]}" gear="1" ctrllimited="false" '
                     f'ctrlrange="-1e6 1e6"/>')
    lines.append(f'{indent}</actuator>')
    if force_sensor and tool_tip is not None:
        lines.append(f'{indent}<sensor>')
        lines.append(f'{indent*2}<force name="ee_force" site="tool_tip_site"/>')
        lines.append(f'{indent*2}<torque name="ee_torque" site="tool_tip_site"/>')
        lines.append(f'{indent}</sensor>')
    lines.append('</mujoco>')
    return '\n'.join(lines)


# ====================================================================
# 2. GAC 外力模拟 — ForceProfile
# ====================================================================

class ForceProfile:
    """外力配置文件 — 定义 F_ext(t) 的时间序列.

    所有方法返回 (6,) 维力的向量.
    除 tangent 外都在体坐标系中定义.
    """

    @staticmethod
    def zero(t):
        """零外力 — 退化模式验证."""
        return np.zeros(6)

    @staticmethod
    def constant(t, force=None):
        """恒定外力 — 验证稳态响应."""
        if force is None:
            force = [10.0, 0, 0, 0, 0, 0]
        return np.array(force, dtype=float)

    @staticmethod
    def pulse(t, start=0.5, duration=0.2, amplitude=None):
        """脉冲外力 — 验证动态响应和恢复."""
        if amplitude is None:
            amplitude = [20.0, 0, 0, 0, 0, 0]
        if start <= t <= start + duration:
            return np.array(amplitude, dtype=float)
        return np.zeros(6)

    @staticmethod
    def spring_contact(t, p_ee, p_surface, stiffness=1000.0):
        """模拟刚性表面接触: F = K_env · penetration.

        :param p_ee: 当前末端位置 (3,)
        :param p_surface: 接触表面位置 (3,)
        :param stiffness: 环境刚度 (N/m)
        :returns: 体坐标系接触力 (6,)
        """
        penetration = p_surface - p_ee
        if penetration[2] > 0:
            f_ext = np.zeros(6)
            f_ext[2] = stiffness * penetration[2]
            return f_ext
        return np.zeros(6)

    @staticmethod
    def tangential(t, p_ee, center=None, radius=0.2,
                   speed=1.0, amplitude=10.0,
                   radial_stiffness=0.0):
        """沿圆弧切线方向的恒力, 带可选径向虚拟弹簧约束.

        惯性系中计算, 调用方需旋转到体坐标系.

        径向虚拟弹簧 (radial_stiffness > 0):
          当机器人偏离圆半径时, 产生径向回复力:
            F_radial = -K_radial · (r - r_des) · radial_unit_vector

        :param p_ee: 当前末端位置 (3,)
        :param center: 圆心 [cx, cy, cz]
        :param radius: 圆半径 (m)
        :param speed: 轨迹速度 (rad/s), 未使用 (位置依赖)
        :param amplitude: 切向力幅值 (N)
        :param radial_stiffness: 径向虚拟弹簧刚度 (N/m), 0=无约束
        :returns: 惯性系外力 (6,) — 调用方需 Rᵀ 转到体坐标系
        """
        if center is None:
            center = [0.5, 0.0, 0.125]
        c = np.array(center, dtype=float)
        dx = p_ee[0] - c[0]
        dy = p_ee[1] - c[1]
        theta = np.arctan2(dy, dx)
        # 切向: [-sin(θ), cos(θ), 0]
        tangent_dir = np.array([-np.sin(theta), np.cos(theta), 0.0])
        f_ext = np.zeros(6)
        f_ext[:3] = tangent_dir * amplitude

        # 径向虚拟弹簧约束
        if radial_stiffness > 0.0:
            dist = np.sqrt(dx**2 + dy**2)
            if dist > 1e-8:
                radial_dir = np.array([dx / dist, dy / dist, 0.0])
                radial_force = -radial_stiffness * (dist - radius)
                f_ext[:3] += radial_dir * radial_force

        return f_ext


# ====================================================================
# 3. GAC 控制仿真主循环
# ====================================================================

def run_verification(robot_urdf, task='regulation',
                     show_viewer=True, max_time=5.0,
                     home_q=None, ee_frame='tool0',
                     link_to_mesh=None, mesh_subdir='',
                     torque_limits=None,
                     # ── GAC 专属参数 ──
                     M_d=None, D_d=None, K_d=None, dt_filter=0.002,
                     force_mode='zero',
                     force_amplitude=None,
                     force_start=1.0, force_duration=0.5,
                     tangent_center=None, tangent_radius=0.2,
                     tangent_amplitude=10.0, tangent_radial_stiffness=500.0,
                     init_pos=None,
                     bandwidth=30.0, damping=1.0,
                     verbose=True, stop_at_end=True, loop=False,
                     # ── 力交互实验 (实验二: 方向解耦) ──
                     experiment='none',
                     decouple_force=10.0, decouple_moment=1.0,
                     decouple_settle=2.0, decouple_measure=1.0,
                     decouple_loop=False, decouple_cycles=2,
                     task_cfg=None):
    """GAC 控制验证主循环.

    步骤:
      1. 生成 MuJoCo 模型 (从 URDF)
      2. 加载 RobotModel (Pinocchio, 从相同 URDF)
      3. 初始化 MuJoCo 仿真
      4. 初始化轨迹 (core.trajectory.build_trajectory)
      5. 构建 GACController
      6. 运行控制循环 (含 F_ext 注入)
      7. 记录并分析结果
    """
    import mujoco

    if task_cfg is None:
        task_cfg = task_config

    # ── 1. 生成 MuJoCo 模型 ──
    urdf_path = os.path.join(URDF_DIR, robot_urdf)
    if not os.path.exists(urdf_path):
        urdf_path = robot_urdf
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"Cannot find URDF: {urdf_path}")

    xml_str = urdf_joints_to_mujoco_xml(urdf_path, ee_frame,
                                         link_to_mesh=link_to_mesh,
                                         mesh_subdir=mesh_subdir,
                                         debug=verbose)

    tmpf = tempfile.NamedTemporaryFile(suffix='.xml', mode='w', delete=False)
    tmpf.write(xml_str)
    tmpf.close()

    if verbose:
        print(f"[MuJoCo XML] written to {tmpf.name}")

    try:
        model = mujoco.MjModel.from_xml_path(tmpf.name)
        data = mujoco.MjData(model)
    except Exception as e:
        print(f"[ERROR] MuJoCo model load failed: {e}")
        print("Generated XML:")
        print(xml_str)
        raise
    finally:
        os.unlink(tmpf.name)

    nv = model.nv

    if verbose:
        print(f"[MuJoCo] nq={model.nq}, nv={nv}, nsite={model.nsite}, "
              f"njnt={model.njnt}")

    # ── 2. 加载 RobotModel ──
    robot = RobotModel(urdf_path, ee_frame_name=ee_frame,
                       robot_name=os.path.basename(robot_urdf),
                       verbose=verbose)

    if home_q is None:
        # 默认舒适位形: 与 robot_configs 'ur12e' 的 home_q 一致
        # (EE 在 [0.50, 0, 0.50], 末端竖直朝下, 避开腕部奇异)
        home_q = np.array([-0.356, -1.498, 1.81, 1.259, 1.571, -0.124])[:robot.nv]

    # ── 3. 初始化状态与轨迹 ──
    dt = model.opt.timestep

    # ── 实验配置 (方向解耦: 7 块 = 基线 + 6 输入;
    #    --decouple-loop 可视化循环: 动作间插复位间隙, 序列循环) ──
    # 末端 body id: 物理外力 (xfrc_applied) 作用点 (作用在其 COM)
    ee_body_id = model.site_bodyid[0] if model.nsite > 0 else 0
    decouple_inputs = None
    if experiment == 'decouple':
        if decouple_loop:
            decouple_inputs = build_decouple_loop_inputs(
                force=decouple_force, moment=decouple_moment)
            decouple_block = decouple_settle + decouple_measure
            decouple_total = (len(decouple_inputs) * decouple_block
                              * decouple_cycles)
            if max_time <= 5.0:
                max_time = decouple_total
                print(f"[GAC decouple-loop] Auto max_time = {max_time:.1f}s "
                      f"({len(decouple_inputs)} 子块 × {decouple_block:.1f}s "
                      f"× {decouple_cycles} 循环; 关闭 viewer 可提前停止)")
        else:
            decouple_inputs = build_decouple_inputs(force=decouple_force,
                                                    moment=decouple_moment)
            decouple_block = decouple_settle + decouple_measure
            decouple_total = len(decouple_inputs) * decouple_block
            if max_time <= 5.0:
                max_time = decouple_total
                print(f"[GAC decouple] Auto max_time = {max_time:.1f}s "
                      f"({len(decouple_inputs)} blocks × {decouple_block:.1f}s)")
    T = int(max_time / dt)

    # 使用 core.trajectory 构建轨迹
    is_regulation = (task == 'regulation')
    if is_regulation:
        from types import SimpleNamespace
        _zero3 = lambda t: np.zeros(3)
        _zero33 = lambda t: np.zeros((3, 3))

        # 如果指定了 init_pos, 用 IK 将机器人摆到该位置
        if init_pos is not None:
            init_pos = np.asarray(init_pos, dtype=float).ravel()
            # 先设 home_q, 取朝向
            data.qpos[:nv] = home_q.copy()
            data.qvel[:nv] = np.zeros(nv)
            mujoco.mj_forward(model, data)
            robot.update(home_q)
            _, R_init = robot.get_pose()
            q_ik = robot.gauss_newton_IK(init_pos, R_init, home_q,
                                          step_size=0.5, tol=1e-6, max_cnt=500)
            if verbose:
                print(f"[IK] init_pos={init_pos}, q_ik={np.round(q_ik, 4)}")
            data.qpos[:nv] = q_ik.copy()
            data.qvel[:nv] = np.zeros(nv)
            mujoco.mj_forward(model, data)
            robot.update(q_ik)
            p_start, R_start = robot.get_pose()
            if verbose:
                print(f"[IK] achieved p={np.round(p_start, 4)}, "
                      f"err={np.linalg.norm(p_start - init_pos):.6e}")
        else:
            data.qpos[:nv] = home_q.copy()
            data.qvel[:nv] = np.zeros(nv)
            mujoco.mj_forward(model, data)
            robot.update(home_q)
            p_start, R_start = robot.get_pose()

        traj_funcs = SimpleNamespace(
            pd_t=lambda t: p_start,
            Rd_t=lambda t: R_start,
            dpd_t=_zero3, dRd_t=_zero33,
            ddpd_t=_zero3, ddRd_t=_zero33,
        )
    else:
        traj_funcs = build_trajectory(task, cfg=task_cfg)
        if verbose:
            print(f"[Trajectory] start = {traj_funcs.pd_t(0).ravel()}")

        pd0 = traj_funcs.pd_t(0).ravel()
        robot.update(data.qpos[:nv].copy())
        _, R_home = robot.get_pose()
        q_ik = robot.gauss_newton_IK(pd0, R_home, home_q,
                                      step_size=0.5, tol=1e-6, max_cnt=500)
        data.qpos[:nv] = q_ik.copy()
        data.qvel[:nv] = np.zeros(nv)
        mujoco.mj_forward(model, data)
        robot.update(q_ik)
        p_start, R_start = robot.get_pose()
        if verbose:
            print(f"[IK] q_ik    = {np.round(q_ik, 4)}")
            print(f"[IK] pos_err = {np.linalg.norm(p_start - pd0):.6e}")

    # ── 4. 朝向渐进混合 (动态任务 / tangent 模式从初始姿态过渡到朝下) ──
    # tangent: 需从 home 朝向旋转约 160° 到"朝下". 0.4s 太猛 (≈7 rad/s)
    # 会使前 4 个关节力矩饱和, 用 3.0s 平滑过渡 (≈0.93 rad/s).
    BLEND_DURATION = 3.0 if force_mode == 'tangent' else 0.4
    is_dynamic_task = not is_regulation
    if is_dynamic_task or force_mode == 'tangent':
        _, R_home_ik = robot.get_pose()
        if verbose:
            Rd0_des = traj_funcs.Rd_t(0).ravel().reshape(3, 3)
            if force_mode == 'tangent':
                Rd0_des = TANGENT_DOWN_R  # tangent: 混合目标是朝下朝向
            init_rot_err = 0.5 * np.linalg.norm(
                np.cross(R_home_ik[:, 0], Rd0_des[:, 0])
                + np.cross(R_home_ik[:, 1], Rd0_des[:, 1])
                + np.cross(R_home_ik[:, 2], Rd0_des[:, 2]))
            print(f"[Blend] Initial orientation error: {init_rot_err:.4f} rad")

    # ── 5. 构建导纳参数 ──
    if M_d is None:
        M_d = [10.0, 10.0, 10.0, 1.0, 1.0, 1.0]
    if K_d is None:
        K_d = [500.0, 500.0, 500.0, 50.0, 50.0, 50.0]

    # tangent 模式: 切向刚度置零 (纯阻尼), 增大修正量限幅
    _max_correction = 0.05  # 默认
    if force_mode == 'tangent':
        if verbose:
            print(f"[GAC tangent] Setting K_d[:2]=0 for continuous motion")
        K_d_orig = list(K_d)
        K_d = [0.0, 0.0, K_d[2], K_d[3], K_d[4], K_d[5]]
        _max_correction = 5.0  # 允许 5m 累积修正 (K_d=0 时无界增长)

    # 有效径向刚度 (K_d=0 的方向用径向虚拟弹簧刚度做参考)
    _radial_K_eff = tangent_radial_stiffness if (
        force_mode == 'tangent' and tangent_radial_stiffness > 0
    ) else 100.0
    if D_d is None:
        # 临界阻尼: D = 2·sqrt(K·M)
        D_d = [2 * np.sqrt(K_d[i] * M_d[i]) if K_d[i] > 0
               else 2 * np.sqrt(_radial_K_eff * M_d[i])
               for i in range(6)]
    else:
        # 即使传入了 D_d, 也对 K_d=0 的方向重新计算阻尼
        # 使阻尼与径向虚拟弹簧刚度匹配, 避免径向过冲
        for i in range(6):
            if K_d[i] == 0:
                D_d[i] = 2.0 * np.sqrt(_radial_K_eff * M_d[i]) if M_d[i] > 0 else 10.0

    # ── 6. 控制器 ──
    controller = GACController(
        robot,
        M_d=M_d, D_d=D_d, K_d=K_d, dt=dt_filter,
        bandwidth=bandwidth, damping=damping,
        torque_limits=torque_limits,
        max_correction=_max_correction,
    )

    if verbose:
        q_actual = data.qpos[:nv].copy()
        robot.update(q_actual)
        model_p, _ = robot.get_pose()
        mujoco_p = data.site_xpos[0].copy() if model.nsite > 0 else np.zeros(3)
        print(f"[FK] MuJoCo  EE: {mujoco_p}")
        print(f"[FK] Pinocchio EE: {model_p}")
        print(f"[FK] pos_diff: {np.linalg.norm(mujoco_p - model_p):.6e}")

    # ── 7. 记录 ──
    log = {
        't': np.zeros(T),
        'p': np.zeros((T, 3)),
        'pd': np.zeros((T, 3)),
        'R': np.zeros((T, 3, 3)),
        'Rd': np.zeros((T, 3, 3)),
        'tau': np.zeros((T, nv)),
        'pos_err': np.zeros(T),
        'rot_err': np.zeros(T),
        'q': np.zeros((T, nv)),
        'dq': np.zeros((T, nv)),
        'f_ext': np.zeros((T, 6)),
        'x_corr': np.zeros((T, 6)),   # 导纳滤波器输出 X_corr (体坐标系)
    }

    # ── 8. Viewer ──
    viewer = None
    trail_cfg = task_cfg.trail
    TRAIL_INTERVAL = trail_cfg.get('interval', 8)
    TRAIL_MAX = trail_cfg.get('max_points', 1200)
    TRAIL_SIZE = trail_cfg.get('sphere_size', 0.006)
    TRAIL_COLOR = np.array(trail_cfg.get('color', [1.0, 0.2, 0.2, 0.85]),
                           dtype=float)
    trail_actual = []
    if show_viewer:
        try:
            from mujoco.viewer import launch_passive
            viewer = launch_passive(model, data)
            time.sleep(0.5)
        except Exception as e:
            print(f"[Viewer] Failed to launch: {e}")
            print("[Viewer] Run with --no-viewer to suppress this warning.")
            show_viewer = False

    # ── 外力模式默认参数 ──
    if force_amplitude is None:
        force_amplitude = [10.0, 0, 0, 0, 0, 0]
    if tangent_center is None:
        tangent_center = [0.5, 0.0, 0.125]

    # ── 9. 主循环 ──
    if verbose:
        print(f"\n{'='*60}")
        print(f"Running GAC simulation: task={task}, "
              f"force_mode={force_mode}, T={T} steps ({max_time}s)")
        print(f"{'='*60}")

    # ── tangent 模式: 惯性系导纳状态 (不与 R_cur 耦合, 避免 z 振荡) ──
    if force_mode == 'tangent':
        pos_corr_inertial = np.zeros(3)   # 惯性系位置修正量 [x, y, z]
        vel_corr_inertial = np.zeros(3)   # 惯性系速度修正量
        radial_k_eff = (tangent_radial_stiffness if tangent_radial_stiffness > 0
                        else 100.0)
        D_radial = (2.0 * np.sqrt(radial_k_eff * M_d[0])
                    if M_d[0] > 0 else 10.0)
        D_z = (2.0 * np.sqrt(K_d[2] * M_d[2])
               if M_d[2] > 0 else 10.0)

    t0 = time.time()
    for i in range(T):
        t = i * dt

        # 关闭 viewer 立即停止仿真 (所有模式通用)
        if viewer is not None and not viewer.is_running():
            if verbose:
                print(f"[Viewer] closed — stopping simulation early "
                      f"(t={t:.1f}s).")
            break

        # ── 期望轨迹 ──
        pd = traj_funcs.pd_t(t).ravel()
        Rd_des = traj_funcs.Rd_t(t).reshape((3, 3))

        # tangent 模式: 期望朝向强制为"朝下" (末端垂直向下, 与 GIC circle 一致),
        # 位置仍由惯性系导纳修正, 朝向与位置解耦, 互不影响.
        if force_mode == 'tangent':
            Rd_des = TANGENT_DOWN_R

        needs_orient_blend = is_dynamic_task or force_mode == 'tangent'
        if needs_orient_blend and t < BLEND_DURATION:
            alpha = t / BLEND_DURATION
            Rd = rotmat_slerp(R_home_ik, Rd_des, alpha)
        else:
            Rd = Rd_des
        blend_factor = min(1.0, t / BLEND_DURATION) if needs_orient_blend else 1.0
        dpd = traj_funcs.dpd_t(t).ravel()
        dRd = traj_funcs.dRd_t(t).reshape((3, 3)) * blend_factor
        ddpd = traj_funcs.ddpd_t(t).ravel()
        ddRd = traj_funcs.ddRd_t(t).reshape((3, 3)) * blend_factor

        # tangent 混合期间: 覆盖朝向是 slerp 插值, 无解析导数.
        # 用有限差分估计 dRd, 使 wd 反映实际旋转, 避免混合期内姿态跟踪滞后.
        if force_mode == 'tangent' and t < BLEND_DURATION:
            alpha_p = min((t + dt) / BLEND_DURATION, 1.0)
            Rd_p = rotmat_slerp(R_home_ik, Rd_des, alpha_p)
            dRd = (Rd_p - Rd) / dt

        vd = Rd.T @ dpd.reshape((-1, 1))
        wd = vee_map(Rd.T @ dRd)
        dvd = (Rd.T @ ddpd.reshape((-1, 1))
               - hat_map(wd) @ Rd.T @ dpd.reshape((-1, 1)))
        dwd = vee_map(Rd.T @ ddRd - hat_map(wd) @ Rd.T @ dRd)

        # ── 读取当前 MuJoCo 状态 ──
        q = data.qpos[:nv].copy()
        dq = data.qvel[:nv].copy()

        # ── 读取当前位姿 (用于外力计算) ──
        has_site = model.nsite > 0
        if has_site:
            p_ee = data.site_xpos[0].copy()
            R_cur = data.site_xmat[0].copy().reshape((3, 3))
        else:
            robot.update(q)
            p_ee, R_cur = robot.get_pose()

        # ── 计算 F_ext ──
        if experiment == 'decouple':
            # 方向解耦: 世界系恒力/力偶, 物理施加 + 感知回读 (双力通路)
            k_block = int(t // decouple_block)
            if decouple_loop:
                k_block = k_block % len(decouple_inputs)   # 循环模式: 取模
            elif k_block >= len(decouple_inputs):
                k_block = len(decouple_inputs) - 1
            F_world = decouple_inputs[k_block]
            if model.nsite > 0:
                data.xfrc_applied[ee_body_id, :] = F_world   # 物理力 (世界系, 末端 body COM)
            F_ext_raw = F_world.copy()
            # 感知力: 模拟腕部 FT 传感器回读 (世界系 → 体坐标系 Rᵀ·F),
            # 与硬件部署 (部署计划 M3: FT 集成) 逐层映射一致
            F_ext_ctrl = np.zeros(6)
            F_ext_ctrl[:3] = R_cur.T @ F_world[:3]
            F_ext_ctrl[3:] = R_cur.T @ F_world[3:]
        elif force_mode == 'zero':
            F_ext_raw = ForceProfile.zero(t)
        elif force_mode == 'constant':
            F_ext_raw = ForceProfile.constant(t, force=force_amplitude)
        elif force_mode == 'pulse':
            F_ext_raw = ForceProfile.pulse(
                t, start=force_start, duration=force_duration,
                amplitude=force_amplitude)
        elif force_mode == 'spring':
            # 接触面 z=0.0 (桌面高度)
            p_surface = np.array([p_ee[0], p_ee[1], 0.0])
            F_ext_raw = ForceProfile.spring_contact(t, p_ee, p_surface)
        elif force_mode == 'tangent':
            # 惯性系外力 (不旋转到体坐标系, 避免 R_cur 耦合)
            F_ext_inertial = ForceProfile.tangential(
                t, p_ee, center=tangent_center,
                radius=tangent_radius, amplitude=tangent_amplitude,
                radial_stiffness=tangent_radial_stiffness)

            # 惯性系 3-DOF 导纳更新 (位置修正)
            # xy: K=0 (自由跟随切向力, 径向弹簧维持半径)
            # z:  K=K_d[2] (刚性维持高度)
            M_xyz = np.array([M_d[0], M_d[1], M_d[2]])
            K_xyz = np.array([0.0, 0.0, K_d[2]])
            D_xyz = np.array([D_radial, D_radial, D_z])
            acc = (F_ext_inertial[:3] - D_xyz * vel_corr_inertial
                   - K_xyz * pos_corr_inertial) / M_xyz
            vel_corr_inertial += acc * dt
            pos_corr_inertial += vel_corr_inertial * dt

            # 修正量限幅 (与 max_correction=5.0 一致)
            corr_norm = np.linalg.norm(pos_corr_inertial)
            if corr_norm > 5.0:
                pos_corr_inertial *= 5.0 / corr_norm

            # 不给 GACFilter 外力 (完全避开体坐标系导纳的 R_cur 耦合)
            # F_ext_raw 保存实际惯性系外力供日志/显示, F_ext_ctrl=0 给控制器
            F_ext_ctrl = np.zeros(6)
            F_ext_raw = np.zeros(6)
            F_ext_raw[:3] = F_ext_inertial[:3]  # 日志记录实际外力
        else:
            F_ext_raw = np.zeros(6)

        # ── 控制器外力 (tangent 模式用零避免体坐标系耦合;
        #    decouple 分支已在上面设置感知力 Rᵀ·F, 需保留) ──
        F_ext_ctrl = (F_ext_ctrl if (force_mode == 'tangent'
                                     or experiment == 'decouple')
                      else F_ext_raw)

        # ── 惯性系轨迹修正 (tangent 模式: 叠加 pos_corr_inertial) ──
        if force_mode == 'tangent':
            pd_track = pd.copy()
            pd_track[:3] = pd[:3] + pos_corr_inertial
        else:
            pd_track = pd

        # ── GAC 控制 ──
        tau_cmd = controller.compute(
            q, dq, pd_track, Rd, vd, wd, dvd, dwd, F_ext=F_ext_ctrl)

        # ── 应用力矩到 MuJoCo ──
        data.ctrl[:] = tau_cmd[:model.nu]
        mujoco.mj_step(model, data)

        # ── 记录 ──
        log['t'][i] = t
        log['q'][i] = q
        log['dq'][i] = dq

        if has_site:
            site_p = data.site_xpos[0].copy()
            site_R = data.site_xmat[0].copy().reshape((3, 3))
        else:
            robot.update(q)
            site_p, site_R = robot.get_pose()

        log['p'][i] = site_p
        log['pd'][i] = pd
        log['R'][i] = site_R
        log['Rd'][i] = Rd
        log['tau'][i] = tau_cmd
        log['f_ext'][i] = F_ext_raw.ravel()
        if experiment == 'decouple':
            log['x_corr'][i] = controller.filter_state['X_corr']

        ep = site_p - pd
        eR = -0.5 * (np.cross(site_R[:, 0], Rd[:, 0])
                     + np.cross(site_R[:, 1], Rd[:, 1])
                     + np.cross(site_R[:, 2], Rd[:, 2]))
        log['pos_err'][i] = np.linalg.norm(ep)
        log['rot_err'][i] = np.linalg.norm(eR)

        # ── Viewer trail ──
        if viewer:
            if i % TRAIL_INTERVAL == 0:
                trail_actual.append(site_p.copy())
                if len(trail_actual) > TRAIL_MAX:
                    trail_actual.pop(0)
            if i % 5 == 0:
                ngeom = min(len(trail_actual), viewer.user_scn.maxgeom)
                if ngeom > 1:
                    for j in range(ngeom):
                        pos = trail_actual[j]
                        mujoco.mjv_initGeom(
                            viewer.user_scn.geoms[j],
                            mujoco.mjtGeom.mjGEOM_SPHERE,
                            np.array([TRAIL_SIZE, 0, 0]),
                            pos, np.eye(3).flatten(), TRAIL_COLOR,
                        )
                    viewer.user_scn.ngeom = ngeom
                viewer.sync()

        if verbose and (i % 500 == 0 or i == T - 1):
            print(f"  t={t:.3f}s | pos_err={log['pos_err'][i]:.6f} | "
                  f"rot_err={log['rot_err'][i]:.6f} | "
                  f"|F_ext|={np.linalg.norm(F_ext_raw):.2f} | "
                  f"tau_norm={np.linalg.norm(tau_cmd):.2f}")

    t_elapsed = time.time() - t0
    if verbose:
        print(f"\nSimulation finished in {t_elapsed:.2f}s "
              f"({(T / t_elapsed):.0f} Hz)")

    # ── 10. 连续循环 ──
    loop_active = loop and show_viewer and viewer is not None
    if loop_active:
        t_cont = T * dt
        i = T
        if verbose:
            print("[Loop] Continuous mode: task keeps running. "
                  "Close viewer to stop.")
        while viewer.is_running():
            t_cur = i * dt

            pd = traj_funcs.pd_t(t_cur).ravel()
            Rd = traj_funcs.Rd_t(t_cur).reshape((3, 3))
            dpd = traj_funcs.dpd_t(t_cur).ravel()
            dRd = traj_funcs.dRd_t(t_cur).reshape((3, 3))
            ddpd = traj_funcs.ddpd_t(t_cur).ravel()
            ddRd = traj_funcs.ddRd_t(t_cur).reshape((3, 3))

            vd = Rd.T @ dpd.reshape((-1, 1))
            wd = vee_map(Rd.T @ dRd)
            dvd = (Rd.T @ ddpd.reshape((-1, 1))
                   - hat_map(wd) @ Rd.T @ dpd.reshape((-1, 1)))
            dwd = vee_map(Rd.T @ ddRd - hat_map(wd) @ Rd.T @ dRd)

            q = data.qpos[:nv].copy()
            dq = data.qvel[:nv].copy()

            # 循环中外力为 0
            F_ext_raw = np.zeros(6)

            tau_cmd = controller.compute(
                q, dq, pd, Rd, vd, wd, dvd, dwd, F_ext=F_ext_raw)
            data.ctrl[:] = tau_cmd[:model.nu]
            mujoco.mj_step(model, data)

            if has_site:
                ee_p = data.site_xpos[0].copy()
            else:
                robot.update(q)
                ee_p = robot.get_pose()[0]
            if i % TRAIL_INTERVAL == 0:
                trail_actual.append(ee_p)
                if len(trail_actual) > TRAIL_MAX:
                    trail_actual.pop(0)
            if i % 5 == 0:
                ngeom = min(len(trail_actual), viewer.user_scn.maxgeom)
                if ngeom > 1:
                    for j in range(ngeom):
                        pos = trail_actual[j]
                        mujoco.mjv_initGeom(
                            viewer.user_scn.geoms[j],
                            mujoco.mjtGeom.mjGEOM_SPHERE,
                            np.array([TRAIL_SIZE, 0, 0]),
                            pos, np.eye(3).flatten(), TRAIL_COLOR,
                        )
                    viewer.user_scn.ngeom = ngeom
                viewer.sync()
            i += 1

    if viewer:
        if not loop_active and stop_at_end:
            print("[Viewer] Simulation paused at final pose. "
                  "Close the viewer window to exit.")
            while viewer.is_running():
                viewer.sync()
                time.sleep(0.02)
        elif not loop_active:
            time.sleep(1)
        viewer.close()

    # ── 11. 方向解耦实验分析 (循环模式为可视化, 跳过定量报告) ──
    if experiment == 'decouple' and not decouple_loop:
        try:
            res = extract_decouple(log, decouple_settle, decouple_measure,
                                   inputs=decouple_inputs, use_xc=True)
            log['decouple'] = res
            print_decouple_report(res, 'GAC')
            save_path = os.path.join(PROJECT_DIR, 'figures', 'decouple',
                                     'gac_decouple.png')
            plot_coupling_matrix(res, 'GAC', save_path=save_path)
        except Exception as e:
            print(f"[GAC decouple] Analysis failed: {e}")

    return log, robot


# ====================================================================
# 4. 绘图与结果分析
# ====================================================================

def plot_results(log, save_path=None):
    """绘制跟踪性能图."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("[Plot] matplotlib not available, skipping")
        print(f"  Final pos_err: {log['pos_err'][-1]:.6f}")
        print(f"  Max pos_err: {np.max(log['pos_err']):.6f}")
        print(f"  Final rot_err: {log['rot_err'][-1]:.6f}")
        print(f"  Max rot_err: {np.max(log['rot_err']):.6f}")
        return

    t = log['t']
    nv = log['q'].shape[1]

    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle('GAC Control Verification - MuJoCo + Pinocchio', fontsize=14)

    # 1. 位置跟踪
    ax = axes[0, 0]
    ax.plot(t, log['p'][:, 0], 'b-', label='x', linewidth=1)
    ax.plot(t, log['p'][:, 1], 'g-', label='y', linewidth=1)
    ax.plot(t, log['p'][:, 2], 'r-', label='z', linewidth=1)
    ax.plot(t, log['pd'][:, 0], 'b--', label='x_des', linewidth=0.5, alpha=0.5)
    ax.plot(t, log['pd'][:, 1], 'g--', label='y_des', linewidth=0.5, alpha=0.5)
    ax.plot(t, log['pd'][:, 2], 'r--', label='z_des', linewidth=0.5, alpha=0.5)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Position [m]')
    ax.set_title('End-Effector Position Tracking')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # 2. 朝向误差
    ax = axes[0, 1]
    angle_err = np.zeros(len(t))
    for i in range(len(t)):
        if i % 50 == 0:
            R_err = log['R'][i].T @ log['Rd'][i]
            cos_angle = (np.trace(R_err) - 1) / 2
            angle_err[i] = np.arccos(np.clip(cos_angle, -1, 1))
    ax.plot(t, angle_err, 'm-', linewidth=1)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Orientation Error [rad]')
    ax.set_title('Orientation Tracking Error')
    ax.grid(True, alpha=0.3)

    # 3. 位置误差范数
    ax = axes[1, 0]
    ax.plot(t, log['pos_err'], 'b-', linewidth=1)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('||pos_err|| [m]')
    ax.set_title('Position Error Norm')
    ax.grid(True, alpha=0.3)

    # 4. 旋转误差范数
    ax = axes[1, 1]
    ax.plot(t, log['rot_err'], 'r-', linewidth=1)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('||rot_err||')
    ax.set_title('Rotation Error Norm')
    ax.grid(True, alpha=0.3)

    # 5. 关节力矩
    ax = axes[2, 0]
    for j in range(nv):
        ax.plot(t, log['tau'][:, j], label=f'τ_{j}', linewidth=0.8)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Torque [Nm]')
    ax.set_title('Joint Torques')
    ax.legend(fontsize=7, ncol=2)
    ax.grid(True, alpha=0.3)

    # 6. 3D 轨迹
    ax = fig.add_subplot(3, 2, 6, projection='3d')
    ax.plot(log['pd'][:, 0], log['pd'][:, 1], log['pd'][:, 2],
            'g--', label='desired', linewidth=1, alpha=0.7)
    ax.plot(log['p'][:, 0], log['p'][:, 1], log['p'][:, 2],
            'b-', label='actual', linewidth=1)
    ax.scatter(log['p'][0, 0], log['p'][0, 1], log['p'][0, 2],
               c='r', s=30, label='start')
    ax.scatter(log['p'][-1, 0], log['p'][-1, 1], log['p'][-1, 2],
               c='k', s=30, label='end')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.set_title('3D Trajectory')
    ax.legend(fontsize=8)

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"[Plot] Saved to {save_path}")

    print(f"\n{'='*50}")
    print("Simulation Summary:")
    print(f"  Mean pos_err: {np.mean(log['pos_err']):.6f} m")
    print(f"  Max  pos_err: {np.max(log['pos_err']):.6f} m")
    print(f"  Final pos_err: {log['pos_err'][-1]:.6f} m")
    print(f"  Mean rot_err: {np.mean(log['rot_err']):.6f}")
    print(f"  Max  rot_err: {np.max(log['rot_err']):.6f}")
    print(f"  Mean |tau|: {np.mean(np.linalg.norm(log['tau'], axis=1)):.2f} Nm")
    if 'f_ext' in log:
        print(f"  Max |F_ext|: {np.max(np.linalg.norm(log['f_ext'], axis=1)):.2f} N")
    print(f"{'='*50}")


# ====================================================================
# 5. 交叉验证: Pinocchio vs MuJoCo
# ====================================================================

def cross_validate_models(urdf_path, ee_frame='tool0', test_q=None):
    """比较 Pinocchio RobotModel 与 MuJoCo 的运动学/雅可比一致性."""
    import mujoco

    print(f"\n{'='*60}")
    print("Cross-Validation: Pinocchio RobotModel vs MuJoCo")
    print(f"{'='*60}")

    robot = RobotModel(urdf_path, ee_frame_name=ee_frame, verbose=False)
    nv = robot.nv

    xml_str = urdf_joints_to_mujoco_xml(urdf_path, ee_frame, debug=False)
    tmpf = tempfile.NamedTemporaryFile(suffix='.xml', mode='w', delete=False)
    tmpf.write(xml_str)
    tmpf.close()

    model = mujoco.MjModel.from_xml_path(tmpf.name)
    data = mujoco.MjData(model)
    os.unlink(tmpf.name)

    if test_q is None:
        np.random.seed(42)
        test_configs = [np.random.uniform(-1.0, 1.0, nv) for _ in range(5)]
    else:
        test_configs = [test_q]

    has_site = model.nsite > 0
    results = []
    for idx_q, q in enumerate(test_configs):
        data.qpos[:nv] = q.copy()
        data.qvel[:nv] = np.zeros(nv)
        mujoco.mj_forward(model, data)

        if has_site:
            mujoco_p = data.site_xpos[0].copy()
        else:
            mujoco_p = np.zeros(3)

        robot.update(q)
        robot_p, robot_R = robot.get_pose()
        pos_diff = np.linalg.norm(mujoco_p - robot_p) if has_site else -1

        if has_site and model.nv >= nv:
            jac_pos_mj = np.zeros((3, model.nv))
            jac_rot_mj = np.zeros((3, model.nv))
            mujoco.mj_jacSite(model, data, jac_pos_mj, jac_rot_mj, 0)
            J_mj = np.vstack([jac_pos_mj[:, :nv], jac_rot_mj[:, :nv]])
        else:
            J_mj = None

        J_geom = robot.get_jacobian()
        if J_mj is not None:
            jac_diff = np.linalg.norm(J_mj - J_geom) / max(
                1e-10, np.linalg.norm(J_mj))
        else:
            jac_diff = -1

        results.append({'q': q, 'pos_diff': pos_diff, 'jac_diff': jac_diff})
        print(f"  Test config {idx_q + 1}: q={np.round(q, 3)}")
        if has_site:
            print(f"    pos_diff = {pos_diff:.6e}")
        if J_mj is not None:
            print(f"    jac_diff = {jac_diff:.6e} (rel)")

    return results


# ====================================================================
# 6. 主入口
# ====================================================================

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='GAC Control Verification with MuJoCo')
    parser.add_argument('--robot', type=str, default='ur12e',
                        choices=['ur12e', 'ur3', 'franka'],
                        help='Robot to simulate')
    parser.add_argument('--task', type=str, default='regulation',
                        choices=['regulation', 'circle', 'line'],
                        help='Trajectory task')
    parser.add_argument('--max-time', type=float, default=5.0,
                        help='Simulation time [s]')
    parser.add_argument('--no-viewer', action='store_true',
                        help='Disable MuJoCo viewer (headless mode)')
    parser.add_argument('--save-plot', type=str, default=None,
                        help='Save plot to file')
    parser.add_argument('--cross-validate', action='store_true',
                        help='Run cross-validation only')
    parser.add_argument('--no-stop', action='store_true',
                        help='Do not pause at final pose')
    parser.add_argument('--no-loop', action='store_true',
                        help='Disable continuous task loop')

    # ── 外力模式 ──
    parser.add_argument('--force-mode', type=str, default='zero',
                        choices=['zero', 'constant', 'pulse',
                                 'spring', 'tangent'],
                        help='External force mode')
    parser.add_argument('--force-amplitude', type=float, nargs=6,
                        default=[10.0, 0, 0, 0, 0, 0],
                        help='Force amplitude [fx, fy, fz, tx, ty, tz]')
    parser.add_argument('--force-start', type=float, default=1.0,
                        help='Force start time (s) for pulse mode')
    parser.add_argument('--force-duration', type=float, default=0.5,
                        help='Force duration (s) for pulse mode')

    # ── 切向力参数 ──
    parser.add_argument('--tangent-circle-center', type=float, nargs=3,
                        default=[0.5, 0.0, 0.125],
                        help='Circle center [cx, cy, cz]')
    parser.add_argument('--tangent-radius', type=float, default=0.2,
                        help='Circle radius (m), default 0.2')
    parser.add_argument('--tangent-amplitude', type=float, default=10.0,
                        help='Tangential force amplitude (N)')
    parser.add_argument('--tangent-radial-stiffness', type=float, default=500.0,
                        help='Radial virtual spring stiffness (N/m), '
                             '0=no radial constraint. '
                             'When >0, D_d for K_d=0 directions is '
                             'auto-tuned for critical damping')

    # ── 初始位置 ──
    parser.add_argument('--init-pos', type=float, nargs=3, default=None,
                        help='Initial EE position [x, y, z] via IK '
                             '(default: home_q FK, tangent: circle start)')

    # ── 导纳参数 ──
    parser.add_argument('--M-d', type=float, nargs=6,
                        default=[10.0, 10.0, 10.0, 1.0, 1.0, 1.0],
                        help='Virtual mass [m, m, m, I, I, I]')
    parser.add_argument('--D-d', type=float, nargs=6,
                        default=None,
                        help='Virtual damping (default: critical)')
    parser.add_argument('--K-d', type=float, nargs=6,
                        default=[500.0, 500.0, 500.0, 50.0, 50.0, 50.0],
                        help='Virtual stiffness')
    parser.add_argument('--bandwidth', type=float, default=30.0,
                        help='GAC inner tracking bandwidth (rad/s)')
    parser.add_argument('--damping', type=float, default=1.0,
                        help='GAC inner tracking damping ratio')

    # ── 力交互实验 (实验二: 方向解耦) ──
    parser.add_argument('--experiment', type=str, default='none',
                        choices=['none', 'decouple'],
                        help='External force experiment '
                             '(decouple = 方向解耦, 7 块: 基线+6 输入)')
    parser.add_argument('--decouple-force', type=float, default=None,
                        help='解耦实验轴向力幅值 (N), 默认 10.0')
    parser.add_argument('--decouple-moment', type=float, default=None,
                        help='解耦实验力偶幅值 (Nm), 默认 1.0')
    parser.add_argument('--decouple-settle', type=float, default=None,
                        help='每块过渡时间 (s), 默认 2.0')
    parser.add_argument('--decouple-measure', type=float, default=None,
                        help='每块稳态测量时间 (s), 默认 1.0')
    parser.add_argument('--decouple-loop', action='store_true',
                        help='可视化循环模式: 动作间插复位间隙, 序列循环运行 '
                             '(幅值/时长取 experiments.decouple_loop 配置)')

    args = parser.parse_args()

    if args.D_d is not None:
        D_d_user = args.D_d
    else:
        # 临界阻尼
        D_d_user = [2 * np.sqrt(args.K_d[i] * args.M_d[i])
                    for i in range(6)]

    # 实验参数默认值 (来自 task_config.experiments;
    #   --decouple-loop 时取 decouple_loop 段: 更大位移/更长时长)
    _dec_cfg = (task_config.experiments.get('decouple_loop', {})
                if args.decouple_loop
                else task_config.experiments.get('decouple', {}))
    decouple_force = (args.decouple_force if args.decouple_force is not None
                      else _dec_cfg.get('force', 10.0))
    decouple_moment = (args.decouple_moment if args.decouple_moment is not None
                       else _dec_cfg.get('moment', 1.0))
    decouple_settle = (args.decouple_settle if args.decouple_settle is not None
                       else _dec_cfg.get('settle', 2.0))
    decouple_measure = (args.decouple_measure
                        if args.decouple_measure is not None
                        else _dec_cfg.get('measure', 1.0))
    decouple_loop = args.decouple_loop
    decouple_cycles = _dec_cfg.get('cycles', 2)

    # 选择 URDF
    if args.robot in ('ur12e', 'ur3'):
        cfg = get_robot_config(args.robot)
        urdf_file = cfg['urdf']
        ee_frame = cfg['ee_frame']
        home_q = cfg['home_q'].copy()
        link_to_mesh = cfg['link_to_mesh']
        mesh_subdir = cfg['mesh_subdir']
        torque_limits = cfg['full_torque_limits']
    else:  # franka
        urdf_file = 'franka_panda.urdf'
        ee_frame = 'panda_hand_tcp'
        home_q = np.array([0.0, -0.3, 0.0, -2.5, 0.0, 2.5, 0.0, 0.02, 0.02])
        link_to_mesh = None
        mesh_subdir = ''
        torque_limits = None

    urdf_path = os.path.join(URDF_DIR, urdf_file)
    if not os.path.exists(urdf_path):
        print(f"[ERROR] URDF not found: {urdf_path}")
        sys.exit(1)

    if args.cross_validate:
        cross_validate_models(urdf_path, ee_frame)
        sys.exit(0)

    # ── tangent 模式的默认初始位置 (圆弧起点, 舒适高度) ──
    if args.force_mode == 'tangent':
        # tangent 模式需要更长的仿真时间才能看到完整圆周运动
        if args.max_time <= 5.0:
            args.max_time = 30.0
            print(f"[GAC tangent] Auto-increasing max_time to {args.max_time}s "
                  f"for tangent mode (use --max-time to override)")
    if args.init_pos is not None:
        init_pos = args.init_pos
    elif args.force_mode == 'tangent':
        # 默认: 圆弧起点, z=0.25 (手肘弯曲, 有运动空间)
        c = np.array(args.tangent_circle_center, dtype=float)
        r = args.tangent_radius
        init_pos = [c[0] + r, c[1], 0.25]
        print(f"[GAC tangent] Default init_pos = {np.round(init_pos, 3)} "
              f"(on circle, z=0.25)")
    else:
        init_pos = None

    # 运行 GAC 验证
    do_loop = (not args.no_loop and args.task != 'regulation'
               and not args.no_viewer)
    log, robot = run_verification(
        urdf_file,
        task=args.task,
        show_viewer=not args.no_viewer,
        max_time=args.max_time,
        home_q=home_q,
        ee_frame=ee_frame,
        link_to_mesh=link_to_mesh,
        mesh_subdir=mesh_subdir,
        torque_limits=torque_limits,
        # GAC 参数
        # 实验模式下 dt_filter 与控制循环同步 (sim dt=0.001),
        # 否则滤波器以 2× 速率积分, 频响/暂态与设计不符.
        M_d=args.M_d, D_d=D_d_user, K_d=args.K_d,
        dt_filter=(0.001 if args.experiment != 'none' else 0.002),
        force_mode=args.force_mode,
        force_amplitude=args.force_amplitude,
        force_start=args.force_start,
        force_duration=args.force_duration,
        tangent_center=args.tangent_circle_center,
        tangent_radius=args.tangent_radius,
        tangent_amplitude=args.tangent_amplitude,
        tangent_radial_stiffness=args.tangent_radial_stiffness,
        init_pos=init_pos,
        bandwidth=args.bandwidth, damping=args.damping,
        verbose=True,
        stop_at_end=not args.no_stop,
        loop=do_loop,
        # 力交互实验 (实验二)
        experiment=args.experiment,
        decouple_force=decouple_force, decouple_moment=decouple_moment,
        decouple_settle=decouple_settle, decouple_measure=decouple_measure,
        decouple_loop=decouple_loop, decouple_cycles=decouple_cycles,
        # 按 --robot 匹配任务参数 (circle/line 几何)
        task_cfg=task_config.get_task_config(args.robot),
    )

    # 绘图
    plot_results(log, save_path=args.save_plot)

    print("\n✅ GAC verification complete!")
