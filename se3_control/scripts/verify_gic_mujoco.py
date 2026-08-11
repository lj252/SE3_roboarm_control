#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SE(3) 几何阻抗控制的 MuJoCo + Pinocchio 联合验证
================================================

验证目标:
  1. RobotModel (Pinocchio) 的运动学/动力学计算正确性
  2. GIC 控制律与 RobotModel 集成的可行性
  3. MuJoCo 物理仿真中的控制稳定性

方法:
  - 读取 UR12e URDF, 生成对应的 MuJoCo 模型
  - RobotModel 提供控制律所需的运动学/动力学计算
  - MuJoCo 负责物理推演 (前向动力学)
  - 对比 GIC 控制下的轨迹跟踪性能

用法:
  conda activate roboarm
  cd /path/to/se3_control

  # 默认: 可视化 + UR12e + 调节任务
  python scripts/verify_gic_mujoco.py

  # 无头模式 (SSH/服务器)
  python scripts/verify_gic_mujoco.py --no-viewer

  # 指定任务与机器人
  python scripts/verify_gic_mujoco.py --robot ur12e --task regulation
  python scripts/verify_gic_mujoco.py --robot ur12e --task circle
  python scripts/verify_gic_mujoco.py --robot franka --task line

  # 保存结果图
  python scripts/verify_gic_mujoco.py --task circle --save-plot my_plot.png

  # 仿真结束后不暂停 (默认会停在最终位姿方便检查)
  python scripts/verify_gic_mujoco.py --no-stop

  # 画圆任务连续循环 (默认 viewer 开启时自动循环)
  python se3_control/scripts/verify_gic_mujoco.py --task circle

  # 关闭连续循环 (仿真结束后停止)
  python se3_control/scripts/verify_gic_mujoco.py --task circle --no-loop

  # 仅做模型交叉验证 (不运行控制)
  python scripts/verify_gic_mujoco.py --cross-validate
"""

import os
import sys
import time
import argparse
import xml.etree.ElementTree as ET
import numpy as np

# ─── 路径设置 ──────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)   # se3_control/
sys.path.insert(0, PROJECT_DIR)

# 导入 Pinocchio 封装
from robot_model.robot_model import RobotModel

# 导入 SE(3) 数学工具 (core.se3_math — 纯 NumPy, 自含, 无 GUFIC 依赖)
from core.se3_math import (
    vee_map, hat_map, rpy_to_rotmat, rotmat_to_xyz_euler, rotmat_slerp,
)
# 轨迹生成 (core.trajectory — 从 task_config 读取参数)
from core.trajectory import build_trajectory
# GIC 控制律 (core.gic_controller — 自适应带宽/阻尼)
from core.gic_controller import GICController

# 导入任务参数配置
sys.path.insert(0, PROJECT_DIR)
from config import task_config

# 导入机器人参数配置
from config.robot_configs import get_robot_config, get_mesh_dir

# 力交互实验分析库 (实验二: 方向解耦)
from core.experiment_analysis import (build_decouple_inputs,
                                      build_decouple_loop_inputs,
                                      extract_decouple,
                                      print_decouple_report,
                                      plot_coupling_matrix)

URDF_DIR = os.path.join(PROJECT_DIR, 'urdf')


# ====================================================================
# 1. URDF → MuJoCo XML 转换
# ====================================================================

def urdf_rpy_to_mjcf_euler(rpy):
    """URDF RPY → MJCF euler (eulerseq='xyz') 转换.

    URDF RPY: R = Rz(yaw) * Ry(pitch) * Rx(roll)   (XYZ 内旋)
    MJCF:     R = Rx(rx) * Ry(ry) * Rz(rz)            (XYZ 内旋)

    两者旋转矩阵相同, 但 euler 参数值不同 (分解顺序不同).
    此函数将 URDF 的 [roll, pitch, yaw] 转换为 MJCF 的 [rx, ry, rz].
    """
    R = rpy_to_rotmat(rpy)
    return rotmat_to_xyz_euler(R)


def parse_urdf_kinematics(urdf_path, debug=False):
    """解析 URDF, 提取运动学树 (仅主线关节链).

    返回:
      joints: list of dict, 每个关节包含:
        name, parent, child, type, origin_xyz, origin_rpy,
        axis_xyz, lower, upper, effort, velocity
      links: 所有 link 名称集合
      ee_link: 默认末端 link (最后的 child)
    """
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    # 收集所有 link
    links = {link.get('name') for link in root.findall('link')}

    # 收集所有 joint
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
            axis_xyz = np.array([1, 0, 0])  # default

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
            'name': name,
            'type': jtype,
            'parent': parent,
            'child': child,
            'origin_xyz': origin_xyz,
            'origin_rpy': origin_rpy,
            'axis_xyz': axis_xyz,
            'lower': lower,
            'upper': upper,
            'effort': effort,
            'velocity': velocity,
        })

    # 找到主线关节链 (revolute + continuous 关节按父-子关系排序)
    rev_joints = [j for j in joints if j['type'] in ('revolute', 'continuous')]

    # 构建完整父子关系图 (包含所有关节类型)
    all_children = set()
    child_to_parent = {}  # child link name → parent link name (通过任何关节)
    for j in joints:
        all_children.add(j['child'])
        child_to_parent[j['child']] = j['parent']

    all_parents = {j['parent'] for j in joints}
    root_link = next(iter(all_parents - all_children), 'world')

    # 从 root_link 出发, 沿固定关节链向下, 找到第一个 revolute 关节的 parent
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

    # current 现在是第一个 revolute 关节的 parent link
    # 对 revolute 关节按父-子链排序
    sorted_joints = []
    # 使用 parent→joint 映射, 而非 child→joint, 以避免无限循环
    parent_to_rev_joint = dict((j['parent'], j) for j in rev_joints)

    # 从 current 出发, 找到第一个 revolute 关节
    first_rev = None
    for j in rev_joints:
        if j['parent'] == current:
            first_rev = j
            break

    if first_rev is not None:
        sorted_joints.append(first_rev)
        current = first_rev['child']
        # 沿 revolute 关节链向下追踪
        # 查找关节的条件: 当前 body 是某个 joint 的 parent
        while current in parent_to_rev_joint:
            j = parent_to_rev_joint[current]
            sorted_joints.append(j)
            current = j['child']

    if debug:
        print(f"[URDF] root: {root_link}")
        print(f"[URDF] revolute chain: {[j['name'] for j in sorted_joints]}")
        print(f"[URDF] total links: {len(links)}")

    ee_link = sorted_joints[-1]['child'] if sorted_joints else None

    return sorted_joints, links, ee_link


def urdf_joints_to_mujoco_xml(urdf_path, ee_frame_name='tool0',
                               timestep=0.001, gravity=np.array([0, 0, -9.81]),
                               link_to_mesh=None, mesh_subdir='',
                               debug=False):
    """将 URDF 关节链转换为 MuJoCo XML 字符串.

    转换规则:
      - URDF 的 joint origin.xyz → MJCF 中 child body 的 pos
      - URDF 的 joint origin.rpy → MJCF 中 child body 的 euler
      - URDF 的 axis (在关节坐标系中) → MJCF 中 joint axis (在 body 坐标系中)
        因为 MJCF 的 body 坐标系与 URDF 的关节坐标系相同 (通过 origin 定义),
        所以 axis 不需要旋转转换.
    """
    joints, links, _ = parse_urdf_kinematics(urdf_path, debug)

    # 也解析惯性数据
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
                'mass': mass,
                'com': com_xyz,
                'com_rpy': com_rpy,
                'ixx': ixx, 'iyy': iyy, 'izz': izz,
                'ixy': ixy, 'ixz': ixz, 'iyz': iyz,
            }

    # 解析碰撞/视觉网格的 origin 偏移 (每个 link 对应 STL 的 pos/rpy)
    mesh_origins = {}
    for link_el in root.findall('link'):
        name = link_el.get('name')
        origin = None
        # 优先用 collision, 回退到 visual
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

    # 辅助: 将惯性张量从 URDF 惯性坐标系旋转到 body 坐标系
    def rotate_inertia_to_body(rpy, ixx, iyy, izz, ixy, ixz, iyz):
        """旋转惯性张量到 body 坐标系 (MuJoCo 不支持 rotated inertial frame + fullinertia)."""
        R = rpy_to_rotmat(rpy)
        # 构建 3x3 惯性矩阵
        I_inertial = np.array([[ixx, ixy, ixz],
                               [ixy, iyy, iyz],
                               [ixz, iyz, izz]])
        # 旋转到 body 系: I_body = R @ I_inertial @ R.T
        I_body = R @ I_inertial @ R.T
        return (I_body[0,0], I_body[1,1], I_body[2,2],
                I_body[0,1], I_body[0,2], I_body[1,2])

    # link → mesh 名称映射
    if link_to_mesh is None:
        # 默认: UR12e 网格映射 (向后兼容)
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
    # 视觉 RGBA
    MESH_RGBA = '0.737 0.737 0.768 1'

    # 将 mesh_origins 中的 rpy 字符串转为 MJCF euler 字符串
    mesh_eulers = {}
    for ln, mo in mesh_origins.items():
        rpy_arr = np.array([float(v) for v in mo['rpy'].split()])
        mjcf_euler = urdf_rpy_to_mjcf_euler(rpy_arr)
        mesh_eulers[ln] = mjcf_euler

    # 生成 MuJoCo XML
    indent = '  '

    lines = []
    lines.append('<?xml version="1.0"?>')
    lines.append(f'<mujoco model="urdf_converted">')
    mesh_dir = os.path.abspath(os.path.join(os.path.dirname(urdf_path), 'meshes', mesh_subdir))
    lines.append(f'{indent}<compiler angle="radian" coordinate="local" meshdir="{mesh_dir}/"/>')
    lines.append(f'{indent}<option timestep="{timestep}" gravity="{gravity[0]} {gravity[1]} {gravity[2]}" impratio="10"/>')

    # Defaults (无附加阻尼/惯量, 使用 URDF 原始动力学)
    lines.append(f'{indent}<default>')
    lines.append(f'{indent*2}<geom type="mesh" contype="0" conaffinity="0" rgba="0.737 0.737 0.768 1"/>')
    lines.append(f'{indent}</default>')

    # Asset: 网格文件
    lines.append(f'{indent}<asset>')
    for link_name, mesh_name in LINK_TO_MESH.items():
        lines.append(f'{indent*2}<mesh name="{mesh_name}" file="{mesh_name}.stl"/>')
    lines.append(f'{indent}</asset>')

    # World
    lines.append(f'{indent}<worldbody>')
    lines.append(f'{indent*2}<light directional="true" diffuse=".8 .8 .8" pos="0 0 5" dir="1.5 1 -2"/>')
    lines.append(f'{indent*2}<geom name="floor" pos="0 0 -0.5" size="2 2 0.5" type="plane" condim="1"/>')

    # 构建 body 树
    # 找到根 body (它的 parent 不在任何 revolute joint 的 child 中)
    if joints:
        # 根 body 名字 = 第一个 joint 的 parent
        root_body = joints[0]['parent']

        # ── 计算从 world 到 root_body 的累积固定变换 ──
        # 重新解析所有固定 joint, 构建 parent→joint 映射
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
                    xyz = np.array([float(v) for v in origin_el.get('xyz', '0 0 0').split()])
                    rpy = np.array([float(v) for v in origin_el.get('rpy', '0 0 0').split()])
                else:
                    xyz, rpy = np.zeros(3), np.zeros(3)
                all_fixed_joints[child] = {'parent': parent, 'xyz': xyz, 'rpy': rpy, 'name': name}

        # 从 root_body 向上追踪到 world, 累积变换
        root_pos = np.zeros(3)
        root_rpy = np.zeros(3)
        current_link = root_body
        # 由于所有变换都是 static, 向上追踪时, 当前 link 是某个 fixed joint 的 child
        # 我们需要找到以 current_link 为 child 的固定 joint, 然后累积其逆变换
        # 正向: world → joint → parent_link → joint → ... → root_body
        # 所以累积时, 我们反向追踪: root_body → parent → ... → world
        chain_rev = []  # 逆序的固定 joint 链
        while current_link in all_fixed_joints:
            j = all_fixed_joints[current_link]
            chain_rev.append(j)
            current_link = j['parent']
        # 现在按从 world 到 root_body 的顺序应用变换
        for j in reversed(chain_rev):
            # 累积: 新的 pos = R_prev @ j.xyz + pos_prev
            R_j = rpy_to_rotmat(j['rpy'])
            root_pos = R_j @ root_pos + j['xyz']
            # 对于累积 rpy: 新朝向 = R_j @ R_prev
            # 简单做法: 将当前 pos/rpy 表示为完整变换后累加
            # 这里简化: 只累加旋转 (通过矩阵乘法)
            R_current = rpy_to_rotmat(root_rpy)
            R_new = R_j @ R_current
            root_rpy = rotmat_to_xyz_euler(R_new)

        if debug:
            print(f"[MJCF] Root body '{root_body}' fixed transform: pos={root_pos}, rpy={root_rpy}")

        # 添加根 body (固定在 world 上, 考虑累积固定变换)
        root_pos_str = f'{root_pos[0]:.10f} {root_pos[1]:.10f} {root_pos[2]:.10f}'
        root_rpy_str = f'{root_rpy[0]:.10f} {root_rpy[1]:.10f} {root_rpy[2]:.10f}'
        lines.append(f'{indent*2}<body name="{root_body}" pos="{root_pos_str}" euler="{root_rpy_str}">')
        # 添加根 body 的惯性
        if root_body in inertia_data and inertia_data[root_body]['mass'] > 0:
            d = inertia_data[root_body]
            com_rpy = [float(v) for v in d['com_rpy'].split()]
            ixx_b, iyy_b, izz_b, ixy_b, ixz_b, iyz_b = rotate_inertia_to_body(
                com_rpy, d['ixx'], d['iyy'], d['izz'], d['ixy'], d['ixz'], d['iyz'])
            lines.append(f'{indent*3}<inertial mass="{d["mass"]}" pos="{d["com"]}" '
                         f'fullinertia="{ixx_b} {iyy_b} {izz_b} {ixy_b} {ixz_b} {iyz_b}"/>')

        # 根 body 视觉: STL 网格 (应用 URDF visual/collision origin 偏移)
        base_mesh = LINK_TO_MESH.get(root_body)
        if base_mesh and root_body in mesh_origins:
            mo = mesh_origins[root_body]
            me = mesh_eulers.get(root_body)
            if me is not None:
                euler_str = f'{me[0]:.10f} {me[1]:.10f} {me[2]:.10f}'
                lines.append(f'{indent*3}<geom type="mesh" mesh="{base_mesh}" '
                             f'pos="{mo["xyz"]}" euler="{euler_str}"/>')
            else:
                lines.append(f'{indent*3}<geom type="mesh" mesh="{base_mesh}" '
                             f'pos="{mo["xyz"]}"/>')

        # 递归添加子 body
        def add_body_chain(parent_name, depth=3):
            """从父 body 出发, 找到以它为 parent 的 revolute joint, 添加其 child body.

            body 标签在 depth 层, 内部内容 (inertial, joint, geom) 在 depth+1 层.
            """
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

                    pos_str = f'{xyz[0]:.10f} {xyz[1]:.10f} {xyz[2]:.10f}'
                    # URDF rpy → MJCF euler (eulerseq="xyz" 约定)
                    mjcf_euler = urdf_rpy_to_mjcf_euler(rpy)
                    euler_str = f'{mjcf_euler[0]:.10f} {mjcf_euler[1]:.10f} {mjcf_euler[2]:.10f}'
                    axis_str = f'{axis[0]:.10f} {axis[1]:.10f} {axis[2]:.10f}'
                    range_str = f'{lower:.10f} {upper:.10f}'

                    outer_indent = indent * depth       # body 标签
                    inner_indent = indent * (depth + 1)  # body 内部内容

                    lines.append(f'{outer_indent}<body name="{child_name}" pos="{pos_str}" euler="{euler_str}">')
                    # 惯性子元素 (body 内第一级)
                    if child_name in inertia_data and inertia_data[child_name]['mass'] > 0:
                        d = inertia_data[child_name]
                        com_rpy = [float(v) for v in d['com_rpy'].split()]
                        ixx_b, iyy_b, izz_b, ixy_b, ixz_b, iyz_b = rotate_inertia_to_body(
                            com_rpy, d['ixx'], d['iyy'], d['izz'], d['ixy'], d['ixz'], d['iyz'])
                        lines.append(f'{inner_indent}<inertial mass="{d["mass"]}" pos="{d["com"]}" '
                                     f'fullinertia="{ixx_b} {iyy_b} {izz_b} {ixy_b} {ixz_b} {iyz_b}"/>')
                    # 关节
                    lines.append(f'{inner_indent}<joint name="{jname}" type="hinge" axis="{axis_str}" range="{range_str}"/>')

                    # 视觉几何: STL 网格 (应用 URDF collision/visual origin 偏移)
                    mesh_name = LINK_TO_MESH.get(child_name)
                    if mesh_name and child_name in mesh_origins:
                        mo = mesh_origins[child_name]
                        me = mesh_eulers.get(child_name)
                        if me is not None:
                            euler_str = f'{me[0]:.10f} {me[1]:.10f} {me[2]:.10f}'
                            lines.append(f'{inner_indent}<geom type="mesh" mesh="{mesh_name}" '
                                         f'pos="{mo["xyz"]}" euler="{euler_str}"/>')
                        else:
                            lines.append(f'{inner_indent}<geom type="mesh" mesh="{mesh_name}" '
                                         f'pos="{mo["xyz"]}"/>')

                    # 子关节 / 末端 site
                    child_has_child = any(j2['parent'] == child_name for j2 in joints)
                    if child_has_child:
                        add_body_chain(child_name, depth + 1)
                    else:
                        # 末端 site 放到 ee_frame 处: 沿末端固定关节链 (wrist_3-flange→flange-tool0)
                        # 合成变换 (URDF 中 ee 常挂在最后一个 revolute 之后的固定关节上, 如 UR3 tool0 偏移).
                        # 与 Pinocchio FK(ee_frame) 对齐; 无固定链 (leaf==ee) 时保持原行为 (零回归).
                        ee_pos = np.zeros(3)
                        ee_euler = np.zeros(3)
                        if ee_frame_name != child_name:
                            # 从 ee 沿 fixed 关节链 (child→parent) 反向走到 leaf; 天然 innermost-first,
                            # 且按 child→parent 唯一路径走, 不会误入同父节点的旁支 (如 ur12e 的 ft_frame).
                            chain_rev = []
                            cur = ee_frame_name
                            while cur in all_fixed_joints:
                                jf = all_fixed_joints[cur]
                                chain_rev.append(jf)
                                cur = jf['parent']
                            if cur == child_name and chain_rev:
                                ee_R = np.eye(3)
                                for jf in chain_rev:  # innermost-first: flange-tool0 → wrist_3-flange
                                    R_j = rpy_to_rotmat(jf['rpy'])
                                    ee_pos = R_j @ ee_pos + jf['xyz']
                                    ee_R = R_j @ ee_R
                                ee_euler = rotmat_to_xyz_euler(ee_R)
                        ee_pos_str = f'{ee_pos[0]:.10f} {ee_pos[1]:.10f} {ee_pos[2]:.10f}'
                        ee_euler_str = f'{ee_euler[0]:.10f} {ee_euler[1]:.10f} {ee_euler[2]:.10f}'
                        lines.append(f'{inner_indent}<site name="end_effector" type="sphere" size="0.005" '
                                     f'pos="{ee_pos_str}" euler="{ee_euler_str}" rgba="1 0 0 1"/>')

                    lines.append(f'{outer_indent}</body>')

        add_body_chain(root_body)

        lines.append(f'{indent*2}</body>')

    # 关闭 worldbody
    lines.append(f'{indent}</worldbody>')

    # Actuators (力矩控制模式)
    lines.append(f'{indent}<actuator>')
    for j in joints:
        lines.append(f'{indent*2}<motor name="{j["name"]}_actuator" joint="{j["name"]}" gear="1" ctrllimited="false" ctrlrange="-1e6 1e6"/>')
    lines.append(f'{indent}</actuator>')

    lines.append('</mujoco>')

    return '\n'.join(lines)


# ────────────────────────────────────────────────────────────────────
# 以下内联实现均已迁移至 core/:
#   - GIC 控制律          → core.gic_controller.GICController
#   - 轨迹生成            → core.trajectory.build_trajectory
#   - 朝向 SLERP          → core.se3_math.rotmat_slerp
#   - SE(3) 数学          → core.se3_math (vee_map/hat_map/rpy/xyz_euler)
#   - 固定增益加载        → 已随固定增益路径移除 (见 GIC_plan Phase 2)
# 与旧内联版差异仅一处: 旧版 e_pos = Rᵀ·Rd·Rdᵀ·(p-pd) = Rᵀ·(p-pd) (RdᵀRd=I),
# 等价于 core 版的 e_pos = Rᵀ·(p-pd)。自适应带宽/阻尼从 task_config.controller 读取。
# ────────────────────────────────────────────────────────────────────


# ====================================================================
# 2. 仿真运行
# ====================================================================

def run_verification(robot_urdf, task='regulation', show_viewer=True,
                     max_time=5.0, home_q=None, ee_frame='tool0',
                     link_to_mesh=None, mesh_subdir='',
                     torque_limits=None,
                     verbose=True, stop_at_end=True, loop=False,
                     # ── 力交互实验 (实验二: 方向解耦) ──
                     experiment='none',
                     decouple_force=10.0, decouple_moment=1.0,
                     decouple_settle=2.0, decouple_measure=1.0,
                     decouple_loop=False, decouple_cycles=2,
                     task_cfg=None):
    """主验证循环.

    步骤:
      1. 生成 MuJoCo 模型 (从 URDF)
      2. 加载 RobotModel (Pinocchio, 从相同 URDF)
      3. 初始化 MuJoCo 仿真
      4. 初始化轨迹
      5. 运行 GIC 控制循环
      6. 记录并分析结果

    :param task_cfg: 按机器人合并的任务配置 (task_config.get_task_config(robot));
                     None 时用模块默认 (ur3). 保证轨迹参数与 --robot 一致.
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

    # 写入临时文件供 MuJoCo 加载 (from_xml_string 可能会有些问题)
    import tempfile
    tmpf = tempfile.NamedTemporaryFile(suffix='.xml', mode='w', delete=False)
    tmpf.write(xml_str)
    tmpf.close()

    if verbose:
        # 打印 MuJoCo 模型摘要
        # print("[MuJoCo XML]")
        # for line in xml_str.split('\n'):
        #     print(f"  {line}")
        print(f"[MuJoCo XML] written to {tmpf.name}")

    try:
        model = mujoco.MjModel.from_xml_path(tmpf.name)
        data = mujoco.MjData(model)
    except Exception as e:
        print(f"[ERROR] MuJoCo model load failed: {e}")
        # 打印 XML 以调试
        print("Generated XML:")
        print(xml_str)
        raise
    finally:
        os.unlink(tmpf.name)

    nv = model.nv  # MuJoCo 自由度

    if verbose:
        print(f"[MuJoCo] nq={model.nq}, nv={nv}, nsite={model.nsite}, njnt={model.njnt}")
        print(f"[MuJoCo] joint names: {[model.joint(i).name for i in range(model.njnt)]}")
        if model.nsite > 0:
            print(f"[MuJoCo] site names: {[model.site(i).name for i in range(model.nsite)]}")

    # ── 2. 加载 RobotModel ──
    robot = RobotModel(urdf_path, ee_frame_name=ee_frame,
                       robot_name=os.path.basename(robot_urdf), verbose=verbose)

    if home_q is None:
        # 默认舒适位形: 与 robot_configs 'ur12e' 的 home_q 一致
        # (EE 在 [0.50, 0, 0.50], 末端竖直朝下, 避开腕部奇异)
        home_q = np.array([-0.356, -1.498, 1.81, 1.259, 1.571, -0.124])[:robot.nv]

    if verbose:
        print(f"[Home] q = {home_q}")

    # ── 3. 初始化状态与轨迹 ──
    dt = model.opt.timestep

    # ── 实验配置 (方向解耦: 7 块 = 基线 + 6 输入, 物理外力施加;
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
                print(f"[GIC decouple-loop] Auto max_time = {max_time:.1f}s "
                      f"({len(decouple_inputs)} 子块 × {decouple_block:.1f}s "
                      f"× {decouple_cycles} 循环; 关闭 viewer 可提前停止)")
        else:
            decouple_inputs = build_decouple_inputs(force=decouple_force,
                                                    moment=decouple_moment)
            decouple_block = decouple_settle + decouple_measure
            decouple_total = len(decouple_inputs) * decouple_block
            if max_time <= 5.0:
                max_time = decouple_total
                print(f"[GIC decouple] Auto max_time = {max_time:.1f}s "
                      f"({len(decouple_inputs)} blocks × {decouple_block:.1f}s)")
    T = int(max_time / dt)

    # 根据任务类型初始化轨迹
    if task == 'regulation':
        # 先设 home_q 到 MuJoCo
        data.qpos[:nv] = home_q.copy()
        data.qvel[:nv] = np.zeros(nv)
        mujoco.mj_forward(model, data)
        robot.update(home_q)
        p_start, R_start = robot.get_pose()
        # 调节任务: 以起始位姿为期望位姿
        pd_t = lambda t: p_start
        Rd_t = lambda t: R_start
        dpd_t = lambda t: np.zeros(3)
        dRd_t = lambda t: np.zeros((3, 3))
        ddpd_t = lambda t: np.zeros(3)
        ddRd_t = lambda t: np.zeros((3, 3))
    else:
        # 动态任务: 从 config 读取参数生成轨迹
        # (直接使用 config 坐标, 不做偏移)
        pd_t, Rd_t, dpd_t, dRd_t, ddpd_t, ddRd_t = build_trajectory(task, cfg=task_cfg)
        if verbose:
            print(f"[Trajectory] start  = {pd_t(0).ravel()}")
            print(f"[Trajectory] center = {getattr(task_cfg, task, {}).get('center', 'N/A')}")

        # ★ IK: 将机械臂摆到轨迹起点附近, 消除初始位置瞬态
        # 注意: IK 只用轨迹位置 + home 朝向 (因为轨迹朝向 Rd_default 与 home 差异大,
        #       会导致 IK 不收敛; 朝向误差由控制器平滑处理)
        pd0 = pd_t(0).ravel()
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
            print(f"[IK] p_ik    = {p_start}")
            print(f"[IK] pd(0)   = {pd0}")
            print(f"[IK] pos_err = {np.linalg.norm(p_start - pd0):.6e}")
            print(f"[IK] rot_err = {np.linalg.norm(R_start - R_home):.6e}")

    # ── 4. 朝向渐进混合 (非 regulation 任务) ──
    # 避免初始朝向误差 (R_home vs Rd(0)) 导致力矩饱和和跟踪发散
    BLEND_DURATION = 0.4  # 0.4 秒内从 home 朝向过渡到轨迹朝向
    is_dynamic_task = (task != 'regulation')
    if is_dynamic_task:
        # 保存 IK 后的实际朝向, 作为混合起点
        _, R_home_ik = robot.get_pose()
        if verbose:
            Rd0_des = Rd_t(0).ravel().reshape(3, 3)
            init_rot_err = 0.5 * np.linalg.norm(
                np.cross(R_home_ik[:, 0], Rd0_des[:, 0])
                + np.cross(R_home_ik[:, 1], Rd0_des[:, 1])
                + np.cross(R_home_ik[:, 2], Rd0_des[:, 2]))
            print(f"[Blend] Initial orientation error: {init_rot_err:.4f} rad")
            print(f"[Blend] Blending over {BLEND_DURATION}s to avoid torque saturation")

    if verbose:
        print(f"[Trajectory] task={task}")
        print(f"[Verify] Desired p0 = {pd_t(0)}")

    # ── 5. 控制器 ──
    controller = GICController(
        robot,
        bandwidth=task_cfg.controller.get('bandwidth', 30.0),
        damping=task_cfg.controller.get('damping', 1.0),
        torque_limits=torque_limits,
    )

    if verbose:
        # 验证正运动学一致性 (使用 MuJoCo 中的实际状态)
        q_actual = data.qpos[:nv].copy()
        robot.update(q_actual)
        model_p, _ = robot.get_pose()
        mujoco_p = data.site_xpos[0].copy() if model.nsite > 0 else np.zeros(3)
        print(f"[FK] MuJoCo  EE: {mujoco_p}")
        print(f"[FK] Pinocchio EE: {model_p}")
        print(f"[FK] pos_diff: {np.linalg.norm(mujoco_p - model_p):.6e}")

    # ── 5. 记录 ──
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
        'f_ext': np.zeros((T, 6)),   # 物理施加外力 (世界系)
    }

    # ── 6. Viewer ──
    viewer = None
    # 从 config 读取轨迹可视化参数
    trail_cfg = task_cfg.trail
    TRAIL_INTERVAL = trail_cfg.get('interval', 8)
    TRAIL_MAX = trail_cfg.get('max_points', 1200)
    TRAIL_SIZE = trail_cfg.get('sphere_size', 0.006)
    TRAIL_COLOR = np.array(trail_cfg.get('color', [1.0, 0.2, 0.2, 0.85]), dtype=float)
    trail_actual = []   # actual EE trajectory trail
    if show_viewer:
        try:
            from mujoco.viewer import launch_passive
            viewer = launch_passive(model, data)
            time.sleep(0.5)
        except Exception as e:
            print(f"[Viewer] Failed to launch: {e}")
            print("[Viewer] Run with --no-viewer to suppress this warning.")
            show_viewer = False

    # ── 7. 主循环 ──
    if verbose:
        print(f"\n{'='*60}")
        print(f"Running GIC simulation: task={task}, T={T} steps ({max_time}s)")
        print(f"{'='*60}")

    t0 = time.time()
    for i in range(T):
        t = i * dt

        # 关闭 viewer 立即停止仿真 (所有模式通用)
        if viewer is not None and not viewer.is_running():
            if verbose:
                print(f"[Viewer] closed — stopping simulation early "
                      f"(t={t:.1f}s).")
            break

        # 期望轨迹
        pd = pd_t(t).ravel()
        Rd_des = Rd_t(t).reshape((3, 3))
        # 朝向渐进混合: 前 BLEND_DURATION 秒从 home 朝向平滑过渡到期望朝向
        if is_dynamic_task and t < BLEND_DURATION:
            alpha = t / BLEND_DURATION
            Rd = rotmat_slerp(R_home_ik, Rd_des, alpha)
        else:
            Rd = Rd_des
        # 由于 Rd 被修改, dRd 也需对应调整: 混合期间使用衰减后的轨迹 dRd
        blend_factor = min(1.0, t / BLEND_DURATION) if is_dynamic_task else 1.0
        dpd = dpd_t(t).ravel()
        dRd = dRd_t(t).reshape((3, 3)) * blend_factor
        ddpd = ddpd_t(t).ravel()
        ddRd = ddRd_t(t).reshape((3, 3)) * blend_factor

        # 期望体速度
        vd = Rd.T @ dpd.reshape((-1, 1))
        wd = vee_map(Rd.T @ dRd)
        dvd = (Rd.T @ ddpd.reshape((-1, 1))
               - hat_map(wd) @ Rd.T @ dpd.reshape((-1, 1)))
        dwd = vee_map(Rd.T @ ddRd - hat_map(wd) @ Rd.T @ dRd)

        # 读取当前 MuJoCo 状态
        q = data.qpos[:nv].copy()
        dq = data.qvel[:nv].copy()

        # ── 物理外力施加 (实验二: 方向解耦) — GIC 被动响应, 不读力 ──
        if experiment == 'decouple':
            k_block = int(t // decouple_block)
            if decouple_loop:
                k_block = k_block % len(decouple_inputs)   # 循环模式: 取模
            elif k_block >= len(decouple_inputs):
                k_block = len(decouple_inputs) - 1
            F_world = decouple_inputs[k_block]
            if model.nsite > 0:
                # 世界系恒力/力偶施加到末端 body COM (物理作用, 控制器不知情)
                data.xfrc_applied[ee_body_id, :] = F_world
            log['f_ext'][i] = F_world
        else:
            log['f_ext'][i] = np.zeros(6)

        # 用 RobotModel 计算 GIC 控制力矩 (被动阻抗, Fe_raw=None)
        tau_cmd = controller.compute(q, dq, pd, Rd, vd, wd, dvd, dwd)

        # 应用力矩到 MuJoCo
        data.ctrl[:] = tau_cmd[:model.nu]

        # 步进 MuJoCo 物理仿真
        mujoco.mj_step(model, data)

        # 记录
        log['t'][i] = t
        log['q'][i] = q
        log['dq'][i] = dq

        # 从 MuJoCo 读取当前位姿 (而不是从 RobotModel, 因为 MuJoCo 是物理)
        if model.nsite > 0:
            site_p = data.site_xpos[0].copy()
            site_R = data.site_xmat[0].copy().reshape((3, 3))
        else:
            # fallback: 从 RobotModel 读
            robot.update(q)
            site_p, site_R = robot.get_pose()

        log['p'][i] = site_p
        log['pd'][i] = pd
        log['R'][i] = site_R
        log['Rd'][i] = Rd
        log['tau'][i] = tau_cmd

        # 误差
        ep = site_p - pd
        eR = -0.5 * (np.cross(site_R[:, 0], Rd[:, 0])
                     + np.cross(site_R[:, 1], Rd[:, 1])
                     + np.cross(site_R[:, 2], Rd[:, 2]))
        log['pos_err'][i] = np.linalg.norm(ep)
        log['rot_err'][i] = np.linalg.norm(eR)

        # Viewer and trajectory trail
        if viewer:
            # accumulate trail points (sliding window)
            if i % TRAIL_INTERVAL == 0:
                trail_actual.append(site_p.copy())
                if len(trail_actual) > TRAIL_MAX:
                    trail_actual.pop(0)
            # draw trail and sync
            if i % 5 == 0:
                ngeom = min(len(trail_actual), viewer.user_scn.maxgeom)
                if ngeom > 1:
                    for j in range(ngeom):
                        pos = trail_actual[j]
                        mujoco.mjv_initGeom(
                            viewer.user_scn.geoms[j],
                            mujoco.mjtGeom.mjGEOM_SPHERE,
                            np.array([TRAIL_SIZE, 0, 0]),
                            pos,
                            np.eye(3).flatten(),
                            TRAIL_COLOR,
                        )
                    viewer.user_scn.ngeom = ngeom
                viewer.sync()

        # 进度
        if verbose and (i % 500 == 0 or i == T - 1):
            print(f"  t={t:.3f}s | pos_err={log['pos_err'][i]:.6f} | "
                  f"rot_err={log['rot_err'][i]:.6f} | "
                  f"tau_norm={np.linalg.norm(tau_cmd):.2f}")

    t_elapsed = time.time() - t0
    if verbose:
        print(f"\nSimulation finished in {t_elapsed:.2f}s "
              f"({(T / t_elapsed):.0f} Hz)")

    # ── 8. 连续循环 (仅在有 viewer 时有效) ──
    loop_active = loop and show_viewer and viewer is not None
    if loop_active:
        t_cont = T * dt  # 从主循环结束时间继续
        i = T
        if verbose:
            print(f"[Loop] Continuous mode: task '{task}' keeps running. Close viewer to stop.")
        while viewer.is_running():
            t = i * dt

            # 同上的控制逻辑
            pd = pd_t(t).ravel()
            Rd = Rd_t(t).reshape((3, 3))
            dpd = dpd_t(t).ravel()
            dRd = dRd_t(t).reshape((3, 3))
            ddpd = ddpd_t(t).ravel()
            ddRd = ddRd_t(t).reshape((3, 3))

            vd = Rd.T @ dpd.reshape((-1, 1))
            wd = vee_map(Rd.T @ dRd)
            dvd = (Rd.T @ ddpd.reshape((-1, 1))
                   - hat_map(wd) @ Rd.T @ dpd.reshape((-1, 1)))
            dwd = vee_map(Rd.T @ ddRd - hat_map(wd) @ Rd.T @ dRd)

            q = data.qpos[:nv].copy()
            dq = data.qvel[:nv].copy()
            tau_cmd = controller.compute(q, dq, pd, Rd, vd, wd, dvd, dwd)
            data.ctrl[:] = tau_cmd[:model.nu]
            mujoco.mj_step(model, data)

            # 读取 EE 位置并更新轨迹 trail
            if model.nsite > 0:
                ee_p = data.site_xpos[0].copy()
            else:
                robot.update(q)
                ee_p = robot.get_pose()[0]
            if i % TRAIL_INTERVAL == 0:
                trail_actual.append(ee_p)
                if len(trail_actual) > TRAIL_MAX:
                    trail_actual.pop(0)  # 滑动窗口: 移除最旧的点

            if i % 5 == 0:
                # 绘制 trail
                ngeom = min(len(trail_actual), viewer.user_scn.maxgeom)
                if ngeom > 1:
                    for j in range(ngeom):
                        pos = trail_actual[j]
                        mujoco.mjv_initGeom(
                            viewer.user_scn.geoms[j],
                            mujoco.mjtGeom.mjGEOM_SPHERE,
                            np.array([TRAIL_SIZE, 0, 0]),
                            pos,
                            np.eye(3).flatten(),
                            TRAIL_COLOR,
                        )
                    viewer.user_scn.ngeom = ngeom
                viewer.sync()
            i += 1
            # 不休眠: Python int 不会溢出, 轨迹函数(如 sin)支持大数值
            # 持续运行的轨迹点会: 在末端坐标系不断更新, trail 覆盖旧点

    if viewer:
        if not loop_active and stop_at_end:
            print("[Viewer] Simulation paused at final pose. Close the viewer window to exit.")
            while viewer.is_running():
                viewer.sync()
                time.sleep(0.02)
        elif not loop_active:
            time.sleep(1)
        viewer.close()

    # ── 9. 方向解耦实验分析 (循环模式为可视化, 跳过定量报告) ──
    if experiment == 'decouple' and not decouple_loop:
        try:
            res = extract_decouple(log, decouple_settle, decouple_measure,
                                   inputs=decouple_inputs)
            log['decouple'] = res
            print_decouple_report(res, 'GIC')
            save_path = os.path.join(PROJECT_DIR, 'figures', 'decouple',
                                     'gic_decouple.png')
            plot_coupling_matrix(res, 'GIC', save_path=save_path)
        except Exception as e:
            print(f"[GIC decouple] Analysis failed: {e}")

    return log, robot


# ====================================================================
# 3. 绘图与结果分析
# ====================================================================

def plot_results(log, save_path=None):
    """绘制跟踪性能图."""
    try:
        import matplotlib
        matplotlib.use('Agg')  # 无头模式
        import matplotlib.pyplot as plt
    except ImportError:
        print("[Plot] matplotlib not available, skipping")
        # 简单打印数值
        print(f"  Final pos_err: {log['pos_err'][-1]:.6f}")
        print(f"  Max pos_err: {np.max(log['pos_err']):.6f}")
        print(f"  Final rot_err: {log['rot_err'][-1]:.6f}")
        print(f"  Max rot_err: {np.max(log['rot_err']):.6f}")
        return

    t = log['t']
    nv = log['q'].shape[1]

    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle('GIC Control Verification - MuJoCo + Pinocchio', fontsize=14)

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

    # 2. 朝向误差 (用旋转角度表示)
    ax = axes[0, 1]
    # 简化的朝向误差: 从 R 和 Rd 计算角度
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
    ax = axes[2, 1]
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
    print(f"{'='*50}")

    # plt.show()  # 在无头模式不显示


# ====================================================================
# 4. 对比验证: Pinocchio vs MuJoCo 雅可比/惯性
# ====================================================================

def cross_validate_models(urdf_path, ee_frame='tool0', test_q=None):
    """在多个随机配置下, 比较 Pinocchio RobotModel 与 MuJoCo 的输出.

    对比项:
      - 正运动学 (末端位置)
      - 几何雅可比 (6×nv)
      - 惯性矩阵 M(q)
    """
    import mujoco

    print(f"\n{'='*60}")
    print("Cross-Validation: Pinocchio RobotModel vs MuJoCo")
    print(f"{'='*60}")

    # 加载 RobotModel
    robot = RobotModel(urdf_path, ee_frame_name=ee_frame, verbose=False)
    nv = robot.nv

    # 生成 MuJoCo 模型
    xml_str = urdf_joints_to_mujoco_xml(urdf_path, ee_frame, debug=False)
    import tempfile
    tmpf = tempfile.NamedTemporaryFile(suffix='.xml', mode='w', delete=False)
    tmpf.write(xml_str)
    tmpf.close()

    model = mujoco.MjModel.from_xml_path(tmpf.name)
    data = mujoco.MjData(model)
    os.unlink(tmpf.name)

    if test_q is None:
        # 生成几个随机测试配置
        np.random.seed(42)
        test_configs = []
        for _ in range(5):
            q = np.random.uniform(-1.0, 1.0, nv)
            test_configs.append(q)
    else:
        test_configs = [test_q]

    results = []
    has_site = model.nsite > 0
    if not has_site and test_q is None:
        print("[WARN] MuJoCo model has no sites. Will compute FK from qpos.")
    if has_site:
        site_id = 0

    for idx_q, q in enumerate(test_configs):
        # MuJoCo FK
        data.qpos[:nv] = q.copy()
        data.qvel[:nv] = np.zeros(nv)
        mujoco.mj_forward(model, data)

        if has_site:
            mujoco_p = data.site_xpos[0].copy()
        else:
            # 用 Pinocchio 的 FK 作为参考
            mujoco_p = np.zeros(3)
            print(f"  [WARN] No MuJoCo site, using Pinocchio FK for config {idx_q}")

        # RobotModel FK
        robot.update(q)
        robot_p, robot_R = robot.get_pose()

        # 位置误差
        pos_diff = np.linalg.norm(mujoco_p - robot_p) if has_site else -1

        # MuJoCo 雅可比 (需要 site)
        if has_site and model.nv >= nv:
            jac_pos_mj = np.zeros((3, model.nv))
            jac_rot_mj = np.zeros((3, model.nv))
            mujoco.mj_jacSite(model, data, jac_pos_mj, jac_rot_mj, site_id)
            J_mj = np.vstack([jac_pos_mj[:, :nv], jac_rot_mj[:, :nv]])
        else:
            J_mj = None

        # RobotModel 雅可比
        J_geom = robot.get_jacobian()

        if J_mj is not None:
            jac_diff = np.linalg.norm(J_mj - J_geom) / max(1e-10, np.linalg.norm(J_mj))
        else:
            jac_diff = -1

        results.append({
            'q': q,
            'pos_diff': pos_diff,
            'jac_diff': jac_diff,
        })

        print(f"  Test config {idx_q + 1}: q={np.round(q, 3)}")
        if has_site:
            print(f"    pos_diff = {pos_diff:.6e}")
        if J_mj is not None:
            print(f"    jac_diff = {jac_diff:.6e} (rel)")
            print(f"    ||J_mj||  = {np.linalg.norm(J_mj):.4f}")
            print(f"    ||J_pin|| = {np.linalg.norm(J_geom):.4f}")

    return results


# ====================================================================
# 5. 主入口
# ====================================================================

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='GIC Control Verification with MuJoCo')
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
                        help='Do not pause at final pose (default: pause)')
    parser.add_argument('--no-loop', action='store_true',
                        help='Disable continuous task loop (default: loop for circle/line)')

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

    # 选择 URDF — 从 robot_configs 加载 UR 系列机器人参数
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

    # 运行 GIC 验证
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
        verbose=True,
        stop_at_end=not args.no_stop,
        loop=do_loop,
        # 力交互实验 (实验二)
        experiment=args.experiment,
        decouple_force=decouple_force, decouple_moment=decouple_moment,
        decouple_settle=decouple_settle, decouple_measure=decouple_measure,
        decouple_loop=decouple_loop, decouple_cycles=decouple_cycles,
        # 按 --robot 匹配任务参数 (circle/line 几何与控制器)
        task_cfg=task_config.get_task_config(args.robot),
    )

    # 绘图
    plot_results(log, save_path=args.save_plot)

    print("\n✅ Verification complete!")
