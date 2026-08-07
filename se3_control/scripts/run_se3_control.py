#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SE(3) 几何控制在 MuJoCo 仿真中的主入口
=========================================

基于已抽离的 core/ 模块（se3_math / trajectory / gic_controller）运行
MuJoCo 物理仿真验证。支持 regulation / circle / line 任务与多机器人。

用法:
  conda activate roboarm
  cd se3_control

  # 默认: 可视化 + UR12e + 调节任务
  python scripts/run_se3_control.py

  # 指定任务与机器人
  python scripts/run_se3_control.py --robot ur3 --task circle

  # 无头模式 (SSH/服务器)
  python scripts/run_se3_control.py --task circle --no-viewer

  # 保存结果图
  python scripts/run_se3_control.py --task circle --save-plot circle.png

  # 仅做模型交叉验证 (不运行控制)
  python scripts/run_se3_control.py --cross-validate

架构:
  run_se3_control.py  (仿真入口 — URDF→XML, MuJoCo 步进, 可视化, 记录)
       ↓ 使用
  core/trajectory.py       — 轨迹生成
  core/gic_controller.py   — GIC 控制律
  core/se3_math.py         — SE(3) 数学工具
       ↓ 使用
  robot_model/robot_model.py  (Pinocchio 封装)
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

# 导入核心模块
from core.se3_math import (
    vee_map, hat_map, rpy_to_rotmat, rotmat_to_xyz_euler,
    rotmat_slerp,
)
from core.trajectory import build_trajectory
from core.gic_controller import GICController

# 导入机器人模型
from robot_model.robot_model import RobotModel

# 导入配置
from config import task_config
from config.robot_configs import get_robot_config, get_mesh_dir

URDF_DIR = os.path.join(PROJECT_DIR, 'urdf')


# ====================================================================
# 1. URDF → MuJoCo XML 转换
# ====================================================================

def urdf_rpy_to_mjcf_euler(rpy):
    """URDF RPY → MJCF euler (eulerseq='xyz') 转换."""
    R = rpy_to_rotmat(rpy)
    return rotmat_to_xyz_euler(R)


def parse_urdf_kinematics(urdf_path, debug=False):
    """解析 URDF, 提取运动学树 (仅主线关节链).

    返回:
      joints: list of dict — 排序后的 revolute/continuous 关节
      links: 所有 link 名称集合
      ee_link: 默认末端 link
    """
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

    # 主线关节链 (revolute + continuous)
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
                               debug=False):
    """将 URDF 关节链转换为 MuJoCo XML 字符串."""
    joints, links, _ = parse_urdf_kinematics(urdf_path, debug)

    # 解析惯性数据
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    inertia_data = {}
    for link_el in root.findall('link'):
        name = link_el.get('name')
        inertial = link_el.find('inertial')
        if inertial is not None:
            mass = float(inertial.find('mass').get('value', 0))
            origin = inertial.find('origin')
            com_xyz = origin.get('xyz', '0 0 0') if origin is not None else '0 0 0'
            com_rpy = origin.get('rpy', '0 0 0') if origin is not None else '0 0 0'

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

    # 解析网格 origin
    mesh_origins = {}
    for link_el in root.findall('link'):
        name = link_el.get('name')
        origin = link_el.find('collision')
        if origin is None:
            origin = link_el.find('visual')
        if origin is not None:
            origin = origin.find('origin')
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

    LINK_TO_MESH = link_to_mesh or {
        'base_link_inertia': 'base_vis',
        'shoulder_link': 'shoulder_vis',
        'upper_arm_link': 'upperarm_vis',
        'forearm_link': 'forearm_vis',
        'wrist_1_link': 'wrist1_vis',
        'wrist_2_link': 'wrist2_vis',
        'wrist_3_link': 'wrist3_vis',
    }

    mesh_eulers = {}
    for ln, mo in mesh_origins.items():
        rpy_arr = np.array([float(v) for v in mo['rpy'].split()])
        mesh_eulers[ln] = urdf_rpy_to_mjcf_euler(rpy_arr)

    indent = '  '
    lines = []
    lines.append('<?xml version="1.0"?>')
    lines.append(f'<mujoco model="urdf_converted">')
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
        lines.append(f'{indent*2}<mesh name="{mesh_name}" file="{mesh_name}.stl"/>')
    lines.append(f'{indent}</asset>')

    lines.append(f'{indent}<worldbody>')
    lines.append(f'{indent*2}<light directional="true" diffuse=".8 .8 .8" '
                 f'pos="0 0 5" dir="1.5 1 -2"/>')
    lines.append(f'{indent*2}<geom name="floor" pos="0 0 -0.5" '
                 f'size="2 2 0.5" type="plane" condim="1"/>')

    if joints:
        root_body = joints[0]['parent']
        tree_fixed = ET.parse(urdf_path)
        root_fixed = tree_fixed.getroot()
        all_fixed_joints = {}
        for joint_el in root_fixed.findall('joint'):
            if joint_el.get('type') == 'fixed':
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
                    'parent': parent, 'xyz': xyz, 'rpy': rpy,
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
            ixx_b, iyy_b, izz_b, ixy_b, ixz_b, iyz_b = rotate_inertia_to_body(
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
                euler_str = f'{me[0]:.10f} {me[1]:.10f} {me[2]:.10f}'
                lines.append(f'{indent*3}<geom type="mesh" mesh="{base_mesh}" '
                             f'pos="{mo["xyz"]}" euler="{euler_str}"/>')
            else:
                lines.append(f'{indent*3}<geom type="mesh" mesh="{base_mesh}" '
                             f'pos="{mo["xyz"]}"/>')

        def add_body_chain(parent_name, depth=3):
            nonlocal lines
            for j in joints:
                if j['parent'] != parent_name:
                    continue
                child_name = j['child']
                pos_str = (f'{j["origin_xyz"][0]:.10f} '
                           f'{j["origin_xyz"][1]:.10f} '
                           f'{j["origin_xyz"][2]:.10f}')
                mjcf_euler = urdf_rpy_to_mjcf_euler(j['origin_rpy'])
                euler_str = (f'{mjcf_euler[0]:.10f} '
                             f'{mjcf_euler[1]:.10f} '
                             f'{mjcf_euler[2]:.10f}')
                axis_str = (f'{j["axis_xyz"][0]:.10f} '
                            f'{j["axis_xyz"][1]:.10f} '
                            f'{j["axis_xyz"][2]:.10f}')
                range_str = f'{j["lower"]:.10f} {j["upper"]:.10f}'

                outer_indent = indent * depth
                inner_indent = indent * (depth + 1)

                lines.append(f'{outer_indent}<body name="{child_name}" '
                             f'pos="{pos_str}" euler="{euler_str}">')

                if child_name in inertia_data and inertia_data[child_name]['mass'] > 0:
                    d = inertia_data[child_name]
                    com_rpy_v = [float(v) for v in d['com_rpy'].split()]
                    ixx_b, iyy_b, izz_b, ixy_b, ixz_b, iyz_b = \
                        rotate_inertia_to_body(
                            com_rpy_v, d['ixx'], d['iyy'], d['izz'],
                            d['ixy'], d['ixz'], d['iyz'])
                    lines.append(f'{inner_indent}<inertial mass="{d["mass"]}" '
                                 f'pos="{d["com"]}" '
                                 f'fullinertia="{ixx_b} {iyy_b} {izz_b} '
                                 f'{ixy_b} {ixz_b} {iyz_b}"/>')

                lines.append(f'{inner_indent}<joint name="{j["name"]}" '
                             f'type="hinge" axis="{axis_str}" '
                             f'range="{range_str}"/>')

                mesh_name = LINK_TO_MESH.get(child_name)
                if mesh_name and child_name in mesh_origins:
                    mo = mesh_origins[child_name]
                    me = mesh_eulers.get(child_name)
                    if me is not None:
                        me_str = (f'{me[0]:.10f} {me[1]:.10f} {me[2]:.10f}')
                        lines.append(
                            f'{inner_indent}<geom type="mesh" '
                            f'mesh="{mesh_name}" pos="{mo["xyz"]}" '
                            f'euler="{me_str}"/>')
                    else:
                        lines.append(
                            f'{inner_indent}<geom type="mesh" '
                            f'mesh="{mesh_name}" pos="{mo["xyz"]}"/>')

                if any(j2['parent'] == child_name for j2 in joints):
                    add_body_chain(child_name, depth + 1)
                else:
                    lines.append(
                        f'{inner_indent}<site name="end_effector" '
                        f'type="sphere" size="0.005" pos="0 0 0" '
                        f'rgba="1 0 0 1"/>')

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
    lines.append('</mujoco>')

    return '\n'.join(lines)


# ====================================================================
# 2. 绘图与结果分析
# ====================================================================

def plot_results(log, save_path=None):
    """绘制跟踪性能图."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print(f"[Plot] matplotlib not available")
        print(f"  Final pos_err: {log['pos_err'][-1]:.6f}")
        print(f"  Max  pos_err: {np.max(log['pos_err']):.6f}")
        print(f"  Final rot_err: {log['rot_err'][-1]:.6f}")
        print(f"  Max  rot_err: {np.max(log['rot_err']):.6f}")
        return

    t = log['t']
    nv = log['q'].shape[1]

    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle('GIC Control - MuJoCo Simulation', fontsize=14)

    # 1. 位置跟踪
    ax = axes[0, 0]
    ax.plot(t, log['p'][:, 0], 'b-', label='x', lw=1)
    ax.plot(t, log['p'][:, 1], 'g-', label='y', lw=1)
    ax.plot(t, log['p'][:, 2], 'r-', label='z', lw=1)
    ax.plot(t, log['pd'][:, 0], 'b--', label='x_des', lw=0.5, alpha=0.5)
    ax.plot(t, log['pd'][:, 1], 'g--', label='y_des', lw=0.5, alpha=0.5)
    ax.plot(t, log['pd'][:, 2], 'r--', label='z_des', lw=0.5, alpha=0.5)
    ax.set_xlabel('Time [s]'); ax.set_ylabel('Position [m]')
    ax.set_title('End-Effector Position Tracking')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # 2. 朝向误差
    ax = axes[0, 1]
    angle_err = np.zeros(len(t))
    for i in range(len(t)):
        R_err = log['R'][i].T @ log['Rd'][i]
        cos_angle = (np.trace(R_err) - 1) / 2
        angle_err[i] = np.arccos(np.clip(cos_angle, -1, 1))
    ax.plot(t, angle_err, 'm-', lw=1)
    ax.set_xlabel('Time [s]'); ax.set_ylabel('Orientation Error [rad]')
    ax.set_title('Orientation Tracking Error')
    ax.grid(True, alpha=0.3)

    # 3. 位置误差范数
    ax = axes[1, 0]
    ax.plot(t, log['pos_err'], 'b-', lw=1)
    ax.set_xlabel('Time [s]'); ax.set_ylabel('||pos_err|| [m]')
    ax.set_title('Position Error Norm')
    ax.grid(True, alpha=0.3)

    # 4. 旋转误差范数
    ax = axes[1, 1]
    ax.plot(t, log['rot_err'], 'r-', lw=1)
    ax.set_xlabel('Time [s]'); ax.set_ylabel('||rot_err||')
    ax.set_title('Rotation Error Norm')
    ax.grid(True, alpha=0.3)

    # 5. 关节力矩
    ax = axes[2, 0]
    for j in range(nv):
        ax.plot(t, log['tau'][:, j], label=f'τ_{j}', lw=0.8)
    ax.set_xlabel('Time [s]'); ax.set_ylabel('Torque [Nm]')
    ax.set_title('Joint Torques')
    ax.legend(fontsize=7, ncol=2); ax.grid(True, alpha=0.3)

    # 6. 3D 轨迹
    ax = fig.add_subplot(3, 2, 6, projection='3d')
    ax.plot(log['pd'][:, 0], log['pd'][:, 1], log['pd'][:, 2],
            'g--', label='desired', lw=1, alpha=0.7)
    ax.plot(log['p'][:, 0], log['p'][:, 1], log['p'][:, 2],
            'b-', label='actual', lw=1)
    ax.scatter(log['p'][0, 0], log['p'][0, 1], log['p'][0, 2],
               c='r', s=30, label='start')
    ax.scatter(log['p'][-1, 0], log['p'][-1, 1], log['p'][-1, 2],
               c='k', s=30, label='end')
    ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]'); ax.set_zlabel('Z [m]')
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


# ====================================================================
# 3. 主仿真循环
# ====================================================================

def run_simulation(robot_urdf, task='regulation', show_viewer=True,
                   max_time=5.0, home_q=None, ee_frame='tool0',
                   link_to_mesh=None, mesh_subdir='',
                   torque_limits=None, bandwidth=None, damping=None,
                   verbose=True, stop_at_end=True, loop=False):
    """GIC 控制 MuJoCo 仿真主循环.

    步骤:
      1. 从 URDF 生成 MuJoCo 模型
      2. 加载 RobotModel (Pinocchio, 从相同 URDF)
      3. 初始化仿真与轨迹
      4. 运行 GIC 控制循环
      5. 记录并分析结果
    """
    import mujoco

    # ── 1. 生成 MuJoCo 模型 ──
    urdf_path = os.path.join(URDF_DIR, robot_urdf)
    if not os.path.exists(urdf_path):
        urdf_path = robot_urdf
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"Cannot find URDF: {urdf_path}")

    xml_str = urdf_joints_to_mujoco_xml(
        urdf_path, ee_frame,
        link_to_mesh=link_to_mesh,
        mesh_subdir=mesh_subdir,
        debug=verbose,
    )

    import tempfile
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
        print(f"[MuJoCo] nq={model.nq}, nv={nv}, nsite={model.nsite}")

    # ── 2. 加载 RobotModel ──
    robot = RobotModel(urdf_path, ee_frame_name=ee_frame,
                       robot_name=os.path.basename(robot_urdf),
                       verbose=verbose)

    if home_q is None:
        home_q = np.array([0.0, -1.2, 0.5, -0.8, 0.3, 0.5])[:robot.nv]

    if verbose:
        print(f"[Home] q = {home_q}")

    # ── 3. 初始化轨迹 ──
    dt = model.opt.timestep
    T = int(max_time / dt)

    if task == 'regulation':
        data.qpos[:nv] = home_q.copy()
        data.qvel[:nv] = np.zeros(nv)
        mujoco.mj_forward(model, data)
        robot.update(home_q)
        p_start, R_start = robot.get_pose()
        # 调节任务: 以起始位姿为期望
        pd_t = lambda t: p_start
        Rd_t = lambda t: R_start
        dpd_t = lambda t: np.zeros(3)
        dRd_t = lambda t: np.zeros((3, 3))
        ddpd_t = lambda t: np.zeros(3)
        ddRd_t = lambda t: np.zeros((3, 3))
    else:
        # 动态任务: 从 task_config 读取参数
        funcs = build_trajectory(task, cfg=task_config)
        pd_t, Rd_t = funcs.pd_t, funcs.Rd_t
        dpd_t, dRd_t = funcs.dpd_t, funcs.dRd_t
        ddpd_t, ddRd_t = funcs.ddpd_t, funcs.ddRd_t

        if verbose:
            print(f"[Trajectory] start  = {pd_t(0).ravel()}")

        # ★ IK: 将机械臂摆到轨迹起点
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
            print(f"[IK] pos_err = {np.linalg.norm(p_start - pd0):.6e}")

    # ── 4. 朝向渐进混合 ──
    BLEND_DURATION = 0.4
    is_dynamic_task = (task != 'regulation')
    if is_dynamic_task:
        _, R_home_ik = robot.get_pose()
        if verbose:
            Rd0_des = Rd_t(0).ravel().reshape(3, 3)
            init_rot_err = 0.5 * np.linalg.norm(
                np.cross(R_home_ik[:, 0], Rd0_des[:, 0])
                + np.cross(R_home_ik[:, 1], Rd0_des[:, 1])
                + np.cross(R_home_ik[:, 2], Rd0_des[:, 2]))
            print(f"[Blend] Initial orientation error: {init_rot_err:.4f} rad")

    # ── 5. 控制器 ──
    if bandwidth is None:
        bandwidth = task_config.controller.get('bandwidth', 30.0)
    if damping is None:
        damping = task_config.controller.get('damping', 1.0)

    controller = GICController(
        robot,
        bandwidth=bandwidth,
        damping=damping,
        torque_limits=torque_limits,
    )

    if verbose:
        print(f"[GIC] bandwidth={bandwidth}, damping={damping}")
        # 验证正运动学一致性
        q_actual = data.qpos[:nv].copy()
        robot.update(q_actual)
        model_p, _ = robot.get_pose()
        mujoco_p = data.site_xpos[0].copy() if model.nsite > 0 else np.zeros(3)
        print(f"[FK] MuJoCo  EE: {mujoco_p}")
        print(f"[FK] Pinocchio EE: {model_p}")
        print(f"[FK] pos_diff: {np.linalg.norm(mujoco_p - model_p):.6e}")

    # ── 6. 记录 ──
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
    }

    # ── 7. Viewer ──
    viewer = None
    trail_cfg = task_config.trail
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
            show_viewer = False

    # ── 8. 主循环 ──
    if verbose:
        print(f"\n{'='*60}")
        print(f"Running GIC simulation: task={task}, T={T} steps ({max_time}s)")
        print(f"{'='*60}")

    t0 = time.time()
    for i in range(T):
        t = i * dt

        # 期望轨迹
        pd = pd_t(t).ravel()
        Rd_des = Rd_t(t).reshape((3, 3))

        # 朝向渐进混合 (前 BLEND_DURATION 秒)
        if is_dynamic_task and t < BLEND_DURATION:
            alpha = t / BLEND_DURATION
            Rd = rotmat_slerp(R_home_ik, Rd_des, alpha)
        else:
            Rd = Rd_des

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

        # 读取 MuJoCo 状态
        q = data.qpos[:nv].copy()
        dq = data.qvel[:nv].copy()

        # GIC 控制力矩
        tau_cmd = controller.compute(q, dq, pd, Rd, vd, wd, dvd, dwd)

        # 应用力矩 → MuJoCo 步进
        data.ctrl[:] = tau_cmd[:model.nu]
        mujoco.mj_step(model, data)

        # 记录
        log['t'][i] = t
        log['q'][i] = q
        log['dq'][i] = dq

        if model.nsite > 0:
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

        # 误差
        ep = site_p - pd
        eR = -0.5 * (np.cross(site_R[:, 0], Rd[:, 0])
                     + np.cross(site_R[:, 1], Rd[:, 1])
                     + np.cross(site_R[:, 2], Rd[:, 2]))
        log['pos_err'][i] = np.linalg.norm(ep)
        log['rot_err'][i] = np.linalg.norm(eR)

        # Viewer 轨迹绘制
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

        # 进度
        if verbose and (i % 500 == 0 or i == T - 1):
            print(f"  t={t:.3f}s | pos_err={log['pos_err'][i]:.6f} | "
                  f"rot_err={log['rot_err'][i]:.6f} | "
                  f"tau_norm={np.linalg.norm(tau_cmd):.2f}")

    t_elapsed = time.time() - t0
    if verbose:
        print(f"\nSimulation finished in {t_elapsed:.2f}s "
              f"({(T / t_elapsed):.0f} Hz)")

    # ── 9. 连续循环 (仅 viewer 开启时) ──
    loop_active = loop and show_viewer and viewer is not None
    if loop_active:
        i = T
        if verbose:
            print(f"[Loop] Continuous mode. Close viewer to stop.")
        while viewer.is_running():
            t = i * dt
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

            if model.nsite > 0:
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

    # ── 10. 清理 ──
    if viewer:
        if not loop_active and stop_at_end:
            print("[Viewer] Paused at final pose. Close window to exit.")
            while viewer.is_running():
                viewer.sync()
                time.sleep(0.02)
        elif not loop_active:
            time.sleep(1)
        viewer.close()

    return log, robot


# ====================================================================
# 4. 对比验证: Pinocchio vs MuJoCo
# ====================================================================

def cross_validate_models(urdf_path, ee_frame='tool0', test_q=None,
                           link_to_mesh=None, mesh_subdir=''):
    """在多个随机配置下比较 Pinocchio RobotModel 与 MuJoCo 的输出."""
    import mujoco

    print(f"\n{'='*60}")
    print("Cross-Validation: Pinocchio RobotModel vs MuJoCo")
    print(f"{'='*60}")

    robot = RobotModel(urdf_path, ee_frame_name=ee_frame, verbose=False)
    nv = robot.nv

    xml_str = urdf_joints_to_mujoco_xml(
        urdf_path, ee_frame, debug=False,
        link_to_mesh=link_to_mesh, mesh_subdir=mesh_subdir)
    import tempfile
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

    for idx_q, q in enumerate(test_configs):
        data.qpos[:nv] = q.copy()
        data.qvel[:nv] = np.zeros(nv)
        mujoco.mj_forward(model, data)

        mujoco_p = data.site_xpos[0].copy() if has_site else np.zeros(3)

        robot.update(q)
        robot_p, _ = robot.get_pose()

        pos_diff = np.linalg.norm(mujoco_p - robot_p) if has_site else -1

        if has_site and model.nv >= nv:
            jac_pos_mj = np.zeros((3, model.nv))
            jac_rot_mj = np.zeros((3, model.nv))
            mujoco.mj_jacSite(model, data, jac_pos_mj, jac_rot_mj, 0)
            J_mj = np.vstack([jac_pos_mj[:, :nv], jac_rot_mj[:, :nv]])
        else:
            J_mj = None

        J_geom = robot.get_jacobian()

        jac_diff = (np.linalg.norm(J_mj - J_geom) / max(1e-10, np.linalg.norm(J_mj))
                    if J_mj is not None else -1)

        print(f"  Test config {idx_q + 1}: q={np.round(q, 3)}")
        if has_site:
            print(f"    pos_diff = {pos_diff:.6e}")
        if J_mj is not None:
            print(f"    jac_diff = {jac_diff:.6e} (rel)")


# ====================================================================
# 5. 主入口
# ====================================================================

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='SE(3) GIC Control — MuJoCo Simulation')
    parser.add_argument('--robot', type=str, default='ur12e',
                        choices=['ur12e', 'ur3', 'franka'],
                        help='Robot to simulate')
    parser.add_argument('--task', type=str, default='regulation',
                        choices=['regulation', 'circle', 'line'],
                        help='Trajectory task')
    parser.add_argument('--max-time', type=float, default=5.0,
                        help='Simulation time [s]')
    parser.add_argument('--no-viewer', action='store_true',
                        help='Disable MuJoCo viewer')
    parser.add_argument('--save-plot', type=str, default=None,
                        help='Save plot to file')
    parser.add_argument('--cross-validate', action='store_true',
                        help='Run cross-validation only')
    parser.add_argument('--no-stop', action='store_true',
                        help='Do not pause at final pose')
    parser.add_argument('--no-loop', action='store_true',
                        help='Disable continuous task loop')
    parser.add_argument('--bandwidth', type=float, default=None,
                        help='GIC bandwidth (overrides task_config)')
    parser.add_argument('--damping', type=float, default=None,
                        help='GIC damping ratio (overrides task_config)')
    args = parser.parse_args()

    # ── 机器人配置 ──
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
        cross_validate_models(urdf_path, ee_frame,
                              link_to_mesh=link_to_mesh,
                              mesh_subdir=mesh_subdir)
        sys.exit(0)

    # ── 运行仿真 ──
    do_loop = (not args.no_loop and args.task != 'regulation'
               and not args.no_viewer)
    log, robot = run_simulation(
        urdf_file,
        task=args.task,
        show_viewer=not args.no_viewer,
        max_time=args.max_time,
        home_q=home_q,
        ee_frame=ee_frame,
        link_to_mesh=link_to_mesh,
        mesh_subdir=mesh_subdir,
        torque_limits=torque_limits,
        bandwidth=args.bandwidth,
        damping=args.damping,
        verbose=True,
        stop_at_end=not args.no_stop,
        loop=do_loop,
    )

    # ── 绘图 ──
    plot_results(log, save_path=args.save_plot)

    print("\n✅ Simulation complete!")
