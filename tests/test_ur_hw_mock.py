#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UR 机械臂硬件接口 Mock 测试 (支持 UR12e / UR3)
==================================================

验证 URHW/UR12eHW/UR3HW 是否正确调用了 ur_rtde 底层方法。
不需要连接真实机械臂。

用法:
  # 测试 UR12e (默认)
  python se3_control/scripts/test_ur_hw_mock.py

  # 测试 UR3
  python se3_control/scripts/test_ur_hw_mock.py --robot ur3

  # 向后兼容 (仍在原位置)
  python se3_control/scripts/test_ur12e_hw_mock.py
"""

import argparse
import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "se3_control"))

# ── 解析参数 ──────────────────────────────────────────────────
parser = argparse.ArgumentParser(description="UR 机械臂硬件接口 Mock 测试")
parser.add_argument("--robot", type=str, default="ur12e",
                    choices=["ur12e", "ur3"],
                    help="机器人类型 (默认: ur12e)")
args, _ = parser.parse_known_args()

# ── Mock ur_rtde (必须在导入任何硬件类之前完成) ────────────
mock_rtde_recv = MagicMock()
mock_rtde_ctrl = MagicMock()

# 先把 rtde_receive 和 rtde_control 注入 sys.modules
# 这样 hardware/ur_hw.py 中的 "import rtde_receive" 会拿到 mock
patcher_recv = patch.dict("sys.modules", {
    "rtde_receive": MagicMock(),
    "rtde_control": MagicMock(),
})
patcher_recv.start()

import rtde_receive, rtde_control
rtde_receive.RTDEReceiveInterface = MagicMock(return_value=mock_rtde_recv)
rtde_control.RTDEControlInterface = MagicMock(return_value=mock_rtde_ctrl)

# ── 现在安全导入 hardware 类 (不会触发真实 ur_rtde import) ──
from config.robot_configs import get_robot_config, get_hw_class

cfg = get_robot_config(args.robot)
RobotHW = get_hw_class(args.robot)
TORQUE_LIMITS = cfg['torque_limits']
JOINT_NAMES = cfg['joint_names']
N_JOINTS = len(JOINT_NAMES)
robot_label = cfg['name']


# ── 配置 mock 默认行为 ──────────────────────────────────────
q_default = [0.1, -1.2, 0.8, -1.5, 0.2, 0.0]
dq_default = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

mock_rtde_recv.isConnected.return_value = True
mock_rtde_recv.getActualQ.return_value = q_default[:N_JOINTS]
mock_rtde_recv.getActualQd.return_value = dq_default[:N_JOINTS]
mock_rtde_recv.getTimestamp.return_value = 1000.0
mock_rtde_recv.isEmergencyStopped.return_value = False
mock_rtde_recv.isProtectiveStopped.return_value = False
mock_rtde_recv.getSafetyStatusBits.return_value = 0
mock_rtde_recv.disconnect.return_value = None

mock_rtde_ctrl.isConnected.return_value = True
mock_rtde_ctrl.isProgramRunning.return_value = True
mock_rtde_ctrl.initPeriod.return_value = None
mock_rtde_ctrl.waitPeriod.return_value = 0.004
mock_rtde_ctrl.directTorque.return_value = None
mock_rtde_ctrl.disconnect.return_value = None


# ── 测试工具 ────────────────────────────────────────────────
passed, failed = 0, 0

def t(name, ok, detail=""):
    global passed, failed
    if ok:
        passed += 1
        print(f"  ✅ {name}")
    else:
        failed += 1
        print(f"  ❌ {name}  {detail}")


# ════════════════════════════════════════════════════════════
print("=" * 60)
print(f"  {robot_label}HW — ur_rtde 接口对接验证 (Mock)")
print(f"  机器人: {robot_label}  ({N_JOINTS} 关节)")
print("=" * 60)

# ─── Test 1: 初始化 ────────────────────────────────────────
print(f"\n[Test 1] initialize()")
robot = RobotHW(ip="192.168.1.100", verbose=False)
t("构造实例", isinstance(robot, RobotHW))

robot.initialize()
t("connected", robot.is_connected())
t("enabled", robot.is_enabled())
t("调用了 initPeriod()", mock_rtde_ctrl.initPeriod.called)
t("调用了 RTDEReceiveInterface()", rtde_receive.RTDEReceiveInterface.called)
t("调用了 RTDEControlInterface()", rtde_control.RTDEControlInterface.called)

# ─── Test 2: get_joint_states ──────────────────────────────
print(f"\n[Test 2] get_joint_states()")
q, dq = robot.get_joint_states()
t(f"返回 q 为 ndarray ({N_JOINTS},)",
  isinstance(q, np.ndarray) and q.shape == (N_JOINTS,))
t(f"返回 dq 为 ndarray ({N_JOINTS},)",
  isinstance(dq, np.ndarray) and dq.shape == (N_JOINTS,))
t("q 值 = getActualQ()",
  np.allclose(q, q_default[:N_JOINTS]))
t("调用了 getActualQ()", mock_rtde_recv.getActualQ.called)
t("调用了 getActualQd()", mock_rtde_recv.getActualQd.called)

# 通信失败 → 返回缓存
mock_rtde_recv.getActualQ.side_effect = Exception("timeout")
q2, _ = robot.get_joint_states()
t("通信失败返回缓存", np.allclose(q2, q_default[:N_JOINTS]))
mock_rtde_recv.getActualQ.side_effect = None  # 恢复

# ─── Test 3: set_joint_torques ─────────────────────────────
print(f"\n[Test 3] set_joint_torques()")

tau_test = np.array([10.0, -20.0, 5.0, -2.0, 1.0, 0.0])[:N_JOINTS]
robot.set_joint_torques(tau_test)
t("调用了 directTorque()", mock_rtde_ctrl.directTorque.called)
args_list = mock_rtde_ctrl.directTorque.call_args[0][0]
t(f"参数为 list 长度 {N_JOINTS}",
  isinstance(args_list, list) and len(args_list) == N_JOINTS)

# 力矩限幅
limits = robot.get_torque_limits()
big_tau = np.array([9999.0] * N_JOINTS)
robot.set_joint_torques(big_tau)
clipped = np.array(mock_rtde_ctrl.directTorque.call_args[0][0])
t(f"限幅生效 (max ≤ {limits[0]:.0f} Nm)",
  np.all(np.abs(clipped) <= limits + 1e-6))

# 急停状态
robot.emergency_stop()
from hardware.interface import HardwareSafetyError
try:
    robot.set_joint_torques(np.zeros(N_JOINTS))
    t("急停时发力矩应抛出异常", False)
except HardwareSafetyError:
    t("急停时抛出 HardwareSafetyError", True)
except Exception as e:
    t(f"急停时抛出 {type(e).__name__} 而非 HardwareSafetyError", False)
robot.reset_emergency_stop()

# ─── Test 4: 急停 ──────────────────────────────────────────
print(f"\n[Test 4] emergency_stop()")
robot.emergency_stop()
t("is_emergency_stopped = True", robot.is_emergency_stopped)

# 检查是否发了零力矩
zero_sent = any(
    np.allclose(c[0][0], [0.0] * N_JOINTS)
    for c in mock_rtde_ctrl.directTorque.call_args_list
)
t("急停时发出零力矩", zero_sent)

robot.reset_emergency_stop()
t("复位后 is_emergency_stopped = False", not robot.is_emergency_stopped)

# ─── Test 5: wait_next_cycle ───────────────────────────────
print(f"\n[Test 5] wait_next_cycle()")
dt = robot.wait_next_cycle()
t("返回 float ≈ 0.004", abs(dt - 0.004) < 1e-6)
t("调用了 waitPeriod()", mock_rtde_ctrl.waitPeriod.called)

# ─── Test 6: get_ft_sensor ─────────────────────────────────
print(f"\n[Test 6] get_ft_sensor()")
ft = robot.get_ft_sensor()
t("返回 ndarray (6,)", isinstance(ft, np.ndarray) and ft.shape == (6,))
t("无传感器返回零向量", np.allclose(ft, np.zeros(6)))

# ─── Test 7: get_error_state ───────────────────────────────
print(f"\n[Test 7] get_error_state()")
t("正常状态 = 0", robot.get_error_state() == 0)
mock_rtde_recv.isEmergencyStopped.return_value = True
t("急停状态 = 1", robot.get_error_state() == 1)
mock_rtde_recv.isEmergencyStopped.return_value = False

# ─── Test 8: shutdown ──────────────────────────────────────
print(f"\n[Test 8] shutdown()")
robot.shutdown()
t("connected = False", not robot.is_connected())
t("enabled = False", not robot.is_enabled())
t("调用了 ctrl.disconnect()", mock_rtde_ctrl.disconnect.called)
t("调用了 recv.disconnect()", mock_rtde_recv.disconnect.called)
t("幂等: 再次 shutdown 不报错", robot.shutdown() is None)

# ─── Test 9: torque_limits ─────────────────────────────────
print(f"\n[Test 9] torque_limits")
# 验证默认力矩限幅与配置一致
default_limits = robot.get_torque_limits()
t(f"默认限幅匹配 {robot_label} 配置",
  np.allclose(default_limits, TORQUE_LIMITS))

# set/get
new_limits = np.array([100.0, 100.0, 50.0, 20.0, 20.0, 20.0])[:N_JOINTS]
robot.set_torque_limits(new_limits)
t("set 后 get 一致", np.allclose(robot.get_torque_limits(), new_limits))

# ─── Test 10: get_joint_names ──────────────────────────────
print(f"\n[Test 10] get_joint_names()")
names = robot.get_joint_names()
t(f"返回 list 长度 {N_JOINTS}",
  isinstance(names, list) and len(names) == N_JOINTS)
t("关节名与 URDF 对齐", names == JOINT_NAMES)

# ─── 汇总 ──────────────────────────────────────────────────
print(f"\n{'=' * 60}")
print(f"  机器人: {robot_label}")
print(f"  结果: {passed} 通过 / {failed} 失败 / {passed + failed} 总计")
print(f"{'=' * 60}")

sys.exit(0 if failed == 0 else 1)
