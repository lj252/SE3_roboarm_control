# RobotHWInterface 与 ur_rtde 对接验证

> 关联文档: [interface_plan.md](../plan/interface_plan.md) | [interface_URtest_usages.md](../usages/interface_URtest_usages.md)
> 验证目标: UR12eHW / UR3HW ↔ ur_rtde 接口对接正确性
>
> **新功能**: 所有脚本支持 ``--robot ur12e|ur3`` 参数，自动切换机械臂配置。

---

## 概述

本项目的硬件抽象层分三层：

```
URHW / UR12eHW / UR3HW  (se3_control/hardware/)
    │  我们写的封装: get_joint_states(), set_joint_torques(), ...
    │  URHW (通用) ← UR12eHW (UR12e 子类)
    │               ← UR3HW   (UR3 子类)
    ↓
ur_rtde  (RTDE 协议)
    │  底层驱动: getActualQ(), directTorque(), waitPeriod(), ...
    ↓
UR 控制箱 → 物理机械臂 (UR12e / UR3)
```

验证这两层是否正确对接，分 **三个层次** 逐步确认：

| 层次 | 名称 | 需要真机 | 验证内容 |
|---|---|---|---|
| **Layer 1** | Mock 单元测试 | ❌ 不需要 | 我们的代码是否正确调用了 ur_rtde 方法 |
| **Layer 2** | 实机接口测试 | ✅ 需要 | 在真机上每个方法能否正确执行 |
| **Layer 3** | Round-trip 脉冲测试 | ✅ 需要 | 发力矩 → 读到运动响应，双向通路打通 |

---

## Layer 1: Mock 测试

### 目的

不连接真实机器人，通过模拟 ur_rtde 接口，验证 `UR12eHW` 在正确的时机以正确的参数调用了底层 ur_rtde 方法。

### 验证清单

| # | 测试项 | 验证点 |
|---|---|---|
| 1 | `initialize()` | 调用了 `RTDEReceiveInterface()`、`RTDEControlInterface()`、`initPeriod()` |
| 2 | `get_joint_states()` | 调用了 `getActualQ()`、`getActualQd()`，返回值形状 (6,) |
| 3 | `get_joint_states()` 异常 | 通信失败时返回上一帧缓存，不崩溃 |
| 4 | `set_joint_torques()` | 调用了 `directTorque()`，参数为 list 长度 6 |
| 5 | 力矩限幅 | 超过 `torque_limits` 的值被截断 |
| 6 | 急停禁止发力矩 | `emergency_stop()` 后调用 `set_joint_torques()` 抛出 `HardwareSafetyError` |
| 7 | 急停零力矩 | 急停时发出 `directTorque([0,0,0,0,0,0])` |
| 8 | `wait_next_cycle()` | 调用了 `waitPeriod()`，返回 ≈ 0.004 |
| 9 | `get_ft_sensor()` | 无传感器时返回零向量 (6,) |
| 10 | `get_error_state()` | 正常 = 0，急停 = 1 |
| 11 | `shutdown()` | 调用了 `disconnect()`，幂等 |

### 用法

```bash
conda activate roboarm
cd /media/lj252/Data/catkin_ws/roboarm_test/SE3_roboarm_control

# UR12e (默认)
python3 se3_control/scripts/test_ur_hw_mock.py --robot ur12e

# UR3
python3 se3_control/scripts/test_ur_hw_mock.py --robot ur3

# 向后兼容
python3 se3_control/scripts/test_ur12e_hw_mock.py
```

### 预期输出

```
============================================================
  UR12eHW — ur_rtde 接口对接验证
============================================================

[Test 1] initialize()
  ✅ 构造 UR12eHW 实例
  ✅ connected
  ✅ enabled
  ✅ 调用了 initPeriod()
  ✅ 调用了 RTDEReceiveInterface()
  ✅ 调用了 RTDEControlInterface()

... (全部 33 项)

============================================================
  结果: 33 通过 / 0 失败 / 33 总计
============================================================
```

> **全部 33 项通过**，说明 `UR12eHW` 对 ur_rtde 的调用在代码层面完全正确。
>
> 如果测试失败，说明 `ur12e_hw.py` 的某处逻辑错误——最常见的原因是方法签名变更或异常处理遗漏。

---

## Layer 2: 实机接口测试

### 目的

在真实 UR12e 上逐项验证 `UR12eHW` 的每个方法在实际通信链路中能正确执行。

### 前置条件

- [ ] UR12e 已开机，教示器在 **Remote Control** 模式
- [ ] 电脑与 UR12e 在同一网段，`ping` 通
- [ ] 急停按钮可触及，臂处于安全位置
- [ ] 已完成 Layer 1 Mock 测试（全部通过）

### 验证流程

#### Step 2.1: 读状态验证

验证 `get_joint_states()` 能从真实机械臂读取到合理的关节数据。

```bash
# UR12e (默认)
python3 se3_control/scripts/test_joint_states.py --ip 192.168.1.100 --duration 5

# UR3
python3 se3_control/scripts/test_joint_states.py --robot ur3 --ip 192.168.1.101 --duration 5
```

**对接验证要点**:

| 观察项 | 含义 |
|---|---|
| `q` 值与教示器显示的关节角度一致 | `getActualQ()` 读到的数据正确到达 |
| `dq` ≈ 0（臂静止时） | `getActualQd()` 速度读数正常 |
| 通信频率 ≈ 250 Hz | RTDE 链路稳定，无丢帧 |

#### Step 2.2: 发力矩验证

验证 `set_joint_torques()` 的力矩指令能传递到 UR 电机。

```bash
# UR12e
python3 se3_control/scripts/test_gravity_comp.py --ip 192.168.1.100 --phase-a-duration 3

# UR3
python3 se3_control/scripts/test_gravity_comp.py --robot ur3 --ip 192.168.1.101 --phase-a-duration 3
```

**对接验证要点**:

| 现象 | 含义 |
|---|---|
| Phase A 中臂自然下落 | `directTorque(zeros)` 成功关闭了 UR 内部力矩，力矩指令到达电机 |
| 下落过程无卡顿/抖动 | RTDE 通信连续，无中断 |

#### Step 2.3: 重力补偿验证

验证 `get_joint_states()` → `RobotModel.get_bias_torque()` → `set_joint_torques()` 全链路。

```bash
# UR12e
python3 se3_control/scripts/test_gravity_comp.py --ip 192.168.1.100 --phase-b-duration 10

# UR3
python3 se3_control/scripts/test_gravity_comp.py --robot ur3 --ip 192.168.1.101 --phase-b-duration 10
```

**对接验证要点**:

| 现象 | 含义 |
|---|---|
| Phase B 中臂保持静止（漂移 < 5 mm） | **全双向链路打通**: 读 q → Pinocchio 算重力 → directTorque 发力矩 |
| 力矩值稳定无跳变 | RTDE 通信连续无中断 |

---

## Layer 3: Round-trip 脉冲测试

### 目的

最直观的验证——你发力矩，关节运动，你读到运动响应。证明**双向 RTDE 通信完全打通**。

### 原理

```
你发出力矩 τ = [5, 0, 0, 0, 0, 0] Nm
      ↓ (directTorque)
UR12e shoulder_pan 关节收到 5 Nm
      ↓ (物理)
关节产生微小转动 Δq ≈ 0.1~0.5°
      ↓ (getActualQ)
你读到的 q 发生了变化
      ↓
✅ 双向通路验证通过
```

### 用法

```bash
conda activate roboarm
cd /media/lj252/Data/catkin_ws/roboarm_test/SE3_roboarm_control

python3 -c "
from se3_control.hardware.ur3_hw import UR3HW
import numpy as np, time

with UR3HW('192.168.1.11', verbose=False) as robot:
    # 读初始状态
    q0, _ = robot.get_joint_states()
    print(f'初始 q:          {np.round(q0, 4)}')
    print(f'shoulder_pan:    {np.rad2deg(q0[0]):.2f}°')

    # 发一个小力矩脉冲 (shoulder_pan, 5 Nm)
    tau = np.array([5.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    print(f'发力矩:          {tau} Nm')
    robot.set_joint_torques(tau)
    time.sleep(0.5)

    # 再读状态
    q1, dq1 = robot.get_joint_states()
    delta = q1 - q0
    print(f'脉冲后 q:         {np.round(q1, 4)}')
    print(f'shoulder_pan 变化: {np.rad2deg(delta[0]):.4f}°')
    print(f'脉冲后 dq:        {np.round(dq1, 6)}')
    print()
    if abs(delta[0]) > 1e-4:
        print('✅ 双向通路打通:')
        print('   发力矩 → 关节运动 → 读到的 q 变化')
    else:
        print('⚠️  未检测到运动 — 可能力矩太小或 UR 内部位置环抵消了')
"
```

### 预期输出

```
初始 q:          [ 0.05   -1.20    0.80   -1.50    0.20    0.00]
shoulder_pan:    2.86°
发力矩:          [5. 0. 0. 0. 0. 0.] Nm
脉冲后 q:         [ 0.0530 -1.2000  0.8000 -1.5000  0.2000  0.0000]
shoulder_pan 变化: 0.17°
脉冲后 dq:        [ 0.0032  0.0000  0.0000  0.0000  0.0000  0.0000]

✅ 双向通路打通:
   发力矩 → 关节运动 → 读到的 q 变化
```

### 扩展：多关节脉冲

如果想验证全部 6 个关节的通路，可以依次给每个关节发短脉冲：

```bash
python3 -c "
from se3_control.hardware.ur3_hw import UR3HW
import numpy as np, time

JOINT_NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow',
               'wrist_1', 'wrist_2', 'wrist_3']

with UR3HW('192.168.1.11', verbose=False) as robot:
    q0, _ = robot.get_joint_states()
    print(f'初始: {np.round(np.rad2deg(q0), 2)}°')

    for j in range(6):
        tau = np.zeros(6)
        tau[j] = 3.0  # 3 Nm 脉冲, 所有关节
        robot.set_joint_torques(tau)
        time.sleep(0.3)

    q1, _ = robot.get_joint_states()
    delta_deg = np.rad2deg(q1 - q0)
    for j in range(6):
        status = '✅' if abs(delta_deg[j]) > 0.01 else '⚠️'
        print(f'  {status} {JOINT_NAMES[j]}: {delta_deg[j]:.3f}°')
"
```

---

## 三层次验证总结

```
Layer 1: Mock 测试 (34 项)
   无需真机, 验证代码逻辑
   运行 test_ur_hw_mock.py --robot ur12e|ur3
       ↓ 全部通过
Layer 2: 实机接口测试
   需要真机, 验证通信链路
   运行 test_joint_states.py + test_gravity_comp.py --robot ur12e|ur3
       ↓ 全部通过
Layer 3: Round-trip 脉冲测试
   需要真机, 最直观的"对接上了"的证据
   运行脉冲脚本, 观察关节响应
       ↓ 全部通过
结论: URHW ↔ ur_rtde 对接完全正确 ✅
```

---

## 故障排查

### Mock 测试失败

| 失败项 | 可能原因 |
|---|---|
| `construct UR12eHW` 失败 | `ur_rtde` 未安装？mock 未正确加载 |
| `directTorque` 未被调用 | `set_joint_torques()` 逻辑错误 |
| 限幅未生效 | `clip_torques()` 实现错误 |

### 实机测试失败

| 失败项 | 可能原因 |
|---|---|
| `initialize()` 连接失败 | IP 不对 / 教示器不在 Remote Control 模式 |
| `get_joint_states()` 超时 | 网络延迟 / RTDE 被其他客户端占用 |
| Phase A 臂不下落 | UR 内部重力补偿仍在运行 |
| Phase B 漂移过大 | URDF 惯性参数不准确 |

### Round-trip 测试失败

| 现象 | 可能原因 |
|---|---|
| shoulder_pan 完全不动 | 力矩太小 (< 3 Nm) 被内部位置环抵消 |
| 所有关节不动 | `directTorque` 未成功调用（检查返回错误） |
| 只有部分关节动 | 某些关节摩擦力 > 3 Nm |

---

*文档创建日期: 2026-07-26*
*关联脚本: test_ur12e_hw_mock.py, test_joint_states.py, test_gravity_comp.py*
