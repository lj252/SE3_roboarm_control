# tests/ — 测试脚本使用说明

> 关联: [GAC_plan.md](../se3_control/docs/plan/GAC_plan.md) | [GIC_plan.md](../se3_control/docs/plan/GIC_plan.md)

---

## 目录

- [1. 运行环境](#1-运行环境)
- [2. 测试一览](#2-测试一览)
- [3. 控制器单元测试](#3-控制器单元测试)
- [4. 硬件接口 Mock 测试](#4-硬件接口-mock-测试)
- [5. 实机测试（需要连接真实机械臂）](#5-实机测试需要连接真实机械臂)
- [6. 测试执行策略](#6-测试执行策略)

---

## 1. 运行环境

```bash
conda activate roboarm
cd /media/lj252/Data/catkin_ws/roboarm_test/SE3_roboarm_control
```

所有测试脚本自动通过 `sys.path` 定位项目模块，可在任意工作目录下运行。

---

## 2. 测试一览

| 脚本 | 分类 | 需要 URDF | 需要真实机械臂 | 执行耗时 |
|---|---|---|---|---|
| `test_gac_controller.py` | 控制器单元测试 | ✅ UR12e/UR3 | ❌ | ~1s |
| `test_ur_hw_mock.py` | 硬件 Mock 测试 | ❌ | ❌ | ~1s |
| `test_ur12e_hw_mock.py` | 硬件 Mock 测试（向后兼容） | ❌ | ❌ | ~1s |
| `test_joint_states.py` | 实机硬件验证 | ❌ | ✅ UR12e/UR3 | ~5s |
| `test_gravity_comp.py` | 实机硬件验证 | ✅ UR12e/UR3 | ✅ UR12e/UR3 | ~15s |
| `test_regulation.py` | 实机控制验证 | ✅ UR12e/UR3 | ✅ UR12e/UR3 | ~15s |

---

## 3. 控制器单元测试

### test_gac_controller.py — GAC 导纳控制器全面测试

**35 项测试**，覆盖 GACFilter、SO(3) 指数映射、朝向修正、GACController、GIC↔GAC 互换性。

```bash
# 运行全部测试
python3 tests/test_gac_controller.py

# 运行指定测试类
python3 -m unittest tests.test_gac_controller.TestGACFilter -v
python3 -m unittest tests.test_gac_controller.TestGACControllerRobot -v
python3 -m unittest tests.test_gac_controller.TestGICGACInterchangeability -v
python3 -m unittest tests.test_gac_controller.TestSO3Exp -v
```

**测试架构**：

| 测试类 | 测试数 | 依赖 |
|---|---|---|
| `TestGACFilter` | 10 | 无（纯 NumPy） |
| `TestSO3Exp` | 5 | 无（纯 NumPy） |
| `TestCorrectOrientation` | 5 | 无（纯 NumPy） |
| `TestGACControllerRobot` | 9 | `se3_control/urdf/ur12e.urdf` |
| `TestGICGACInterchangeability` | 4 | `se3_control/urdf/ur12e.urdf` |

**跳过条件**：URDF 文件缺失时，依赖 URDF 的测试自动跳过。

---

## 4. 硬件接口 Mock 测试

### test_ur_hw_mock.py — UR 机械臂硬件接口 Mock 测试

**34 项测试**，通过 mock `ur_rtde` 库验证 `URHW` / `UR12eHW` / `UR3HW` 的接口行为，不需要连接真实机械臂。

```bash
# 测试 UR12e（默认）
python3 tests/test_ur_hw_mock.py

# 测试 UR3
python3 tests/test_ur_hw_mock.py --robot ur3

# 指定 IP（mock 模式下不真正连接）
python3 tests/test_ur_hw_mock.py --ip 192.168.1.11
```

**测试内容**：

| 测试组 | 验证内容 |
|---|---|
| Test 1 — initialize() | 构造、连接、使能、RTDE 初始化调用 |
| Test 2 — get_joint_states() | 形状、值正确、通信失败返回缓存 |
| Test 3 — set_joint_torques() | directTorque 调用、力矩限幅、急停异常 |
| Test 4 — emergency_stop() | 急停状态、零力矩发送、复位 |
| Test 5 — wait_next_cycle() | 周期等待、返回值 |
| Test 6 — get_ft_sensor() | 无传感器时返回零向量 |
| Test 7 — get_error_state() | 正常/急停状态码 |
| Test 8 — shutdown() | 断开连接、幂等性 |
| Test 9 — torque_limits | 默认限幅匹配配置、set/get |
| Test 10 — get_joint_names() | 关节名列表与 URDF 对齐 |

### test_ur12e_hw_mock.py — 向后兼容包装器

代理到 `test_ur_hw_mock.py --robot ur12e`，供旧脚本引用：

```bash
# 等价于 test_ur_hw_mock.py --robot ur12e
python3 tests/test_ur12e_hw_mock.py
```

---

## 5. 实机测试（需要连接真实机械臂）

> ⚠️ 实机测试前请确认：
> - 机械臂处于**远程控制模式**（Remote Control）
> - 急停按钮已释放
> - 臂处于安全位置，下方无遮挡
> - 手持示教器在旁边

### test_joint_states.py — 关节状态读取验证

**目标**：验证 `get_joint_states()` 方法的通信链路、数据完整性、读取频率和延迟。

```bash
# UR12e（默认 IP）
python3 tests/test_joint_states.py

# 指定 IP 和持续时间
python3 tests/test_joint_states.py --ip 192.168.1.100 --duration 10

# UR3
python3 tests/test_joint_states.py --robot ur3 --ip 192.168.1.11
```

**输出示例**：
```
=== 统计结果 ===
总采样数:     1250
平均频率:     249.8 Hz
关节位置标准差: [0.000012 0.000008 ...] rad
读取延迟:     平均 0.12 ms, 最大 0.45 ms
```

**安全等级**：🟢 低风险（只读，不下发力矩）

---

### test_gravity_comp.py — 力矩下发 + 重力补偿验证

**目标**：分两阶段验证力矩链路和重力补偿。

| Phase | 操作 | 预期行为 |
|---|---|---|
| **Phase A** | 下发零力矩，持续 3s | 臂在重力作用下自然下落 |
| **Phase B** | 下发重力补偿力矩，持续 10s | 臂保持静止，TCP 漂移 < 5mm |

```bash
# UR12e（默认）
python3 tests/test_gravity_comp.py

# UR3
python3 tests/test_gravity_comp.py --robot ur3 --ip 192.168.1.11

# 自定义测试时长
python3 tests/test_gravity_comp.py --phase-a-duration 5 --phase-b-duration 20
```

**输出示例**：
```
Phase B 统计结果:
  TCP 初始位置:  [0.412 -0.023  0.145]
  TCP 最终位置:  [0.413 -0.024  0.143]
  最终漂移:      1.23 mm ✅
  最大漂移:      2.45 mm
```

**安全等级**：🟡 中风险（下发力矩，但仅为重力补偿）
- Phase A 中臂会下落，确保下方无遮挡
- 随时准备按 Ctrl+C 或急停按钮

---

### test_regulation.py — 简化 GIC 位置保持验证

**目标**：运行简化 GIC 控制律，在 regulation 模式下保持末端位置，验证完整"读-算-发"闭环。

```bash
# UR12e，默认增益 Kp=50
python3 tests/test_regulation.py

# 自定义增益和时长
python3 tests/test_regulation.py --kp 100 100 100 --kr 80 80 80 --duration 30

# UR3
python3 tests/test_regulation.py --robot ur3 --ip 192.168.1.11 --kp 30 30 30
```

**控制律**（简化 GIC，无自适应惯性）：
```
F_body = -Kp·ep - KR·eR - Kd·Vb
tau_cmd = Jbᵀ·F_body + tau_bias
```

**命令行参数**：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--kp` | `50 50 50` | 位置刚度 [kx, ky, kz] (N/m) |
| `--kr` | `50 50 50` | 旋转刚度 [krx, kry, krz] (Nm/rad) |
| `--kd` | `10 10 10 5 5 5` | 阻尼系数 (Ns/m, Nms/rad) |
| `--duration` | `15.0` | 测试持续时间 (秒) |
| `--dt` | `0.004` | 控制周期 (秒, 250Hz) |
| `--ip` | 来自配置 | UR 控制箱 IP |
| `--urdf` | 来自配置 | URDF 路径覆盖 |
| `--ee-frame` | 来自配置 | EE frame 名称覆盖 |

**输出示例**：
```
测试统计 [UR12e]:
  平均频率:       248.5 Hz
  最终位置误差:   0.52 mm
  平均位置误差:   0.38 mm
  最大位置误差:   0.83 mm
  ✅ Step 4 (UR12e) 通过
     位置保持精度: ±0.8 mm
```

**安全等级**：🔴 中高风险（下发 GIC 控制力矩）
- 从极低 Kp=50 开始，逐步递增
- 异常时自动触发 `emergency_stop()`
- 力矩限幅为 URDF 限位的 50%

---

## 6. 测试执行策略

### 推荐执行顺序

```
1. test_gac_controller.py         # 控制器逻辑正确性
2. test_ur_hw_mock.py             # 硬件接口逻辑正确性
3. test_joint_states.py           # 实机通信链路（只读）
4. test_gravity_comp.py           # 实机力矩 + 重力补偿
5. test_regulation.py              # 实机闭环控制
```

### 回归测试

```bash
# 全部单元测试 + Mock 测试（不接机械臂）
conda run -n roboarm \
  python3 tests/test_gac_controller.py && \
  python3 tests/test_ur_hw_mock.py

# UR3 变体
conda run -n roboarm \
  python3 tests/test_ur_hw_mock.py --robot ur3
```

---

*文档创建日期: 2026-07-29*
