# RobotHWInterface 实机测试说明

> 关联文档: [interface_plan.md](./interface_plan.md) | [deploy_se3_to_hardware_plan.md](../../docs/deploy_se3_to_hardware_plan.md)
> 测试目标: UR12e / UR3 | 驱动库: ur_rtde
>
> **新功能**: 所有脚本支持 ``--robot ur12e|ur3`` 参数，自动切换配置。

---

## 0. 测试前准备

### 0.1 硬件检查清单

- [ ] UR12e 已开机，教示器无报错
- [ ] 教示器处于 **远程控制模式 (Remote Control)**
  - 路径: 右上角菜单 → `Remote Control` → 点击 `Connect`（如果未自动连接）
- [ ] 控制箱 IP 地址已知（默认 `192.168.1.100`）
- [ ] 电脑与 UR12e 在同一网段（e.g. `192.168.1.x`）
- [ ] 急停按钮可触及
- [ ] 臂下方及周围无人员/障碍物
- [ ] 臂处于安全位置（建议手动移动到工作空间中央，远离限位）

### 0.2 机械臂切换

所有测试脚本现在支持 ``--robot`` 参数选择机械臂类型：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--robot` | `ur12e` | 机械臂类型：`ur12e` 或 `ur3` |

切换机械臂时，以下参数自动从 ``config/robot_configs.py`` 加载：
- URDF 文件路径
- 默认 IP 地址
- 关节力矩限幅
- 末端执行器 frame 名称
- 关节名称列表

示例:
```bash
# UR12e (默认)
python3 se3_control/scripts/test_joint_states.py
python3 se3_control/scripts/test_joint_states.py --robot ur12e --ip 192.168.1.100

# UR3
python3 se3_control/scripts/test_joint_states.py --robot ur3 --ip 192.168.1.101
```

### 0.3 连接验证

```bash
# 确认网络连通
ping 192.168.1.100

# 确认 Conda 环境
conda activate roboarm

# 确认 ur_rtde 可用
python3 -c "import rtde_receive; import rtde_control; print('ur_rtde OK')"
```

### 0.4 测试流程总览

```
Step 2 (test_joint_states.py)   --robot ur12e|ur3
  └─ 只读模式: 验证通信链路和数据完整性
      ↓ 通过后
Step 3 (test_gravity_comp.py)   --robot ur12e|ur3
  └─ 发力矩模式: 验证力矩下发 + 重力补偿
      ↓ 通过后
Step 4 (test_regulation.py)     --robot ur12e|ur3
  └─ 闭环模式: 验证 GIC 位置保持控制
```

> **安全原则**: 每一步必须通过才能进入下一步。如果 Step 2 通信不稳定，不要进入 Step 3/4。

---

## 1. Step 2: 关节状态读取验证

### 1.1 目的

验证 RTDE 通信链路正常，关节位置/速度数据可稳定读取。

### 1.2 用法

```bash
cd /media/lj252/Data/catkin_ws/roboarm_test/SE3_roboarm_control
conda activate roboarm

# 最小用法（使用默认 IP 192.168.1.100，默认测试 5 秒）
python3 se3_control/scripts/test_joint_states.py

# 指定 IP 和测试时长
python3 se3_control/scripts/test_joint_states.py --ip 192.168.1.101 --duration 10
```

### 1.3 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--robot` | `ur12e` | 机械臂类型: `ur12e` / `ur3` |
| `--ip` | ur12e: `192.168.1.100`<br>ur3: `192.168.1.101` | UR 控制箱 IP（默认从配置加载） |
| `--duration` | `5.0` | 测试持续时间（秒） |
| `--dt` | `0.004` | 读取周期（秒），对应 250 Hz |

### 1.4 预期输出

```
[UR12e@192.168.1.100] UR12e @ 192.168.1.100 连接成功
  初始 q:  [0.05  -1.20  0.80  -1.50  0.10  0.00]
  初始 dq: [0.00  0.00  0.00  0.00  0.00  0.00]

  关节位置 q:  [0.05  -1.20  0.80  -1.50  0.10  0.00]
  关节速度 dq: [0.00  0.00  0.00  0.00  0.00  0.00]

=== 统计结果 ===
总采样数:     1250
实际耗时:     5.002 s
平均频率:     249.9 Hz

关节位置统计 (rad):
  均值: [0.05  -1.20  0.80  -1.50  0.10  0.00]
  标准差: [0.0001  0.0002  0.0001  0.0001  0.0000  0.0000]

关节速度统计 (rad/s):
  均值: [0.00  0.00  0.00  0.00  0.00  0.00]
  最大绝对值: [0.02  0.03  0.02  0.01  0.01  0.01]

Step 2 全部通过 ✅
```

### 1.5 通过标准

| 检查项 | 标准 | 失败处理 |
|---|---|---|
| 连接成功 | `is_connected() == True` | 检查 IP/网段/防火墙 |
| 数据完整性 | 无 NaN/Inf | 检查网线/RTDE 版本 |
| 位置范围 | 所有 `\|q\| < 2π` | 检查编码器 |
| 静止速度 | `std(dq) < 1.0 rad/s` | 检查机械刹车是否释放 |
| 通信频率 | `≥ 200 Hz` | 检查网络延迟（`ping`） |

---

## 2. Step 3: 力矩下发 + 重力补偿验证

### 2.1 目的

- 验证 `set_joint_torques()` 能正确下发力矩指令
- 验证 Pinocchio 计算的重力补偿能抵消重力，使臂保持静止

### 2.2 用法

```bash
cd /media/lj252/Data/catkin_ws/roboarm_test/SE3_roboarm_control
conda activate roboarm

# 默认参数
python3 se3_control/scripts/test_gravity_comp.py

# 指定 IP 和控制周期
python3 se3_control/scripts/test_gravity_comp.py --ip 192.168.1.100 --dt 0.004
```

### 2.3 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--robot` | `ur12e` | 机械臂类型: `ur12e` / `ur3` |
| `--ip` | ur12e: `192.168.1.100`<br>ur3: `192.168.1.101` | UR 控制箱 IP（默认从配置加载） |
| `--urdf` | 从配置自动加载 | URDF 文件路径（可手动覆盖） |
| `--ee-frame` | 从配置自动加载 | 末端 frame 名称（可手动覆盖） |
| `--phase-a-duration` | `3.0` | 零力矩阶段时长（秒） |
| `--phase-b-duration` | `10.0` | 重力补偿阶段时长（秒） |
| `--dt` | `0.004` | 控制周期（250 Hz） |

### 2.4 测试流程

```
Phase A: 下发零力矩 (3 秒)
  tau = [0, 0, 0, 0, 0, 0]
  → 臂在重力作用下缓慢下落（正常现象）
  → 验证力矩指令已到达电机

Phase B: 重力补偿 (10 秒)
  tau = tau_bias  (Pinocchio 计算)
  → 臂应基本保持静止
  → 验证重力补偿正确性
```

### 2.5 预期输出

```
Phase A: 下发零力矩 (3.0 秒)
  t=0.50s  位置: [0.500, 0.000, 0.600]  漂移: 0.0 mm
  t=1.50s  位置: [0.498, 0.001, 0.592]  漂移: 8.2 mm
  t=2.50s  位置: [0.496, 0.002, 0.581]  漂移: 19.5 mm
  → 臂在下降 ✅

Phase B: 重力补偿 (10.0 秒)
  t=0.50s  位置: [0.496, 0.002, 0.581]  漂移: 0.0 mm ✅
  t=2.00s  位置: [0.496, 0.002, 0.580]  漂移: 1.2 mm ✅
  t=5.00s  位置: [0.496, 0.002, 0.580]  漂移: 1.5 mm ✅
  t=10.0s  位置: [0.496, 0.002, 0.579]  漂移: 2.1 mm ✅

Phase B 统计结果:
  最终漂移:     2.1 mm ✅
  重力补偿均值: [0.00  -121.29  -39.30  0.00  0.00  0.00] Nm
  重力补偿标准差: [0.01  0.05  0.03  0.00  0.00  0.00] Nm

✅ Step 3 全部通过
```

### 2.6 通过标准

| 检查项 | 标准 | 失败处理 |
|---|---|---|
| Phase A 下落 | TCP 下降 > 5 mm | 检查 `directTorque` 模式是否正确 |
| Phase B 漂移 | TCP 漂移 < 5 mm | 检查 URDF 惯性参数 |
| 力矩稳定 | 标准差 < 1 Nm | 检查控制频率稳定性 |
| 无报错 | `get_error_state() == 0` | 查看教示器错误信息 |

### 2.7 安全说明

- **Phase A 中臂会自然下落** — 确保下方无遮挡
- 初始位置越低，下落距离越短。建议臂在较高位置开始测试
- 如果下落过快，立即按急停或 Ctrl+C
- 如果重力补偿后臂抖动，按 Ctrl+C 停止（Kd 过小或 Kp 过大）

---

## 3. Step 4: 简化 GIC 位置保持验证

### 3.1 目的

验证完整"读状态 → 正解 → 控制律 → 发力矩"闭环在 UR12e 上正确运转。

### 3.2 用法

```bash
cd /media/lj252/Data/catkin_ws/roboarm_test/SE3_roboarm_control
conda activate roboarm

# 极低增益 Kp=50 (默认, 安全)
python3 se3_control/scripts/test_regulation.py

# 手动调参
python3 se3_control/scripts/test_regulation.py \
  --kp 200 200 200 \
  --kr 150 150 150 \
  --kd 30 30 30 15 15 15 \
  --duration 30
```

### 3.3 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--robot` | `ur12e` | 机械臂类型: `ur12e` / `ur3` |
| `--ip` | ur12e: `192.168.1.100`<br>ur3: `192.168.1.101` | UR 控制箱 IP（默认从配置加载） |
| `--urdf` | 从配置自动加载 | URDF 文件路径（可手动覆盖） |
| `--ee-frame` | 从配置自动加载 | 末端 frame 名称（可手动覆盖） |
| `--duration` | `15.0` | 测试时长（秒） |
| `--dt` | `0.004` | 控制周期（250 Hz） |
| `--kp` | `50 50 50` | 位置刚度 (N/m) |
| `--kr` | `50 50 50` | 旋转刚度 (Nm/rad) |
| `--kd` | `10 10 10 5 5 5` | 阻尼 (Ns/m, Nms/rad) |

### 3.4 控制律说明

```python
# 简化 GIC Regulation (无自适应惯性)
Vb      = Jb @ dq                    # 体速度
ep      = R^T @ (p - pd)             # 体坐标系位置误差
eR      = vee(Rd^T @ R - R^T @ Rd)   # 朝向误差
F_body  = -Kp @ ep - KR @ eR         # 虚拟力 (体坐标系)
tau_tilde = Jb^T @ (F_body - Kd @ Vb)  # 任务空间力矩
tau_cmd = tau_tilde + tau_bias       # 附加重力补偿
```

### 3.5 预期输出

```
GIC 控制循环启动 (15.0 秒) ...
  t=  1.00s  ||ep||=  0.03mm  ||tau||=  75.2Nm  p=[0.500, 0.000, 0.600]
  t=  2.00s  ||ep||=  0.05mm  ||tau||=  75.1Nm  p=[0.500, 0.000, 0.600]
  ...
  t= 15.00s  ||ep||=  0.51mm  ||tau||=  75.3Nm  p=[0.500, 0.000, 0.600]

=== 测试统计 ===
  平均频率:       249.7 Hz
  最终位置误差:   0.51 mm
  平均位置误差:   0.33 mm
  最大位置误差:   1.24 mm
  位置标准差:    [0.21, 0.15, 0.18] mm

✅ Step 4 通过
```

### 3.6 通过标准

| Kp 范围 | 预期位置精度 | 说明 |
|---|---|---|
| Kp=50 | ±2 mm | 极低增益，位置保持较弱 |
| Kp=200 | ±0.5 mm | 中等刚度 |
| Kp=500 | ±0.2 mm | 较高刚度（注意抖动） |
| Kp=1000+ | ±0.1 mm | 高刚度（需配合高阻尼） |

### 3.7 调参指南

**安全调参顺序**: 每次只调一个参数，观测 10 秒以上

```
1. 先调 Kp (位置刚度)
   Kp=50   → 评估稳态误差
   Kp=200  → 误差应减小
   Kp=500  → 注意是否出现抖动
   
2. 再调 KR (旋转刚度)
   KR 通常与 Kp 同级或略小
   如果末端扭转化, 增大 KR
   
3. 最后调 Kd (阻尼)
   Kd 应随 Kp 增大而增大
   出现抖动时增大 Kd
   推荐: Kd ≈ 2 * sqrt(Kp * M_eff)  (临界阻尼)
```

### 3.8 常见问题

| 现象 | 可能原因 | 解决方法 |
|---|---|---|
| 末端高频抖动 | Kd 过小 | 增大 Kd |
| 末端缓慢漂移 | Kp 过小 | 增大 Kp |
| 末端静止但有轻微震荡 | Kp/Kd 不匹配 | 增大 Kd 或减小 Kp |
| 突然跳变 | 通信超时 | 检查网络/cache 机制 |
| 重力补偿不足 | URDF 参数不准确 | 检查 link 质量/质心 |

---

## 4. 快速故障排查

### 4.1 连接问题

| 症状 | 检查项 |
|---|---|
| `RTDEReceiveInterface 连接失败` | `ping` 通吗？教示器在 Remote Control 模式吗？ |
| `RTDEControlInterface 连接失败` | UR 安全配置是否允许远程控制？ |
| 连接成功但读取超时 | 网线质量？交换器？尝试降低频率（增大 `--dt`） |
| 连接断断续续 | 检查 TCP 连接数，关闭其他 RTDE 客户端 |

### 4.2 力矩问题

| 症状 | 检查项 |
|---|---|
| 零力矩时臂不动 | UR 内部重力补偿可能已开启，检查教示器设置 |
| 重力补偿后臂上漂 | URDF 质心参数偏大 |
| 重力补偿后臂下沉 | URDF 质量参数偏小 |
| 发力矩时报错 | 力矩限幅过低？尝试增大 `torque_limits` |

### 4.3 控制问题

| 症状 | 检查项 |
|---|---|
| 控制频率不达标 | 减小 `--dt`，但不要低于 0.002 (500 Hz) |
| 位置误差大 | 增大 Kp，或检查 URDF 运动学参数 |
| 末端奇异 | 检查当前位形是否接近奇异点（腕部对齐） |

---

## 5. 参考

- [ur_rtde 文档](https://sdurobotics.gitlab.io/ur_rtde/)
- [Pinocchio 文档](https://stack-of-tasks.github.io/pinocchio/)
- [UR12e URDF](https://github.com/ros-industrial/ur_description)
- SE(3) 控制相关论文: Seo et al., "Geometric Unified Force-Impedance Control"

## 4. 附录: 机械臂参数参考

### 4.1 UR12e

| 关节 | URDF Effort (Nm) | 安全限幅 50% (Nm) |
|---|---|---|
| shoulder_pan | 330 | 165 |
| shoulder_lift | 330 | 165 |
| elbow | 150 | 75 |
| wrist_1 | 54 | 27 |
| wrist_2 | 54 | 27 |
| wrist_3 | 54 | 27 |

### 4.2 UR3

| 关节 | URDF Effort (Nm) | 安全限幅 50% (Nm) |
|---|---|---|
| shoulder_pan | 56 | 28 |
| shoulder_lift | 56 | 28 |
| elbow | 28 | 14 |
| wrist_1 | 12 | 6 |
| wrist_2 | 12 | 6 |
| wrist_3 | 12 | 6 |

> **注意**: UR3 的力矩限幅远小于 UR12e。使用 UR3 进行重力补偿和 GIC 测试时，建议适当降低增益以保护机械臂。

---

*文档创建日期: 2026-07-26*
*首个部署目标: UR12e @ 500 Hz / 250 Hz*
