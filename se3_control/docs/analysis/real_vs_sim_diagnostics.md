# 实机 vs 仿真 差异诊断（"仿真画圆正常、真机向上抬然后折叠撞"）

> 用途：真机上跑 `run_se3_control.py` 时**复现异常动作**（臂向上抬、肘/前臂向基部
> 折叠、碰撞），录下运动过程数据，用同一套指标把**实机**与 **MuJoCo 预览** 逐列对照，
> 定位差异来源。三个工具配合使用，全部在**仓库根目录**运行（conda env `roboarm`）。

| 工具 | 作用 | 在哪个环节用 |
|---|---|---|
| `--log-dir`（run_se3_control / --preview） | 每控制周期写**同格式**全分辨率 CSV（q / dq / q_servo / dq_des / tau / tau_lim / 误差 / TCP） | 实机跑任务时 + 仿真预览时，各录一份 |
| `monitor_rtde.py` | **只读**实时录原始 RTDE 数据（q / dq / 电流 / TCP / 速度缩放 / 安全事件 / momentum） | 实机跑任务时，另一个终端同步开 |
| `analyze_arm_log.py` | 读 CSV → 自动判定 + 出图 | 事后分析 |

---

## 1. 三工具配套工作流（实机）

```bash
conda activate roboarm
mkdir -p logs/run_01

# 终端 1：先回 home（确保起步位形一致，这是实机撞的常见根因之一）
python go_home.py --robot ur3 --yes

# 终端 2：同时开 RTDE 只读记录（500 Hz，Ctrl+C 结束）
python monitor_rtde.py --robot ur3 --rate 500 --out logs/run_01/rtde.csv

# 终端 1：跑任务，--log-dir 记录控制回路每周期数据
python se3_control/scripts/run_se3_control.py --robot ur3 --control-mode servoJ \
    --task circle --center 0.40 0.0 0.35 --radius 0.05 --duration 16 --bandwidth 10 \
    --log-dir logs/run_01

# 仿真对照：同一参数 + --log-dir，生成同格式 CSV
python se3_control/scripts/run_se3_control.py --robot ur3 --control-mode servoJ \
    --task circle --center 0.40 0.0 0.35 --radius 0.05 --duration 16 --bandwidth 10 \
    --preview --log-dir logs/sim_01

# 分析：实机 vs 仿真 叠图 + 自动判定（通配符可一次给多份）
python analyze_arm_log.py \
    --log logs/run_01/Phase2_*.csv  --label 实机 \
    --log logs/sim_01/sim_*.csv     --label 仿真 \
    --rtde logs/run_01/rtde.csv \
    --out logs/analysis_01
```

产物：`logs/analysis_01/` 下 `errors.png`（误差对比）、`windup.png`（参考积分漂移）、
`torque.png`（力矩饱和）、`cartesian.png`（TCP 轨迹）、`compare.png`（实机 vs 仿真
直接叠图）、`rtde_q.png`（RTDE 原始数据），以及控制台的逐项判定报告。

---

## 2. CSV 里每个信号的意义（怎么看）

`--log-dir` 的 CSV 列序见 `se3_control/core/arm_log.py`（实机与仿真**完全一致**，可直接
按 q 对齐后逐列对照）。诊断时重点盯这几个信号：

| 信号 | 列 | 说明 | 异常判据 |
|---|---|---|---|
| **参考积分漂移** | `q_servo − q` | 下发给 servoJ 的目标位与实际关节角的差。真机 servoJ 是增益式内层跟踪器，`q_servo` 靠参考积分生成；内层追不上 → 积分漂移 | 后半段 RMS > 50 mrad 即为可疑（仿真理想闭环≈0） |
| **力矩饱和** | `tau` vs `tau_lim` | GIC 力矩顶到关节限幅 = 臂达不到期望，误差开始累积 | 任一关节饱和占比 > 20% |
| **参考速度饱和** | `dq_des` vs ±2.0 | 桥接器期望速度顶到 `dq_max=2.0 rad/s` 限幅 = 级联不稳定的征兆 | 占比 > 0（接近 2.0） |
| **误差发散** | `pos_err` / `rot_err` | 后半段斜率是否持续为正且终值大 | 斜率 > 2 mm/s 且终值 > 8 cm |
| **“向上抬/折叠”** | `p_z − pd_ref_z` | 真机先抬再塌的笛卡尔表现 | 后半段最高抬 > 5 cm **且**最低塌 < −5 cm |
| **保护性停止** | `monitor` CSV 的 `safety_mode` | `PROTECTIVE_STOP=2` = 触发安全限位（Phase0 即停常见） | 值从 0 跳变到 2/4 |

`monitor_rtde.py` 还记录 `speed_scaling`（速度缩放会改变 servoJ 内层增益行为）、
`momentum`（碰撞冲击量证据）、`current`（CB3 无关节力矩，电机电流是力矩最接近的
代理）。CB3 没有 `getActualJointTorques`，所以**真机力矩只能从电流 + 控制回路 CSV 的
tau（期望值）间接判断**。

---

## 3. 为什么仿真正常、真机乱动（已知根因清单）

`analyze_arm_log.py` 的判定覆盖以下已知根因；实测数据能直接对上哪一条就说明问题在哪：

1. **起步位形不在 home**：预览从 home 起步，真机从折叠/低位起步 → 混合路径扫向
   home 过程中肘部贴基座。→ 先 `go_home.py`，或 `--preview-start-q <当前q>` 从真实起步模拟。
2. **servoJ 内层参考积分漂移**（`q_servo − q` 大）：真机 servoJ 是增益式跟踪器，
   无模型前馈，高频 circle 任务下内层追不上 → `q_servo` 积分越积越大。→ 降低
   `--bandwidth` 或改 `--control-mode directTorque`（需 e-Series）。
3. **力矩饱和**（`tau` 顶到 `tau_lim`）：GIC 期望力矩超过关节能力 → 臂达不到期望，
   误差累积放大。→ 检查 `tau` 曲线哪个关节顶满。
4. **真实伺服滞后/速度缩放**：`speed_scaling < 100%` 时内层伺服行为与仿真不一致。
5. **安全限位触发**：`safety_mode = PROTECTIVE_STOP` → Phase0 即停；看 `go_home.py`
   与 `ur-rtde-directtorque-rootcause` 的结论（首发力矩触发安全限位）。
6. **安全模式 REDUCED 未复位**（2026-08-11 实机首次取证发现）：`safety_mode` 全程 = 1
   （REDUCED）而非 NORMAL(0)。REDUCED 模式启用降级限速/限力，内层 servoJ 追不上参考、
   加重积分漂移。→ 跑任务前在示教器复位到 NORMAL，确认 `safety_mode=0`（见 §4）。

---

## 4. 复位安全模式后重跑（本次实机问题的关键前置）

> 2026-08-11 首次实机取证发现：全程 `safety_mode = 1 (REDUCED)` 而非 NORMAL(0)。
> REDUCED 模式下 UR 启用降级的安全限速/限力，内层 servoJ 更追不上参考，
> 是"仿真正常、真机折叠撞"的重要推手。**跑任务前必须回到 NORMAL。**

### 复位到 NORMAL 后，重跑同一任务的完整命令

```bash
conda activate roboarm
mkdir -p logs/run_02

# ① 复位安全模式（示教器）
#    若示教器显示保护性停止 / RECOVERY：按示教器上的复位 + 上电(Power)按钮
#    恢复 NORMAL，或重新上电。然后只读检查 safety_mode 是否回到 0：
python monitor_rtde.py --robot ur3 --out logs/run_02/check_safety.csv --duration 3

# ② 回 home（起步位形=home，排除"起步非 home"根因）
python go_home.py --robot ur3 --yes

# ③ 开 RTDE 只读记录 + 跑任务（两个终端）
#    终端 2：
python monitor_rtde.py --robot ur3 --rate 500 --out logs/run_02/rtde.csv
#    终端 1（bandwidth 10→6 降低内层负担；想先复现对比可保持 10）：
python se3_control/scripts/run_se3_control.py --robot ur3 --control-mode servoJ \
    --task circle --center 0.40 0.0 0.35 --radius 0.05 --duration 16 --bandwidth 6 \
    --log-dir logs/run_02

# ④ 仿真对照（同一参数，先确认仿真安全）
python se3_control/scripts/run_se3_control.py --robot ur3 --control-mode servoJ \
    --task circle --center 0.40 0.0 0.35 --radius 0.05 --duration 16 --bandwidth 6 \
    --preview --log-dir logs/sim_02

# ⑤ 分析（重点看实机的积分漂移/饱和/发散/折叠是否消失）
python analyze_arm_log.py \
    --log logs/run_02/Phase2_*.csv --label 实机 \
    --log logs/sim_02/sim_*.csv     --label 仿真 \
    --rtde logs/run_02/rtde.csv --out logs/analysis_02
```

判定通过标准：`实机` 的 `积分漂移 RMS`、`力矩饱和占比`、`误差发散`、`折叠特征`
相对首次运行（记录见 `logs/run_01/实验记录_20260811_1035.md`）明显下降、与 `仿真`
曲线接近；`rtde.csv` 里 `safety_mode` 保持 0，无 `PROTECTIVE_STOP`/`RECOVERY` 跳变。

---

## 5. 先仿真对照再上真机（闭环预览）

```bash
# 预览（MuJoCo 闭环，不碰真机）+ 自动碰撞判定 + 记录同格式 CSV
python se3_control/scripts/run_se3_control.py --robot ur3 --control-mode servoJ \
    --task circle --center 0.40 0.0 0.35 --radius 0.05 --duration 16 --bandwidth 10 \
    --preview --log-dir logs/sim_01

# 从真实起步位形模拟（先 --dry-run 读当前 q）
python se3_control/scripts/run_se3_control.py --robot ur3 --control-mode servoJ \
    --task circle --center 0.40 0.0 0.35 --radius 0.05 --duration 16 --bandwidth 10 \
    --preview-start-q -0.5 -1.3 1.1 -1.5 -1.6 2.6 --log-dir logs/sim_02
```

预览通过（报告 `✓ 无碰撞风险`）后再上真机；上真机时开 `monitor_rtde.py` + `--log-dir`。

---

## 6. 常见问题

- **`analyze_arm_log.py --log` 通配符不匹配**：bash 通配符需要引号或脚本内 glob 展开，
  本脚本内部用 `glob`，直接传 `Phase2_*.csv` 即可（不用加引号也能展开）。
- **`--log` 与 `--label` 数量对不上**：`--log` 支持通配符，展开后的文件数必须与
  `--label` 一一对应。
- **CSV 是几份**：`run_se3_control` 每个 Phase（Phase0/Phase2）各写一份；分析实机
  circle 主阶段用 `Phase2_*`。
- **没有真实 RTDE 也想看分析效果**：`logs/_synth/` 下有合成样例，可先跑
  `python analyze_arm_log.py --log logs/_synth/healthy.csv --label 健康
   --log logs/_synth/diverging.csv --label 发散` 熟悉输出。
