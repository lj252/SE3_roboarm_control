# 实机 vs 仿真 差异诊断（"仿真画圆正常、真机向上抬然后折叠撞"）

> 用途：真机上跑 `run_se3_control.py` 时**复现异常动作**（臂向上抬、肘/前臂向基部
> 折叠、碰撞），录下运动过程数据，用同一套指标把**实机**与 **MuJoCo 预览** 逐列对照，
> 定位差异来源。三个工具配合使用，全部在**仓库根目录**运行（conda env `roboarm`）。

| 工具 | 作用 | 在哪个环节用 |
|---|---|---|
| `--log-dir`（run_se3_control / --preview） | 每控制周期写**同格式**全分辨率 CSV（q / dq / q_servo / dq_des / tau / tau_lim / 误差 / TCP） | 实机跑任务时 + 仿真预览时，各录一份 |
| `tests/monitor/monitor_rtde.py` | **只读**实时录原始 RTDE 数据（q / dq / 电流 / TCP / 速度缩放 / 安全事件 / momentum） | 实机跑任务时，另一个终端同步开 |
| `tests/monitor/analyze_arm_log.py` | 读 CSV → 自动判定 + 出图 | 事后分析 |

---

## 1. 三工具配套工作流（实机）

```bash
conda activate roboarm
mkdir -p logs/run_01

# 终端 1：先回 home（确保起步位形一致，这是实机撞的常见根因之一）
python tests/monitor/go_home.py --robot ur3 --yes

# 终端 2：同时开 RTDE 只读记录（500 Hz，Ctrl+C 结束）
python tests/monitor/monitor_rtde.py --robot ur3 --rate 500 --out logs/run_01/rtde.csv

# 终端 1：跑任务，--log-dir 记录控制回路每周期数据
python se3_control/scripts/run_se3_control.py --robot ur3 --control-mode servoJ \
    --task circle --center 0.40 0.0 0.35 --radius 0.05 --duration 16 --bandwidth 10 \
    --log-dir logs/run_01

# 仿真对照：同一参数 + --log-dir，生成同格式 CSV
python se3_control/scripts/run_se3_control.py --robot ur3 --control-mode servoJ \
    --task circle --center 0.40 0.0 0.35 --radius 0.05 --duration 16 --bandwidth 10 \
    --preview --log-dir logs/sim_01

# 分析：实机 vs 仿真 叠图 + 自动判定（通配符可一次给多份）
python tests/monitor/analyze_arm_log.py \
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
| **速度缩放异常** | RTDE `target_speed_fraction` / `speed_scaling_combined` | 安全控制器施加的组合速度上限，**< 1.0 即实机在降速运行**（run_04 为 0.24；run_02 稳定时为 1.0，两者 `safety_mode` 都是 REDUCED） | 启动时 < 0.9 应拒绝/告警；servoJ 参考速度上限必须按此缩放（见 §8） |
| **保护性停止** | `monitor` CSV 的 `safety_mode` | `PROTECTIVE_STOP=2` = 触发安全限位（Phase0 即停常见） | 值从 0 跳变到 2/4 |

`tests/monitor/monitor_rtde.py` 还记录 `speed_scaling`（速度缩放会改变 servoJ 内层增益行为）、
`momentum`（碰撞冲击量证据）、`current`（CB3 无关节力矩，电机电流是力矩最接近的
代理）。CB3 没有 `getActualJointTorques`，所以**真机力矩只能从电流 + 控制回路 CSV 的
tau（期望值）间接判断**。

---

## 3. 为什么仿真正常、真机乱动（已知根因清单）

`tests/monitor/analyze_arm_log.py` 的判定覆盖以下已知根因；实测数据能直接对上哪一条就说明问题在哪：

1. **起步位形不在 home**：预览从 home 起步，真机从折叠/低位起步 → 混合路径扫向
   home 过程中肘部贴基座。→ 先 `tests/monitor/go_home.py`，或 `--preview-start-q <当前q>` 从真实起步模拟。
2. **servoJ 内层参考积分漂移**（`q_servo − q` 大）：真机 servoJ 是增益式跟踪器，
   无模型前馈，高频 circle 任务下内层追不上 → `q_servo` 积分越积越大。→ 降低
   `--bandwidth` 或改 `--control-mode directTorque`（需 e-Series）。
3. **力矩饱和**（`tau` 顶到 `tau_lim`）：GIC 期望力矩超过关节能力 → 臂达不到期望，
   误差累积放大。→ 检查 `tau` 曲线哪个关节顶满。
4. **真实伺服滞后/速度缩放**：`speed_scaling < 100%` 时内层伺服行为与仿真不一致。
5. **安全限位触发**：`safety_mode = PROTECTIVE_STOP` → Phase0 即停；看 `tests/monitor/go_home.py`
   与 `ur-rtde-directtorque-rootcause` 的结论（首发力矩触发安全限位）。
6. **安全模式 REDUCED 未复位**（2026-08-11 实机首次取证发现）：`safety_mode` 全程 = 1
   （REDUCED）而非 NORMAL(0)。REDUCED 模式启用降级限速/限力，内层 servoJ 追不上参考、
   加重积分漂移。→ 跑任务前在示教器复位到 NORMAL，确认 `safety_mode=0`（见 §4）。
7. **circle 画圆倾斜平面**（2026-08-11 定位，**仿真与真机一致地倾斜**，不是"仿真正常
   真机乱动"类差异）：根因是 GIC/GAC 控制器里**期望速度拼装顺序错误**——`np.hstack((vd, wd))`
   把两个 (3,1) 列向量拼成 (3,2) 再行优先 reshape 成交错序 `[vx,wx,vy,wy,vz,wz]`，把 `vd_y`
   （圆切向 ≈34 mm/s）当成 z 通道参考，产生 ~11 mm z 向稳态误差 → 圆平面倾斜。已修复
   （详见 §7）。此问题 `tests/monitor/analyze_arm_log.py` 判不出来（两套都"正常"），要靠平面拟合发现。

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
python tests/monitor/monitor_rtde.py --robot ur3 --out logs/run_02/check_safety.csv --duration 3

# ② 回 home（起步位形=home，排除"起步非 home"根因）
python tests/monitor/go_home.py --robot ur3 --yes

# ③ 开 RTDE 只读记录 + 跑任务（两个终端）
#    终端 2：
python tests/monitor/monitor_rtde.py --robot ur3 --rate 500 --out logs/run_02/rtde.csv
#    终端 1（bandwidth 10→6 降低内层负担；想先复现对比可保持 10）：
python se3_control/scripts/run_se3_control.py --robot ur3 --control-mode servoJ \
    --task circle --center 0.40 0.0 0.35 --radius 0.05 --duration 16 --bandwidth 6 \
    --log-dir logs/run_02

# ④ 仿真对照（同一参数，先确认仿真安全）
python se3_control/scripts/run_se3_control.py --robot ur3 --control-mode servoJ \
    --task circle --center 0.40 0.0 0.35 --radius 0.05 --duration 16 --bandwidth 6 \
    --preview --log-dir logs/sim_02

# ⑤ 分析（重点看实机的积分漂移/饱和/发散/折叠是否消失）
python tests/monitor/analyze_arm_log.py \
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

预览通过（报告 `✓ 无碰撞风险`）后再上真机；上真机时开 `tests/monitor/monitor_rtde.py` + `--log-dir`。

---

## 6. 常见问题

- **`tests/monitor/analyze_arm_log.py --log` 通配符不匹配**：bash 通配符需要引号或脚本内 glob 展开，
  本脚本内部用 `glob`，直接传 `Phase2_*.csv` 即可（不用加引号也能展开）。
- **`--log` 与 `--label` 数量对不上**：`--log` 支持通配符，展开后的文件数必须与
  `--label` 一一对应。
- **CSV 是几份**：`run_se3_control` 每个 Phase（Phase0/Phase2）各写一份；分析实机
  circle 主阶段用 `Phase2_*`。
- **没有真实 RTDE 也想看分析效果**：`logs/_synth/` 下有合成样例，可先跑
  `python tests/monitor/analyze_arm_log.py --log logs/_synth/healthy.csv --label 健康
   --log logs/_synth/diverging.csv --label 发散` 熟悉输出。

---

## 7. circle 画圆倾斜平面的根因与修复（2026-08-11）

**现象**：circle 任务画出的圆不在水平面上，而是倾斜平面。**仿真与真机几乎完全一致**
（这是关键线索——若只有真机倾斜、仿真正常，应优先查 §3 的实机特有根因）：

| 来源 | 平面拟合 | z-std |
|---|---|---|
| 仿真 `logs/sim_dt_full`（直接 GIC，理想闭环） | `z = 0.054x − 0.270y + 0.244` | 10.5 mm |
| 真机 `logs/run_03`（UR3，校准后坐标） | `z = 0.051x − 0.266y + 0.242` | 11.0 mm |

### 根因：期望速度拼装顺序错误（非低带宽）

`se3_control/core/gic_controller.py` 原写法：

```python
Vd = np.hstack((vd, wd)).reshape((-1, 1))   # 错误
```

`eval_body_twist` 返回的 `vd`/`wd` 是 **(3,1) 列向量**，`np.hstack((vd, wd))` 把两个
(3,1) 沿 axis=1 拼成 **(3,2) 矩阵**，再行优先 `reshape((-1,1))` 得到**交错序**
`[vx, wx, vy, wy, vz, wz]`；而 `adjoint_g_ed`、`e_op`、`ev`、`M̃` 全部按**块序**
`[vx,vy,vz,wx,wy,wz]` 排列。错位后，圆轨迹上的切向速度 `vd_y`（≈34 mm/s）被当成
`Vd_z` 注入 z 通道参考，经阻尼项 `2ζω·ev_z` 产生 **~11 mm 的 z 向稳态误差 → 圆平面倾斜**。

> **为什么看起来像"仿真带宽低"**：监控脚本 `tests/monitor/monitor_sim.py` 重建控制器 τ 时犯了
> **同样的拼装错误**，拟合出的"有效带宽 0.5"是重建伪影。修复后 `--fit-bandwidth`
> 精确拟合回 **ω_eff=6.001、残差 0.0 mN·m**（控制器本来就跑在配置的 ω=6）。

### 修复（已应用，74 个测试全部通过）

先 `ravel()` 到 (3,) 再 `np.concatenate`，得到块序 `[v; w]`，共 3 处：

| 文件 | 修复内容 |
|---|---|
| `core/gic_controller.py:100` | Vd/dVd 拼装改块序（根因） |
| `core/gac_controller.py:420` | **同款拼装 bug** + `adjoint_g_ed_deriv` 参数错位（期望速度进了当前速度槽位、期望加速度进了期望速度槽位；正确应传当前体速度 `Vb[:3],Vb[3:]`） |
| `tests/monitor/monitor_sim.py:144` | 同款拼装 bug（解释了"带宽 0.5"假象） |

### 验证证据（闭环可控实验）

- **消除性实验**：把拼装改成块序 `np.vstack((vd,wd))` → e_w z RMS **11.3 → 0.2 mm**，
  `corr(e_z,e_x)` 从 −1.000（直线相关=平面）→ +0.002（水平）；
- **真实 MuJoCo 预览仿真修复后**：e_w z 向 RMS **11.26 → 0.23 mm**，平面残差 z-std
  0.11 mm，圆恢复水平；
- **GAC circle 仿真对比**：修复后稳态 pos_err **8.7 → 0.096 mm**（约 90 倍改善）；
- 其余假设全部排除：M̃ 非对角耦合、P 投影偏差、前馈 dVd*、低带宽——均不是主因。

### 影响与排查提示

- **实验2 球面接触（GAC）的早期数值受此 bug 影响**，若重跑实验2 结果会变化；
- 这是 SE(3) 控制器里最容易再犯的一类 bug：凡"把两个 (3,1) 拼成 6 向量"处都要用
  `np.concatenate([a.ravel(), b.ravel()])`，不要 `np.hstack(...).reshape(-1,1)`。新增
  控制器/监控脚本时 grep 检查 `hstack(.*vd.*wd` 与 `.reshape((-1, 1))`；
- 此问题 `tests/monitor/analyze_arm_log.py` 的逐项判定全"正常"（仿真真机都如此），**必须靠平面拟合**
  （`tests/monitor/monitor_sim.py` 的平面残差 z-std，或画 `p_z` 随角度变化）才能暴露。

完整排查过程见 `docs/project_overview.md` §11.3。

---

## 8. 实机降速运行下的发散（run_04，2026-08-11）

> 场景：**为安全起见真机不可能全速跑**（或处于降速的 REDUCED 状态）。但控制器的
> 参考生成器按"臂能全速移动"设计，一旦实际速度被安全系统压到低速，级联失稳会以
> "先沉后抬、最后甩起"的形式发散。本节记录 run_04 实机发散的全过程、根因与
> 建议的工程化防护。

### 8.1 现象

```bash
# 实机（run_04）与仿真（sim_04）同一命令：
python se3_control/scripts/run_se3_control.py --robot ur3 --control-mode servoJ \
    --task circle --center -0.35 0.0 0.16 --radius 0.05 --duration 16 --bandwidth 6 \
    --log-dir logs/run_04
```

| 来源 | 结果 |
|---|---|
| 仿真 `sim_04` | 圆**完美**：pos_err→0，无漂移/无饱和/无发散 |
| 实机 `run_04` | **开始向下沉**（TCP z→0.089，低于圆平面 0.16），随后**整体抬高直到最高点**（z→0.486），末段 pos_err 发散到 56 cm、rot_err 2.29 rad，用户安全断开连接 |

> 同一命令在方案 A 修复后（参考上限按实际速度缩放）即可稳定运行，见 **§8.5**。

### 8.2 取证数据

**`tests/monitor/analyze_arm_log.py` 判定（run_04 Phase2，t∈[0,10.6]s）**：

| 指标 | 实机 | 仿真 |
|---|---|---|
| 积分漂移 RMS(后半) | 关节2 = **3.25 rad**（q_servo−q 最大关节1 = 4.33 rad） | 0 |
| 力矩饱和占比 | 关节0 = **22%** | 0% |
| dq_des 饱和占比 | 关节1 = **30%** | 0% |
| pos_err 斜率/终值 | +99.6 mm/s / **56 cm** | 0 |
| rot_err 终值 | 2.29 rad | 0 |

**RTDE 对照——run_02（稳定）vs run_04（发散）的关键差异**：

| 信号 | run_02（16s 稳定） | run_04（发散） |
|---|---|---|
| `safety_mode` | 1 (REDUCED) | 1 (REDUCED) |
| `target_speed_fraction` | **1.0（100%）** | **0.24（24%，全程恒定）** |
| `speed_scaling_combined` | ~0.99 | ≤0.24，末段跌到 0 |
| TCP z 范围 | [0.210, 0.238] | [0.089, 0.486] |
| momentum 峰值 | — | 2.0 |

> run_02 同样 `safety_mode=1 (REDUCED)` 但**速度缩放 100%** → 稳定。run_04 的
> 差异不是 REDUCED 本身，而是**速度缩放被压到 24%**。判断"是否降速"要看
> `target_speed_fraction`，**不要只看 `safety_mode`**。

### 8.3 根因

**servoJ 参考积分器按"臂能全速移动"假设前进，但实机实际关节速度被速度缩放下限死
→ 参考跑赢实际 → 参考积分漂移 → 力矩饱和 → 误差发散。**

机制（`core/servo_bridge.py`）：

```
τ → ddq → dq_des(积分, 限幅 ±dq_max=2.0 rad/s) → q_servo(积分) → servoJ
```

- 桥接器 `dq_max=2.0 rad/s` 按 UR3 满速（前 3 关节 ~2.175 rad/s）设计；
- 24% 速度缩放下 UR 伺服实际最大关节速度只剩 **~0.52 rad/s**；
- GIC 误差持续为正 → `ddq` 顶到 ±20 rad/s² 饱和 → `dq_des` 停在 ~2.0；
- `ref_damp=15` 把 `dq_des` 拉向 `dq` 的阻尼**拽不住**（ddq 饱和贡献更大）→
  `q_servo` 以 ~2 rad/s 前进、实际臂只 ~0.5 rad/s → `q_servo−q` 以 ~1.5 rad/s 累积；
- 误差增长 → GIC 力矩增长 → 关节0 力矩饱和(22%) → 发散；方向由饱和关节决定，
  最后臂整体甩起、腕关节翻转（joint5 转 4.37 rad，rot_err 2.29 rad）。

时间线（Phase2）：t<0.5s 漂移即开始 → t≈2.7s 沉到 z=0.095（"向下沉"）→
t≈8s 甩到 z=0.453（"整体抬高"）→ 10.6s 误差 56 cm → 手动断开。

**为什么仿真复现不了**：`core/mujoco_preview.py` 的内层伺服是理想计算力矩伺服
`τ = M·(ddq_des + 2ω(dq_des−dq) + ω²(q_servo−q)) + bias`（ω=30 rad/s），**没有任何
关节速度上限**，不建模 UR 速度缩放 → 臂想多快就多快 → 参考永远不会跑赢 →
圆完美。这类"实机被限速"的故障只有给内层伺服建模速度缩放后才能被 `--preview` 预判。

> **run_05_forced/05b 补充：不是所有降速发散都是 windup**。v1（只缩放参考上限）在 24%
> 下仍发散，机制是**闭环失稳**（外环带宽 6 > 降速后内环有效带宽 ~2 → 级联不稳定，
> λ=+0.31/s，z 振荡 1.44 rad/s）。v2（加 ref_damp÷s）则**追落体坠向基部**（参考单调
> 往下追下坠臂，q_servo−q 涨到 310 mrad）。最终方案 A = 带宽×s + 参考上限×s + 参考
> 阻尼保持基值且加丢跟踪门限，见 **§8.5** 顶部。

### 8.4 建议的解决方法（防止此类问题）

**核心原则：低速运行本身没问题，问题在于"参考生成器不知道实际限速"。**
让控制器知道并匹配实际速度缩放，即可安全低速运行。

| 方案 | 做法 | 效果 |
|---|---|---|
| **A. 速度缩放感知的自适应桥接（首选）✅ 已实现（最终版）** | 启动时从 RTDE 读 `target_speed_fraction`（或 `--servo-speed-fraction <s>` 覆盖），运行中每 10 个控制周期从 RTDE 连续更新（降速**立即生效**、恢复**缓慢爬升**），并按 `s` 缩放：① 参考速度/加速度上限 `dq_max_eff = dq_max × s`；② 外环 GIC 有效带宽 `ω_eff = ω × s`（级联稳定）；③ 参考阻尼 `ref_damp` **保持基值不缩放**，且加"伺服丢跟踪门限"：`\|参考−臂\| > 0.15 rad` 时只刹不追 | ①参考不跑赢被限速的臂；②级联稳定（外环带宽 < 降速后内环有效带宽）；③正常跟踪保持对称阻尼（收敛），丢跟踪（臂以超参考速度下坠）时不追臂 → **任意降速下稳定**。2026-08-11 最终版已实现（`core/servo_bridge.py`：`speed_fraction`/`set_speed_fraction`/`_dq_track_gate`，`ur_hw.py`：`get_speed_scaling`，`run_se3_control.py`：启动读 RTDE + 周期更新 + `resolve_main_bandwidth` 按 s 缩带宽，`mujoco_preview.py` 透传），84 测试通过；**演进**：首版只做①仍发散（run_05_forced），v2 加 ③`ref_damp÷s` 反而坠向基部（run_05b），v3 改 ③为门限只刹不追，见 **§8.5** |
| **B. 跑前自检安全闸门** | 跑任务前用 `monitor_rtde.py` 只读 1 s，断言 `target_speed_fraction` 与期望一致（期望全速则 ≥0.9）；不符则**拒绝运行并提示复位/降参** | run_04 这类"机器人其实在降速"在臂动之前就被拦住 |
| **C. 参考漂移硬限幅（反积分饱和安全网）** | 桥接器加硬保护：`\|q_servo−q\| > Δq_safe`（如 0.5 rad）时冻结参考（`dq_des→0`）并把 `q_servo` 拉回 `q` | 无论因何追不上（降速/饱和/负载），发散被降级为受控停顿，不甩臂 |
| **D. 仿真建模速度缩放（未实现）** | `mujoco_preview` 内层伺服把关节速度钳到 `s × vmax`（钳 `dq_des` 或步进后钳 qvel）；`--servo-speed-fraction` 参数已透传（当前只到桥接器，缩放参考上限） | 实现后 `--preview --servo-speed-fraction 0.24` 才能**在仿真里复现** run_04 故障，用于先验证 A–C 再上真机。**现状**：MuJoCo 内层伺服无速度上限，仿真里传 0.24 圆仍是完美的 |
| **E. 运维规范** | 明确每次跑的真实速度缩放并写进命令/记录；故意低速时同步降 `--bandwidth`（级联稳定性）与 `--servo-dq-max` | 消除"以为全速、实则降速"的认知错位 |

**落地进度**：**A（速度缩放感知）✅ 已实现**（2026-08-11，最终版：参考上限 ×s + 外环带宽 ×s + 参考阻尼保持基值且加"丢跟踪门限只刹不追"，84 测试通过；运行命令见 §8.5）。
剩余建议顺序：**B（自检闸门）** —— 一行 RTDE 读取，立刻挡住复发；**D（仿真建模）** ——
`--preview --servo-speed-fraction 0.24` 参数已预留，但 MuJoCo 内层伺服速度钳制**尚未实现**
（现只缩放桥接器参考上限），实现后才能在仿真里复现 run_04 故障；**C（漂移硬限幅）**
作为任何速度下的最后安全网。**C 尤为重要**：即使 A 已完整，任何未建模的滞后/饱和都可能
让级联失稳，C 是最后的安全网。

> 注意：即使启用 A/B，`analyze_arm_log.py` 也应从 `--rtde` 文件自动汇报
> `target_speed_fraction`，把"降速"直接写进判定报告，避免靠人眼翻 RTDE。

### 8.5 方案 A 落地：运行命令（2026-08-11 完整版已实现）

**演进（三轮实测逐个排查）**：

- **run_05_forced（v1，只做"参考上限 ×s"）仍发散**：`dq_des` 全程被钳在 0.48（缩放生效），
  但臂 dq 峰值 **0.68 > 参考**、`q_servo−q` 有界 ~0.25 rad（≈0.5s 滞后），位置误差**指数**
  发散 λ=+0.31/s、z 以 1.44 rad/s 振荡。机制是**闭环失稳**：降速后内环有效带宽塌到 ~2 rad/s，
  外环 GIC 6 rad/s 远超 → 级联不稳定。→ 加 **外环带宽 ×s**（`resolve_main_bandwidth`，ω→1.44）。
- **run_05b（v2，加 `ref_damp÷s`=62.5）坠向基部**：新症状"一直向下沉撞基部"。取证：GIC 力矩
  方向全对（tau2 100% 抵抗下沉、均值 −5.7 N·m），但参考 q_servo **单调往下追**（1.236→2.999），
  `dq_des` 钉在 +0.48 上限；臂 dq 峰值 0.71 **超过参考**，`q_servo−q` 涨到 310 mrad，位置误差
  线性增长（+46 mm/s，非指数）。根因：**放大的 ref_damp 把参考变成"追落体器"**——臂在 24%
  降速下刹不住、以超参考的速度下坠时，对称阻尼项 `ref_damp·(dq−dq_des)` 把 `dq_des` 拉向实测
  下坠速度 → 参考满速往下追 → GIC 的纠正加速度（每步权重仅 dt）被压过 → 坠向基部。用 run_05b
  实测臂速度曲线复算：ref_damp=62.5 → 追落体饱和 27%（与实测吻合）、ref_damp=15 → 仍 14%、
  只刹不追 → **0%**。→ **回退 ref_damp÷s**（保持基值 15），并加**丢跟踪门限**（见下）。

**最终方案 A（`core/servo_bridge.py` + `run_se3_control.py`，84 测试通过）**：
1. **参考上限 ×s**：`dq_max/qdd_max × s` —— 参考不跑赢被限速的臂；
2. **外环 GIC 带宽 ×s**：`resolve_main_bandwidth` 把有效带宽降为 `ω × s`
   （`--bandwidth 6` 在 24% 下自动降到 ~1.4 rad/s，任务变慢但稳定）；
3. **参考阻尼保持基值 + 丢跟踪门限**：`ref_damp` 不随 s 缩放；compute 里
   `|参考−臂| > 0.15 rad`（伺服丢跟踪，如臂坠）时阻尼只**刹车**不**追臂**
   （`min(dq−dq_des, 0)`），正常跟踪时保持对称阻尼保证收敛。
   这样：正常运行时对称耦合不丢，坠落时参考不被下坠臂拖走，GIC 能把参考拉回目标。

> **给用户的安全建议**：即使完整方案 A 已实现，真机复测仍应 `monitor_rtde.py` 同时
> 录 RTDE，跑完用 `analyze_arm_log.py` 判定是否还有漂移/饱和/发散。若仍乱动，先看
> RTDE `target_speed_fraction` 实际值——它才是 ground truth；方案 B/C 作为后续防线。

**实机（推荐——自动读 RTDE，无需新参数）**：

```bash
conda activate roboarm
mkdir -p logs/run_05

# ① 先确认安全模式回到 NORMAL（safety_mode=0）与回 home
python tests/monitor/monitor_rtde.py --robot ur3 --out logs/run_05/check_safety.csv --duration 3
python tests/monitor/go_home.py --robot ur3 --yes

# ② RTDE 只读记录 + 跑任务（两个终端）
#    终端 2：
python tests/monitor/monitor_rtde.py --robot ur3 --rate 500 --out logs/run_05/rtde.csv
#    终端 1（不传 --servo-speed-fraction：启动时自动读实际速度缩放并缩放参考上限）：
python se3_control/scripts/run_se3_control.py --robot ur3 --control-mode servoJ \
    --task circle --center -0.35 0.0 0.16 --radius 0.05 --duration 16 --bandwidth 6 \
    --log-dir logs/run_05
```

启动时若检测到 `speed_scaling < 0.9` 会打 ⚠️ 告警并自动缩放参考上限 ×s 与外环
带宽 ×s（参考阻尼保持基值）；即使全速启动、运行中安全系统阶梯降速，`run_tracking`
也会每 10 个控制周期从 RTDE 重新读取并**立即**压下来（安全优先），恢复时缓慢爬升。

**CLI 覆盖 `--servo-speed-fraction <s>` 的语义**：只是**启动时的初值**。
真机上运行中每 10 个控制周期仍会从 RTDE 重新读实际速度缩放并覆盖它——若实机
实际满速 (s=1.0)，参考上限/带宽会缓慢爬回满速；若实机降到 0.1，立即降到 0.1
（参考阻尼保持基值 15，不随 s 缩放）。
这是**安全优先**的正确行为：控制器永远跟随机器人真实状态，不被人为参数架空。

```bash
# 例：显式指定 s=0.24 作为启动初值（真机上运行中仍跟随 RTDE 实际值；
#     完整版还会把有效带宽从 6 自动降到 ~1.4 rad/s——任务变慢但稳定）
python se3_control/scripts/run_se3_control.py --robot ur3 --control-mode servoJ \
    --task circle --center -0.35 0.0 0.16 --radius 0.05 --duration 16 --bandwidth 6 \
    --servo-speed-fraction 0.24 --log-dir logs/run_05_forced
```

> **该命令就是 run_05_forced 原样复现用的命令**：v1（只有参考上限 ×s）在此发散
> （闭环失稳），v2（+带宽 ×s +阻尼 ÷s）在此坠向基部（追落体）。**最终版
> （+带宽 ×s +阻尼保持基值 +丢跟踪门限）应稳定**。日志中 `main_bandwidth` 应显示
> ~1.4 而非 6——若显示 6 说明缩放未生效，先查 `target_speed_fraction`。

**仿真预演降速（现状——方案 D 未做前）**：`--preview --servo-speed-fraction <s>`
把 `s` 传给桥接器（参考上限按 0.24 缩放）**并**经 `resolve_main_bandwidth` 把有效带宽
按 s 缩放（仿真里能看到带宽告警），但 **MuJoCo 内层伺服仍是理想计算力矩伺服、无关节
速度上限**——参考以 24% 速度前进、理想伺服完美跟上，圆仍完美，**不能复现** run_04 的
"参考跑赢被限速臂"故障（该故障只有做完方案 D、内层伺服把关节速度钳到 `s × vmax` 后
才能复现）。当前命令用于验证"控制器侧在降速参考下不炸"：

```bash
python se3_control/scripts/run_se3_control.py --robot ur3 --control-mode servoJ \
    --task circle --center -0.35 0.0 0.16 --radius 0.05 --duration 16 --bandwidth 6 \
    --preview --servo-speed-fraction 0.24 --log-dir logs/sim_05_forced
```

**验证稳定**：`analyze_arm_log.py` 判定应不再有积分漂移 / 力矩饱和 / 误差发散
（run_04 同参数、未缩放时：q_servo−q 漂到 4.3 rad、力矩饱和 22%、pos_err 56 cm；
run_05_forced 首版：q_servo−q 有界 ~0.25 rad 但 pos_err 指数发散 λ=+0.31/s）：

```bash
python tests/monitor/analyze_arm_log.py \
    --log logs/run_05/Phase2_*.csv --label 实机 \
    --log logs/sim_05/sim_*.csv      --label 仿真 \
    --rtde logs/run_05/rtde.csv --out logs/analysis_05
```

> 预期：实机与仿真曲线接近、无发散。`rtde.csv` 里 `target_speed_fraction` 若 < 1.0，
> 三项缩放已自动生效，不影响稳定——这正是本方案要消除的"参考跑赢被限速臂" +
> "带宽跑赢降速内环"。
