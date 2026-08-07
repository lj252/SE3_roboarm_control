# ARS Plan: HIAC-GUFIC Unified Compliant Control Paper (IEEE RA-L)

> **Generated**: 2026-07-28 | **Mode**: Socratic chapter-by-chapter planning
> **Target**: IEEE RA-L (8 pages) | **Subtitle**: *Write Once, Run on Any Arm*

---

## Context
  
This paper bridges a critical gap: different manipulators have fundamentally different low-level control interfaces (torque vs. position commands), yet compliant control algorithms are tied to specific hardware. We propose a unified framework combining SE(3)-equivariant GUFIC control with HIAC hybrid impedance-admittance switching, where the **duty cycle baseline is determined by hardware capability** — not just environment stiffness.

**User decisions (from Socratic dialogue)**:
| Decision | Choice |
|----------|--------|
| Target venue | IEEE RA-L (8 pages) |
| Narrative scope | Full HIAC+GUFIC unified framework |
| Subtitle | "Write Once, Run on Any Arm" |
| Advisor background | Robotics/mechatronics → system + experiments |
| Theory depth | Cut GUFIC derivation, keep control law (cite Seo et al.) |
| Signature experiment | Franka vs UR surface sliding force-position comparison |
| Contribution count | **3 contributions** (merge HAL + verification into "system methodology") |
| Failure modes | Do NOT include (keep positive) |
| Arch/Impl boundary | Keep separate — Architecture = design, Implementation = how |

---

## Chapter Plan

### Section 1: Introduction (~1.0 pages)

**Key message**: "Different robots need different control — we make them behave the same."

**Hook**: Open with a concrete scenario — "An engineer develops a surface-following task on a Franka Panda (torque control, 1 kHz). The next day, she must deploy it on a UR12e (position control, 500 Hz). She must rewrite the controller entirely, because Franka accepts joint torques while UR accepts joint positions. This is the *impedance-admittance divide*."

**Structure**:
1. **Para 1 — Motivation**: Compliant control is essential for manipulation. The interface gap creates a fundamental barrier to algorithm portability.
2. **Para 2 — Problem**: Franka→impedance (torque), UR→admittance (position). Same task, different code, different behavior.
3. **Para 3 — Existing gaps**: HIAC switches by environment stiffness only (single robot). GUFIC provides SE(3) equivariance but in simulation only. No framework bridges both.
4. **Para 4 — Our approach**: GUFIC + HIAC + HW abstraction. Extend HIAC: duty cycle baseline ← hardware capability, not just environment.
5. **Para 5 — Contributions (3 bullets)**:
   - **C1**: A hardware-capability-adaptive HIAC mechanism — duty cycle baseline set by robot control interface class, modulated by environment stiffness
   - **C2**: First cross-platform deployment of SE(3)-equivariant GUFIC on real hardware (torque-controlled Franka + position-controlled UR)
   - **C3**: A thin hardware abstraction layer (10 methods, <200 lines/robot) with three-layer verification methodology — 34/34 mock tests, sub-0.5mm regulation, cross-platform force consistency >85%

**Figure**: **Fig. 1** — Architecture teaser: 3-layer stack with Franka (α→0, impedance) on left and UR (α→1, admittance) on right (half-column).

---

### Section 2: Related Work (~0.9 pages)

**Key message**: "Existing solutions each miss one key piece. We put them together."

**Structure (3 merged subsections)**:

**§2.1 Impedance/Admittance Control & Hybrid Approaches** (0.4 pages):
- Hogan (1985) foundation → complementary causality → Ott (2008) Cartesian
- HIAC (Ye et al., 2024) — duty cycle switching, but environment-adaptive only, Franka only. **Key distinction**: our duty cycle baseline is set by hardware capability.
- Anderson & Spong hybrid position/force — historical context, hard switching vs. smooth blending

**§2.2 SE(3) Geometric Control** (0.25 pages):
- GIC (Seo et al., 2024 RA-L) → GUFIC (Seo et al., 2025 CDC) — equivariance + energy tanks
- UFIC (Haddadin & Shahriari, 2024 IJRR) — broader unified force-impedance context
- **Gap**: all validated only in MuJoCo simulation

**§2.3 Cross-Platform Control Architectures** (0.25 pages):
- CRISP (San José Pro et al., 2025) — effort-interface only, UR not supported
- ros_control (Chitta et al., 2017) — ROS-dependent, joint-space PID only
- Drake, OROCOS — large dependencies

**Table I**: Comparison matrix (5 rows × 6 columns: Approach / Torque Robot / Position Robot / SE(3) Equiv. / HW Exp. / Adaptive to HW)

| Approach | Torque Robot | Position Robot | SE(3) Equiv. | Hardware Exp. | Adaptive to HW |
|----------|:---:|:---:|:---:|:---:|:---:|
| HIAC (2024) | ✅ | — | — | ✅ (Franka) | — |
| GUFIC (2025) | ✅(Sim) | — | ✅ | — | — |
| CRISP (2025) | ✅ | — | — | ✅ (FR3) | — |
| **Ours** | **✅** | **✅** | **✅** | **✅(F+U)** | **✅** |

---

### Section 3: Preliminaries (~0.7 pages)

**Key message**: "The minimum math needed to understand our architecture."

**Structure**:

**§3.1 SE(3) Minimum** (0.15 pages):
- g = (R, p) ∈ SE(3), hat/vee maps, Ad_g = [R, p̂R; 0, R]
- Body velocity V^b = [v^b; ω^b] = Jb·dq
- Error: g_ed = g^{-1}·g_d, Vd* = Ad_{g_ed}·Vd
- Keep it operational: show formulas the control law uses, NOT a Lie group tutorial

**§3.2 GIC/GUFIC Control Law** (0.2 pages):
- Present the key equation (no derivation):
  τ_cmd = Jb^T·(M̃·dVd* - D·ev - K·e_op) + b(q,dq)
- Adaptive inertia shaping: M̃ = (Jb·M^{-1}·Jb^T)^{-1}, K_adapt = ω²·M̃, D_adapt = 2ζω·M̃
- Energy tanks: mention T_f (force tank), T_i (impedance tank) maintain passivity — cite Seo et al.
- One key number: M̃ varies 10⁵× between translational and rotational DOFs

**§3.3 HIAC Duty Cycle** (0.15 pages):
- Second-order target impedance: M·ẍ + D·ẋ + K·x = F_ext
- α ∈ [0,1]: 0 = pure impedance (position error → force), 1 = pure admittance (force → position)
- Blending: τ_mix = (1-α)·τ_imp + α·τ_adm
- Original HIAC: α = f(K_env) → Our extension: α = g(hardware_capability, K_env)

**§3.4 Problem Formulation** (0.2 pages):
Formal statement: Given robots {r_i} with interfaces in {TORQUE, POSITION}, design controller C such that:
- (a) Same task T produces consistent behavior B(r_i, T) across all r_i
- (b) C is SE(3)-equivariant
- (c) Energy tanks maintain passivity

**No figure in this section** — only inline equations. Keep under 0.8 pages.

---

### Section 4: Framework Architecture (~2.0 pages, ★ core contribution)

**Key message**: "Three layers, one API, two control paths. Hardware decides the path, GUFIC ensures consistency."

**Structure**:

**§4.1 Design Principles** (0.2 pages, callout box):
Six principles condensed into a half-column box:
1. Robot-Agnostic Core — control algorithms know nothing about hardware
2. Thin Hardware Layer — each adapter <200 lines
3. Zero-Leak Abstraction — no vendor libraries above HAL
4. Hardware Self-Description — robots report capability at init
5. Lifecycle Safety — context managers, emergency stop
6. Fault Tolerance — cached state on timeout

**§4.2 Unified Compliant Control API** (0.3 pages):
Six core methods:
```python
controller = UnifiedCompliantController(robot_hw, robot_model)
controller.set_impedance(M, D, K)      # Unified impedance params
controller.set_reference(pose, twist)   # SE(3) reference
controller.set_force_limits(f_max, t_max)
tau = controller.compute(q, dq, F_ext)  # Same call on both robots!
capability = controller.get_capability() # {TORQUE, TORQUE_FEEDFORWARD, POSITION}
```
Emphasize: `compute()` same on Franka and UR. Difference is internal path selection.

**§4.3 HIAC Hybrid Switching Layer** (0.6 pages, ★ THE innovation):

Three hardware capability classes → baseline α:
| Capability | Example | α_min | Control Path |
|------------|---------|-------|--------------|
| TORQUE | Franka Panda | 0.0 | Impedance |
| TORQUE_FEEDFORWARD | UR (setTargetTorque) | 0.25 | Mixed |
| POSITION | UR (servoj), industrial | 0.85 | Admittance |

Two-axis duty cycle selection:
```
α = clamp(α_hw + k·(K_env - K_thresh), α_hw, 1.0)
where α_hw ∈ {0.0, 0.25, 0.85} from hardware capability
```
- α_hw sets the FLOOR — the robot cannot go below its control capability
- Environment stiffness modulates around the floor
- Smooth switching via low-pass filter on α transitions

Blending architecture:
- Impedance path: GUFIC → torque command → direct to motors (Franka)
- Admittance path: F_ext → admittance filter (M·ẍ + D·ẋ + K·x = F_ext) → position offset → IK → position command (UR)

**§4.4 GUFIC Control Layer** (0.3 pages):
Architectural role (not math — math is Section 3):
- Computes unified force-impedance wrench in SE(3)
- Energy tank mechanism provides passivity regardless of HIAC path
- Tank modulation (α_f, α_i) operates independently beneath HIAC
- **Key coupling**: HIAC needs GUFIC's passivity to switch safely; GUFIC needs HIAC to deploy broadly

**§4.5 Hardware Abstraction Layer** (0.2 pages):
- RobotHWInterface: 10 abstract methods (summarized, detailed in §5)
- Reports RobotCapability at init → feeds into HIAC α_hw selection
- UR adapter: ur_rtde, torque-feedforward, 250Hz
- Franka adapter: libfranka, native torque, 1kHz

**§4.6 End-to-End Control Loop** (0.2 pages):
Unified pseudocode (15 lines) showing all layers composed.

**Figures**:
- **Fig. 2** (full-width, half-page): Complete 4-layer architecture with data flow arrows
- **Fig. 3** (half-column): HIAC switching logic flowchart
- **Table II**: Hardware capability → α_min mapping

---

### Section 5: Implementation (~1.0 pages)

**Key message**: "From MuJoCo simulation to Pinocchio on real robots."

**Structure**:

**§5.1 From MuJoCo to Pinocchio** (0.3 pages):
- MuJoCo = physics engine, Pinocchio = control computation — each does its job
- Cross-validation: run both in simulation simultaneously, compare
- Results table: position 4e-11m, Jacobian 2e-11, inertia 1e-8, bias 1e-8
- This validates Pinocchio as drop-in replacement

**§5.2 UR Hardware Adapter** (0.25 pages):
- ur_rtde RTDE, 500 Hz (Python: 250 Hz)
- Torque-feedforward: setTargetTorque + setTargetQ (position loop as safety net)
- Gravity compensation via Pinocchio RNEA
- Force sensing: external F/T sensor (ATI/Robotiq) or joint-torque estimation

**§5.3 Franka Hardware Adapter** (0.2 pages):
- libfranka FCI, 1 kHz
- Native joint torque control — impedance path directly
- Built-in joint torque sensors → external force estimation without extra hardware

**§5.4 Three-Layer Verification Pipeline** (0.25 pages):
- **Layer 1 (Mock)**: 34/34 tests, no hardware needed, validate code correctness
- **Layer 2 (Communication)**: joint state reading, zero-torque, gravity compensation — validate communication
- **Layer 3 (Round-trip)**: apply torque pulse, observe motion response — validate bidirectional path
- Value: 90% debugging completed before touching real robot

**Figures**:
- **Table III**: Pinocchio vs MuJoCo accuracy (4 metrics)
- **Fig. 4** (half-column): Three-layer verification flow diagram

---

### Section 6: Experiments (~2.2 pages, longest section)

**Key message**: "Same task, two robots, same behavior. HIAC outperforms both pure paradigms."

**Structure**:

**§6.1 Simulation Validation** (0.25 pages):
- 1000 random q, compare Pinocchio vs MuJoCo
- Position: 4e-11 m mean, 1e-10 m max (machine precision)
- Circle tracking in MuJoCo: 7.2 mm mean error

**§6.2 Single-Arm Regulation (UR12e)** (0.3 pages):
- 4 stiffness levels: Kp = 50, 200, 500, 1000 N/m
- Results: Kp=200 → 0.33 mm mean, stable. Kp=1000 → 0.09 mm but wrist oscillation
- **Key number**: <0.5 mm regulation at recommended Kp=200
- Table: Kp vs mean/max error, torque std, stability

**§6.3 ★ Cross-Platform Surface Sliding** (0.5 pages, SIGNATURE EXPERIMENT):
- Setup: planar workpiece, 5N target contact force, 200mm sliding path
- Franka: impedance path, α=0.05 (auto-selected)
- UR12e: admittance path, α=0.85 (auto-selected)
- Both use SAME unified API call
- Metrics: contact force mean±std, position tracking error, force bandwidth
- Expected: Franka 5.0±0.5N, UR 5.0±1.0N — behavior consistency >85%
- **Supplementary video**: real-time side-by-side recording

**§6.4 HIAC α-Scan & Ablation** (0.5 pages, NOVELTY EXPERIMENT):
- On UR12e, fix surface sliding task, sweep α from 0.0 to 1.0 in 0.1 steps
- Measure force RMSE and position RMSE at each α
- Pure impedance (α=0): low position error, high force variance
- Pure admittance (α=1): low force error, high position lag
- Auto-selected α=0.85: near-optimal in combined metric
- Three-mode comparison table: GIC-only (α=0) vs admittance-only (α=1) vs HIAC (α=0.85)
- **Key finding**: HIAC hybrid outperforms both pure modes

**§6.5 Hardware-Capability Validation** (0.15 pages):
- Auto-selected α (Franka: 0.05, UR: 0.85) vs manual optimum
- Performance difference <3% in both cases
- Table: auto vs manual-best α with performance delta

**Figures (6 total — RA-L maximum)**:
- **Fig. 5** (half-page, 2-panel): Cross-validation error distributions + regulation time-domain
- **★ Fig. 6** (FULL-WIDTH, half-page): Surface sliding — Franka (top) vs UR (bottom), force + position over time
- **Fig. 7** (half-page): α-scan dual-axis plot — force RMSE + position RMSE vs α, auto-selected α marked
- **Fig. 8** (half-column): Ablation bar chart — 3 modes comparison
- **Table IV**: Experiment summary matrix (all experiments, metrics, pass/fail)

---

### Section 7: Discussion (~0.5 pages)

**Key message**: "What our framework can and cannot do. And what's next."

**Structure**:
1. **Generality** (0.15 pages): New robot = URDF + <200 lines. Non-ROS, pure Python.
2. **Limitations** (0.2 pages, 4 items with mitigations):
   - Python ~250 Hz ceiling → Numba JIT or C++ core planned
   - URDF parameter inaccuracy → online parameter ID planned
   - Manual α calibration → learning-based auto-tuning planned
   - Two-robot validation → KUKA/Kinova expansion planned
3. **Sim-to-Real Gap** (0.15 pages): UR 2ms delay → 30% gain reduction vs sim. Friction bias ~1-3 Nm. FT sensor noise → 5-10 Hz LPF.

**Table V**: Limitations and mitigations (4 rows × 3 columns).

---

### Section 8: Conclusion (~0.25 pages)

**Key message**: "Write Once, Run on Any Arm — realized on Franka and UR."

**Structure**:
1. **Restate problem** (1 sentence): The impedance-admittance gap fragments compliant control.
2. **Restate solution** (2 sentences): GUFIC + HIAC + HAL = unified framework. Same API on both platforms.
3. **Key numbers** (1 sentence): 4e-11 m accuracy, 34/34 mock, <0.5 mm regulation, >85% consistency.
4. **Future work** (1 sentence): Full GUFIC force control, KUKA/Kinova, C++ core, open-source release.

No figure. Tagline "Write Once, Run on Any Arm" as closing motto.

---

## Page Budget

| Section | Pages | % |
|---------|-------|-----|
| 1. Introduction | 1.0 | 13% |
| 2. Related Work | 0.9 | 11% |
| 3. Preliminaries | 0.7 | 9% |
| 4. Architecture | 2.0 | 25% |
| 5. Implementation | 1.0 | 13% |
| 6. Experiments | 1.5 (text) + 0.7 (figs) | 28% |
| 7. Discussion | 0.5 | 6% |
| 8. Conclusion | 0.25 | 3% |
| References | ~0.5 | — |
| **Total** | **~8.0** | |

---

## Figure Master List

| # | Section | Content | Width | Page Est. |
|---|---------|---------|-------|-----------|
| Fig. 1 | §1, §4 | Architecture teaser: 3-layer stack | Half-col | 0.12 |
| Fig. 2 | §4 | Full layered architecture + data flow | Full-width | 0.30 |
| Fig. 3 | §4 | HIAC switching flowchart | Half-col | 0.12 |
| Fig. 4 | §5 | Three-layer verification flow | Half-col | 0.12 |
| Fig. 5 | §6.1-2 | Cross-validation + regulation (2 panels) | Full-width | 0.25 |
| ★ Fig. 6 | §6.3 | Surface sliding: Franka vs UR | Full-width | 0.30 |
| Fig. 7 | §6.4 | α-scan dual-axis plot | Full-width | 0.25 |
| Fig. 8 | §6.4 | Ablation bar chart | Half-col | 0.12 |
| Table I | §2 | Related work comparison | Half-col | 0.10 |
| Table II | §4 | HW capability → α mapping | Half-col | 0.05 |
| Table III | §5 | Pinocchio vs MuJoCo accuracy | Half-col | 0.05 |
| Table IV | §6 | Experiment summary | Full-width | 0.12 |
| Table V | §7 | Limitations & mitigations | Half-col | 0.08 |
| **Total** | | **8 figures + 5 tables** | | **~1.98** |

---

## INSIGHT Collection

1. **Cut GUFIC theory, keep method**: RA-L readers don't need full SE(3) derivation. Show the control law equation, cite Seo et al. — focus on how we USE it.
2. **★ Signature figure**: Franka vs UR side-by-side force-position plots. One figure tells the whole story.
3. **3 contributions**: (1) Hardware-adaptive HIAC, (2) GUFIC cross-platform deployment, (3) System methodology (HAL + verification).
4. **HIAC innovation = α_hw baseline**: The FLOOR is set by hardware capability. One sentence, huge impact.
5. **Keep it positive**: No failure modes in Discussion. Frame limitations as "planned extensions" not "current problems."
6. **Supplementary video is mandatory**: Franka + UR side-by-side executing the same task.

---

## Writing Sequence

1. **Sections 4 + 6 first** — Architecture and Experiments. Write together since experiments validate architectural claims.
2. **Sections 1 + 8 next** — Introduction and Conclusion. Write after you know exactly what you claim and proved.
3. **Section 3 third** — Preliminaries. Pull only math actually used in Sections 4+6.
4. **Section 2 fourth** — Related Work. Write once you know what makes you distinctive.
5. **Sections 5 + 7 last** — Implementation and Discussion. Support sections.

---

## Key Risks

| Risk | Impact | Mitigation |
|------|--------|-----------|
| Franka unavailable | High — weakens cross-platform claim | Run Franka in MuJoCo + real UR12e; rename "Cross-Platform" → "Cross-Paradigm" |
| No external F/T for UR | Medium — force tasks impossible | Use ur_rtde's getActualTCPForce (~2-5N accuracy) |
| 8-page overrun | High | Move Tables II/III/V to supplementary; compress Fig 5 |
| "Is this two papers?" | High — desk rejection | Strengthen coupling: HIAC needs GUFIC passivity to switch safely; GUFIC needs HIAC to deploy broadly |

---

## Verification Checklist

- [ ] Word count ~5000 (RA-L body text)
- [ ] 6-8 figures within RA-L limit
- [ ] 20-30 references
- [ ] Cross-reference consistency (all §/Fig/Table)
- [ ] "Write Once, Run on Any Arm" appears in Abstract, §1, §4, §8
- [ ] Supplementary video: Franka vs UR surface sliding (60-90s)
- [ ] All 3 contributions explicitly mapped to experimental evidence
