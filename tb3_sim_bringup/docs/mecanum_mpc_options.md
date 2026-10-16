# MPC control for the mecanum Waffle Pi — options (research notes)

Status (updated 2026-10-16): **option C was implemented** as the package `mecanum_mpc`: an LTV-MPC with torque inputs on the exact Chaplygin model, for the own mecanum robot rather than the Waffle Pi. That controller (v1) is kept on the [`v1` branch](https://github.com/Pvaradi05/Mecanum-Robot-MPC-TDK/tree/v1/mecanum_mpc). On `main` it has been removed, and a new controller with a different method is being written from scratch. These notes are the record of v1: references below to `mecanum_mpc`, its README and its tools mean the `v1` branch. See the dated sections from ["Update 2026-10-05"](#update-2026-10-05-goal--dynamic-path-tracking-option-c-linear-vs-nonlinear-mpc) on. The candidate survey below is the original research from 2026-10-01 and is kept as it was.
Context: the mecanum variant in this package (`tb3_waffle_pi_mecanum_sim.launch.py`, see the [README](../README.md#mecanum-variant)) is driven by gz-sim's `MecanumDrive` from `/cmd_vel` (`TwistStamped`, uses `linear.x`, `linear.y`, `angular.z`). Any controller below only needs to publish that.

Repo stats/status below were checked on 2026-10-01 via the GitHub API — re-check before relying on them.

## Bottom line

- Public MPC code for mecanum robots **exists but is thin**: one maintained mecanum-oriented MPC (Neobotix), one mainstream sampling-based MPC with an omni model (Nav2 MPPI), and otherwise ROS 1 / Python-only / unlicensed research code.
- **Almost nobody publishes performance numbers.** Repos show GIFs; the only figure found is Neobotix's "~50 Hz on an Intel i5". Papers with real-hardware numbers don't ship code. → we'll have to benchmark ourselves (see [Evaluation plan](#evaluation-plan)).

## Candidates

### Tier 1 — usable on Jazzy now

**A. Nav2 MPPI controller, `motion_model: "Omni"`** — sampling-based MPC (Model Predictive Path Integral), Nav2's flagship local controller.
- Install: `ros-jazzy-nav2-mppi-controller` (apt candidate 1.3.13 on 2026-10-01; not installed yet). Needs the rest of Nav2 + a map + localization (AMCL or SLAM).
- Omni-relevant settings (from Nav2 docs and ROBOTIS's AI Worker write-up): `motion_model: "Omni"`; non-zero `vy_max` / `vy_min`; disable or minimise `PreferForwardCritic`; lower `PathAlignCritic` weight so heading decouples from path direction; keep `GoalAngleCritic` moderate; keep `CostCritic` + `PathFollowCritic`.
- Mecanum-specific upstream work, **both open/unmerged** as of 2026-10-01:
  - PR #6460 *AxisAlignCritic* (opened 2026-09-06) — penalises simultaneous vx+vy. Motivation: on real mecanum hardware 45° diagonal motion drives only two wheels and slips; pure forward/lateral drives all four. Tested in Gazebo Classic + Isaac Sim.
  - PR #6390 — velocity limits as an ellipse instead of a rectangle (today the diagonal can exceed the per-axis max, and speed-seeking critics push toward that corner, fighting heading-alignment critics). Related issue #6330.
- Performance evidence: qualitative only (ROBOTIS GIFs: smoother in tight spaces than Regulated Pure Pursuit). No numbers.
- Pros: lowest effort, standard, maintained, C++, obstacle avoidance via costmap. Cons: not a "classic" gradient-based MPC; tuning is critic-weight juggling.

**B. neobotix/neo_mpc_planner2** — gradient-based MPC as a Nav2 controller plugin, written for Neobotix MPO-700 (quasi-omni) and stated to work on the mecanum MPO-500.
- MIT, 66 stars, 16 open issues, last commit 2025-12-01. Branches: `humble`, `kilted`, `rolling`, `fix-update-footprint-jazzy` (no plain `jazzy` branch → expect some build work).
- Optimiser: SciPy (Python 3, sample config says SciPy 1.6.3), with an optimisation server node; depends on `neo_nav2_py_costmap2D` and `neo_srvs2`.
- Parameters: per-axis velocity/accel limits (`max_vel_x/y/theta`, `acc_x/y/theta_limit`), weights `w_trans`, `w_orient`, `w_control`, `w_terminal`, `w_costmap`, `w_footprint`, lookahead distances, `control_steps`.
- Performance: "average rate of 50 Hz" on an Intel i5. Limitation: **stops** when an unknown obstacle is in the predicted path (doesn't re-plan around it).
- Pros: genuine MPC, built by a company that ships mecanum robots. Cons: Python/SciPy solver, Jazzy port effort, small community.

### Tier 2 — research/hobby code (port or borrow ideas)

| Repo | What | ROS | Last push | Licence | Notes |
|---|---|---|---|---|---|
| duynamrcv/mpc_casadi | CasADi NMPC, omni robot tracking + navigation | ROS 1 (catkin) | 2022-04 | none | 50 stars; no results published |
| duynamrcv/nmpc_casadi_python | CasADi NMPC, 3-wheel omni, kinematic + dynamic model, obstacle avoidance | none (Python sim) | 2022-01 | none | 33 stars; formulation reference only |
| duynamrcv/mpc_ros | ACADO MPC | ROS 1 | 2021-12 | none | |
| Geonhee-LEE/mpc_ros | NMPC path tracking | ROS 1 (melodic) | 2023-05 | Apache-2.0 | 978 stars but **diff-drive** |
| matthew-t-watson/CollinearMecanumDrive | MATLAB/Simulink MPC | — | 2020-06 | GPL-3.0 | balancing collinear-mecanum robot; theory only |

No licence = legally can't reuse the code, only the ideas.

### Tier 3 — frameworks to build our own

- **tud-amr/mpc_planner** — C++ MPC motion planner, ROS + ROS 2, solvers acados (free) or Forces Pro (licensed; their paper results used Forces Pro). Apache-2.0, 405 stars, last push 2025-03. Built around Clearpath Jackal (diff-drive) → swap in a mecanum model.
- **SokhengDin/NMPC-ACADOS-CPP** — small template: define model in Python, generate C, run in C++. Diff-drive/bicycle examples only. 17 stars, no licence.
- Plain **acados** or **CasADi** (neither installed; `casadi` not importable on this machine). Jazzy apt also has `ros-jazzy-acado-vendor` (older ACADO toolkit).
- Ruled out: `easynav_mpc_controller` (diff-drive only, NLopt), `ackermann_nlmpc` (car-like).

### Literature with hardware results (no code)
- Sliding-mode-observer-based MPC for a mecanum robot — lower RMSE/overshoot/chattering vs fuzzy sliding mode; two real-world experiments.
- ANFIS-tuned MPC — claims >50% lower distance error and >70% lower angle error vs traditional MPC.
- "MPC for a Mecanum-wheeled robot navigating among obstacles" / "…in dynamical environments" (ResearchGate).
Useful for the problem formulation if we write our own (option C).

## Options for implementation (ranked by effort)

1. **A — Nav2 + MPPI Omni.** Lowest effort, highest chance of "just works". Prerequisite: Nav2 + a map (do SLAM with Cartographer/slam_toolbox first, or localise in a saved map of `turtlebot3_world`).
2. **B — neo_mpc_planner2.** The closest true MPC built for mecanum. Expect Jazzy porting work. Good to run head-to-head against A.
3. **C — own tracking NMPC node** (CasADi or acados), publishing `/cmd_vel`.
   - Model: `ẋ = vx·cosθ − vy·sinθ`, `ẏ = vx·sinθ + vy·cosθ`, `θ̇ = ω`; inputs `(vx, vy, ω)`.
   - Mecanum-specific improvement nobody above does: constrain the **per-wheel speeds** instead of a per-axis box, `|vx ∓ vy ∓ (lx+ly)ω| / r ≤ ω_wheel_max` (the IK verified in this package: `lx+ly = 0.224 m`, `r = 0.033 m`). That's the robot's true velocity envelope (a polytope), and it also handles the diagonal-speed problem PR #6390 addresses.
   - Optional: penalise vx·vy to mimic AxisAlignCritic.
   - Scaffold option: tud-amr/mpc_planner with the model swapped.
4. **Make the sim less ideal** (independent of the choice above). Current sim has zero lateral roller resistance, perfect motors and slip-free odometry, so the diagonal-slip problem that motivates PR #6460 **cannot appear**. Lowering the `fdir1` grip, adding wheel-speed / odometry noise, or limiting wheel effort would let us see whether MPC helps where real robots struggle.

## Evaluation plan

Reuse the ground-truth approach from the mecanum verification (read `/world/default/pose/info` from Gazebo; time everything in **sim time** — WSL stalls make wall-clock measurements unreliable, real-time factor dips to ~0.06).

Metrics per controller, on the same paths (straight, lateral-only, diagonal, figure-8, tight corridor with obstacles):
- cross-track error (RMSE, max) and heading error vs ground truth
- time to goal / completion rate
- solve time per step (mean, p95) and achieved control rate
- how often wheel-speed limits saturate
- smoothness (jerk in `/cmd_vel`)

## Open questions to settle before implementing

- Goal: autonomous navigation with obstacle avoidance (→ A or B, needs Nav2 + map), or precise trajectory tracking of given paths (→ C)?
- Is real hardware planned? If yes, prioritise slip robustness (option 4 + AxisAlignCritic-style penalty).
- Python acceptable in the control loop, or C++ only?
- ~~Final robot geometry~~ — settled 2026-10-05: own robot is in (`own_mecanum_sim.launch.py`, see README): `lx = 0.064`, `ly = 0.1526` → `lx+ly = 0.2166 m`, `r = 0.03028 m`. Use these (not the Waffle Pi's 0.224 / 0.033) in the per-wheel constraint of option C. Its rollers are physically modelled, so it already slips a little (strafe ~2–3 %), which partly covers option 4.

## Sources
- Nav2 MPPI: https://docs.ros.org/en/humble/p/nav2_mppi_controller/standard_docs/README.html · https://navigation.ros.org/configuration/packages/configuring-mppic.html
- navigation2 PR #6460: https://github.com/ros-navigation/navigation2/pull/6460 · PR #6390: https://github.com/ros-navigation/navigation2/pull/6390 · issue #6330: https://github.com/ros-navigation/navigation2/issues/6330
- ROBOTIS AI Worker × Nav2: https://ai.robotis.com/ai_worker/technical_story_ai_worker_Nav2
- https://github.com/neobotix/neo_mpc_planner2
- https://github.com/duynamrcv/mpc_casadi · https://github.com/duynamrcv/nmpc_casadi_python · https://github.com/duynamrcv/mpc_ros
- https://github.com/Geonhee-LEE/mpc_ros · https://github.com/matthew-t-watson/CollinearMecanumDrive
- https://github.com/tud-amr/mpc_planner · https://github.com/SokhengDin/NMPC-ACADOS-CPP
- https://index.ros.org/p/easynav_mpc_controller/
- Papers: https://www.researchgate.net/publication/381104612 · https://www.researchgate.net/publication/354480639 · https://www.researchgate.net/publication/334319114

## Update 2026-10-05: goal = dynamic path tracking (option C), linear vs nonlinear MPC

Goal settled with the user: **track circle, ellipse, figure-8 and square paths** with an MPC built on the **exact (Chaplygin) dynamics** of Zeidis & Zimmermann, *Dynamics of a four-wheeled mobile robot with Mecanum wheels*, ZAMM 2019, doi:10.1002/zamm.201900173.

Findings ([analysis/](analysis/), run with `python3`):
- `chaplygin_check.py` — the paper's exact model eq. (64) is **identical** (max diff 8e-9) to the body-frame form
  `me·u̇ − ms·v·r = Fξ`, `me·v̇ + ms·u·r = Fη`, `Je·ṙ = τ`, with `me = ms + 4J1/R²`, `Je = JC + 4J1(ρ+l)²/R²`, body wrench `= Jᵀ M` (J from eq. 32). Note the Coriolis terms use `ms` (body), not `me`; that is exactly the non-holonomic correction. The only nonlinearities are these bilinear terms plus the cos ψ / sin ψ pose kinematics.
- `lin_check.py` — own robot, 0.2 m/s: the bilinear term equals the centripetal force, at most 0.12 / 0.32 / 0.58 N (circle / ellipse / figure-8), and only with a **tangent heading**. With a **fixed heading** it is exactly 0, so the ideal model is exactly linear (the paper's translational case, eqs. 72–74). Over a 1 s horizon, starting 5 cm / 10° / 0.05 m/s off the reference, LTV linearisation mispredicts the position by ≈ 1 cm. Re-linearising at the current state: 0 (fixed heading) to 1.4 cm (figure-8, tangent heading).
- Forces needed at 0.2 m/s (≤ 1.3 N) are far below what the motors can give (4 × 0.6 N·m / 0.03 m ≈ 80 N), so the torque limits never bind at teleop speeds. Dynamics only start to matter near the motor speed limit (35 rad/s · 0.03 m ≈ 1 m/s) or with slip.
- `J1` dominates: with the gearmotor's reflected inertia (9e-4 kg·m², a guess in robot_config.json), `me` is 7.1 kg against `ms` = 3.0 kg.

Practical consequences for the implementation:
- The sim's `MecanumDrive` commands ideal wheel **velocities**. A torque-input MPC needs effort-controlled wheel joints (e.g. gz `ApplyJointForce` per joint, or ros2_control effort interfaces) and its own odometry.
- 4 torques for 3 DOF: use the body wrench as the MPC input and allocate to the 4 wheel torques (minimum norm), or keep 4 inputs and regularise.
- The chassis CoM is 16 mm ahead of the wheel centre (robot_config.json); the paper assumes it is centred.

## LTV-MPC implemented (2026-10-05)

Package `src/mecanum_mpc` (see its README): LTV-MPC with **torque inputs** on the Chaplygin model, parameters read from the own robot's URDF. Sim with `own_mecanum_sim.launch.py drive:=effort` (ApplyJointForce wheels, ground-truth `/odom`). Run: `ros2 launch mecanum_mpc mpc_sim.launch.py shape:=… heading:=…`.

Gazebo, physical 6-sphere rollers, 0.2 m/s, one lap, N = 20, Δt = 0.05 s. Metrics after the first 3 s (start-up breakaway). Logs in `~/ros2_ws/mpc_logs/`, plot `results_2026-10-05.png`.

| path | heading | tracking RMS / max | heading err RMS / max | max wheel torque |
|---|---|---|---|---|
| circle (r 0.75 m) | fixed | 3.6 / 5.9 mm | 0.40 / 1.41° | 0.014 N·m |
| circle | tangent | 1.7 / 2.9 mm | 0.47 / 0.98° | 0.009 N·m |
| ellipse (1.0 × 0.5 m) | fixed | 3.2 / 6.9 mm | 0.30 / 1.02° | 0.016 N·m |
| ellipse | tangent | 1.8 / 3.1 mm | 0.57 / 1.63° | 0.012 N·m |
| figure-8 (2 × 1 m) | fixed | 3.4 / 6.2 mm | 0.39 / 1.47° | 0.020 N·m |
| figure-8 | tangent | 1.9 / 6.4 mm | 0.57 / 1.72° | 0.017 N·m |
| square (1.5 m) | fixed | 3.2 / 5.4 mm | 0.31 / 1.12° | 0.017 N·m |
| square | tangent | 2.0 / 5.7 mm | 0.45 / 1.66° | 0.015 N·m |

QP solve: 0.6 ms median, 1.2 ms p95; a full control step in Python takes 2–7 ms. The first 3 s peak at about 10 mm: the wheels need 0.01–0.02 N·m to break away from standstill.

Observations, not yet explained:
- **Fixed heading has about 2× the error of tangent heading** (steady ~3 mm). Fixed heading means more sideways and diagonal rolling, where the rollers spin. Rolling resistance and roller inertia are not in the model (see the mecanum_mpc README, "Sim vs model"). The CoM offset (13.9 mm) is another candidate. → Narrowed down on 2026-10-16: the extra error is a lag along the path while strafing, not a deviation from the path. See [below](#fixed-vs-tangent-heading-the-difference-is-timing-2026-10-16).
- **One-off glitches:** gz `OdometryPublisher` emits single yaw-rate samples of ±600 rad/s (about 2π/0.01 s). The node now rejects them; before that, they caused one "infeasible" QP per run.
- **Control-timing gap:** one run had a 0.28 s gap between control steps (a WSL stall while the sim kept going), which gave a 27.6 mm spike during start-up.

Next candidates: NMPC (acados SQP-RTI) on the same model and paths; higher speeds (fixed-heading wheel-speed limit ≈ 0.64 m/s); integral or disturbance-observer action against the bias; adding roller inertia and rolling resistance to the model.

## Corner-radius benchmark (2026-10-12)

**Question:** how does the LTV-MPC cope as the square's corners get sharper, and what does the feedforward wrench `w_ref` contribute?

**Setup:**
- 1.5 m square at 0.2 m/s, one lap per run, a fresh headless Gazebo per run.
- Corner radius 22.5 cm (the default), 12, 6, 3 and 0 cm (sharp).
- Both heading modes.
- Three feedforward variants:
  - **sampled**: `w_ref` at the step time, the default;
  - **none**: `force_reference:=false`;
  - **averaged**: `average_force_reference:=true`, mean over each 50 ms step. Run only at radius 0.
- The 22.5 cm and 3 cm runs were repeated twice more. Between repeats, RMS varied by under 0.2 mm and max by up to 2 mm.
- The scripts, logs and charts are in `~/ros2_ws/mpc_logs/corner_radius_benchmark/`, outside this repo (`run_benchmark.py`, `plot_benchmark.py`, `corner_radius_{tangent,fixed}.png`, `summary.csv`).

Tracking error RMS / max [mm], from t = 3 s to the end of the lap:

| corner radius | tangent, sampled | tangent, none | tangent, averaged | fixed, sampled | fixed, none | fixed, averaged |
|---|---|---|---|---|---|---|
| 22.5 cm | 2.1 / 6.9 | 1.9 / 6.1 | | 3.2 / 5.5 | 3.3 / 6.3 | |
| 12 cm | 1.9 / 3.9 | 1.9 / 4.0 | | 3.2 / 5.4 | 3.1 / 5.4 | |
| 6 cm | 1.9 / 4.6 | 1.9 / 3.9 | | 3.1 / 5.6 | 3.2 / 5.5 | |
| 3 cm | 1.9 / 5.0 | 2.1 / 5.5 | | 3.1 / 6.0 | 3.2 / 6.9 | |
| 0 (sharp) | **4.4 / 24.5** | 2.2 / 10.6 | **2.0 / 7.8** | **6.1 / 33.8** | 3.3 / 12.6 | **3.2 / 9.5** |

Findings:
- **Rounded corners, even 3 cm: the feedforward makes no difference** beyond run-to-run spread. At 0.2 m/s the forces are tiny (wheel torques ≤ 0.045 N·m), and the feedback does the work. That may change at higher speeds, which haven't been tested.
- **Sharp corners: the sampled feedforward hurts.** The reference velocity jumps, so the exact `w_ref` is an impulse. Sampled at one instant (0, or 160–220 N) and held for 50 ms, it overshoots. Averaged over the step, it is at most 6–9 N.
  - At every corner there is a spike of 24–34 mm, and wheel torque reaches 0.13–0.20 N·m, against 0.03–0.05 N·m otherwise.
  - Averaging `w_ref` over the step removes the spikes: RMS is as good as on rounded corners, and the max is lower than with no feedforward.
  - **Use `average_force_reference:=true` with sharp corners.** It is still off by default.
- **Distance from the path** is 0.7–1.4 mm RMS (max 2.6–7.4 mm) for every run except sharp corners with the sampled feedforward (3.5–4.0 mm RMS). That holds for both heading modes, so most of the tracking error is along the path, i.e. timing.
- **The `crosstrack_*` columns in `summary.csv` are not valid.** They measure the distance to the nearest logged reference point, and those points are 10 mm apart, which gives about 2.8 mm RMS even for a robot exactly on the path. The cross-track numbers here were recomputed against the path polyline. The same metric is in `mecanum_mpc evaluate`.

## Fixed vs tangent heading: the difference is timing (2026-10-16)

Fixed-heading square from the benchmark logs (radius 22.5, 12 and 3 cm), split by which way the robot drives on each side:

| side driven | position error RMS | along the path (mean) | across the path (mean) |
|---|---|---|---|
| forward (u = ±0.2 m/s) | 2.0 mm | 1.3 mm **ahead** | ≤ 0.3 mm |
| sideways (v = ±0.2 m/s) | 3.8–3.9 mm | 3.6 mm **behind** | ≤ 0.3 mm |
| (tangent heading, always forward) | 2.1 mm | 0.9 mm ahead | 0.6 mm |

**The extra error of fixed heading is a constant lag while strafing.** The robot stays on the path but runs late.
- **Why a constant lag:** some steady resistance in sideways motion is missing from the model. Only the MPC's feedback opposes it, and the feedback has no integral action.
- **Plausible source, not verified:** driving straight forward, the roller in contact doesn't turn; driving sideways, the rollers spin. The roller joints have damping 1e-5 N·m·s/rad in `robot.urdf`, plus spin inertia, and neither is in the model.
- **The small lead when driving forward** may be the 0.003 N·m friction feedforward overcompensating. Also unverified. The weighed masses (next section) point the same way: on the lighter robot the lead grows.

Fix candidates:
- integral or disturbance-observer action;
- a friction feedforward that depends on the direction of travel;
- roller damping in the model.

## Weighed masses (2026-10-16)

The robot was weighed: 1927.4 g for the body without wheels and 51.65 g per wheel, 2.134 kg in total. `robot.urdf` now has these masses, with every component homogeneous. Method and table: tb3_sim_bringup README, "Masses". Applied by [`analysis/set_masses.py`](analysis/set_masses.py).

**Model parameters, old guesses → weighed:**

| parameter | old | weighed |
|---|---|---|
| m_s | 3.0 kg | 2.134 kg |
| m_e | 3.17 kg | 2.21 kg |
| J_e | | 0.0304 kg·m² |
| J₁ | | 1.85e-5 kg·m² |
| CoM offset | 13.9 mm | 1.0 mm |

**Everything measured before this date used the old masses:** the tables above, the benchmark, and `lin_check.py`, whose parameters are hardcoded. The friction feedforward (0.003 N·m) and the "Sim vs model" numbers in the mecanum_mpc README were also measured on the 3.0 kg robot.

Quick check, one headless lap each, metrics after t = 3 s:

| run | old masses | weighed masses | mean offset along the path, weighed |
|---|---|---|---|
| square (1.5 m), fixed | 3.2 / 5.4 mm | **2.5 / 5.6 mm** | forward sides 2.3 mm ahead, sideways sides 1.35 mm behind (was 1.3 / 3.6) |
| figure-8, tangent | 1.9 / 6.4 mm | **2.5 / 4.0 mm** | 2.05 mm ahead (was ~0.9) |

Tracking error is RMS / max. The sim stays stable with the lighter rollers (2.1 g instead of 5 g): no failed solves and no fallbacks.
- **Fixed heading improved:** less sideways lag.
- **Tangent heading got worse:** the robot runs ahead of the reference.

Both fit a friction feedforward that is now too large for the lighter robot: rolling resistance scales with the load, and 0.003 × 2.134/3.0 ≈ 0.002 N·m. Next: re-measure the rolling resistance, or retune `friction_torque`, then re-run the result tables.
