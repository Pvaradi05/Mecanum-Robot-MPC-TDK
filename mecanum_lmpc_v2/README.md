# mecanum_lmpc_v2

**Linear MPC v2** for the own mecanum robot. It is a condensed linear time-varying MPC after Kühne, Gomes da Silva Jr. and Lages, *Mobile robot trajectory tracking using model predictive control* (SBAI/LARS 2005, eq. 15–20). It predicts with the exact Chaplygin dynamics of Zeidis & Zimmermann, *Dynamics of a four-wheeled mobile robot with Mecanum wheels* (ZAMM 2019), discretised with RK4.

- **State:** z = [X, Y, ψ, ω₁, ω₂, ω₃]: the world-frame pose and the speeds of the front-left, front-right and rear-left wheels (ω₄ = ω₁ + ω₂ − ω₃).
- **Input:** the four wheel torques u = [M1, M2, M3, M4], in N·m.

The design, every equation and the decisions D1–D8 are in the plan document "Chaplygin LTV-MPC for the own mecanum robot: implementation plan". Since 2026-10-30, `lmpc.py` is the LTV-MPC of the RL comparison, synced from [Kemenylegeny/Mecanum-tracking-tdk](https://github.com/Kemenylegeny/Mecanum-tracking-tdk) @ 411436a. That file started as this package's `lmpc.py` @ fe01f88 and added the DARE terminal weight, a torque-change penalty S, a path-frame position weight and the wheel-speed state, and removed the hard torque-rate limit. The sync changes only its imports, and moves `solve_dare` unchanged to `mecanum_mpc_common/dare.py` so the NMPC uses the same function.

The node, the offline simulation and the tests run it in the **latest configuration** of that repository (the defaults of its `scripts/rsl_rl/mpc_controller.py`), `mecanum_lmpc_v2.LATEST`:
- **Weights:** Q_BO, the best of its closed-loop Bayesian optimisation in Isaac Lab: q = (10⁷, 10⁷, 10⁶, 17.59, 17.59, 15.22) on the errors of (X, Y, ψ, Ẋ, Ẏ, ψ̇), with the velocity weights acting on the wheel speeds as J3ᵀ diag(q_v) J3; R = 400·I₄. **In Gazebo it diverges** (2026-10-30, square path, even without a start error: a growing step-to-step oscillation). With Bryson's weights, `-p Q:=[40000.0,40000.0,3283.0,2500.0,2500.0,100.0]`, it tracks to 1.0 mm RMS; the NMPC uses those for now.
- **Terminal weight:** the DARE solution at the last horizon step, warm-started with the previous step's.
- **Constraints:** the torque box only. No torque-rate limit, no soft wheel-speed limit, no S, no path-frame weight.
- **Measurement:** the wheel speeds from the odometry's base velocity through the rolling constraints ("kinematic").

The model, reference, motor limits, ROS node and tools are shared with the nonlinear MPC (`mecanum_nmpc`) through `mecanum_mpc_common`, so the two controllers differ only in their optimiser.

The v1 controller (a different LTV-MPC) is on branch `v1`. Nothing here is copied from it.

## Run

```bash
# prerequisites: ROS 2 Jazzy, Gazebo Harmonic, tb3_sim_bringup, mecanum_mpc_common; QP solver from pip:
pip install --user --break-system-packages quadprog
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash

ros2 launch mecanum_lmpc_v2 lmpc_v2_sim.launch.py shape:=square heading:=tangent
#   shape:=circle|ellipse|figure8|square   heading:=fixed|tangent   speed:=0.1
#   loop:=true (repeat the path until Ctrl-C)   size:=0 (default size)   corner_radius:=-1 (15 %)
#   world:=empty_world   gui:=true|false

ros2 run mecanum_lmpc_v2 offline_sim [--plant perturbed]                   # no ROS, no Gazebo
ros2 run mecanum_lmpc_v2 sign_test --ros-args -p use_sim_time:=true        # with the sim running
ros2 run mecanum_mpc_common evaluate ~/ros2_ws/mpc_logs/lmpc_v2/*.csv --skip 3   # metrics
python3 -m pytest src/mecanum_lmpc_v2/test -q                              # plan tests 1-7
```

The launch starts `own_mecanum_sim.launch.py drive:=effort` (torque-driven wheels, ground-truth `/odom`). After 8 s it starts `lmpc_v2_node`. On the first odometry message the path is anchored at the robot's pose, and the run starts once the torque bridge is connected. Without `loop`, the robot drives the path once and then holds the end pose.

**Topics:**
- Wheel torques: `/model/own_mecanum/joint/<corner>_wheel_joint/cmd_force`.
- Visualisation: `/lmpc_v2/reference` and `/lmpc_v2/predicted` (nav_msgs/Path).
- Status: `/lmpc_v2/status` (Float64MultiArray, field names in the layout label).

Each run writes one CSV row per control step to `~/ros2_ws/mpc_logs/lmpc_v2/<shape>_<heading>_<speed>_<time>.csv`. The last two columns are `slack_max`, the largest wheel-speed slack of the step (0 while the soft limit is off), and `dare_iters`, the Riccati iterations of the step's terminal weight.

**Parameters:** the common ones (`T`, `N`, `Q`, `R`, `u_max`, `omega_max`), and the switches of the comparison repository's `mpc_controller.py`, defaults = `LATEST`: `terminal` (`dare` or `scaled`, P = `P_scale`·Q), `P_scale` (5.0), `S` (0 = off), `Q_path` ([qa, qc], [0, 0] = off), `wheel_speed_limit` (false).

Overhead videos and charts of runs: `record_video` and `plot_runs` in `mecanum_mpc_common` (see its README).

## Motor limits (DYNAMIXEL XM430-W210-T, 12.0 V)

Each wheel is driven by one ROBOTIS DYNAMIXEL XM430-W210-T, run at 12.0 V (the recommended voltage). The data comes from the ROBOTIS e-manual: https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/. The values are in `mecanum_mpc_common/motor.py`, and each one can be overridden through a node parameter.

| parameter | value | source | used for |
| --- | --- | --- | --- |
| `u_max` | 1.95 N·m | highest torque on the 12 V performance graph (at about 11 rpm, 2.05 A) | torque box in the QP, final clip, start-up check |
| `omega_max` | 8.06 rad/s (77 rpm) | no-load speed at 12 V | start-up check of the reference; rejecting implausible odometry (2·ω_max, LMPC node only); the soft wheel-speed limit when switched on |

The motor has no torque-rate limit in Current Control Mode (Profile Acceleration does not act there). The hard torque-rate limit of D6 (`du_max` = 1.95 N·m per step) was removed on 2026-10-30, as in the comparison repository.

Notes:
- The 3.0 N·m stall torque at 12 V is a locked-rotor value, not a working limit. The performance graph ends at about 1.95 N·m.
- The datasheet gives no continuous torque rating and no rotor inertia.
- At 0.2 m/s, every path needs 81–129 rpm, which is above the 77 rpm limit. So the default path speed is 0.1 m/s, which is feasible on every path.
- The Gazebo model does not enforce these limits on the wheel joints. Only the controllers respect them.

## Soft wheel-speed limit (off by default)

When switched on (`wheel_speed_limit:=true`), the LMPC keeps every wheel's speed within ω_max as a soft constraint: |ω_j| ≤ ω_max + s_j with s_j ≥ 0 and a penalty 10⁴·s + 10⁴·s² per wheel and step, linearised along the reference (exact in the wheel-speed state), with 4N more rows and 4N slack variables in its QP. The comparison repository switched it off on 2026-10-28: at 0.9·v_max it kept the robot from catching up where the reference runs near the wheel-speed limit. The NMPC has no such limit.

## How it is built

| file | plan section |
| --- | --- |
| `lmpc.py` | One control step: linearisation, free response e, prediction matrix B̄, H, f, DARE terminal weight, torque box (+ the optional S, Q_path and soft wheel-speed limit), quadprog (D8), control law, fallback |
| `__init__.py` | `LATEST`, the latest configuration |
| `lmpc_v2_node.py` | The shared node (`mecanum_mpc_common.mpc_node`) with `LinearMPCv2` inside |
| `offline_sim.py` | The shared offline closed loop with `LinearMPCv2` (plan test 7) |
| `sign_test.py` | Plan test 8 |

Shared, in `mecanum_mpc_common`: robot parameters, dynamics and RK4 Jacobians (D2) in both states, reference with step-averaged u_r (D3), the DARE, the weights, motor limits, wheel speeds, node, offline plant and the tools.

The other settings follow the plan's decisions:
- **Horizon:** T = 0.05 s, N = 10, a 0.5 s horizon (D4; N = 20 until 2026-10-21, when a sweep of N = 5, 10 and 20 showed the same tracking for both controllers at a third of the LMPC's compute time).
- **Model:** no friction model and no integral action (D5).
- **Reference heading:** not smoothed. Decided 2026-10-21.

Before 2026-10-30 this package used the world-frame velocity state, the plan's starting weights Q = diag(40000, 40000, 3283, 2500, 2500, 100), R = 400·I₄ and P = 5·Q (D1), a hard torque-rate limit (D6) and the soft wheel-speed limit: commit fe01f88.

## Results

See the plan documents. Offline, against the nominal model, every path tracks to under 0.03 mm RMS.
