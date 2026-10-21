# mecanum_lmpc_v2

**Linear MPC v2** for the own mecanum robot. It is a condensed linear time-varying MPC after Kühne, Gomes da Silva Jr. and Lages, *Mobile robot trajectory tracking using model predictive control* (SBAI/LARS 2005, eq. 15–20). It predicts with the exact Chaplygin dynamics of Zeidis & Zimmermann, *Dynamics of a four-wheeled mobile robot with Mecanum wheels* (ZAMM 2019), discretised with RK4.

- **State:** x = [X, Y, ψ, Ẋ, Ẏ, ψ̇], in the world frame.
- **Input:** the four wheel torques u = [M1, M2, M3, M4], in N·m.

The design, every equation and the decisions D1–D8 are in the plan document "Chaplygin LTV-MPC for the own mecanum robot: implementation plan". This package implements that plan. Since 2026-10-21 it also has the soft wheel-speed limit of the NMPC plan (decision N5, replacing D7).

The model, reference, motor limits, ROS node and tools are shared with the nonlinear MPC (`mecanum_nmpc`) through `mecanum_mpc_common`, so the two controllers differ only in their optimiser. The move into the common package was checked bit-identical: the same torques and predictions on 1,600 test inputs.

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

Each run writes one CSV row per control step to `~/ros2_ws/mpc_logs/lmpc_v2/<shape>_<heading>_<speed>_<time>.csv`. The last column, `slack_max`, is the largest wheel-speed slack of the step.

Overhead videos and charts of runs: `record_video` and `plot_runs` in `mecanum_mpc_common` (see its README).

## Motor limits (DYNAMIXEL XM430-W210-T, 12.0 V)

Each wheel is driven by one ROBOTIS DYNAMIXEL XM430-W210-T, run at 12.0 V (the recommended voltage). The data comes from the ROBOTIS e-manual: https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/. The values are in `mecanum_mpc_common/motor.py`, and each one can be overridden through a node parameter.

| parameter | value | source | used for |
| --- | --- | --- | --- |
| `u_max` | 1.95 N·m | highest torque on the 12 V performance graph (at about 11 rpm, 2.05 A) | torque box in the QP, final clip, start-up check |
| `omega_max` | 8.06 rad/s (77 rpm) | no-load speed at 12 V | soft wheel-speed limit; start-up check of the reference; rejecting implausible odometry (2·ω_max) |
| `du_max` | 1.95 N·m per 50 ms step | = `u_max`: the datasheet gives no torque-rate limit, and Profile Acceleration does not act in Current Control Mode | hard torque-rate limit (D6) |

Notes:
- The 3.0 N·m stall torque at 12 V is a locked-rotor value, not a working limit. The performance graph ends at about 1.95 N·m.
- The datasheet gives no continuous torque rating and no rotor inertia.
- At 0.2 m/s, every path needs 81–129 rpm, which is above the 77 rpm limit. So the default path speed is 0.1 m/s, which is feasible on every path.
- The Gazebo model does not enforce these limits on the wheel joints. Only the controllers respect them.

## Soft wheel-speed limit (NMPC plan, N5)

Both controllers keep every wheel's speed within ω_max, as a soft constraint: |ω_j| ≤ ω_max + s_j with s_j ≥ 0 and a penalty `slack_lin`·s + `slack_quad`·s² per wheel and step. The LMPC linearises the wheel speeds along the reference, ω(x_r,j) + G_j x̃_j, which gives 4N more rows and 4N slack variables in its QP. The NMPC imposes the same limit exactly.

The weights are 10⁴ and 10⁴ (decided 2026-10-21). That is 17× the largest shadow price of the hard limit measured in this QP (585 per rad/s at a 30 cm / 30° start error), so the limit acts as a hard one whenever it can be met, and the QP stays solvable when it cannot. Parameters: `wheel_speed_limit` (default true), `slack_lin`, `slack_quad`.

## How it is built

| file | plan section |
| --- | --- |
| `lmpc.py` | One control step: linearisation, free response e, prediction matrix B̄, H, f, torque box + hard rate limit + soft wheel-speed limit, quadprog (D8), control law, fallback |
| `lmpc_v2_node.py` | The shared node (`mecanum_mpc_common.mpc_node`) with `LinearMPCv2` inside |
| `offline_sim.py` | The shared offline closed loop with `LinearMPCv2` (plan test 7) |
| `sign_test.py` | Plan test 8 |

Shared, in `mecanum_mpc_common`: robot parameters, dynamics and RK4 Jacobians (D2), reference with step-averaged u_r (D3), motor limits, wheel speeds, node, offline plant and the tools.

The settings follow the plan's decisions:
- **Weights:** the plan's starting values, Q = diag(40000, 40000, 3283, 2500, 2500, 100), R = 400·I₄ and P = 5·Q (D1). They have not been tuned.
- **Horizon:** T = 0.05 s, N = 10, a 0.5 s horizon (D4; N = 20 until 2026-10-21, when a sweep of N = 5, 10 and 20 showed the same tracking for both controllers at a third of the LMPC's compute time).
- **Model:** no friction model and no integral action (D5).
- **Constraints:** torque box, hard torque-rate limit (D6), soft wheel-speed limit (N5, replaces D7).
- **Reference heading:** not smoothed. Decided 2026-10-21.

## Results

See the plan documents. Offline, against the nominal model, every path tracks to under 0.03 mm RMS.
