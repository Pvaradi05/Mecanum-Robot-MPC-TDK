# Mecanum robot simulation and MPC path tracking (ROS 2 Jazzy, Gazebo Sim)

Simulation of a four-wheeled mecanum robot in Gazebo (gz sim), and model predictive control that makes it follow circle, ellipse, figure-8 and square paths.

**Status (2026-10-21):** `main` has two controllers for the research comparison of a linear MPC, a nonlinear MPC and (later) an RL policy. Both follow Kühne, Gomes da Silva Jr. and Lages, *Mobile robot trajectory tracking using model predictive control* (SBAI/LARS 2005). Both predict with the exact non-holonomic (Chaplygin) dynamics of Zeidis & Zimmermann, *Dynamics of a four-wheeled mobile robot with Mecanum wheels*, ZAMM 2019, [doi:10.1002/zamm.201900173](https://doi.org/10.1002/zamm.201900173). Their weights are the untuned starting values. The first controller (**v1**, a different LTV-MPC) is kept on the [`v1` branch](https://github.com/Pvaradi05/Mecanum-Robot-MPC-TDK/tree/v1).

## Repository layout

Each package has one role. The two controllers share everything except their optimiser through one common package, so a difference in their results can only come from the controller itself.

| role | package | what it builds |
|---|---|---|
| **Shared** by every controller | [`mecanum_mpc_common`](mecanum_mpc_common/README.md) | robot parameters from the URDF; Chaplygin dynamics and RK4 (NumPy and CasADi); reference paths and reference torques; motor limits (DYNAMIXEL XM430-W210-T); soft wheel-speed limit; the ROS node every controller runs in; offline simulation; evaluation, chart and video tools |
| **Linear MPC** | [`mecanum_lmpc_v2`](mecanum_lmpc_v2/README.md) | `lmpc.py`: linear time-varying MPC (Kühne eq. 15–20), condensed QP solved with quadprog; its node, launch file, offline simulation, sign test and tests |
| **Nonlinear MPC** | [`mecanum_nmpc`](mecanum_nmpc/README.md) | `nmpc.py`: nonlinear MPC (Kühne eq. 7–10), multiple shooting, solved by acados (SQP, Gauss–Newton, HPIPM); its node, launch file, offline simulation and tests |
| Simulation | [`tb3_sim_bringup`](tb3_sim_bringup/README.md) | Gazebo launch files and robot models: the own mecanum robot (physically modelled rollers, from [Mecanum-TDK](https://github.com/Kemenylegeny/Mecanum-TDK), weighed masses), a mecanum TurtleBot3 Waffle Pi variant, keyboard teleop, and the MPC research notes and v1 results in [`docs/`](tb3_sim_bringup/docs/mecanum_mpc_options.md) |

Dependencies: `mecanum_lmpc_v2` → `mecanum_mpc_common` ← `mecanum_nmpc`, and all three use `tb3_sim_bringup` for the robot model and the simulation. The two controllers do not depend on each other. Only the NMPC's tests use the LMPC, to check that the NMPC's first iteration equals the LMPC step.

Shared settings: T = 0.05 s, N = 10, the same cost and the same constraints (torque box, torque-rate limit, soft wheel-speed limit), and a default path speed of 0.1 m/s.

## Quick start

This repository is the `src/` folder of a colcon workspace.

```bash
# prerequisites: ROS 2 Jazzy + Gazebo Harmonic
sudo apt install ros-jazzy-turtlebot3-simulations ros-jazzy-xacro

mkdir -p ~/ros2_ws && cd ~/ros2_ws
git clone <this repo> src
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

# drive the robot yourself (velocity mode)
ros2 launch tb3_sim_bringup own_mecanum_sim.launch.py
ros2 run tb3_sim_bringup tb3_mecanum_teleop_keys

# torque-driven wheels + ground-truth /odom, for a torque-input controller
ros2 launch tb3_sim_bringup own_mecanum_sim.launch.py world:=empty_world drive:=effort

# the controllers (sim included): linear MPC, nonlinear MPC
ros2 launch mecanum_lmpc_v2 lmpc_v2_sim.launch.py shape:=square heading:=tangent
ros2 launch mecanum_nmpc nmpc_sim.launch.py shape:=square heading:=tangent
```

Solvers, from pip: `quadprog` for the linear MPC. For the nonlinear MPC, acados is built from source with its Python interface (and CasADi); the steps are in the [`mecanum_nmpc` README](mecanum_nmpc/README.md).

To run the v1 controller: `git checkout v1`, build, then `ros2 launch mecanum_mpc mpc_sim.launch.py shape:=square heading:=tangent loop:=true` (it needs `pip install --user --break-system-packages osqp`).

## v1 results

All at 0.2 m/s in Gazebo, measured with the earlier 3.0 kg mass estimate:
- **1.7–3.6 mm RMS tracking error** on all four paths. This is the distance to where the reference is at that moment.
- **About 1 mm RMS distance from the path itself.** The rest is timing: with fixed heading, the robot lags about 3.6 mm behind the reference while it drives sideways.
- **Sharp-cornered squares** work when the feedforward is averaged over each control step.
- **With the weighed masses (2.134 kg):** a first check gives 2.5 mm RMS, and the friction feedforward needs retuning.

Details: [MPC notes](tb3_sim_bringup/docs/mecanum_mpc_options.md), and the [`mecanum_mpc` README on `v1`](https://github.com/Pvaradi05/Mecanum-Robot-MPC-TDK/blob/v1/mecanum_mpc/README.md).
