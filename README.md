# Mecanum robot simulation and MPC path tracking (ROS 2 Jazzy, Gazebo Sim)

Simulation of a four-wheeled mecanum robot in Gazebo (gz sim), and model predictive control that makes it follow circle, ellipse, figure-8 and square paths.

**Status (2026-10-16):** a new MPC controller, using a different method, is being written from scratch on `main`. The first controller (**v1**) is kept on the [`v1` branch](https://github.com/Pvaradi05/Mecanum-Robot-MPC-TDK/tree/v1). It is a linear time-varying MPC with wheel-torque inputs, predicting with the exact non-holonomic (Chaplygin) dynamics of Zeidis & Zimmermann, *Dynamics of a four-wheeled mobile robot with Mecanum wheels*, ZAMM 2019, [doi:10.1002/zamm.201900173](https://doi.org/10.1002/zamm.201900173).

| package | contents |
|---|---|
| [`tb3_sim_bringup`](tb3_sim_bringup/README.md) | Gazebo launch files and robot models: the own mecanum robot (physically modelled rollers, from [Mecanum-TDK](https://github.com/Kemenylegeny/Mecanum-TDK), weighed masses), a mecanum TurtleBot3 Waffle Pi variant, keyboard teleop, and the MPC research notes and v1 results in [`docs/`](tb3_sim_bringup/docs/mecanum_mpc_options.md) |

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
```

To run the v1 controller: `git checkout v1`, build, then `ros2 launch mecanum_mpc mpc_sim.launch.py shape:=square heading:=tangent loop:=true` (it needs `pip install --user --break-system-packages osqp`).

## v1 results

All at 0.2 m/s in Gazebo, measured with the earlier 3.0 kg mass estimate:
- **1.7–3.6 mm RMS tracking error** on all four paths. This is the distance to where the reference is at that moment.
- **About 1 mm RMS distance from the path itself.** The rest is timing: with fixed heading, the robot lags about 3.6 mm behind the reference while it drives sideways.
- **Sharp-cornered squares** work when the feedforward is averaged over each control step.
- **With the weighed masses (2.134 kg):** a first check gives 2.5 mm RMS, and the friction feedforward needs retuning.

Details: [MPC notes](tb3_sim_bringup/docs/mecanum_mpc_options.md), and the [`mecanum_mpc` README on `v1`](https://github.com/Pvaradi05/Mecanum-Robot-MPC-TDK/blob/v1/mecanum_mpc/README.md).
