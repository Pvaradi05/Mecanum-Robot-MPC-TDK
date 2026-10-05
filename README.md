# Mecanum robot simulation and MPC path tracking (ROS 2 Jazzy, Gazebo Sim)

Simulation of a four-wheeled mecanum robot in Gazebo (gz sim), and a model predictive controller that makes it follow circle, ellipse, figure-8 and square paths. The controller predicts with the exact non-holonomic (Chaplygin) dynamics of Zeidis & Zimmermann, *Dynamics of a four-wheeled mobile robot with Mecanum wheels*, ZAMM 2019, [doi:10.1002/zamm.201900173](https://doi.org/10.1002/zamm.201900173).

| package | contents |
|---|---|
| [`tb3_sim_bringup`](tb3_sim_bringup/README.md) | Gazebo launch files and robot models: the own mecanum robot (physically modelled rollers, from [Mecanum-TDK](https://github.com/Kemenylegeny/Mecanum-TDK)), a mecanum TurtleBot3 Waffle Pi variant, keyboard teleop, and the MPC research notes in [`docs/`](tb3_sim_bringup/docs/mecanum_mpc_options.md) |
| [`mecanum_mpc`](mecanum_mpc/README.md) | Linear time-varying MPC with wheel-torque inputs: dynamics model, reference paths, ROS node, evaluation tools, tests |

## Quick start

This repository is the `src/` folder of a colcon workspace.

```bash
# prerequisites: ROS 2 Jazzy + Gazebo Harmonic
sudo apt install ros-jazzy-turtlebot3-simulations ros-jazzy-xacro
pip install --user --break-system-packages osqp      # QP solver, not available via rosdep

mkdir -p ~/ros2_ws && cd ~/ros2_ws
git clone <this repo> src
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

# robot follows a square, facing along the path, lap after lap until Ctrl-C
ros2 launch mecanum_mpc mpc_sim.launch.py shape:=square heading:=tangent loop:=true

# drive the robot yourself (velocity mode)
ros2 launch tb3_sim_bringup own_mecanum_sim.launch.py
ros2 run tb3_sim_bringup tb3_mecanum_teleop_keys
```

Results so far (Gazebo, 0.2 m/s): 1.7–3.6 mm RMS tracking error on all four paths. See the [`mecanum_mpc` README](mecanum_mpc/README.md) and the [MPC notes](tb3_sim_bringup/docs/mecanum_mpc_options.md).
