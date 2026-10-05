# tb3_sim_bringup

Launch entrypoint for a TurtleBot3 Waffle Pi Gazebo (gz sim) simulation on ROS2 Jazzy, built to the point where the robot can be driven via `/cmd_vel`.

## What's here

- `launch/tb3_waffle_pi_sim.launch.py` — wraps the upstream `turtlebot3_gazebo` launch files rather than duplicating them. Args: `turtlebot3_model` (default `waffle_pi`), `world` (default `turtlebot3_world`; other valid values — `empty_world`, `turtlebot3_house`, etc. — are listed under `$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/launch/`).
- `launch/own_mecanum_sim.launch.py` + `models/own_mecanum/` + `urdf/own_mecanum.urdf.xacro` — **your own mecanum robot** with physically modelled rollers. See [Own mecanum robot](#own-mecanum-robot).
- `launch/tb3_waffle_pi_mecanum_sim.launch.py` + `models/turtlebot3_waffle_pi_mecanum/` + `urdf/turtlebot3_waffle_pi_mecanum.urdf` + `scripts/tb3_mecanum_teleop_keys.py` — a **mecanum-wheel** Waffle Pi variant that can strafe. See [Mecanum variant](#mecanum-variant) below.

## Prerequisites (already done on this machine, 2026-07-20)

TurtleBot3 has ready-made Jazzy binary packages — no source build needed. They're already ported to the new Gazebo Sim (classic Gazebo isn't packaged for Jazzy/Noble at all):

```bash
sudo apt install -y ros-dev-tools ros-jazzy-turtlebot3-simulations ros-jazzy-turtlebot3-teleop
sudo rosdep init   # first-time only, already done
rosdep update
```

`TURTLEBOT3_MODEL=waffle_pi` and `source ~/ros2_ws/install/setup.bash` are both appended to `~/.bashrc`.

Deliberately **not** installed: the full `ros-jazzy-turtlebot3` meta-package (Nav2/Cartographer/real-hardware bringup), `turtlebot3-description`, `turtlebot3-msgs`, `turtlebot3-bringup` — none are dependencies of `turtlebot3-gazebo` or this sim path. Add them if/when Nav2, SLAM, or real-hardware work is needed.

## Build

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

## Run

```bash
ros2 launch tb3_sim_bringup tb3_waffle_pi_sim.launch.py
# or override the world: ... world:=empty_world
```

Use a **fresh terminal** so it picks up `~/.bashrc`. Note: non-interactive/scripted shells (e.g. CI, tool calls) don't source `.bashrc` at all — its own first lines return immediately unless the shell is interactive — so in that context source manually first:
```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
```

## Driving it

**`/cmd_vel` is `geometry_msgs/msg/TwistStamped`, not plain `Twist`** — confirmed from the `ros_gz_bridge` log at launch time (this is the REP 2005 Twist→TwistStamped migration). Verified working 2026-07-20: a `linear.x=0.2` command moved the robot from x≈0 to x≈1.11m over 3s; an `angular.z=0.5` command rotated it in place (position stayed ~0, orientation quaternion changed).

### Recommended: `tb3_teleop_keys` (fixes the stock script's zero-drift issue)

```bash
ros2 run tb3_sim_bringup tb3_teleop_keys
```

Adapted from `ros-jazzy-turtlebot3-teleop`'s `teleop_keyboard.py` (source at
`/opt/ros/jazzy/lib/python3.12/site-packages/turtlebot3_teleop/script/teleop_keyboard.py`),
which only supports zeroing linear+angular together (`s`/space) — there's no way to reset just
one axis, so during free-form driving it's easy to end up with a small residual velocity on one
axis that's hard to notice/count back to exactly zero. This version adds two independent hotkeys:

```
w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
q   : zero LINEAR velocity only (keeps angular as-is)
e   : zero ANGULAR velocity only (keeps linear as-is)
space key, s : force stop (zero both)
CTRL-C to quit
```

It also fixes a real bug found in the upstream script while adapting it: on Jazzy, the upstream
script's Ctrl-C handler republishes the *last active* velocity instead of zero, so quitting the
script can leave the robot still moving. This version always publishes `(0.0, 0.0)` on exit —
verified end-to-end (not just internally) by holding a nonzero velocity, sending Ctrl-C, and
confirming the actual next message on `/cmd_vel` was zero.

Source: `scripts/tb3_teleop_keys.py`. Max velocities and step sizes match the original
(`TURTLEBOT3_MODEL`-aware: 0.22 m/s / 2.84 rad/s for burger, 0.26 m/s / 1.82 rad/s for
waffle/waffle_pi; steps of 0.01 m/s and 0.1 rad/s).

### Alternatives

```bash
ros2 run turtlebot3_teleop teleop_keyboard        # stock TB3 script — only combined zero (s/space), and the Ctrl-C bug above
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p frame_id:=base_link   # generic fallback
```

One-shot test:
```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: ''}, twist: {linear: {x: 0.2}, angular: {z: 0.0}}}"
```

The diff-drive plugin latches the last command indefinitely (no built-in timeout) — zero it explicitly to stop:
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: ''}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}"
```

## Mecanum variant

A Waffle Pi with the two diff-drive wheels and two casters replaced by four mecanum wheels, so it can strafe and move diagonally. Sensors (lidar, IMU, Pi camera), topics, and bridge config are identical to the stock robot; `/cmd_vel` simply also honours `linear.y` now.

```bash
ros2 launch tb3_sim_bringup tb3_waffle_pi_mecanum_sim.launch.py
#   args: world:=turtlebot3_world|empty_world|turtlebot3_house  (a file in turtlebot3_gazebo/worlds/)
#         x_pose:=-2.0 y_pose:=-0.5   gui:=false (headless)
ros2 run tb3_sim_bringup tb3_mecanum_teleop_keys      # in a second terminal
```

```
w/x : increase/decrease forward velocity  (linear.x,  max 0.26 m/s)
a/d : increase/decrease strafe velocity   (linear.y,  a = left, max 0.26 m/s)
q/e : increase/decrease rotation velocity (angular.z, q = counter-clockwise, max 1.82 rad/s)
1 / 2 / 3 : zero forward / strafe / rotation only
space key, s : force stop (zero all)
CTRL-C to quit (always publishes zero)
```

Axes combine — e.g. `w` + `a` drives diagonally, `a` + `q` strafes while turning. Note `q`/`e` mean *rotate* here, not "zero one axis" as in `tb3_teleop_keys`; the per-axis zeros moved to `1`/`2`/`3`.

### How it works

- **Controller:** gz-sim's built-in `gz::sim::systems::MecanumDrive` system (in the model SDF). It does the inverse kinematics `cmd_vel (vx, vy, wz)` → four wheel velocities, and publishes `/odom` + `odom→base_footprint` TF on the same topics/frames as the stock DiffDrive.
- **Physics of the rollers:** not modelled as individual rollers. Each wheel's collision is a sphere with **anisotropic friction** — `mu=1` along a 45° direction (`fdir1`) and `mu2=0` across it — which is what a mecanum wheel's free rollers effectively give you. `fdir1` uses `gz:expressed_in="base_link"` so the direction stays fixed to the chassis instead of spinning with the wheel. This is the technique from gz-sim's own `worlds/mecanum_drive.sdf` example. It needs the DART physics engine (gz-sim's default, used by all turtlebot3 worlds).
- **Geometry:** wheels at x = +0.016 / −0.144, y = ±0.144 in `base_link` (symmetric about the chassis centre / centre of mass at x = −0.064), radius 0.033 → `wheelbase 0.160`, `wheel_separation 0.288`. Joints: `wheel_{front,rear}_{left,right}_joint`.
- **`base_footprint` moved** to under the chassis centre (`base_joint` is now `0.064 0 0.010`, was `0 0 0.010`). A mecanum base rotates about the centre of its four wheels; leaving `base_footprint` at the old diff-drive axle would make odometry wrong whenever it turns (that point would orbit a 6.4 cm circle while odom claims it's stationary).
- **Two robot descriptions, kept in sync by hand** — same split as upstream TurtleBot3: Gazebo spawns `models/turtlebot3_waffle_pi_mecanum/model.sdf`, while `robot_state_publisher` gets `urdf/turtlebot3_waffle_pi_mecanum.urdf` for TF. The SDF is the one that matters for physics (the `fdir1 … expressed_in` friction setup has no plain-URDF equivalent).

### Verified (2026-10-01, `empty_world`, headless)

Ground-truth body velocity from Gazebo's `/world/default/pose/info`, measured against sim time at steady state:

| `cmd_vel` (vx, vy, wz) | measured vx | measured vy | measured wz |
|---|---|---|---|
| 0.2, 0, 0 | +0.200 | 0.000 | 0.000 |
| 0, +0.2, 0 (strafe left) | 0.000 | +0.200 | 0.000 |
| 0, −0.2, 0 (strafe right) | 0.000 | −0.200 | 0.000 |
| 0, +0.26, 0 (max strafe) | 0.000 | +0.260 | 0.000 |
| 0.15, 0.15, 0 (diagonal) | +0.150 | +0.150 | 0.000 |
| 0, 0, +0.5 | — | — | +0.500 |
| 0, 0, −1.0 | — | — | −1.000 |

Also checked: `/odom` reports `linear.y` while strafing; TF `base_footprint→base_link` = (0.064, 0, 0.010); `/joint_states` lists all four wheels; `/scan`, `/imu`, `/camera/image_raw` publish; teleop keys produce the expected `/cmd_vel` on every axis and Ctrl-C mid-motion publishes zero.

On this WSL2 machine the sim intermittently runs below real time (`/stats` real_time_factor dips as low as ~0.06; over a full test, ~1.55 s sim per 3 s wall), so robots move less far per wall-clock second than the commanded speed suggests. Velocities in sim time are exact.

## Own mecanum robot

Your own robot from [Kemenylegeny/Mecanum-TDK `assets/robots/own_mecanum`](https://github.com/Kemenylegeny/Mecanum-TDK/tree/main/assets/robots/own_mecanum), with **physically modelled rollers**. Added 2026-10-05; the Waffle Pi variants above are unchanged.

```bash
ros2 launch tb3_sim_bringup own_mecanum_sim.launch.py   # same args: world:=… x_pose:= y_pose:= gui:=false
ros2 run tb3_sim_bringup tb3_mecanum_teleop_keys       # same teleop, same /cmd_vel
```

Same topics as the Waffle Pi (`/cmd_vel`, `/odom`, `/tf`, `/scan`, `/imu`, `/joint_states`), so the upstream bridge config, teleop and later Nav2/MPC work unchanged. **No camera** — none on the model yet (`/camera/camera_info` exists but stays silent).

### Files

- `models/own_mecanum/robot.urdf` + `meshes/` — copied **unchanged** from the repo (only these are needed; `stl/` and the JSONs are generator inputs/analysis, their values are already baked into the URDF). To update the robot, copy a newer `robot.urdf` + `meshes/` over them.
- `urdf/own_mecanum.urdf.xacro` — includes `robot.urdf` and adds everything simulation-specific: `base_footprint`, lidar (`base_scan`) and IMU (`imu_link`) frames + sensors, roller friction (μ = 0.8, middle of the 0.5–1.1 rubber range in `robot_config.json`), `MecanumDrive` and `JointStatePublisher` (all 40 joints).
- `launch/own_mecanum_sim.launch.py` — expands the xacro, rewrites the URDF's relative `meshes/…` paths to `package://tb3_sim_bringup/models/own_mecanum/meshes/…`, and spawns from `robot_description`. **One description for both Gazebo and `robot_state_publisher`** (no hand-synced SDF copy, unlike the Waffle Pi variant).

### How it differs from the Waffle Pi mecanum

- **Rollers:** each wheel = hub (no collision) + 9 free-spinning roller links on revolute joints, each roller's collision = **6 spheres** along its axis (radii 5.04/6.10/6.64/6.64/6.10/5.04 mm) approximating the barrel shape; 216 contact spheres total. Rollers use plain isotropic friction — the mecanum effect comes from the rollers actually spinning, not from the `fdir1` trick. Roller pattern is the same "X" as the Waffle Pi variant.
- **Geometry:** wheels at x = ±0.064, y = ±0.1526, z = −0.0415 in `base_link`; effective rolling radius 0.03028 (envelope of the sphere chains) → `wheelbase 0.128`, `wheel_separation 0.3052`, `wheel_radius 0.03028`. `base_footprint` is 0.0718 below `base_link`, under the wheel centre. Mass ≈ 3 kg (chassis 2.6 kg guessed in `robot_config.json`). Chassis–ground clearance is only ~5 mm.
- **Lidar:** the scanner is part of `chassis_visual.stl` (round head ~62 mm across on top, RPLIDAR-A1-like base). Found by analysing the mesh: head centred at x = 0.0015, y = 0, z = 0.048–0.0665 in `base_link` → scan plane `base_scan` at (0.0015, 0, 0.057), i.e. 0.129 m above ground. Same settings as the TB3 LDS (360 samples, 0.12–3.5 m, 10 Hz). The 0.12 m min range keeps rays off the robot's own housing.
- **IMU:** placeholder at the `base_link` origin until the real mount is known.

### Verified (2026-10-05, headless)

Ground truth from `/world/default/pose/info`, sim time, `empty_world`:

| `cmd_vel` (vx, vy, wz) | GT vx | GT vy | GT wz | `/odom` |
|---|---|---|---|---|
| 0.2, 0, 0 | +0.200 | 0.000 | 0.000 | exact |
| 0, +0.2, 0 | −0.001 | +0.196 | −0.001 | exact |
| 0, −0.2, 0 | −0.001 | −0.195 | +0.001 | exact |
| 0, +0.26, 0 | −0.001 | +0.253 | +0.005 | exact |
| 0.15, 0.15, 0 | +0.148 | +0.143 | +0.008 | exact |
| 0, 0, +0.5 | — | — | +0.497 | exact |
| 0, 0, −1.0 | — | — | −0.994 | exact |

Unlike the idealised Waffle Pi variant, the roller model **slips a little** (strafe ~2–3 % short, slight yaw drift on diagonals) while wheel odometry still reports the commanded values — a real odometry-vs-truth gap, as on hardware. Also checked: rests level on the rollers (`base_footprint` z ≈ −0.1 mm), `/joint_states` has 40 joints, TF includes every roller, `/imu` reads 9.81 m/s² in `imu_link`, `/scan` is all-inf in `empty_world` (no self-hits) and sees walls in `turtlebot3_world` (325/360 finite, min 0.53 m).

### Cost of the roller model

Pure physics, no sensors, real-time cap off, 10 s sim at 1 ms steps (DART is single-threaded): **~8.7 s CPU for the own robot vs ~2.1 s for the Waffle Pi sphere model (~4×)** → ~0.87 ms per step, so physics alone only just keeps up with real time. Full launch headless: ~2 cores for `gz sim`, real-time factor 0.92–0.98 in `empty_world`, **0.70–0.75 in `turtlebot3_world`**. (The Waffle Pi variant uses *more* total CPU, ~2.7–3 cores, because it also renders a camera.) The 233k-triangle `chassis_visual.stl` matters only with the GUI/RViz; physics uses the 656-triangle collision mesh.

## Notes

- `~/ros2_ws/src/my_robot_description` is a **separate, unrelated** older experiment (a custom "box_bot" URDF + `voice_controller*.py` scripts on `/model/box_bot/cmd_vel`) — untouched by this work.
- `~/ros2_ws` is not a git repo, so this file (rather than commit history) is the record of how/why this was set up.
- MPC control for the mecanum variant: researched, not implemented — options, trade-offs and an evaluation plan are in [docs/mecanum_mpc_options.md](docs/mecanum_mpc_options.md).
- Natural next steps, not yet done: SLAM/mapping (`turtlebot3_cartographer`), autonomous navigation (`turtlebot3_navigation2` — for the mecanum variant, Nav2's controller needs holonomic settings, e.g. MPPI with `motion_model: Omni`), RViz visualization, a custom world.
