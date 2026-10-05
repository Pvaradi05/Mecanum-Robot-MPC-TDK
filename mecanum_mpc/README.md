# mecanum_mpc

Linear time-varying MPC (LTV-MPC) that makes the own mecanum robot (`tb3_sim_bringup`, see its README, "Own mecanum robot") follow **circle, ellipse, figure-8 and square** paths. The controller commands **wheel torques** and predicts with the **exact non-holonomic (Chaplygin) dynamics** of

> I. Zeidis, K. Zimmermann, *Dynamics of a four-wheeled mobile robot with Mecanum wheels*, ZAMM 99 (2019) e201900173, doi:10.1002/zamm.201900173

Written 2026-10-05. Background (LTV vs NMPC, why linearising is fine here) is in `tb3_sim_bringup/docs/mecanum_mpc_options.md`.

## Run

```bash
ros2 launch mecanum_mpc mpc_sim.launch.py shape:=figure8 heading:=tangent
#   shape:=circle|ellipse|figure8|square   heading:=fixed|tangent   speed:=0.2
#   loop:=false (default: drive `laps` laps, laps:=1, then hold the end pose)
#   loop:=true  (drive the path over and over until Ctrl-C, tracking report after every lap)
#   size:=0 (= default: circle r 0.75 m, ellipse 1.0 x 0.5 m semi-axes, figure-8 2 x 1 m, square 1.5 m)
#   linearize_about:=reference|prediction   world:=empty_world   gui:=true|false
ros2 run mecanum_mpc evaluate ~/ros2_ws/mpc_logs/*.csv --plot runs.png    # metrics + plots
ros2 run mecanum_mpc offline_sim --shape all --heading both               # no Gazebo, nominal model
```

The launch starts the sim with `drive:=effort` (torque-driven wheels, ground-truth `/odom`), waits 8 s, then starts `mpc_node`. The node anchors the path at the robot's current pose and drives it once (or `laps` times) with 2 s speed ramps, then holds the end pose. With `loop:=true` there is one ramp up, then the path repeats at constant speed until shutdown. With heading following the path, the heading keeps counting up by 2π per lap on the circle and square instead of jumping back. Example, circle with heading following the path: lap 1 (incl. start) 2.0 mm RMS, laps 2–3 1.7 mm RMS / 3.0 mm max. Every run writes `~/ros2_ws/mpc_logs/<shape>_<heading>_<speed>_<time>.csv`. RViz topics: `/mpc/reference`, `/mpc/predicted` (frame `odom`), `/mpc/tracking_error`.

**Path on the floor in Gazebo.** When the path is anchored, the node also spawns it into the world as a static, visual-only model `mpc_reference_path`: an orange ribbon 2 cm wide, with a green disc at the start (`gz_draw.py`). It has no collisions, so it doesn't affect the robot. A new run in the same sim replaces it. Turn it off with `-p draw_path:=false`; set `world_name` for worlds not called `default`. This works because in this sim `/odom` is the world frame, which was checked by spawning at (−2, −0.5).

Dependency outside rosdep: the OSQP Python bindings (`pip install --user --break-system-packages osqp`, 1.1.3 on this machine).

## How it works

| file | |
|---|---|
| `model.py` | Dynamics, Jacobians, RK4, ZOH discretisation. **Parameters are read from the URDF**: mass, yaw inertia, wheel+roller spin inertia, rolling radius (roller sphere envelope), wheel positions. |
| `references.py` | The 4 paths → timed reference `x_ref(t)`, plus feedforward wrench `w_ref(t)` from inverse dynamics. |
| `ltv_mpc.py` | The MPC: one sparse QP per step (OSQP). |
| `mpc_node.py` | ROS node: odometry in, wheel torques out, logging. |
| `evaluate.py`, `offline_sim.py` | Log analysis, closed loop against the Python model. |
| `test/` | Model = paper eq. (64), Jacobians = finite differences, offline tracking (`python3 -m pytest test/`). |

**Model** (equivalent to the paper's eq. 64, verified to 1e-8 in `test/`). State `x = [X, Y, ψ, u, v, r]` (world pose, body velocities), input `w = [F_ξ, F_η, τ]` (body wrench):

```
Ẋ = u cosψ − v sinψ     m_e u̇ = F_ξ + m_s v r
Ẏ = u sinψ + v cosψ     m_e v̇ = F_η − m_s u r          m_e = m_s + 4 J₁/R²
ψ̇ = r                   J_e ṙ = τ                       J_e = J_C + 4 J₁ L²/R²,  L = l_x + l_y
```

Wheel torques ↔ wrench: `w = Wᵀ M` with `R φ̇ᵢ = u + s_vᵢ v + s_rᵢ L r` (paper eq. 11). The 4 torques are the minimum-norm solution `M = pinv(Wᵀ) w`.

**LTV-MPC.** Every 50 ms, over N = 20 steps (1 s):
1. Linearise and discretise (ZOH) the dynamics along the reference: `δx_{k+1} = A_k δx_k + B_k δw_k + c_k`.
2. Solve one QP:
   - cost: `Σ |x−x_ref|²_Q + |w−w_ref|²_R + |Δw|²_Rd` plus a terminal cost (5 Q);
   - limits: |wheel torque| ≤ 0.6 N·m (the motor's continuous torque guess), |wheel speed| ≤ 30 rad/s;
   - the heading error is wrapped.
3. Apply `w_ref + δw₀`.

Defaults are `Q = diag(1000, 1000, 250, 5, 5, 1)`, `R = diag(0.01, 0.01, 1)`, `Rd = diag(0.1, 0.1, 10)`. `linearize_about:=prediction` linearises around the previous solution instead, which is one SQP real-time iteration of an NMPC.

**References.** Paths start at the robot, heading along its x axis. Constant speed with cosine ramps. `heading:=fixed` keeps ψ constant: the robot never rotates, so the bilinear (Coriolis) terms are 0 along the reference. `heading:=tangent` follows the path direction, smoothed with σ = 0.25 s so the square's corners don't demand infinite angular acceleration. The square has rounded corners (radius 15 % of the side).

The node warns if a reference needs more than 30 rad/s wheel speed. With fixed heading the worst case is diagonal travel (|u|+|v| = v√2), which limits the path speed to 30·R/√2 ≈ **0.64 m/s** for every shape. With heading following the path, the turning adds wheel speed: limits are ≈ 0.70 m/s for the circle and ≈ 0.50 m/s for the ellipse, figure-8 and square.

**Friction feedforward.** The sim's rollers have rolling resistance that the paper's model doesn't have (see "Sim vs model"). The node therefore adds `friction_torque · tanh(φ̇_ref / 0.5 rad/s)` per wheel (default 0.003 N·m).

Parameters: `ros2 param list /mecanum_mpc` (Q, R, Rd, horizon, dt, torque_max, wheel_speed_max, friction_torque, extra_wheel_inertia, …).

## Sim vs model (measured 2026-10-05, `drive:=effort`)

- **Breakaway:** at rest, wheels need **0.01–0.02 N·m each** before anything moves.
- **Rolling resistance** while moving: ≈ 0.003 N·m per wheel (≈ 0.35 N total). Likely cause: the 6-sphere roller approximation has a ~0.07 mm ripple in its rolling radius, and the robot sits in a dip at standstill.
- **Effective mass** from torque steps: ≈ 3.9 kg against the model's m_e = 3.17 kg. Probably roller spin inertia, which the paper's infinitesimal-roller model ignores.
- **Odometry glitch:** gz `OdometryPublisher` emits one yaw-rate sample of about ±600 rad/s each time the yaw passes ±180° (an angle wrap in its differentiation; seen once per lap in loop mode). The node rejects twists the wheels can't produce.
- **Odometry:** in effort mode, `/odom` is ground truth from Gazebo's `OdometryPublisher`, i.e. perfect state estimation.
- **Hardware limits:** gearmotor inertia (`robot_config.json` guess: 9e-4 kg·m² reflected) is **not** in the URDF/sim. Set `extra_wheel_inertia` when moving to hardware.
- **CoM offset:** the URDF's CoM is 13.9 mm ahead of the wheel centre; the model assumes 0.

## Results

See the table in `tb3_sim_bringup/docs/mecanum_mpc_options.md` ("LTV-MPC implemented").
