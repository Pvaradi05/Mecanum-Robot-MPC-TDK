# mecanum_mpc_common

The shared base of the own mecanum robot's controllers: the linear MPC v2 (`mecanum_lmpc_v2`), the nonlinear MPC (`mecanum_nmpc`), and later the RL policy. Everything the controllers must agree on lives here once, so a difference in their results can only come from the controllers themselves (NMPC plan, decision N7).

| Module | Contents |
| --- | --- |
| `params.py` | Robot parameters from `robot.urdf`: m_s, J_C, J1, R, ρ, l and the wheel signs; m_e, J_e, κ, W, G, G⁺ |
| `dynamics.py` | Chaplygin model f(x, u) in the world-frame state [X, Y, ψ, Ẋ, Ẏ, ψ̇], RK4 step F and its chain-rule Jacobians (NumPy) |
| `wheel_dynamics.py` | The same model in the wheel-speed state z = [X, Y, ψ, ω₁, ω₂, ω₃], the state of both controllers since 2026-10-30; conversions from the world state and the encoders; the velocity weights on the wheel speeds. Copied unchanged from Kemenylegeny/Mecanum-tracking-tdk @ 411436a |
| `wheel_dynamics_casadi.py` | The wheel-speed-state model and RK4 step in CasADi symbols, for acados; tested against `wheel_dynamics.py` |
| `dare.py` | The DARE terminal weight of both controllers (the comparison repository's `solve_dare`, unchanged) |
| `weights.py` | The weights: Q_BO (the LTV-MPC's, the LMPC's default), Q_BRYSON (the NMPC's default for now), R = 400·I₄ |
| `reference.py` | Paths, cosine speed ramps, fixed or raw tangent heading, inverse dynamics, step-averaged reference torques, anchoring, feasibility check |
| `motor.py` | Wheel motor limits: DYNAMIXEL XM430-W210-T at 12.0 V |
| `wheel_speed.py` | Wheel speeds from the world-frame state, their Jacobian, and the slack weights of the LMPC's optional soft wheel-speed limit |
| `conventions.py` | Step status codes and controller display names |
| `mpc_node.py` | The ROS node every controller uses: measurement (converted to the wheel-speed state for a controller with `state == 'wheel'`), reference, timer, topics, CSV log; a controller subclasses it |
| `offline_sim.py` | Closed loop against the model (nominal or perturbed plant) for any controller |
| `evaluate.py`, `plot_runs.py`, `record_video.py` | Metrics, charts and overhead videos of runs |

```bash
ros2 run mecanum_mpc_common evaluate ~/ros2_ws/mpc_logs/<controller>/*.csv --skip 3
ros2 run mecanum_mpc_common plot_runs --controller nmpc <run_dir> ... --joined <out_stem>
ros2 run mecanum_mpc_common record_video --ros-args -p use_sim_time:=true -p controller:=nmpc \
    -p shape:=square -p heading:=tangent -p frames_dir:=/tmp/frames   # sim running, before the node
ros2 run mecanum_mpc_common record_video encode /tmp/frames <run.csv> square_tangent.mp4
```

The video tool spawns a visual-only line along the reference path and a static camera 20 m above it, looking straight down. Gazebo saves every frame at 30 fps in sim time. The camera renders only while its topic has a subscriber, so the recorder keeps a bridge subscription open. Rendering slows the sim to about 0.2× real time; the controllers run on sim time, so their results do not change.
