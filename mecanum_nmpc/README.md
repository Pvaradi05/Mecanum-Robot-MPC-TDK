# mecanum_nmpc

**Nonlinear MPC** for the own mecanum robot, after section 3 of Kühne, Gomes da Silva Jr. and Lages, *Mobile robot trajectory tracking using model predictive control* (SBAI/LARS 2005, eq. 7–10). It predicts with the exact Chaplygin dynamics of Zeidis & Zimmermann (ZAMM 2019), discretised with RK4, and solves the nonlinear problem with acados.

The design, the equations and the decisions N1–N7 are in the plan document "Chaplygin NMPC for the own mecanum robot: implementation plan". Since 2026-10-30, everything except the optimiser is that of the **latest LTV-MPC** of the RL comparison ([Kemenylegeny/Mecanum-tracking-tdk](https://github.com/Kemenylegeny/Mecanum-tracking-tdk) @ 411436a, synced into `mecanum_lmpc_v2`): model, states, inputs, cost, terminal weight, constraints, horizon and measurement. The weights are, for now, the exception (see the table). The NMPC's own parts are the solver (acados) and the problem it solves: the nonlinear model with states and torques as decision variables (multiple shooting), instead of the LTV-MPC's model linearised along the reference in a condensed QP. A difference between the two controllers' results therefore comes from the nonlinear prediction alone.

## The controller

| | |
| --- | --- |
| State, input | z = [X, Y, ψ, ω₁, ω₂, ω₃]: pose and the speeds of the front-left, front-right and rear-left wheels (ω₄ = ω₁ + ω₂ − ω₃); u = [M1, M2, M3, M4] wheel torques |
| Model | the Chaplygin model in these states (`mecanum_mpc_common/wheel_dynamics.py`), RK4 over T |
| Problem | Kühne's eq. 7–10: torques and states as decision variables (multiple shooting, N2), z₀ = measurement, z_j+1 = F(z_j, u_j) |
| Cost | the LTV-MPC's (N4): stage z̃ᵀQz̃ + ũᵀRũ, terminal z̃_NᵀPz̃_N |
| Weights | **for now Bryson's rule, not the LTV-MPC's** (decided 2026-10-30): q = (40000, 40000, 3283, 2500, 2500, 100) on the errors of (X, Y, ψ, Ẋ, Ẏ, ψ̇); the velocity weights act on the wheel speeds as J3ᵀ diag(q_v) J3. R = 400·I₄. The LTV-MPC's Q_BO = (10⁷, 10⁷, 10⁶, 17.59, 17.59, 15.22) diverges in Gazebo for both controllers, even without a start error; `-p Q:=[1e7,1e7,1e6,17.5888,17.5888,15.22]` selects it |
| Terminal weight | P = solution of the DARE at (A_N−1, B_N−1), the RK4 Jacobians at the reference's last step, warm-started with the previous step's P: the LTV-MPC's `terminal='dare'`, the same function (`mecanum_mpc_common/dare.py`) |
| Constraints | the torque box ±1.95 N·m only (N5): no torque-rate limit and no soft wheel-speed limit, so no slack variables |
| Solver | acados (N1): SQP, Gauss–Newton Hessian, HPIPM, merit-function line search, generated C code |
| Stopping | KKT residuals ≤ 10⁻⁶, capped at 10 iterations and 40 ms (N3); a capped step applies the last iterate and is logged "not converged" |
| Start | every step from the reference (N6), so the first SQP iteration solves exactly the LTV-MPC's QP |
| Horizon | T = 0.05 s, N = 10 (0.5 s), the same as the LTV-MPC |
| Measurement | the wheel speeds from the odometry's base velocity through the rolling constraints (the LTV-MPC's default, "kinematic"). Unlike the LMPC node, the NMPC node takes every odometry twist as it comes (`ODOM_GUARD` off) |

The model is the RK4 step written in CasADi (`mecanum_mpc_common/wheel_dynamics_casadi.py`, tested against the NumPy model), given to acados as a discrete model, so acados integrates exactly the LTV-MPC's F. acados's default scaling of stage costs by the time step is switched off, so the cost is the LTV-MPC's. The terminal weight changes every step and is set at run time (`cost_set(N, 'W', 2P)`). Nothing else acts on the torques: no filter, no feedforward compensation, only the final clip to the box, as in the LTV-MPC.

Before 2026-10-30 the NMPC used the world-frame velocity state, the Bryson weights with P = 5·Q, a hard torque-rate limit (as 4 extra states) and a soft wheel-speed limit: commit fe01f88.

## Install acados (once)

```bash
cd ~ && git clone https://github.com/acados/acados.git && cd acados
git checkout v0.6.0 && git submodule update --recursive --init
mkdir -p build && cd build && cmake -DACADOS_WITH_QPOASES=OFF -DACADOS_WITH_OPENMP=OFF .. && make install -j8
# Python interface + CasADi into the user site; pin the system NumPy/SciPy/Matplotlib so ROS keeps them
printf 'numpy==1.26.4\nscipy==1.11.4\nmatplotlib==3.6.3\n' > /tmp/pins.txt
pip install --user --break-system-packages -e ~/acados/interfaces/acados_template -c /tmp/pins.txt
```

No environment variables are needed. `nmpc.py` sets `ACADOS_SOURCE_DIR` to `~/acados` if it is unset, and loads acados's libraries itself, which is what `LD_LIBRARY_PATH` would do. On the first run, acados downloads its template renderer, then generates and compiles the solver into `~/ros2_ws/build/mecanum_nmpc/acados_generated/<settings hash>/`. Later runs with the same settings reuse it, and other settings get their own folder. acados v0.6.0 warns that CasADi 3.8.1 is newer than it was tested with; all tests pass with it.

## Run

```bash
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash

ros2 launch mecanum_nmpc nmpc_sim.launch.py shape:=square heading:=tangent
#   the same arguments as the LMPC launch: shape, heading, speed, loop, size, corner_radius, world, gui

ros2 run mecanum_nmpc offline_sim [--plant perturbed] [--start-offset X Y PSI]   # no ROS, no Gazebo
ros2 run mecanum_mpc_common evaluate ~/ros2_ws/mpc_logs/nmpc/*.csv --skip 3
python3 -m pytest src/mecanum_mpc_common/test src/mecanum_nmpc/test -q          # plan tests 1, 3-10
```

**Topics:** wheel torques as in the LMPC; `/nmpc/reference`, `/nmpc/predicted` and `/nmpc/status`.

**Parameters:** the common ones (`T`, `N`, `Q`, `R`, `u_max`, `omega_max`), plus `max_iter` (10), `time_budget` (0.04 s) and `kkt_tol` (10⁻⁶).

**Log:** `~/ros2_ws/mpc_logs/nmpc/<shape>_<heading>_<speed>_<time>.csv`. It has the LMPC's columns (`qp_ms` is the acados solve time), plus:

| column | meaning |
| --- | --- |
| `iterations` | SQP iterations of the step |
| `kkt` | largest KKT residual at exit |
| `acados_status` | 0 converged, 2 iteration cap, 3 minimum step, 7 time budget, other = failure |
| `alpha_first` | line-search step of the first iteration (1 = full step) |
| `dare_iters` | Riccati iterations of the step's terminal weight |
| `M1_first` … `M4_first` | torques after the first iteration: the LTV-MPC's torques when `alpha_first` = 1 |

## Tests

| file | plan tests |
| --- | --- |
| `mecanum_mpc_common/test/test_common.py` | 1: CasADi model = NumPy model (wheel-speed state), values to 10⁻¹², RK4 Jacobians to 10⁻⁹, at 1000 random points; the wheel-speed state model = `dynamics.py` in other coordinates |
| `mecanum_lmpc_v2/test/test_lmpc_v2.py` | 2: the LTV-MPC's tests, in the latest configuration |
| `test/test_nmpc.py` | against the latest LTV-MPC with the NMPC's weights: the same problem (Q, R, box, the identical DARE terminal weight step by step); 3: on the reference; 4: first iteration = LTV-MPC step to 10⁻⁶ N·m, offsets 1 mm to 30 cm; 5: KKT and model gaps at convergence; 6: the same solution from other starting points; 7: only the torque box limits the plan; 8: offline closed loop, nominal and perturbed; 9: 5/10/30 cm start offsets, both controllers; 10: timing |
