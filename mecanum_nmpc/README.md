# mecanum_nmpc

**Nonlinear MPC** for the own mecanum robot, after section 3 of Kühne, Gomes da Silva Jr. and Lages, *Mobile robot trajectory tracking using model predictive control* (SBAI/LARS 2005, eq. 7–10). It predicts with the exact Chaplygin dynamics of Zeidis & Zimmermann (ZAMM 2019), discretised with RK4, and solves the nonlinear problem with acados.

The design, the equations and the decisions N1–N7 are in the plan document "Chaplygin NMPC for the own mecanum robot: implementation plan". Everything except the optimiser is shared with the linear MPC v2 (`mecanum_lmpc_v2`) through `mecanum_mpc_common`: model, reference, motor limits, wheel-speed limit, cost, horizon, ROS node, logs and tools. A difference between the two controllers' results therefore comes from the nonlinear prediction alone.

## The controller

| | |
| --- | --- |
| Problem | Kühne's eq. 7–10: torques and states as decision variables (multiple shooting, N2), x₀ = measurement, x_j+1 = F(x_j, u_j) |
| Cost | identical to the LMPC (N4): Q = diag(40000, 40000, 3283, 2500, 2500, 100), R = 400·I₄, P = 5·Q |
| Constraints | torque box ±1.95 N·m, hard torque-rate limit, soft wheel-speed limit 8.06 rad/s with slack weights 10⁴/10⁴ (N5), the same as the LMPC's but exact |
| Solver | acados (N1): SQP, Gauss–Newton Hessian, HPIPM, merit-function line search, generated C code |
| Stopping | KKT residuals ≤ 10⁻⁶, capped at 10 iterations and 40 ms (N3); a capped step applies the last iterate and is logged "not converged" |
| Start | every step from the reference (N6), so the first SQP iteration solves exactly the LMPC's QP |
| Horizon | T = 0.05 s, N = 10 (0.5 s), the same as the LMPC (D4; 20 until 2026-10-21) |

acados bounds each stage, not differences between stages. The torque-rate limit therefore uses 4 extra states, the previous torques: z_j = [x_j; u_j−1]. The model is the RK4 step written in CasADi (`mecanum_mpc_common/dynamics_casadi.py`), given to acados as a discrete model, so acados integrates exactly the LMPC's F. acados's default scaling of stage costs by the time step is switched off, so the cost is the LMPC's.

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

**Parameters:** the LMPC's, plus `max_iter` (10), `time_budget` (0.04 s) and `kkt_tol` (10⁻⁶).

**Log:** `~/ros2_ws/mpc_logs/nmpc/<shape>_<heading>_<speed>_<time>.csv`. It has the LMPC's columns (`qp_ms` is the acados solve time), plus:

| column | meaning |
| --- | --- |
| `iterations` | SQP iterations of the step |
| `kkt` | largest KKT residual at exit |
| `slack_max` | largest wheel-speed slack of the step (rad/s) |
| `acados_status` | 0 converged, 2 iteration cap, 3 minimum step, 7 time budget, other = failure |
| `alpha_first` | line-search step of the first iteration (1 = full step) |
| `M1_first` … `M4_first` | torques after the first iteration: the LMPC's torques when `alpha_first` = 1 |

## Tests

| file | plan tests |
| --- | --- |
| `mecanum_mpc_common/test/test_common.py` | 1: CasADi model = NumPy model, values to 10⁻¹², RK4 Jacobians to 10⁻⁹, at 1000 random points |
| `mecanum_lmpc_v2/test/test_lmpc_v2.py` | 2: the LMPC's 21 tests, plus the bit-identity check of the refactor (done once, 1,600 inputs) |
| `test/test_nmpc.py` | 3: on the reference; 4: first iteration = LMPC step to 10⁻⁶ N·m, offsets 1 mm to 30 cm; 5: KKT and model gaps at convergence; 6: the same solution from other starting points; 7: wheel-speed limit in both controllers; 8: offline closed loop, nominal and perturbed; 9: 5/10/30 cm start offsets, both controllers; 10: timing |
