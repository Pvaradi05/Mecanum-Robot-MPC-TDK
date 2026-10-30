"""Weights of the controllers, on the errors of [X, Y, psi, X_dot, Y_dot, psi_dot] (with the
wheel-speed state the velocity weights act through the rolling constraints, J3^T diag(q_v) J3,
wheel_dynamics.velocity_weight) and on each wheel torque.

    Q_BO, R_BO   the LTV-MPC's since 2026-10-30 (Kemenylegeny/Mecanum-tracking-tdk @ 411436a,
                 scripts/rsl_rl/mpc_controller.py Q_BO): the best of its closed-loop Bayesian
                 optimisation in Isaac Lab (q_pos, q_psi, q_v, q_omega; r = 400 fixed). The LMPC's
                 default. In Gazebo both controllers diverge with it (2026-10-30, square path, even
                 without a start error: a growing step-to-step oscillation)
    Q_BRYSON     Bryson's rule, the plan's starting values (D1) and the LTV-MPC's until 2026-10-29,
                 with r = 400 (R_BO). The NMPC's default since 2026-10-30, "for now" (decided
                 2026-10-30): both controllers track in Gazebo with it, ~1.0 mm RMS on the square
"""
Q_BO = (1e7, 1e7, 1e6, 17.5888, 17.5888, 15.22)
R_BO = (400.0, 400.0, 400.0, 400.0)
Q_BRYSON = (40000.0, 40000.0, 3283.0, 2500.0, 2500.0, 100.0)
