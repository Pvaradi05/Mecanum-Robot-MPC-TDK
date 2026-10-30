from mecanum_mpc_common.weights import Q_BO, R_BO

LATEST = dict(Q=Q_BO, R=R_BO, terminal='dare', S=None, Q_path=None, wheel_speed_limit=False,
              state='wheel')
"""The latest LTV-MPC (Kemenylegeny/Mecanum-tracking-tdk @ 411436a, the defaults of
scripts/rsl_rl/mpc_controller.py): Q_BO with r = 400, DARE terminal weight, wheel-speed state
z = [X, Y, psi, w1, w2, w3], and only the torque box (no torque-change penalty S, no path-frame
weight, no soft wheel-speed limit; lmpc.py has no torque-rate limit). LinearMPCv2(p, T, N, **LATEST)
is the node's, the offline simulation's and the tests' controller, and the NMPC's reference."""
