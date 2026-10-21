"""Wheel speeds as a function of the state, for the soft wheel-speed limit (NMPC plan, N5).

    omega = W [cos psi X_dot + sin psi Y_dot;  -sin psi X_dot + cos psi Y_dot;  psi_dot]
    |omega_i| <= omega_max + s_i,  s_i >= 0,  cost  SLACK_LIN s_i + SLACK_QUAD s_i^2

Both controllers use the same limit and the same penalty: the LMPC linearised along the
reference, the NMPC exactly.
"""
import numpy as np

from .params import RobotParams

# Decided 2026-10-21: 17x the largest shadow price of the hard limit measured in the LMPC QP
# (585 per rad/s at a 30 cm / 30 deg start error), so the soft limit acts as a hard one whenever
# it can be met (exact penalty) and the problem stays solvable when it cannot.
SLACK_LIN = 1.0e4      # cost per rad/s above omega_max, per wheel and step
SLACK_QUAD = 1.0e4     # cost per (rad/s)^2 above omega_max, per wheel and step


def wheel_speeds(x, p: RobotParams):
    """x (6,) or (n, 6) world-frame states -> wheel speeds (4,) or (n, 4) in rad/s."""
    x = np.asarray(x, dtype=float)
    c, s = np.cos(x[..., 2]), np.sin(x[..., 2])
    nu = np.stack([c * x[..., 3] + s * x[..., 4], -s * x[..., 3] + c * x[..., 4], x[..., 5]], -1)
    return nu @ p.W.T


def wheel_speed_jacobian(x, p: RobotParams):
    """d omega / d x at one state x (6,) -> (4, 6)."""
    c, s = np.cos(x[2]), np.sin(x[2])
    J = np.zeros((3, 6))
    J[0, 2], J[1, 2] = -s * x[3] + c * x[4], -c * x[3] - s * x[4]
    J[0, 3], J[0, 4] = c, s
    J[1, 3], J[1, 4] = -s, c
    J[2, 5] = 1.0
    return p.W @ J
