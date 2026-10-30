"""The Chaplygin dynamics of dynamics.py (Zeidis & Zimmermann, ZAMM 2019) with three wheel speeds as the velocity states
(the MPC's default since 2026-10-29):

    z = [X, Y, psi, w1, w2, w3]     world-frame pose and the speeds of wheels 1-3 (front-left, front-right, rear-left)
    u = [M1, M2, M3, M4]            wheel torques [N m]

The rolling constraints (params.W, eq. 11) tie the four wheel speeds to the three body velocities
nu = [V_Cxi, V_Ceta, psi_dot]: w = W nu. Three of them are independent: nu = W3^-1 [w1, w2, w3] (W3 = rows 1-3 of W),
and the fourth follows, w4 = w1 + w2 - w3 (n^T W = 0, n = params.NULL_TORQUE). The model is the time derivative of z:

    [X_dot, Y_dot] = R(psi) [V_Cxi, V_Ceta],   psi_dot = nu_3
    [w1, w2, w3]_dot = W3 nu_dot
    nu_dot = [ (m_s/m_e) psi_dot V_Ceta + F_xi / m_e,  -(m_s/m_e) psi_dot V_Cxi + F_eta / m_e,  tau / J_e ],
    [F_xi, F_eta, tau] = G u

i.e. the wheel angular accelerations of the Chaplygin equations directly (nu_dot is dynamics.f in the body frame:
R^T a - psi_dot S nu with a = dynamics.f's acceleration). The velocity dynamics do not depend on the pose, and every
wheel speed is linear in z (the wheel-speed limit needs no linearisation).
"""
import numpy as np

from .params import RobotParams

STATE_WHEELS = (0, 1, 2)
"""The wheels whose speeds are states (front-left, front-right, rear-left); rear-right = w1 + w2 - w3."""

_MATS = {}


def _mats(p: RobotParams):
    """(W3, J3, G, the wheel-acceleration input matrix) of p, built once per parameter set and read-only: params.W
    builds W on every access, which was half of an MPC step when f and the Jacobians called it (same values)."""
    key = (p.m_s, p.J_C, p.J1, p.R, p.rho, p.l)
    m = _MATS.get(key)
    if m is None:
        W = p.W[list(STATE_WHEELS)]
        G = p.G
        m = (W, np.linalg.inv(W), G, W @ np.diag([1.0 / p.m_e, 1.0 / p.m_e, 1.0 / p.J_e]) @ G)
        for a in m:
            a.flags.writeable = False
        _MATS[key] = m
    return m


def W3(p: RobotParams):
    """Rows of the rolling constraints of the state wheels: [w1, w2, w3] = W3 nu  (3x3)."""
    return _mats(p)[0]


def J3(p: RobotParams):
    """Body velocities from the state wheel speeds: nu = J3 [w1, w2, w3]  (3x3)."""
    return _mats(p)[1]


def all_wheels(p: RobotParams):
    """All four wheel speeds from the three state wheels: w = E [w1, w2, w3]  (4x3)."""
    return p.W @ J3(p)


def f(z, u, p: RobotParams):
    """Continuous dynamics z_dot = f(z, u)."""
    psi = z[2]
    Wm, Jw, G, _ = _mats(p)
    a, b, r = Jw @ z[3:6]
    F_xi, F_eta, tau = G @ u
    mu = p.m_s / p.m_e
    c, s = np.cos(psi), np.sin(psi)
    nu_dot = np.array([mu * r * b + F_xi / p.m_e, -mu * r * a + F_eta / p.m_e, tau / p.J_e])
    return np.r_[c * a - s * b, s * a + c * b, r, Wm @ nu_dot]


def jacobians(z, u, p: RobotParams):
    """A_c = df/dz (6x6) and B_c = df/du (6x4) of the continuous dynamics."""
    psi = z[2]
    Wm, Jw, _, Bw = _mats(p)
    a, b, r = Jw @ z[3:6]
    mu = p.m_s / p.m_e
    c, s = np.cos(psi), np.sin(psi)
    A = np.zeros((6, 6))
    A[0, 2], A[1, 2] = -s * a - c * b, c * a - s * b
    A[0, 3:] = c * Jw[0] - s * Jw[1]
    A[1, 3:] = s * Jw[0] + c * Jw[1]
    A[2, 3:] = Jw[2]
    dnu = np.array([[0.0, mu * r, mu * b], [-mu * r, 0.0, -mu * a], [0.0, 0.0, 0.0]])  # d nu_dot / d nu
    A[3:, 3:] = Wm @ dnu @ Jw
    B = np.zeros((6, 4))
    B[3:] = Bw
    return A, B


def rk4_step(z, u, T, p: RobotParams):
    """One RK4 step with u held over T: z_{k+1} = F(z_k, u_k)."""
    k1 = f(z, u, p)
    k2 = f(z + T / 2 * k1, u, p)
    k3 = f(z + T / 2 * k2, u, p)
    k4 = f(z + T * k3, u, p)
    return z + T / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def rk4_jacobians(z, u, T, p: RobotParams):
    """Exact Jacobians of the RK4 step, A_d = dF/dz (6x6), B_d = dF/du (6x4), by the chain rule through the four
    stages (as dynamics.rk4_jacobians)."""
    I = np.eye(6)
    k1 = f(z, u, p)
    z2 = z + T / 2 * k1
    k2 = f(z2, u, p)
    z3 = z + T / 2 * k2
    k3 = f(z3, u, p)
    z4 = z + T * k3
    A1, B1 = jacobians(z, u, p)
    A2, B2 = jacobians(z2, u, p)
    A3, B3 = jacobians(z3, u, p)
    A4, B4 = jacobians(z4, u, p)
    K2x, K2u = A2 @ (I + T / 2 * A1), T / 2 * A2 @ B1 + B2
    K3x, K3u = A3 @ (I + T / 2 * K2x), T / 2 * A3 @ K2u + B3
    K4x, K4u = A4 @ (I + T * K3x), T * A4 @ K3u + B4
    A_d = I + T / 6 * (A1 + 2 * K2x + 2 * K3x + K4x)
    B_d = T / 6 * (B1 + 2 * K2u + 2 * K3u + K4u)
    return A_d, B_d


def from_world(x, p: RobotParams):
    """World-frame states [X, Y, psi, X_dot, Y_dot, psi_dot] (6,) or (n, 6) -> wheel-speed states, assuming rolling
    without slip (the velocities rotated into the body frame, times W3)."""
    x = np.asarray(x, dtype=float)
    c, s = np.cos(x[..., 2]), np.sin(x[..., 2])
    nu = np.stack([c * x[..., 3] + s * x[..., 4], -s * x[..., 3] + c * x[..., 4], x[..., 5]], -1)
    return np.concatenate([x[..., :3], nu @ W3(p).T], -1)


def to_world(z, p: RobotParams):
    """Wheel-speed states (6,) or (n, 6) -> world-frame states [X, Y, psi, X_dot, Y_dot, psi_dot]."""
    z = np.asarray(z, dtype=float)
    nu = z[..., 3:6] @ J3(p).T
    c, s = np.cos(z[..., 2]), np.sin(z[..., 2])
    return np.concatenate([z[..., :3], np.stack([c * nu[..., 0] - s * nu[..., 1], s * nu[..., 0] + c * nu[..., 1],
                                                 nu[..., 2]], -1)], -1)


def from_encoders(x, w, p: RobotParams):
    """The measured state: the pose of the world-frame state x (6,) or (n, 6), the wheel speeds from the four encoders
    w (4,) or (n, 4), MPC order and signs. The four speeds are projected onto the rolling constraints (least squares,
    nu = W+ w): this drops their part along n = (1, 1, -1, -1), which no motion without slip produces (roller slip),
    and keeps all four encoders in the estimate."""
    w = np.asarray(w, dtype=float)
    nu = w @ p.G_pinv     # (W+ w)^T; W+ = G_pinv^T
    return np.concatenate([np.asarray(x, dtype=float)[..., :3], nu @ W3(p).T], -1)


def world_velocity(z, p: RobotParams):
    """(X_dot, Y_dot) of a wheel-speed state (6,)."""
    a, b, _ = J3(p) @ z[3:6]
    c, s = np.cos(z[2]), np.sin(z[2])
    return np.array([c * a - s * b, s * a + c * b])


def wheel_speeds(z, p: RobotParams):
    """All four wheel speeds (4,) or (n, 4) of wheel-speed states (6,) or (n, 6), rad/s."""
    return np.asarray(z, dtype=float)[..., 3:6] @ all_wheels(p).T


def wheel_speed_jacobian(z, p: RobotParams):
    """d omega / d z (4, 6): constant, the wheel speeds are linear in the state."""
    G = np.zeros((4, 6))
    G[:, 3:] = all_wheels(p)
    return G


def velocity_weight(q, p: RobotParams):
    """The world-state velocity weights diag(q) on (X_dot, Y_dot, psi_dot) as a weight on the wheel-speed states:
    J3^T diag(q) J3 (q_X = q_Y: rotation-invariant, so it weights the body velocities the same). Equal costs for equal
    motions; the matrix is full (the wheels share the body velocities)."""
    Jw = J3(p)
    return Jw.T @ np.diag(np.asarray(q, dtype=float)) @ Jw
