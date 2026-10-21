"""Exact (Chaplygin) dynamics of Zeidis & Zimmermann (ZAMM 2019) in the state and input of the plan
(sections "Exact dynamics in the required state and input" and "RK4 discretisation").

    x = [X, Y, psi, X_dot, Y_dot, psi_dot]     world-frame pose and its time derivatives
    u = [M1, M2, M3, M4]                        wheel torques [N m]

    X_ddot = -kappa psi_dot Y_dot + (cos psi F_xi - sin psi F_eta) / m_e
    Y_ddot =  kappa psi_dot X_dot + (sin psi F_xi + cos psi F_eta) / m_e
    psi_ddot = tau / J_e,        [F_xi, F_eta, tau] = G u
"""
import numpy as np

from .params import RobotParams


def f(x, u, p: RobotParams):
    """Continuous dynamics x_dot = f(x, u)."""
    psi, Xd, Yd, r = x[2], x[3], x[4], x[5]
    F_xi, F_eta, tau = p.G @ u
    c, s = np.cos(psi), np.sin(psi)
    return np.array([
        Xd,
        Yd,
        r,
        -p.kappa * r * Yd + (c * F_xi - s * F_eta) / p.m_e,
        p.kappa * r * Xd + (s * F_xi + c * F_eta) / p.m_e,
        tau / p.J_e,
    ])


def jacobians(x, u, p: RobotParams):
    """A_c = df/dx (6x6) and B_c = df/du (6x4) of the continuous dynamics."""
    psi, Xd, Yd, r = x[2], x[3], x[4], x[5]
    F_xi, F_eta, _ = p.G @ u
    c, s = np.cos(psi), np.sin(psi)
    A = np.zeros((6, 6))
    A[0, 3] = A[1, 4] = A[2, 5] = 1.0
    A[3, 2] = -(s * F_xi + c * F_eta) / p.m_e
    A[3, 4] = -p.kappa * r
    A[3, 5] = -p.kappa * Yd
    A[4, 2] = (c * F_xi - s * F_eta) / p.m_e
    A[4, 3] = p.kappa * r
    A[4, 5] = p.kappa * Xd
    B = np.zeros((6, 4))
    Rot = np.array([[c, -s], [s, c]])
    B[3:5] = Rot @ p.G[:2] / p.m_e
    B[5] = p.G[2] / p.J_e
    return A, B


def rk4_step(x, u, T, p: RobotParams):
    """One RK4 step with u held over T: x_{k+1} = F(x_k, u_k)."""
    k1 = f(x, u, p)
    k2 = f(x + T / 2 * k1, u, p)
    k3 = f(x + T / 2 * k2, u, p)
    k4 = f(x + T * k3, u, p)
    return x + T / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def rk4_jacobians(x, u, T, p: RobotParams):
    """Exact Jacobians of the RK4 step (decision D2): A_d = dF/dx (6x6), B_d = dF/du (6x4),
    by the chain rule through the four stages z1 = x, z2 = x + T/2 k1, z3 = x + T/2 k2,
    z4 = x + T k3."""
    I = np.eye(6)
    k1 = f(x, u, p)
    z2 = x + T / 2 * k1
    k2 = f(z2, u, p)
    z3 = x + T / 2 * k2
    k3 = f(z3, u, p)
    z4 = x + T * k3
    A1, B1 = jacobians(x, u, p)
    A2, B2 = jacobians(z2, u, p)
    A3, B3 = jacobians(z3, u, p)
    A4, B4 = jacobians(z4, u, p)
    K1x, K1u = A1, B1
    K2x, K2u = A2 @ (I + T / 2 * K1x), T / 2 * A2 @ K1u + B2
    K3x, K3u = A3 @ (I + T / 2 * K2x), T / 2 * A3 @ K2u + B3
    K4x, K4u = A4 @ (I + T * K3x), T * A4 @ K3u + B4
    A_d = I + T / 6 * (K1x + 2 * K2x + 2 * K3x + K4x)
    B_d = T / 6 * (K1u + 2 * K2u + 2 * K3u + K4u)
    return A_d, B_d
