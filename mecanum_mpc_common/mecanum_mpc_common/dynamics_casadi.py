"""The Chaplygin model of dynamics.py in CasADi symbols, for acados (NMPC plan: "Model in
acados"). Same equations, same RK4 step; test_common.py checks both against each other.

    f(x, u)     continuous dynamics
    F(x, u)     one RK4 step over T with u held (the LMPC's dynamics.rk4_step)
    omega(x)    wheel speeds (wheel_speed.wheel_speeds)
"""
import casadi as ca

from .params import RobotParams


def f(x, u, p: RobotParams):
    psi, Xd, Yd, r = x[2], x[3], x[4], x[5]
    w = ca.DM(p.G) @ u
    F_xi, F_eta, tau = w[0], w[1], w[2]
    c, s = ca.cos(psi), ca.sin(psi)
    return ca.vertcat(
        Xd,
        Yd,
        r,
        -p.kappa * r * Yd + (c * F_xi - s * F_eta) / p.m_e,
        p.kappa * r * Xd + (s * F_xi + c * F_eta) / p.m_e,
        tau / p.J_e,
    )


def rk4_step(x, u, T, p: RobotParams):
    k1 = f(x, u, p)
    k2 = f(x + T / 2 * k1, u, p)
    k3 = f(x + T / 2 * k2, u, p)
    k4 = f(x + T * k3, u, p)
    return x + T / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def wheel_speeds(x, p: RobotParams):
    c, s = ca.cos(x[2]), ca.sin(x[2])
    nu = ca.vertcat(c * x[3] + s * x[4], -s * x[3] + c * x[4], x[5])
    return ca.DM(p.W) @ nu
