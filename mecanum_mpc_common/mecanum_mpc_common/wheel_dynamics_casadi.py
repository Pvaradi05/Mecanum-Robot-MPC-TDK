"""The Chaplygin model with wheel-speed states of wheel_dynamics.py in CasADi symbols, for acados.
Same equations, same RK4 step; test_common.py checks both against each other.

    z = [X, Y, psi, w1, w2, w3]   pose and the speeds of wheels 1-3,  u = [M1, M2, M3, M4]
    f(z, u)     continuous dynamics (wheel_dynamics.f)
    F(z, u)     one RK4 step over T with u held (wheel_dynamics.rk4_step)
"""
import casadi as ca

from . import wheel_dynamics
from .params import RobotParams


def f(z, u, p: RobotParams):
    Wm, Jw, G = ca.DM(wheel_dynamics.W3(p)), ca.DM(wheel_dynamics.J3(p)), ca.DM(p.G)
    nu = Jw @ z[3:6]
    a, b, r = nu[0], nu[1], nu[2]
    w = G @ u
    F_xi, F_eta, tau = w[0], w[1], w[2]
    mu = p.m_s / p.m_e
    c, s = ca.cos(z[2]), ca.sin(z[2])
    nu_dot = ca.vertcat(mu * r * b + F_xi / p.m_e, -mu * r * a + F_eta / p.m_e, tau / p.J_e)
    return ca.vertcat(c * a - s * b, s * a + c * b, r, Wm @ nu_dot)


def rk4_step(z, u, T, p: RobotParams):
    k1 = f(z, u, p)
    k2 = f(z + T / 2 * k1, u, p)
    k3 = f(z + T / 2 * k2, u, p)
    k4 = f(z + T * k3, u, p)
    return z + T / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
