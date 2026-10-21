"""Tests of the shared model code (NMPC plan, test 1): the CasADi model used by acados must be the
NumPy model used by the LMPC, value for value and derivative for derivative."""
import os

import casadi as ca
import numpy as np

from mecanum_mpc_common import dynamics, dynamics_casadi, params
from mecanum_mpc_common.wheel_speed import wheel_speed_jacobian, wheel_speeds

URDF = os.path.join(os.path.dirname(__file__), '..', '..', 'tb3_sim_bringup', 'models',
                    'own_mecanum', 'robot.urdf')
T = 0.05


def robot():
    return params.from_urdf(URDF)


def random_points(n=1000, seed=3):
    rng = np.random.default_rng(seed)
    x = rng.normal(scale=[1.0, 1.0, 3.0, 0.3, 0.3, 1.0], size=(n, 6))
    u = rng.normal(scale=0.5, size=(n, 4))
    return x, u


def casadi_functions(p):
    x, u = ca.SX.sym('x', 6), ca.SX.sym('u', 4)
    F = dynamics_casadi.rk4_step(x, u, T, p)
    return (ca.Function('f', [x, u], [dynamics_casadi.f(x, u, p)]),
            ca.Function('F', [x, u], [F, ca.jacobian(F, x), ca.jacobian(F, u)]),
            ca.Function('w', [x], [dynamics_casadi.wheel_speeds(x, p)]))


def test_casadi_model_equals_numpy_model():
    p = robot()
    f_ca, F_ca, w_ca = casadi_functions(p)
    for x, u in zip(*random_points()):
        assert np.allclose(np.array(f_ca(x, u)).ravel(), dynamics.f(x, u, p), rtol=1e-12, atol=1e-12)
        F, A, B = (np.array(v) for v in F_ca(x, u))
        assert np.allclose(F.ravel(), dynamics.rk4_step(x, u, T, p), rtol=1e-12, atol=1e-12)
        A_np, B_np = dynamics.rk4_jacobians(x, u, T, p)
        assert np.allclose(A, A_np, rtol=1e-9, atol=1e-9)
        assert np.allclose(B, B_np, rtol=1e-9, atol=1e-9)
        assert np.allclose(np.array(w_ca(x)).ravel(), wheel_speeds(x, p), rtol=1e-12, atol=1e-12)


def test_wheel_speed_jacobian():
    p = robot()
    for x, _ in zip(*random_points(200)):
        J = wheel_speed_jacobian(x, p)
        h = 1e-6
        J_fd = np.column_stack([(wheel_speeds(x + h * e, p) - wheel_speeds(x - h * e, p)) / (2 * h)
                                for e in np.eye(6)])
        assert np.allclose(J, J_fd, rtol=1e-6, atol=1e-6)
