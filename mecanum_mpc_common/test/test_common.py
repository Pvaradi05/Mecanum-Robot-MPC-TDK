"""Tests of the shared model code (NMPC plan, test 1): the CasADi model used by acados must be the
NumPy model used by the LMPC, value for value and derivative for derivative; the wheel-speed state
model must be dynamics.py in other coordinates."""
import os

import casadi as ca
import numpy as np

from mecanum_mpc_common import dynamics, params, wheel_dynamics, wheel_dynamics_casadi
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
    z, u = ca.SX.sym('z', 6), ca.SX.sym('u', 4)
    F = wheel_dynamics_casadi.rk4_step(z, u, T, p)
    return (ca.Function('f', [z, u], [wheel_dynamics_casadi.f(z, u, p)]),
            ca.Function('F', [z, u], [F, ca.jacobian(F, z), ca.jacobian(F, u)]))


def test_casadi_model_equals_numpy_model():
    p = robot()
    f_ca, F_ca = casadi_functions(p)
    for x, u in zip(*random_points()):
        z = wheel_dynamics.from_world(x, p)
        assert np.allclose(np.array(f_ca(z, u)).ravel(), wheel_dynamics.f(z, u, p),
                           rtol=1e-12, atol=1e-12)
        F, A, B = (np.array(v) for v in F_ca(z, u))
        assert np.allclose(F.ravel(), wheel_dynamics.rk4_step(z, u, T, p), rtol=1e-12, atol=1e-12)
        A_np, B_np = wheel_dynamics.rk4_jacobians(z, u, T, p)
        assert np.allclose(A, A_np, rtol=1e-9, atol=1e-9)
        assert np.allclose(B, B_np, rtol=1e-9, atol=1e-9)


def test_wheel_speed_jacobian():
    p = robot()
    for x, _ in zip(*random_points(200)):
        J = wheel_speed_jacobian(x, p)
        h = 1e-6
        J_fd = np.column_stack([(wheel_speeds(x + h * e, p) - wheel_speeds(x - h * e, p)) / (2 * h)
                                for e in np.eye(6)])
        assert np.allclose(J, J_fd, rtol=1e-6, atol=1e-6)


def test_wheel_state_model():
    """Kemenylegeny/Mecanum-tracking-tdk @ 411436a, tests/test_mpc.py test_wheel_state_model."""
    p = robot()
    rng = np.random.default_rng(0)
    for _ in range(50):
        x = np.r_[rng.normal(0, 1, 2), rng.uniform(-np.pi, np.pi), rng.normal(0, 0.1, 2),
                  rng.normal(0, 0.5)]
        u = rng.normal(0, 0.02, 4)
        z = wheel_dynamics.from_world(x, p)
        assert np.allclose(wheel_dynamics.to_world(z, p), x)
        # z_dot = d from_world(x) / dt: the same motion in other coordinates
        xd, h = dynamics.f(x, u, p), 1e-7
        zd = (wheel_dynamics.from_world(x + h * xd, p)
              - wheel_dynamics.from_world(x - h * xd, p)) / (2 * h)
        assert np.abs(wheel_dynamics.f(z, u, p) - zd).max() < 1e-5
        # 1 s: both RK4s converge to the same solution
        xa, za = x.copy(), z.copy()
        for _ in range(200):
            xa, za = dynamics.rk4_step(xa, u, 0.005, p), wheel_dynamics.rk4_step(za, u, 0.005, p)
        assert np.abs(wheel_dynamics.to_world(za, p) - xa).max() < 1e-5
        # exact Jacobians (continuous and RK4) against finite differences
        A, B = wheel_dynamics.jacobians(z, u, p)
        An = np.stack([(wheel_dynamics.f(z + 1e-6 * e, u, p)
                        - wheel_dynamics.f(z - 1e-6 * e, u, p)) / 2e-6 for e in np.eye(6)], 1)
        assert np.abs(A - An).max() < 1e-5
        _, Bd = wheel_dynamics.rk4_jacobians(z, u, T, p)
        Bn = np.stack([(wheel_dynamics.rk4_step(z, u + 1e-6 * e, T, p)
                        - wheel_dynamics.rk4_step(z, u - 1e-6 * e, T, p)) / 2e-6
                       for e in np.eye(4)], 1)
        assert np.abs(Bd - Bn).max() < 1e-5
    # wheel 4 follows from the rolling constraints; the encoders' slip part (1, 1, -1, -1) is dropped
    w = wheel_dynamics.wheel_speeds(z, p)
    assert np.isclose(w[3], w[0] + w[1] - w[2])
    assert np.allclose(wheel_dynamics.from_encoders(x, w + 0.5 * np.array([1, 1, -1, -1]), p), z)
