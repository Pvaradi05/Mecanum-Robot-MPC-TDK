import os

import numpy as np
from scipy.integrate import solve_ivp

from mecanum_mpc import model
from mecanum_mpc.offline_sim import run
from mecanum_mpc.references import Reference

URDF = os.path.join(os.path.dirname(__file__), '..', '..', 'tb3_sim_bringup', 'models',
                    'own_mecanum', 'robot.urdf')


def test_params_from_urdf():
    p = model.params_from_urdf(URDF)
    assert abs(p.ms - 3.0) < 1e-6
    assert abs(p.R - 0.03028) < 1e-5          # roller envelope, wheel_geometry.json
    assert abs(p.lx - 0.064) < 1e-9 and abs(p.ly - 0.1526) < 1e-9


def test_body_form_equals_paper_chaplygin_eq64():
    """Zeidis & Zimmermann eq. (64) with the paper's robot (101) and torques (102)."""
    m0, m1, R, rho, l, J0, J1, J2 = 3.1, 0.35, 0.05, 0.15, 0.15, 0.032, 6.25e-4, 3.13e-4
    L = rho + l
    ms, JC = m0 + 4 * m1, J0 + 4 * (J2 + m1 * (rho**2 + l**2))
    A = ms * R**2 / 8 + JC * R**2 / (16 * L**2) + J1
    B = JC * R**2 / (16 * L**2)
    C = ms * R**2 / 8 - JC * R**2 / (16 * L**2)
    M = np.array([0.05, 0.25, -0.05, 0.20])

    def exact(t, w):
        w1, w2, w3 = w
        k = R * (B + C) / L
        P = np.array([-k * (w2 + w3) * (w2 - w3), k * (w1 - w3) * (w2 - w3), k * (w1 + w2) * (w2 - w3)])
        Mm = np.array([[2 * (A + C), A + C, -(A + C)], [A + C, 2 * (A + B), -(A + 2 * B - C)],
                       [-(A + C), -(A + 2 * B - C), 2 * (A + B)]])
        return np.linalg.solve(Mm, np.array([M[0] + M[3], M[1] + M[3], M[2] - M[3]]) - P)

    p = model.Params(ms=ms, JC=JC, J1=J1, R=R, lx=rho, ly=l)
    wrench = p.torque_to_wrench() @ M
    T = np.linspace(0, 2, 21)
    ex = solve_ivp(exact, (0, 2), [0, 0, 0], t_eval=T, rtol=1e-10, atol=1e-12)
    w1, w2, w3 = ex.y
    uvr = np.vstack([R / 2 * (w1 + w2), R / 2 * (w3 - w1), R / (2 * L) * (w2 - w3)])
    body = solve_ivp(lambda t, s: model.f(np.r_[0, 0, 0, s], wrench, p)[3:], (0, 2), [0, 0, 0],
                     t_eval=T, rtol=1e-10, atol=1e-12)
    assert np.abs(body.y - uvr).max() < 1e-6


def test_jacobians_match_finite_differences():
    p = model.params_from_urdf(URDF)
    rng = np.random.default_rng(0)
    for _ in range(5):
        x, w = rng.normal(size=6), rng.normal(size=3)
        A, B = model.jacobians(x, p)
        h = 1e-6
        An = np.array([(model.f(x + h * e, w, p) - model.f(x - h * e, w, p)) / (2 * h) for e in np.eye(6)]).T
        Bn = np.array([(model.f(x, w + h * e, p) - model.f(x, w - h * e, p)) / (2 * h) for e in np.eye(3)]).T
        assert np.allclose(A, An, atol=1e-6) and np.allclose(B, Bn, atol=1e-6)


def test_square_corner_radius():
    p = model.params_from_urdf(URDF)
    default = Reference(p, 'square', 1.5, heading='tangent')
    same = Reference(p, 'square', 1.5, heading='tangent', corner_radius=0.225)
    assert np.allclose(default.x, same.x)
    sharp = Reference(p, 'square', 1.5, corner_radius=0.0)
    assert abs(sharp.length - 6.0) < 1e-6 and np.isfinite(sharp.x).all()
    # at a sharp corner the exact force reference is an impulse: huge as a point value,
    # about me * dv / dt as the mean over a control step
    t = np.arange(0, sharp.duration, 0.05)
    assert np.abs(sharp.sample(t)[1]).max() > 100
    assert np.abs(sharp.sample(t, w_window=0.05)[1]).max() < 15


def test_ltv_mpc_tracks_on_nominal_model():
    r = run('figure8', 'tangent', offset=(0.2, -0.1, 0.5), urdf=URDF)
    assert r['failed'] == 0
    assert r['rms_mm'] < 2.0
