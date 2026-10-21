"""Plan tests 1-7 (section "Verification and tests"). Run: python3 -m pytest test/ -q"""
import os

import numpy as np
import pytest
import quadprog
from scipy.integrate import solve_ivp

from mecanum_mpc_common import dynamics, params
from mecanum_lmpc_v2.lmpc import STATUS_SOLVED, LinearMPCv2
from mecanum_lmpc_v2.offline_sim import run
from mecanum_mpc_common.reference import SHAPES, Reference

URDF = os.path.join(os.path.dirname(__file__), '..', '..', 'tb3_sim_bringup', 'models',
                    'own_mecanum', 'robot.urdf')

# paper robot (eq. 101) and torques (eq. 102)
m0, m1, Rw, rho, l, J0, J1, J2 = 3.1, 0.35, 0.05, 0.15, 0.15, 0.032, 6.25e-4, 3.13e-4
M_PAPER = np.array([0.05, 0.25, -0.05, 0.20])


def paper_params():
    return params.RobotParams(m_s=m0 + 4 * m1, J_C=J0 + 4 * (J2 + m1 * (rho ** 2 + l ** 2)),
                              J1=J1, R=Rw, rho=rho, l=l)


def robot():
    return params.from_urdf(URDF)


# ---- test 1: paper equivalence -------------------------------------------------------------------
def test_world_model_equals_paper_eq64():
    p = paper_params()
    L, ms, JC = p.L, p.m_s, p.J_C
    A = ms * Rw ** 2 / 8 + JC * Rw ** 2 / (16 * L ** 2) + J1          # eq. (39)
    B = JC * Rw ** 2 / (16 * L ** 2)
    C = ms * Rw ** 2 / 8 - JC * Rw ** 2 / (16 * L ** 2)
    Mm = np.array([[2 * (A + C), A + C, -(A + C)],                     # eq. (64)
                   [A + C, 2 * (A + B), -(A + 2 * B - C)],
                   [-(A + C), -(A + 2 * B - C), 2 * (A + B)]])
    Q = np.array([M_PAPER[0] + M_PAPER[3], M_PAPER[1] + M_PAPER[3], M_PAPER[2] - M_PAPER[3]])
    k = Rw * (B + C) / L

    def paper_rhs(t, z):                       # z = [x, y, psi, phi1_dot, phi2_dot, phi3_dot]
        _, _, psi, w1, w2, w3 = z
        P = np.array([-k * (w2 + w3) * (w2 - w3), k * (w1 - w3) * (w2 - w3), k * (w1 + w2) * (w2 - w3)])
        wd = np.linalg.solve(Mm, Q - P)
        Vxi, Veta, psid = Rw / 2 * (w1 + w2), Rw / 2 * (w3 - w1), Rw / (2 * L) * (w2 - w3)   # eq. (12)
        return [np.cos(psi) * Vxi - np.sin(psi) * Veta, np.sin(psi) * Vxi + np.cos(psi) * Veta,  # (14)
                psid, *wd]

    T = np.linspace(0, 2, 41)
    zp = solve_ivp(paper_rhs, (0, 2), np.zeros(6), t_eval=T, rtol=1e-11, atol=1e-13).y
    xw = solve_ivp(lambda t, x: dynamics.f(x, M_PAPER, p), (0, 2), np.zeros(6), t_eval=T,
                   rtol=1e-11, atol=1e-13).y
    w1, w2, w3, psi = zp[3], zp[4], zp[5], zp[2]
    Vxi, Veta = Rw / 2 * (w1 + w2), Rw / 2 * (w3 - w1)
    vel_paper = np.vstack([np.cos(psi) * Vxi - np.sin(psi) * Veta,
                           np.sin(psi) * Vxi + np.cos(psi) * Veta, Rw / (2 * L) * (w2 - w3)])
    assert np.abs(xw[3:] - vel_paper).max() < 1e-6
    assert np.abs(xw[:3] - zp[:3]).max() < 1e-6


def test_kappa_one_gives_the_papers_circle():
    class Approx(params.RobotParams):          # the pseudoinverse + Lagrange II model: kappa = 1
        @property
        def kappa(self):
            return 1.0
    pa = paper_params()
    p = Approx(m_s=pa.m_s, J_C=pa.J_C, J1=pa.J1, R=pa.R, rho=pa.rho, l=pa.l)
    Mxi, Meta, Mpsi = (p.G @ M_PAPER) / np.array([p.m_e, p.m_e, p.J_e])
    xs = solve_ivp(lambda t, x: dynamics.f(x, M_PAPER, p), (0, 3), np.zeros(6),
                   t_eval=np.linspace(0, 3, 61), rtol=1e-11, atol=1e-13).y
    radius = np.hypot(xs[0] + Meta / Mpsi, xs[1] - Mxi / Mpsi)           # eq. (98)
    assert np.abs(radius - np.hypot(Mxi, Meta) / abs(Mpsi)).max() < 1e-6


# ---- test 2: Jacobians ---------------------------------------------------------------------------
def test_jacobians_match_finite_differences():
    p, rng, h = robot(), np.random.default_rng(1), 1e-6
    for _ in range(5):
        x, u = rng.normal(size=6), 0.05 * rng.normal(size=4)
        A, B = dynamics.jacobians(x, u, p)
        An = np.array([(dynamics.f(x + h * e, u, p) - dynamics.f(x - h * e, u, p)) / (2 * h)
                       for e in np.eye(6)]).T
        Bn = np.array([(dynamics.f(x, u + h * e, p) - dynamics.f(x, u - h * e, p)) / (2 * h)
                       for e in np.eye(4)]).T
        assert np.allclose(A, An, rtol=1e-6, atol=1e-7) and np.allclose(B, Bn, rtol=1e-6, atol=1e-7)
        Ad, Bd = dynamics.rk4_jacobians(x, u, 0.05, p)
        Adn = np.array([(dynamics.rk4_step(x + h * e, u, 0.05, p) - dynamics.rk4_step(x - h * e, u, 0.05, p))
                        / (2 * h) for e in np.eye(6)]).T
        Bdn = np.array([(dynamics.rk4_step(x, u + h * e, 0.05, p) - dynamics.rk4_step(x, u - h * e, 0.05, p))
                        / (2 * h) for e in np.eye(4)]).T
        assert np.allclose(Ad, Adn, rtol=1e-6, atol=1e-8) and np.allclose(Bd, Bdn, rtol=1e-6, atol=1e-8)


# ---- test 3: RK4 ---------------------------------------------------------------------------------
def test_rk4_order_and_standstill():
    p = robot()
    x0, u = np.array([0, 0, 0.3, 0.2, -0.1, 0.8]), np.array([0.01, -0.02, 0.015, 0.005])
    exact = solve_ivp(lambda t, x: dynamics.f(x, u, p), (0, 1), x0, rtol=1e-12, atol=1e-14).y[:, -1]
    errs = []
    for T in (0.1, 0.05):
        x = x0.copy()
        for _ in range(int(round(1 / T))):
            x = dynamics.rk4_step(x, u, T, p)
        errs.append(np.abs(x - exact).max())
    assert abs(np.log2(errs[0] / errs[1]) - 4) < 0.2
    T = 0.05
    Ad, Bd = dynamics.rk4_jacobians(np.zeros(6), np.zeros(4), T, p)
    _, Bc = dynamics.jacobians(np.zeros(6), np.zeros(4), p)
    Bv = Bc[3:]
    assert np.allclose(Ad, np.block([[np.eye(3), T * np.eye(3)], [np.zeros((3, 3)), np.eye(3)]]), atol=1e-15)
    assert np.allclose(Bd, np.vstack([T ** 2 / 2 * Bv, T * Bv]), atol=1e-15)


# ---- test 4: reference ---------------------------------------------------------------------------
@pytest.mark.parametrize('shape,heading,t_start', [('figure8', 'tangent', 3.0), ('circle', 'fixed', 3.0),
                                                   ('square', 'tangent', 3.2)])   # crosses a joint
def test_reference_consistency(shape, heading, t_start):
    p = robot()
    ref = Reference(p, shape, None, 0.2, heading)
    ref.anchor((0.3, -0.2, 0.7))
    T, N = 0.05, 20
    x_r, u_r = ref.horizon(t_start, T, N)
    assert np.abs(u_r @ params.NULL_TORQUE).max() < 1e-12          # no null torque
    # G u_r = mean wrench over each step (independent fine quadrature of the force part)
    for j in range(N):
        ts = np.linspace(t_start + j * T, t_start + (j + 1) * T, 2001)
        F = ref._force(ts)
        Fmean = ((F[1:] + F[:-1]) / 2).sum(axis=0) * (ts[1] - ts[0]) / T
        w = p.G @ u_r[j]
        assert np.allclose(w[:2], Fmean, atol=2e-3 * max(1.0, np.abs(Fmean).max()))
    # stepping the model with u_r stays on x_r for 1 s
    x = x_r[0].copy()
    for j in range(N):
        x = dynamics.rk4_step(x, u_r[j], T, p)
    assert np.hypot(*(x[:2] - x_r[N, :2])) < 1e-4


# ---- test 5 and 6: prediction and QP -------------------------------------------------------------
def _one_step(offset=(0.01, -0.005, 0.02, 0, 0, 0), t=3.0):
    p = robot()
    ref = Reference(p, 'figure8', None, 0.1, 'tangent')
    ref.anchor((0.0, 0.0, 0.0))
    mpc = LinearMPCv2(p)
    x_r, u_r = ref.horizon(t, mpc.T, mpc.N)
    u, info = mpc.step(x_r[0] + np.array(offset), x_r, u_r, u_r[0])
    return p, mpc, x_r, u_r, info


def test_condensed_prediction_equals_stepping():
    p, mpc, x_r, u_r, info = _one_step()
    rng = np.random.default_rng(2)
    ub = 0.01 * rng.normal(size=4 * mpc.N)
    pred = info['e'] + info['Bbar'] @ ub
    xt = info['x_tilde0']
    for j in range(mpc.N):
        Ad, Bd = dynamics.rk4_jacobians(x_r[j], u_r[j], mpc.T, p)
        c = dynamics.rk4_step(x_r[j], u_r[j], mpc.T, p) - x_r[j + 1]
        xt = Ad @ xt + Bd @ ub[4 * j:4 * j + 4] + c
        assert np.allclose(pred[6 * j:6 * j + 6], xt, atol=1e-12)
    H = info['H']
    assert np.allclose(H, H.T) and np.linalg.eigvalsh(H).min() > 0


def test_qp_unconstrained_and_kkt():
    _, mpc, _, _, info = _one_step()
    assert np.allclose(info['sol'], -np.linalg.solve(info['H'], info['f']), atol=1e-9)
    _, mpc, _, _, info = _one_step(offset=(1.0, 0.8, 1.0, 0, 0, 0))    # far off: limits active
    H, f, b = info['H'], info['f'], info['b']
    x, _, _, _, lam, _ = quadprog.solve_qp(H, -f, mpc.C, b, 0)
    slack = mpc.C.T @ x - b
    assert slack.min() > -1e-9 and (slack < 1e-9).any()                  # feasible, some active
    assert lam.min() >= -1e-12 and np.abs(lam * slack).max() < 1e-9      # complementarity
    scale = np.abs(H).max() * np.abs(x).max() + np.abs(f).max()
    assert np.abs(H @ x + f - mpc.C @ lam).max() < 1e-12 * scale         # stationarity


# ---- test 7: offline closed loop -----------------------------------------------------------------
@pytest.mark.parametrize('shape', SHAPES)
@pytest.mark.parametrize('heading', ['fixed', 'tangent'])
def test_closed_loop_nominal(shape, heading):
    r = run(shape, heading, plant='nominal', urdf=URDF)
    assert r['not_solved'] == 0 and r['rms_mm'] < 1.0


@pytest.mark.parametrize('shape', SHAPES)
def test_closed_loop_perturbed_stays_bounded(shape):
    r = run(shape, 'tangent', plant='perturbed', urdf=URDF)
    assert r['not_solved'] == 0 and r['max_mm'] < 20.0
