"""Tests of the nonlinear MPC (NMPC plan, "Verification", tests 3-10; tests 1-2 are in
mecanum_mpc_common and mecanum_lmpc_v2). The LTV-MPC it is compared with is the latest one,
LinearMPCv2 with mecanum_lmpc_v2.LATEST, with the NMPC's weights (for now Bryson's rule, not
the LTV-MPC's Q_BO), so that both solve the same problem."""
import os

import numpy as np
import pytest
from mecanum_lmpc_v2 import LATEST
from mecanum_lmpc_v2.lmpc import LinearMPCv2
from mecanum_lmpc_v2.offline_sim import run as run_lmpc
from mecanum_mpc_common import motor, params, wheel_dynamics
from mecanum_mpc_common.conventions import STATUS_SOLVED
from mecanum_mpc_common.reference import SHAPES, Reference
from mecanum_mpc_common.weights import Q_BO, Q_BRYSON

from mecanum_nmpc.nmpc import ACADOS_SUCCESS, NonlinearMPC
from mecanum_nmpc.offline_sim import run as run_nmpc

URDF = os.path.join(os.path.dirname(__file__), '..', '..', 'tb3_sim_bringup', 'models',
                    'own_mecanum', 'robot.urdf')
T, N = 0.05, 10
OFFSETS = [(0.001, 0.0, 0.0017), (0.01, 0.0, 0.017), (0.05, 0.0, 0.087), (0.1, 0.0, 0.17),
           (0.3, 0.0, 0.52)]                                  # 1 mm/0.1 deg ... 30 cm/30 deg


@pytest.fixture(scope='module')
def p():
    return params.from_urdf(URDF)


@pytest.fixture(scope='module')
def nmpc(p):
    return NonlinearMPC(p, T, N)


LTV = {**LATEST, 'Q': Q_BRYSON}       # the latest LTV-MPC with the NMPC's weights


def ltv(p):
    return LinearMPCv2(p, T, N, **LTV)


def horizon(p, shape='figure8', heading='tangent', t=12.0):
    """Reference in the wheel-speed state (z_r, u_r)."""
    ref = Reference(p, shape, None, 0.1, heading)
    ref.anchor((0.0, 0.0, 0.0))
    x_r, u_r = ref.horizon(t, T, N)
    return wheel_dynamics.from_world(x_r, p), u_r


def offset_state(z_r0, dp, dpsi):
    return z_r0 + np.array([dp / np.sqrt(2), dp / np.sqrt(2), dpsi, 0, 0, 0])


# ---- the problem is the LTV-MPC's --------------------------------------------------------------
def test_same_problem_as_ltv(p, nmpc):
    lt = ltv(p)
    assert nmpc.state == lt.state == 'wheel'
    assert np.array_equal(nmpc.Q, lt.Q) and np.array_equal(nmpc.R, lt.R)
    assert nmpc.u_max == lt.u_max and (nmpc.T, nmpc.N) == (lt.T, lt.N)
    # the same terminal weight from the same history (DARE, warm-started)
    a, b = NonlinearMPC(p, T, N), ltv(p)
    for t in (6.0, 6.05, 6.1, 12.0):
        z_r, u_r = horizon(p, 'square', 'tangent', t)
        a.step(z_r[0], z_r, u_r, u_r[0])
        b.step(z_r[0], z_r, u_r, u_r[0])
        assert np.array_equal(a.P_dare, b.P_dare) and a.dare_iterations == b.dare_iterations


# ---- test 3: on the reference -----------------------------------------------------------------
@pytest.mark.parametrize('shape,heading', [('figure8', 'tangent'), ('circle', 'fixed'),
                                           ('square', 'tangent')])
def test_on_reference(p, nmpc, shape, heading):
    z_r, u_r = horizon(p, shape, heading, 6.3)
    u, info = nmpc.step(z_r[0], z_r, u_r, u_r[0])
    assert info['status'] == STATUS_SOLVED and info['iterations'] <= 2
    # the reference is consistent with the model up to the D3 residuals c_j, so u = u_r up to
    # their effect: compare with the LTV-MPC, which sees the same residuals
    u_l, _ = ltv(p).step(z_r[0], z_r, u_r, u_r[0])
    assert np.abs(u - u_r[0]).max() < 1e-4 and np.abs(u - u_l).max() < 1e-6


# ---- test 4: the first iteration is the LTV-MPC step ------------------------------------------
@pytest.mark.parametrize('dp,_unused,dpsi', OFFSETS)
def test_first_iteration_equals_ltv(p, dp, _unused, dpsi):
    z_r, u_r = horizon(p)
    z = offset_state(z_r[0], dp, dpsi)
    _, info = NonlinearMPC(p, T, N).step(z, z_r, u_r, u_r[0])    # both from P0 = Q
    u_l, _ = ltv(p).step(z, z_r, u_r, u_r[0])
    assert info['alpha_first'] == 1.0
    assert np.abs(info['u_first'] - u_l).max() < 1e-6


# ---- test 5: convergence ----------------------------------------------------------------------
@pytest.mark.parametrize('dp,_unused,dpsi', OFFSETS[:4])
def test_converged_solution(p, dp, _unused, dpsi):
    nmpc = NonlinearMPC(p, T, N, max_iter=200, time_budget=0.0)      # without the N3 caps
    z_r, u_r = horizon(p)
    z = offset_state(z_r[0], dp, dpsi)
    u, info = nmpc.step(z, z_r, u_r, u_r[0])
    assert info['acados_status'] == ACADOS_SUCCESS and info['kkt'] <= 1e-6
    # the converged states obey the model: re-simulating the plan reproduces the prediction
    sv = nmpc.solver
    zj = z.copy()
    for j in range(N):
        zj = wheel_dynamics.rk4_step(zj, sv.get(j, 'u'), T, p)
        assert np.abs(zj - info['predicted'][j]).max() < 1e-8 * max(1.0, np.abs(zj).max())


# ---- test 6: local minima ---------------------------------------------------------------------
@pytest.mark.parametrize('dp,_unused,dpsi', OFFSETS[1:4])
def test_same_solution_from_other_starts(p, dp, _unused, dpsi):
    m = NonlinearMPC(p, T, N, max_iter=200, time_budget=0.0)
    z_r, u_r = horizon(p)
    z = offset_state(z_r[0], dp, dpsi)
    u_ref, info = m.step(z, z_r, u_r, u_r[0])
    assert info['acados_status'] == ACADOS_SUCCESS
    plan = np.array([m.solver.get(j, 'u') for j in range(N)])
    states = np.array([m.solver.get(j, 'x') for j in range(N + 1)])
    rng = np.random.default_rng(5)
    for guess in ('shifted', 'perturbed', 'perturbed'):
        sv = m.solver
        for j in range(N):           # same problem, a different starting point
            if guess == 'shifted':
                sv.set(j, 'u', plan[min(j + 1, N - 1)])
                sv.set(j, 'x', states[min(j + 1, N)])
            else:
                sv.set(j, 'u', plan[j] + rng.normal(scale=0.02, size=4))
                sv.set(j, 'x', states[j] + rng.normal(scale=[0.02, 0.02, 0.05, 0.5, 0.5, 0.5]))
        sv.set(N, 'x', states[N])
        assert sv.solve() == ACADOS_SUCCESS
        assert np.abs(sv.get(0, 'u') - u_ref).max() < 1e-4


# ---- test 7: the only constraint is the torque box --------------------------------------------
def test_torque_box_only(p):
    # with Q_BO: Bryson's weights do not drive a 30 cm start onto the box
    m = NonlinearMPC(p, T, N, Q=Q_BO, max_iter=200, time_budget=0.0)
    z_r, u_r = horizon(p)
    z = offset_state(z_r[0], 0.3, 0.52)
    _, info = m.step(z, z_r, u_r, -motor.U_MAX * np.ones(4))
    assert info['acados_status'] == ACADOS_SUCCESS
    plan = np.array([m.solver.get(j, 'u') for j in range(N)])
    # a 30 cm / 30 deg start drives the torques onto the box ...
    assert np.abs(plan).max() <= motor.U_MAX + 1e-9 and np.isclose(np.abs(plan).max(), motor.U_MAX)
    # ... and nothing else limits them: no torque-rate limit (from u_applied = -u_max the first
    # step changes by more than the old limit, 1.95 N m per step) and no wheel-speed limit
    assert np.abs(plan[0] + motor.U_MAX).max() > motor.DU_MAX
    assert np.abs(wheel_dynamics.wheel_speeds(info['predicted'], p)).max() > motor.OMEGA_MAX


# ---- test 8: offline closed loop ----------------------------------------------------------------
@pytest.mark.parametrize('shape', SHAPES)
@pytest.mark.parametrize('heading', ['fixed', 'tangent'])
def test_closed_loop_nominal(shape, heading):
    r = run_nmpc(shape, heading, plant='nominal', urdf=URDF)
    assert r['not_solved'] == 0 and r['rms_mm'] < 1.0


@pytest.mark.parametrize('shape', SHAPES)
def test_closed_loop_perturbed_stays_bounded(shape):
    r = run_nmpc(shape, 'tangent', plant='perturbed', urdf=URDF)
    assert r['not_solved'] == 0 and r['max_mm'] < 20.0


# ---- test 9: large start offsets, NMPC and LTV-MPC (recorded, not judged) ----------------------
@pytest.mark.parametrize('dp,_unused,dpsi', OFFSETS[2:])
def test_large_offsets_recover(dp, _unused, dpsi):
    off = (dp / np.sqrt(2), dp / np.sqrt(2), dpsi)
    for run, kw in ((run_nmpc, {}), (run_lmpc, dict(Q=Q_BRYSON))):
        r = run('figure8', 'tangent', plant='nominal', urdf=URDF, start_offset=off, **kw)
        assert r['not_solved'] == 0
        e_end = np.hypot(*(r['log'][-1, 1:3] - r['log'][-1, 7:9]))
        assert e_end < 0.01                                   # back on the path by the end


# ---- test 10: timing --------------------------------------------------------------------------
def test_timing():
    for shape in SHAPES:
        r = run_nmpc(shape, 'tangent', plant='nominal', urdf=URDF)
        assert r['step_ms'][1] < 40.0
