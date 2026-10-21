"""Tests of the nonlinear MPC (NMPC plan, "Verification", tests 3-10; tests 1-2 are in
mecanum_mpc_common and mecanum_lmpc_v2)."""
import os

import numpy as np
import pytest
from mecanum_lmpc_v2.lmpc import LinearMPCv2
from mecanum_lmpc_v2.offline_sim import run as run_lmpc
from mecanum_mpc_common import dynamics, motor, params
from mecanum_mpc_common.conventions import STATUS_SOLVED
from mecanum_mpc_common.reference import SHAPES, Reference
from mecanum_mpc_common.wheel_speed import wheel_speeds

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


def horizon(p, shape='figure8', heading='tangent', t=12.0):
    ref = Reference(p, shape, None, 0.1, heading)
    ref.anchor((0.0, 0.0, 0.0))
    return ref.horizon(t, T, N)


def offset_state(x_r0, dp, dpsi):
    return x_r0 + np.array([dp / np.sqrt(2), dp / np.sqrt(2), dpsi, 0, 0, 0])


# ---- test 3: on the reference -----------------------------------------------------------------
@pytest.mark.parametrize('shape,heading', [('figure8', 'tangent'), ('circle', 'fixed'),
                                           ('square', 'tangent')])
def test_on_reference(p, nmpc, shape, heading):
    x_r, u_r = horizon(p, shape, heading, 6.3)
    u, info = nmpc.step(x_r[0], x_r, u_r, u_r[0])
    assert info['status'] == STATUS_SOLVED and info['iterations'] <= 2
    # the reference is consistent with the model up to the D3 residuals c_j, so u = u_r up to
    # their effect: compare with the LMPC, which sees the same residuals
    u_l, _ = LinearMPCv2(p, T, N).step(x_r[0], x_r, u_r, u_r[0])
    assert np.abs(u - u_r[0]).max() < 1e-4 and np.abs(u - u_l).max() < 1e-6


# ---- test 4: the first iteration is the LMPC step ----------------------------------------------
@pytest.mark.parametrize('dp,_unused,dpsi', OFFSETS)
def test_first_iteration_equals_lmpc(p, nmpc, dp, _unused, dpsi):
    x_r, u_r = horizon(p)
    x = offset_state(x_r[0], dp, dpsi)
    _, info = nmpc.step(x, x_r, u_r, u_r[0])
    u_l, info_l = LinearMPCv2(p, T, N).step(x, x_r, u_r, u_r[0])
    assert info['alpha_first'] == 1.0
    assert np.abs(info['u_first'] - u_l).max() < 1e-6


# ---- test 5: convergence ----------------------------------------------------------------------
@pytest.mark.parametrize('dp,_unused,dpsi', OFFSETS[:4])
def test_converged_solution(p, dp, _unused, dpsi):
    nmpc = NonlinearMPC(p, T, N, max_iter=200, time_budget=0.0)      # without the N3 caps
    x_r, u_r = horizon(p)
    x = offset_state(x_r[0], dp, dpsi)
    u, info = nmpc.step(x, x_r, u_r, u_r[0])
    assert info['acados_status'] == ACADOS_SUCCESS and info['kkt'] <= 1e-6
    # the converged states obey the model: re-simulating the plan reproduces the prediction
    sv = nmpc.solver
    z = x.copy()
    for j in range(N):
        z = dynamics.rk4_step(z, sv.get(j, 'u'), T, p)
        assert np.abs(z - info['predicted'][j]).max() < 1e-8


# ---- test 6: local minima ---------------------------------------------------------------------
@pytest.mark.parametrize('dp,_unused,dpsi', OFFSETS[1:4])
def test_same_solution_from_other_starts(p, dp, _unused, dpsi):
    m = NonlinearMPC(p, T, N, max_iter=200, time_budget=0.0)
    x_r, u_r = horizon(p)
    x = offset_state(x_r[0], dp, dpsi)
    u_ref, info = m.step(x, x_r, u_r, u_r[0])
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
                sv.set(j, 'x', states[j] + np.r_[rng.normal(scale=[0.02, 0.02, 0.05, 0.02, 0.02,
                                                                   0.05]), np.zeros(4)])
        sv.set(N, 'x', states[N])
        assert sv.solve() == ACADOS_SUCCESS
        assert np.abs(sv.get(0, 'u') - u_ref).max() < 1e-4


# ---- test 7: soft wheel-speed limit, both controllers -----------------------------------------
def test_wheel_speed_limit(p):
    x_r, u_r = horizon(p)
    x = offset_state(x_r[0], 0.1, 0.17)
    w_max = motor.OMEGA_MAX
    # without the limit the plans exceed omega_max (the reason for N5) ...
    _, free = NonlinearMPC(p, T, N, wheel_speed_limit=False, max_iter=200,
                           time_budget=0.0).step(x, x_r, u_r, u_r[0])
    assert np.abs(wheel_speeds(free['predicted'], p)).max() > 1.5 * w_max
    # ... with it the NMPC's plan respects it exactly (as far as the slack allows) ...
    _, lim = NonlinearMPC(p, T, N, max_iter=200, time_budget=0.0).step(x, x_r, u_r, u_r[0])
    assert np.abs(wheel_speeds(lim['predicted'], p)).max() <= w_max + lim['slack_max'] + 1e-6
    # ... and the LMPC's plan respects its linearised version
    _, lin = LinearMPCv2(p, T, N).step(x, x_r, u_r, u_r[0])
    assert np.abs(wheel_speeds(lin['predicted'], p)).max() < 1.5 * w_max


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


# ---- test 9: large start offsets, NMPC and LMPC (recorded, not judged) -------------------------
@pytest.mark.parametrize('dp,_unused,dpsi', OFFSETS[2:])
def test_large_offsets_recover(dp, _unused, dpsi):
    off = (dp / np.sqrt(2), dp / np.sqrt(2), dpsi)
    for run in (run_nmpc, run_lmpc):
        r = run('figure8', 'tangent', plant='nominal', urdf=URDF, start_offset=off)
        assert r['not_solved'] == 0
        e_end = np.hypot(*(r['log'][-1, 1:3] - r['log'][-1, 7:9]))
        assert e_end < 0.01                                   # back on the path by the end


# ---- test 10: timing --------------------------------------------------------------------------
def test_timing():
    for shape in SHAPES:
        r = run_nmpc(shape, 'tangent', plant='nominal', urdf=URDF)
        assert r['step_ms'][1] < 40.0
