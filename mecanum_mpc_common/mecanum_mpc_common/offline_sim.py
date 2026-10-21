"""Closed loop of a controller against the model, without ROS or Gazebo (LMPC plan test 7, NMPC
plan tests 8-9). A controller package passes make_controller(p, T, N) and gets the same plant,
reference and metrics as every other controller.

Plant: the same Chaplygin model, RK4 with 10 substeps per control period. The perturbed plant
(only for this test) has its own parameters and a Coulomb friction torque on each wheel.
"""
import argparse
import dataclasses
import os

import numpy as np

from . import dynamics, params
from .conventions import FAILED, STATUS_NOT_CONVERGED
from .reference import SHAPES, Reference
from .wheel_speed import wheel_speeds


def default_urdf():
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('tb3_sim_bringup')
    except Exception:                      # not sourced: the source tree next to this package
        share = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..',
                             'tb3_sim_bringup')
    return os.path.join(share, 'models', 'own_mecanum', 'robot.urdf')


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def plant_step(x, u, T, pp, friction=0.0, substeps=10):
    """Plant: RK4 substeps with the wheel torques reduced by Coulomb friction (smooth sign)."""
    h = T / substeps
    for _ in range(substeps):
        def fx(z):
            c, s = np.cos(z[2]), np.sin(z[2])
            nu = np.array([c * z[3] + s * z[4], -s * z[3] + c * z[4], z[5]])
            return dynamics.f(z, u - friction * np.tanh(pp.W @ nu / 0.05), pp)
        k1 = fx(x)
        k2 = fx(x + h / 2 * k1)
        k3 = fx(x + h / 2 * k2)
        k4 = fx(x + h * k3)
        x = x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return x


def run(make_controller, shape='circle', heading='fixed', speed=0.1, plant='nominal',
        start_offset=(0, 0, 0), urdf=None, T=0.05, N=10, mass_scale=1.2, J1_scale=2.0,
        friction=0.003, extra=()):
    """One run; extra = info keys to collect per step (returned as r['extra'][key])."""
    p = params.from_urdf(urdf or default_urdf())
    pp, fr = p, 0.0
    if plant == 'perturbed':
        pp = dataclasses.replace(p, m_s=mass_scale * p.m_s, J_C=mass_scale * p.J_C,
                                 J1=J1_scale * p.J1)
        fr = friction
    ref = Reference(p, shape, None, speed, heading)
    ref.anchor((0.0, 0.0, 0.0), start_offset)
    mpc = make_controller(p, T, N)
    x = np.zeros(6)
    u_applied = np.zeros(4)
    rows, extras = [], {k: [] for k in extra}
    for k in range(int(round((ref.T_end + 2.0) / T))):
        t = k * T
        x_r, u_r = ref.horizon(t, T, N)
        u, info = mpc.step(x, x_r, u_r, u_applied)
        rows.append(np.r_[t, x, x_r[0], u, info['qp_ms'], info['step_ms'], info['status']])
        for key in extra:
            extras[key].append(info[key])
        u_applied = u
        x = plant_step(x, u, T, pp, fr)
    log = np.array(rows)
    on_path = log[:, 0] <= ref.T_end
    e = np.hypot(log[:, 1] - log[:, 7], log[:, 2] - log[:, 8])[on_path]
    dpsi = np.abs(wrap(log[:, 3] - log[:, 9]))[on_path]
    return dict(log=log, rms_mm=1e3 * np.sqrt(np.mean(e ** 2)), max_mm=1e3 * e.max(),
                psi_max_deg=np.degrees(dpsi.max()), torque_max=np.abs(log[:, 13:17]).max(),
                du_max=np.abs(np.diff(log[:, 13:17], axis=0)).max(),
                wheel_max=np.abs(wheel_speeds(log[:, 1:7], p)).max(),
                qp_ms=np.percentile(log[:, 17], [50, 95]),
                step_ms=np.percentile(log[:, 18], [50, 95]),
                not_solved=int(np.isin(log[:, 19], FAILED).sum()),
                not_converged=int((log[:, 19] == STATUS_NOT_CONVERGED).sum()),
                extra={k: np.array(v) for k, v in extras.items()})


def main(make_controller, description):
    ap = argparse.ArgumentParser(description=description,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--shape', default='all', choices=('all',) + SHAPES)
    ap.add_argument('--heading', default='both', choices=('both', 'fixed', 'tangent'))
    ap.add_argument('--speed', type=float, default=0.1)
    ap.add_argument('--plant', default='nominal', choices=('nominal', 'perturbed'))
    ap.add_argument('--start-offset', type=float, nargs=3, default=[0.0, 0.0, 0.0],
                    metavar=('X', 'Y', 'PSI'))
    a = ap.parse_args()
    shapes = SHAPES if a.shape == 'all' else (a.shape,)
    headings = ('fixed', 'tangent') if a.heading == 'both' else (a.heading,)
    print(f'{"path":8} {"heading":8} | track RMS / max   | psi max | torque max | du max  | '
          f'wheel max | QP ms p50/p95 | step ms p50/p95 | failed | not conv.')
    for s in shapes:
        for h in headings:
            r = run(make_controller, s, h, a.speed, a.plant, tuple(a.start_offset))
            print(f'{s:8} {h:8} | {r["rms_mm"]:6.3f} / {r["max_mm"]:6.3f} mm | '
                  f'{r["psi_max_deg"]:5.2f} deg | {r["torque_max"]:.4f} N m | {r["du_max"]:.4f} | '
                  f'{r["wheel_max"]:5.2f} r/s | {r["qp_ms"][0]:5.2f} / {r["qp_ms"][1]:5.2f} | '
                  f'{r["step_ms"][0]:6.2f} / {r["step_ms"][1]:6.2f} | {r["not_solved"]:6d} | '
                  f'{r["not_converged"]}')
