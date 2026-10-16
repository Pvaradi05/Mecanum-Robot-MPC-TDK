"""Closed-loop test of the LTV-MPC against the nonlinear model, without ROS or Gazebo.

    python3 -m mecanum_mpc.offline_sim --shape figure8 --heading tangent --offset 0.1 0 0.3
"""
import argparse
import os

import numpy as np

from . import model
from .ltv_mpc import LTVMPC, wrap
from .references import Reference

DEFAULT_URDF = os.path.join(os.path.dirname(__file__), '..', '..', 'tb3_sim_bringup', 'models',
                            'own_mecanum', 'robot.urdf')
SIZES = {'circle': 0.75, 'ellipse': 1.0, 'figure8': 1.0, 'square': 1.5}


def run(shape='circle', heading='fixed', speed=0.2, offset=(0.0, 0.0, 0.0), urdf=DEFAULT_URDF,
        dt=0.05, N=20, linearize_about='reference', plant_params=None, corner_radius=None,
        force_reference=True, average_force_reference=False, **mpc_kw):
    p = model.params_from_urdf(urdf)
    plant = plant_params or p
    ref = Reference(p, shape, SIZES[shape], speed=speed, heading=heading,
                    corner_radius=corner_radius)
    mpc = LTVMPC(p, dt=dt, N=N, linearize_about=linearize_about, **mpc_kw)
    x = np.zeros(6)
    x[:3] = offset
    steps = int((ref.duration + 2.0) / dt)
    log = []
    for i in range(steps):
        t = i * dt
        xr, wr = ref.sample(t + dt * np.arange(N + 1),
                            w_window=dt if average_force_reference else 0.0)
        if not force_reference:
            wr = np.zeros_like(wr)
        w, info = mpc.solve(x, xr, wr[:N])
        w_applied = p.torque_to_wrench() @ info['torques']   # what the wheels actually deliver
        log.append(np.r_[t, x, xr[0], info['torques'], info['solve_ms'], info['total_ms'],
                         info['status'] in ('solved', 'solved inaccurate')])
        x = model.rk4(x, w_applied, dt, plant, substeps=10)
    log = np.array(log)
    err = log[:, 1:3] - log[:, 7:9]
    pos_err = np.linalg.norm(err, axis=1)
    psi_err = np.abs(wrap(log[:, 3] - log[:, 9]))
    settle = log[:, 0] > 5.0                                 # after the initial offset is gone
    return dict(log=log, ref=ref,
                rms_mm=1e3 * np.sqrt(np.mean(pos_err[settle] ** 2)),
                max_mm=1e3 * pos_err[settle].max(),
                max_psi_deg=np.degrees(psi_err[settle].max()),
                max_torque=np.abs(log[:, 13:17]).max(),
                solve_ms=np.percentile(log[:, 17], [50, 95]),
                total_ms=np.percentile(log[:, 18], [50, 95]),
                failed=int((log[:, 19] == 0).sum()))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--shape', default='all', choices=['all', 'circle', 'ellipse', 'figure8', 'square'])
    ap.add_argument('--heading', default='both', choices=['both', 'fixed', 'tangent'])
    ap.add_argument('--speed', type=float, default=0.2)
    ap.add_argument('--offset', type=float, nargs=3, default=[0, 0, 0], metavar=('X', 'Y', 'PSI'))
    ap.add_argument('--linearize-about', default='reference', choices=['reference', 'prediction'])
    ap.add_argument('--plot', help='save a PNG of the trajectories here')
    a = ap.parse_args()
    shapes = list(SIZES) if a.shape == 'all' else [a.shape]
    headings = ['fixed', 'tangent'] if a.heading == 'both' else [a.heading]
    results = []
    print(f'{"shape":8} {"heading":8} | RMS err  max err  max psi err | max torque | QP ms p50/p95 | step ms p50/p95 | failed')
    for s in shapes:
        for h in headings:
            r = run(s, h, a.speed, tuple(a.offset), linearize_about=a.linearize_about)
            results.append((s, h, r))
            print(f'{s:8} {h:8} | {r["rms_mm"]:5.1f}mm  {r["max_mm"]:5.1f}mm  {r["max_psi_deg"]:6.2f}deg   | '
                  f'{r["max_torque"]:.3f} Nm  | {r["solve_ms"][0]:5.2f}/{r["solve_ms"][1]:5.2f}   | '
                  f'{r["total_ms"][0]:5.1f}/{r["total_ms"][1]:5.1f}     | {r["failed"]}')
    if a.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(len(headings), len(shapes), figsize=(4.5 * len(shapes), 4.2 * len(headings)),
                                squeeze=False)
        for i, (s, h, r) in enumerate(results):
            ax = axs[headings.index(h), shapes.index(s)]
            L = r['log']
            ax.plot(L[:, 7], L[:, 8], 'k--', lw=1, label='reference')
            ax.plot(L[:, 1], L[:, 2], lw=1.5, label='robot')
            ax.set_aspect('equal')
            ax.grid(True)
            ax.set_title(f'{s}, {h}: RMS {r["rms_mm"]:.1f} mm')
        axs[0, 0].legend()
        plt.tight_layout()
        plt.savefig(a.plot, dpi=80)


if __name__ == '__main__':
    main()
