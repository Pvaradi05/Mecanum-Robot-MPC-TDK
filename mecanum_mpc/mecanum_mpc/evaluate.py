"""Summarise and plot mpc_node CSV logs.

    ros2 run mecanum_mpc evaluate ~/ros2_ws/mpc_logs/*.csv --plot runs.png

Metrics are computed while the path is being driven (reference velocity != 0 or t <= end of
path), excluding the first `--skip` seconds. Tracking error = distance to where the reference is at
that time; cross-track error = distance to the nearest point of the reference path.
"""
import argparse
import os

import numpy as np


def load(path):
    with open(path) as f:
        header = f.readline().strip().split(',')
    data = np.genfromtxt(path, delimiter=',', skip_header=1, dtype=float,
                         usecols=range(len(header) - 1))   # last column (status) is text
    status = np.genfromtxt(path, delimiter=',', skip_header=1, dtype=str, usecols=len(header) - 1)
    d = {h: data[:, i] for i, h in enumerate(header[:-1])}
    d['ok'] = np.isin(status, ['solved', 'solved inaccurate'])
    return d


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def metrics(d, skip=0.0):
    moving = np.abs(np.c_[d['u_ref'], d['v_ref'], d['r_ref']]).max(axis=1) > 1e-9
    end = d['t'][moving].max() if moving.any() else d['t'][-1]
    m = (d['t'] >= skip) & (d['t'] <= end)
    e = np.hypot(d['X'] - d['X_ref'], d['Y'] - d['Y_ref'])[m]
    ref = np.c_[d['X_ref'], d['Y_ref']][moving]
    pts = np.c_[d['X'], d['Y']][m]
    ct = np.array([np.min(np.hypot(*(ref - q).T)) for q in pts])
    psi_e = np.degrees(np.abs(wrap(d['psi'] - d['psi_ref'])))[m]
    M = np.c_[d['M1'], d['M2'], d['M3'], d['M4']][m]
    return dict(duration=end, rms=1e3 * np.sqrt(np.mean(e**2)), max=1e3 * e.max(),
                ct_rms=1e3 * np.sqrt(np.mean(ct**2)), ct_max=1e3 * ct.max(),
                psi_rms=np.sqrt(np.mean(psi_e**2)), psi_max=psi_e.max(),
                torque_max=np.abs(M).max(), torque_rms=np.sqrt(np.mean(M**2)),
                dM_max=np.abs(np.diff(M, axis=0)).max(),
                qp_ms=np.percentile(d['qp_ms'][m], [50, 95]),
                step_ms=np.percentile(d['step_ms'][m], [50, 95]),
                failed=int((~d['ok'][m]).sum()), end_mask=m)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('logs', nargs='+')
    ap.add_argument('--skip', type=float, default=0.0, help='ignore the first seconds')
    ap.add_argument('--plot', help='save a PNG with path, errors and torques per log')
    a = ap.parse_args()
    runs = []
    print(f'{"log":42} | track RMS/max mm | cross-track RMS/max mm | psi RMS/max deg | '
          f'torque max/RMS Nm | QP ms p50/p95 | failed')
    for path in a.logs:
        d = load(path)
        r = metrics(d, a.skip)
        runs.append((path, d, r))
        print(f'{os.path.basename(path)[:42]:42} | {r["rms"]:6.1f} / {r["max"]:6.1f} | '
              f'{r["ct_rms"]:8.1f} / {r["ct_max"]:6.1f}     | {r["psi_rms"]:5.2f} / {r["psi_max"]:5.2f} | '
              f'{r["torque_max"]:.4f} / {r["torque_rms"]:.4f} | {r["qp_ms"][0]:5.2f} / {r["qp_ms"][1]:5.2f} | '
              f'{r["failed"]}')
    if a.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(len(runs), 3, figsize=(16, 4.2 * len(runs)), squeeze=False)
        for i, (path, d, r) in enumerate(runs):
            name = os.path.basename(path).rsplit('_', 2)[0]
            ax = axs[i, 0]
            ax.plot(d['X_ref'], d['Y_ref'], 'k--', lw=1, label='reference')
            ax.plot(d['X'], d['Y'], lw=1.5, label='robot')
            ax.set_aspect('equal')
            ax.grid(True)
            ax.set_title(f'{name}: RMS {r["rms"]:.1f} mm')
            ax.legend(fontsize=8)
            ax = axs[i, 1]
            ax.plot(d['t'], 1e3 * np.hypot(d['X'] - d['X_ref'], d['Y'] - d['Y_ref']), label='position [mm]')
            ax.plot(d['t'], np.degrees(wrap(d['psi'] - d['psi_ref'])) * 10, label='heading x10 [deg]')
            ax.axvline(r['duration'], color='gray', ls=':')
            ax.set_xlabel('t [s]')
            ax.grid(True)
            ax.legend(fontsize=8)
            ax.set_title('tracking error')
            ax = axs[i, 2]
            for k in range(4):
                ax.plot(d['t'], d[f'M{k + 1}'], lw=0.8, label=f'M{k + 1}')
            ax.set_xlabel('t [s]')
            ax.set_ylabel('wheel torque [N m]')
            ax.grid(True)
            ax.legend(fontsize=8, ncol=4)
            ax.set_title('wheel torques (FL, FR, RL, RR)')
        plt.tight_layout()
        plt.savefig(a.plot, dpi=75)
        print(f'plot: {a.plot}')


if __name__ == '__main__':
    main()
