"""Metrics from controller CSV logs (LMPC plan: evaluate.py).

    ros2 run mecanum_mpc_common evaluate ~/ros2_ws/mpc_logs/<controller>/*.csv [--skip 3]

Window: from --skip seconds to the end of the reference motion.
    tracking error     distance to where the reference is at that time
    cross-track error  distance to the reference path polyline (segments between the logged
                       reference points, not the nearest logged point)
    heading error      wrapped psi - psi_r
"""
import argparse
import os

import numpy as np

from .conventions import FAILED, STATUS_NOT_CONVERGED


def load(path):
    with open(path) as f:
        header = f.readline().strip().split(',')
    data = np.genfromtxt(path, delimiter=',', skip_header=1)
    return {h: data[:, i] for i, h in enumerate(header)}


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def dist_to_polyline(q, P):
    """Distance from each point of q (n x 2) to the polyline through P (m x 2)."""
    A, B = P[:-1], P[1:]
    AB = B - A
    L2 = np.maximum((AB ** 2).sum(1), 1e-18)
    out = np.empty(len(q))
    for i, pt in enumerate(q):
        s = np.clip(((pt - A) * AB).sum(1) / L2, 0.0, 1.0)
        out[i] = np.min(np.hypot(*(A + s[:, None] * AB - pt).T))
    return out


def metrics(d, skip=0.0):
    moving = np.abs(np.c_[d['X_dot_r'], d['Y_dot_r'], d['psi_dot_r']]).max(axis=1) > 1e-9
    t_end = d['t'][moving].max() if moving.any() else d['t'][-1]
    m = (d['t'] >= skip) & (d['t'] <= t_end)
    pos = np.c_[d['X'], d['Y']]
    ref = np.c_[d['X_r'], d['Y_r']]
    e = np.hypot(*(pos - ref).T)[m]
    ct = dist_to_polyline(pos[m], ref[d['t'] <= t_end])
    psi = np.degrees(np.abs(wrap(d['psi'] - d['psi_r'])))[m]
    M = np.c_[d['M1'], d['M2'], d['M3'], d['M4']][m]
    return dict(t_end=t_end, rms=1e3 * np.sqrt(np.mean(e ** 2)), max=1e3 * e.max(),
                ct_rms=1e3 * np.sqrt(np.mean(ct ** 2)), ct_max=1e3 * ct.max(),
                psi_rms=np.sqrt(np.mean(psi ** 2)), psi_max=psi.max(),
                torque_max=np.abs(M).max(), torque_rms=np.sqrt(np.mean(M ** 2)),
                qp_ms=np.percentile(d['qp_ms'][m], [50, 95]),
                step_ms=np.percentile(d['step_ms'][m], [50, 95]),
                not_solved=int(np.isin(d['status'][m], FAILED).sum()),
                not_converged=int((d['status'][m] == STATUS_NOT_CONVERGED).sum()))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('logs', nargs='+')
    ap.add_argument('--skip', type=float, default=0.0, help='ignore the first seconds')
    a = ap.parse_args()
    print(f'{"log":44} | track RMS/max mm | cross-track RMS/max mm | psi RMS/max deg | '
          f'torque max/RMS N m | QP ms p50/p95 | not solved')
    for path in a.logs:
        r = metrics(load(path), a.skip)
        print(f'{os.path.basename(path)[:44]:44} | {r["rms"]:6.2f} / {r["max"]:6.2f} | '
              f'{r["ct_rms"]:9.2f} / {r["ct_max"]:6.2f}    | {r["psi_rms"]:5.2f} / {r["psi_max"]:5.2f} | '
              f'{r["torque_max"]:.4f} / {r["torque_rms"]:.4f} | {r["qp_ms"][0]:5.2f} / {r["qp_ms"][1]:5.2f} | '
              f'{r["not_solved"]}')


if __name__ == '__main__':
    main()
