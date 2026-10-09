"""Per-step evaluation statistics of the MPC, published on /mpc/status for the live dashboard.

/mpc/status is a std_msgs/Float64MultiArray. layout.dim[0].label holds the comma-separated field
names, in the same order as `data` (see FIELDS). Units: m, rad, m/s, rad/s, N, N m, ms.
"""
from collections import deque
import math

import numpy as np

from .ltv_mpc import wrap

FIELDS = [
    't', 'lap',
    # tracking
    'X', 'Y', 'psi', 'X_ref', 'Y_ref', 'psi_ref',
    'err_pos', 'err_along', 'err_cross', 'err_psi', 'err_speed',
    # forces: feedforward (model, from the reference) and feedback (MPC correction)
    'F_xi_ff', 'F_eta_ff', 'tau_ff', 'F_xi_fb', 'F_eta_fb', 'tau_fb',
    # size of the non-holonomic (Coriolis) terms ms*v*r, ms*u*r of the dynamics
    'coriolis',
    # wheels: applied torques and measured speeds, + use of the limits now and over the plan
    'M1', 'M2', 'M3', 'M4', 'w1', 'w2', 'w3', 'w4',
    'torque_util', 'speed_util', 'torque_active', 'speed_active',
    # model quality: measured state vs what the MPC predicted 1 step / 1 horizon earlier
    'pred1_pos', 'pred1_psi', 'pred1_speed', 'predN_pos', 'predN_psi',
    # wheel slip: RMS of (real wheel speed - speed the rolling constraint implies) * R [m/s]
    'slip',
    # solver and timing
    'qp_ms', 'step_ms', 'iters', 'cost', 'status', 'period_ms',
]
STATUS_SOLVED, STATUS_INACCURATE, STATUS_FALLBACK = 0.0, 1.0, 2.0


def along_cross(x, xr):
    """Position error split along / across the reference direction of travel."""
    e = np.array([x[0] - xr[0], x[1] - xr[1]])
    c, s = math.cos(xr[2]), math.sin(xr[2])
    vw = np.array([c * xr[3] - s * xr[4], s * xr[3] + c * xr[4]])      # world velocity of ref
    n = np.linalg.norm(vw)
    tdir = vw / n if n > 1e-4 else np.array([c, s])                    # standing: use heading
    return float(e @ tdir), float(-e[0] * tdir[1] + e[1] * tdir[0])   # cross > 0: left of path


class PredictionTracker:
    """Remembers the MPC's predicted trajectories and compares them with what really happened."""

    def __init__(self, N):
        self.N = N
        self.history = deque(maxlen=N + 1)    # predicted trajectories, oldest first

    def update(self, x_meas, predicted):
        out = dict(pred1_pos=np.nan, pred1_psi=np.nan, pred1_speed=np.nan,
                   predN_pos=np.nan, predN_psi=np.nan)
        if self.history:
            p1 = self.history[-1][1]                         # predicted for now, 1 step ago
            out['pred1_pos'] = math.hypot(*(x_meas[:2] - p1[:2]))
            out['pred1_psi'] = abs(wrap(x_meas[2] - p1[2]))
            out['pred1_speed'] = float(np.linalg.norm(x_meas[3:5] - p1[3:5]))
        if len(self.history) == self.N + 1:
            pN = self.history[0][self.N]                     # predicted for now, N steps ago
            out['predN_pos'] = math.hypot(*(x_meas[:2] - pN[:2]))
            out['predN_psi'] = abs(wrap(x_meas[2] - pN[2]))
        self.history.append(np.array(predicted))
        return out


def step_stats(p, mpc, x, xr0, w_ref0, info, wheel_speeds):
    """Everything about one control step except time/lap/prediction/timing."""
    along, cross = along_cross(x, xr0)
    w = info['plan_w'][0]
    plan_M = info['plan_w'] @ mpc.Gp.T                      # (N, 4) planned torques
    plan_om = info['predicted'][1:, 3:] @ p.wheel_jacobian().T
    status = (STATUS_FALLBACK if not info['ok'] else
              STATUS_INACCURATE if info['status'] != 'solved' else STATUS_SOLVED)
    return dict(
        X=x[0], Y=x[1], psi=x[2], X_ref=xr0[0], Y_ref=xr0[1], psi_ref=xr0[2],
        err_pos=math.hypot(along, cross), err_along=along, err_cross=cross,
        err_psi=wrap(x[2] - xr0[2]), err_speed=float(np.linalg.norm(x[3:5] - xr0[3:5])),
        F_xi_ff=w_ref0[0], F_eta_ff=w_ref0[1], tau_ff=w_ref0[2],
        F_xi_fb=w[0] - w_ref0[0], F_eta_fb=w[1] - w_ref0[1], tau_fb=w[2] - w_ref0[2],
        coriolis=p.ms * math.hypot(x[4] * x[5], x[3] * x[5]),
        M1=info['M'][0], M2=info['M'][1], M3=info['M'][2], M4=info['M'][3],
        w1=wheel_speeds[0], w2=wheel_speeds[1], w3=wheel_speeds[2], w4=wheel_speeds[3],
        torque_util=float(np.abs(info['M']).max() / mpc.torque_max),
        speed_util=float(np.abs(wheel_speeds).max() / mpc.wheel_speed_max),
        torque_active=int((np.abs(plan_M) >= 0.999 * mpc.torque_max).any(axis=1).sum()),
        speed_active=int((np.abs(plan_om) >= 0.999 * mpc.wheel_speed_max).any(axis=1).sum()),
        qp_ms=info['solve_ms'], step_ms=info['total_ms'], iters=info['iters'],
        cost=info['cost'], status=status)
