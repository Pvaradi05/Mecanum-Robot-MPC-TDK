"""Linear time-varying MPC after Kuehne, Gomes da Silva Jr., Lages (2005), condensed QP form
(eq. 15-20), on the RK4-discretised Chaplygin model (plan: "Linear MPC after Kuehne et al.",
"Constraints" and "One control step, end to end").

Decisions: D1 constant Q, terminal P = p Q; D2 exact RK4 Jacobians; D3 affine term c_k with
step-averaged u_r; D4 T = 50 ms, N = 10 (20 until 2026-10-21, when a horizon sweep showed no
difference in tracking); D5 pure model; D6 hard torque-rate limit, no rate penalty; D8 quadprog.
D7 (no wheel-speed constraint) is replaced by NMPC plan decision N5: a soft wheel-speed limit in
both controllers, here linearised along the reference.

Per step:
    x~_0 = x_meas - x_r,0 (heading unwrapped onto the reference)
    A_j, B_j = RK4 Jacobians at (x_r,j, u_r,j),  c_j = F(x_r,j, u_r,j) - x_r,j+1
    e_0 = x~_0,  e_j = A_{j-1} e_{j-1} + c_{j-1}                 free response
    Bbar_{j,i} = A_{j-1} Bbar_{j-1,i} (i < j-1), B_{j-1} (i = j-1)  prediction matrix
    H = 2 (Bbar^T Qbar Bbar + Rbar),  f = 2 Bbar^T Qbar e
    min 1/2 ubar^T H ubar + f^T ubar + slack_lin 1^T s + slack_quad s^T s
        s.t.  u_min - u_r,j <= u~_j <= u_max - u_r,j
              -du_max <= D ubar + delta <= du_max
              |w(x_r,j) + G_j (e_j + Bbar_j ubar)| <= w_max + s_j,  s >= 0   (j = 1..N, G = dw/dx)
    u(k) = clip(u_r,0 + u~*_0, u_min, u_max)
"""
import time

import numpy as np
import quadprog
from mecanum_mpc_common import dynamics, motor
from mecanum_mpc_common.conventions import STATUS_FALLBACK, STATUS_SOLVED, STATUS_STOPPED
from mecanum_mpc_common.params import RobotParams
from mecanum_mpc_common.wheel_speed import (SLACK_LIN, SLACK_QUAD, wheel_speed_jacobian,
                                            wheel_speeds)

NX, NU = 6, 4
__all__ = ['LinearMPCv2', 'STATUS_SOLVED', 'STATUS_FALLBACK', 'STATUS_STOPPED', 'wrap']


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class LinearMPCv2:
    def __init__(self, p: RobotParams, T=0.05, N=10, Q=(40000, 40000, 3283, 2500, 2500, 100),
                 R=(400, 400, 400, 400), P_scale=5.0, u_max=motor.U_MAX, du_max=motor.DU_MAX,
                 max_failures=3, omega_max=motor.OMEGA_MAX, wheel_speed_limit=True,
                 slack_lin=SLACK_LIN, slack_quad=SLACK_QUAD):
        self.p, self.T, self.N = p, T, N
        self.u_max, self.du_max, self.max_failures = u_max, du_max, max_failures
        self.omega_max, self.wheel_speed_limit = omega_max, wheel_speed_limit
        self.slack_lin, self.slack_quad = slack_lin, slack_quad
        Q, R = np.diag(Q).astype(float), np.diag(R).astype(float)
        # constant pieces (built once): Qbar, Rbar, D, C
        self.Qbar = np.zeros((NX * N, NX * N))
        for j in range(N):
            self.Qbar[NX * j:NX * (j + 1), NX * j:NX * (j + 1)] = Q if j < N - 1 else P_scale * Q
        self.Rbar = np.kron(np.eye(N), R)
        self.D = np.eye(NU * N) - np.eye(NU * N, k=-NU)
        I = np.eye(NU * N)
        self.C = np.hstack([I, -I, self.D.T, -self.D.T])      # quadprog: C^T ubar >= b
        self.prev_plan = None        # absolute torques of the last solved plan (N x 4)
        self.failures = 0
        self.stopped = False

    def step(self, x_meas, x_r, u_r, u_applied):
        """x_meas (6,), x_r (N+1 x 6), u_r (N x 4), u_applied (4,) -> (u (4,), info)."""
        t0 = time.perf_counter()
        N, T, p = self.N, self.T, self.p
        if self.stopped:
            return np.zeros(NU), dict(status=STATUS_STOPPED, qp_ms=0.0, step_ms=0.0,
                                      predicted=None, x_tilde0=np.zeros(NX), slack_max=0.0)

        # error at the start of the horizon (measured heading unwrapped onto the reference)
        x = np.array(x_meas, dtype=float)
        x[2] = x_r[0, 2] + wrap(x[2] - x_r[0, 2])
        x_t0 = x - x_r[0]

        # linearise along the reference; free response e and prediction matrix Bbar
        A, B, c = [], [], []
        for j in range(N):
            Ad, Bd = dynamics.rk4_jacobians(x_r[j], u_r[j], T, p)
            A.append(Ad)
            B.append(Bd)
            c.append(dynamics.rk4_step(x_r[j], u_r[j], T, p) - x_r[j + 1])
        e = np.zeros(NX * N)
        Bbar = np.zeros((NX * N, NU * N))
        e_prev = x_t0
        for j in range(1, N + 1):
            rows = slice(NX * (j - 1), NX * j)
            e_prev = A[j - 1] @ e_prev + c[j - 1]
            e[rows] = e_prev
            if j > 1:
                Bbar[rows, :NU * (j - 1)] = A[j - 1] @ Bbar[NX * (j - 2):NX * (j - 1), :NU * (j - 1)]
            Bbar[rows, NU * (j - 1):NU * j] = B[j - 1]

        # QP data
        BtQ = Bbar.T @ self.Qbar
        H = 2.0 * (BtQ @ Bbar + self.Rbar)
        H = (H + H.T) / 2
        f = 2.0 * BtQ @ e
        ur_bar = u_r.ravel()
        delta = np.r_[u_r[0] - u_applied, (u_r[1:] - u_r[:-1]).ravel()]
        b = np.r_[-self.u_max - ur_bar, ur_bar - self.u_max,
                  -self.du_max - delta, -self.du_max + delta]
        qp_H, qp_a, qp_C, qp_b = H, -f, self.C, b
        nu = NU * N
        if self.wheel_speed_limit:
            # soft wheel-speed rows linearised along the reference, one slack per wheel and step:
            # variables [u~ (4N), s (4N)]; rows  -G Bbar u~ + s >= w_lin - w_max,
            # G Bbar u~ + s >= -w_max - w_lin,  s >= 0;  w_lin = w(x_r) + G e
            ns = 4 * N
            G = np.zeros((ns, NX * N))
            for j in range(N):
                G[4 * j:4 * (j + 1), NX * j:NX * (j + 1)] = wheel_speed_jacobian(x_r[j + 1], p)
            w_lin = wheel_speeds(x_r[1:], p).ravel() + G @ e
            GB = G @ Bbar
            nc = self.C.shape[1]
            qp_H = np.zeros((nu + ns, nu + ns))
            qp_H[:nu, :nu] = H
            qp_H[nu:, nu:] = 2.0 * self.slack_quad * np.eye(ns)
            qp_a = np.r_[-f, -self.slack_lin * np.ones(ns)]
            qp_C = np.zeros((nu + ns, nc + 3 * ns))
            qp_C[:nu, :nc] = self.C
            qp_C[:nu, nc:nc + ns] = -GB.T
            qp_C[:nu, nc + ns:nc + 2 * ns] = GB.T
            qp_C[nu:, nc:nc + ns] = np.eye(ns)
            qp_C[nu:, nc + ns:nc + 2 * ns] = np.eye(ns)
            qp_C[nu:, nc + 2 * ns:] = np.eye(ns)
            qp_b = np.r_[b, w_lin - self.omega_max, -self.omega_max - w_lin, np.zeros(ns)]

        t_qp = time.perf_counter()
        try:
            sol_full = quadprog.solve_qp(qp_H, qp_a, qp_C, qp_b, 0)[0]
            sol = sol_full[:nu]
            ok = True
        except ValueError:
            ok = False
        qp_ms = (time.perf_counter() - t_qp) * 1e3

        if ok:
            self.failures = 0
            u_plan = u_r + sol.reshape(N, NU)
            self.prev_plan = u_plan
            u = u_plan[0]
            predicted = x_r[1:] + (e + Bbar @ sol).reshape(N, NX)
            status = STATUS_SOLVED
        else:
            self.failures += 1
            predicted = None
            if self.failures >= self.max_failures or self.prev_plan is None:
                self.stopped = self.failures >= self.max_failures
                u = np.zeros(NU)
                status = STATUS_STOPPED if self.stopped else STATUS_FALLBACK
            else:
                u = self.prev_plan[min(self.failures, N - 1)]   # previous plan, shifted
                status = STATUS_FALLBACK
        u = np.clip(u, -self.u_max, self.u_max)
        slack_max = float(sol_full[nu:].max()) if ok and self.wheel_speed_limit else 0.0
        return u, dict(status=status, qp_ms=qp_ms, step_ms=(time.perf_counter() - t0) * 1e3,
                       predicted=predicted, x_tilde0=x_t0, H=H, f=f, e=e, Bbar=Bbar, b=b,
                       sol=sol if ok else None, slack_max=slack_max)
