"""Linear time-varying MPC for the mecanum robot (model.py), solved as one sparse QP per step.

At every control step the nonlinear dynamics are linearised along a trajectory (x_bar, w_bar):
by default the reference itself (classic LTV-MPC). With linearize_about='prediction', the
previous solution shifted by one step is used instead, which is one real-time iteration of an
SQP-NMPC.

    delta_x_{k+1} = Ad_k delta_x_k + Bd_k delta_w_k + c_k,
    c_k = f_d(x_bar_k, w_bar_k) - x_bar_{k+1}     (how inconsistent the linearisation point is)

Decision variables z = [delta_x_0 .. delta_x_N, delta_w_0 .. delta_w_{N-1}]; x = x_bar + delta_x.

    min  sum_k |x_k - x_ref_k|^2_Q + |w_k - w_ref_k|^2_R + |w_k - w_{k-1}|^2_Rd + |x_N - x_ref_N|^2_Qf
    s.t. linearised dynamics, x_0 = measured state
         |wheel torque_i| <= torque_max  (torques = min-norm allocation of w, linear in w)
         |wheel speed_i|  <= wheel_speed_max  (linear in u, v, r; k = 1..N)
"""
import time

import numpy as np
import osqp
from scipy import sparse

from . import model

NX, NU = 6, 3


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class LTVMPC:
    def __init__(self, p: model.Params, dt=0.05, N=20,
                 Q=(1000, 1000, 250, 5, 5, 1), R=(0.01, 0.01, 1.0), Rd=(0.1, 0.1, 10.0), Qf_scale=5.0,
                 torque_max=0.6, wheel_speed_max=30.0, linearize_about='reference'):
        if linearize_about not in ('reference', 'prediction'):
            raise ValueError("linearize_about must be 'reference' or 'prediction'")
        self.p, self.dt, self.N, self.lin = p, dt, N, linearize_about
        self.Q, self.R, self.Rd = np.diag(Q), np.diag(R), np.diag(Rd)
        self.Qf = Qf_scale * self.Q
        self.torque_max, self.wheel_speed_max = torque_max, wheel_speed_max
        self.Gp = p.wrench_to_torque()                       # 4x3
        self.Wv = np.zeros((4, NX))
        self.Wv[:, 3:] = p.wheel_jacobian()                  # wheel speeds from the state
        self.nz = NX * (N + 1) + NU * N
        self.prev_dw = np.zeros(NU)                          # last applied w - w_bar (for Rd)
        self.prev_sol = None                                 # (x (N+1,6), w (N,3)) for warm start
        self._build()

    # ---- QP structure (fixed sparsity, values updated every step) -----------------------------
    def _ix(self, k):
        return slice(NX * k, NX * (k + 1))

    def _iu(self, k):
        o = NX * (self.N + 1)
        return slice(o + NU * k, o + NU * (k + 1))

    def _build(self):
        N = self.N
        # cost Hessian
        P = np.zeros((self.nz, self.nz))
        for k in range(N):
            P[self._ix(k), self._ix(k)] = self.Q
            P[self._iu(k), self._iu(k)] += self.R + self.Rd
            if k > 0:
                P[self._iu(k - 1), self._iu(k - 1)] += self.Rd
                P[self._iu(k), self._iu(k - 1)] -= self.Rd
                P[self._iu(k - 1), self._iu(k)] -= self.Rd
        P[self._ix(N), self._ix(N)] = self.Qf
        self.P = sparse.triu(sparse.csc_matrix(P)).tocsc()

        # constraint rows: dynamics (NX*(N+1)), torques (4N), wheel speeds (4N)
        self.n_dyn, self.n_tq = NX * (N + 1), 4 * N
        self.nc = self.n_dyn + 2 * self.n_tq
        mask = np.zeros((self.nc, self.nz), dtype=bool)
        mask[0:NX, self._ix(0)] = np.eye(NX, dtype=bool)
        for k in range(N):
            r = slice(NX * (k + 1), NX * (k + 2))
            mask[r, self._ix(k)] = True                      # Ad_k (dense block)
            mask[r, self._iu(k)] = True                      # Bd_k
            mask[r, self._ix(k + 1)] |= np.eye(NX, dtype=bool)
            mask[self.n_dyn + 4 * k: self.n_dyn + 4 * (k + 1), self._iu(k)] = True
            rs = slice(self.n_dyn + self.n_tq + 4 * k, self.n_dyn + self.n_tq + 4 * (k + 1))
            mask[rs, self._ix(k + 1)] = self.Wv != 0
        self.rows, self.cols = np.nonzero(mask)
        A = sparse.csc_matrix((np.arange(1, len(self.rows) + 1, dtype=float),
                               (self.rows, self.cols)), shape=mask.shape)
        self.perm = A.data.astype(int) - 1                   # CSC data order -> row-major order
        self.A = A
        self.solver = None

    # ---- one control step ----------------------------------------------------------------------
    def solve(self, x_meas, x_ref, w_ref):
        """x_meas (6,), x_ref (N+1, 6), w_ref (N, 3). Returns (w0, info dict)."""
        t0 = time.perf_counter()
        N, p, dt = self.N, self.p, self.dt

        # linearisation point
        if self.lin == 'prediction' and self.prev_sol is not None:
            xs, ws = self.prev_sol
            x_bar = np.vstack([xs[1:], xs[-1:]])
            w_bar = np.vstack([ws[1:], ws[-1:]])
            x_bar[:, 2] = x_ref[:, 2] + wrap(x_bar[:, 2] - x_ref[:, 2])
        else:
            x_bar, w_bar = x_ref.copy(), w_ref.copy()

        Ad, Bd, c = [], [], []
        for k in range(N):
            a, b = model.discretize(x_bar[k], p, dt)
            Ad.append(a)
            Bd.append(b)
            c.append(model.rk4(x_bar[k], w_bar[k], dt, p) - x_bar[k + 1])

        # constraint matrix values
        Ad_full = np.zeros((self.nc, self.nz))
        Ad_full[0:NX, self._ix(0)] = np.eye(NX)
        for k in range(N):
            r = slice(NX * (k + 1), NX * (k + 2))
            Ad_full[r, self._ix(k)] = Ad[k]
            Ad_full[r, self._iu(k)] = Bd[k]
            Ad_full[r, self._ix(k + 1)] -= np.eye(NX)
            Ad_full[self.n_dyn + 4 * k: self.n_dyn + 4 * (k + 1), self._iu(k)] = self.Gp
            rs = slice(self.n_dyn + self.n_tq + 4 * k, self.n_dyn + self.n_tq + 4 * (k + 1))
            Ad_full[rs, self._ix(k + 1)] = self.Wv
        Ax = Ad_full[self.rows, self.cols][self.perm]

        # bounds
        e0 = x_meas - x_bar[0]
        e0[2] = wrap(e0[2])
        lo, hi = np.empty(self.nc), np.empty(self.nc)
        lo[0:NX] = hi[0:NX] = e0
        for k in range(N):
            lo[NX * (k + 1): NX * (k + 2)] = hi[NX * (k + 1): NX * (k + 2)] = -c[k]
        M_bar = w_bar @ self.Gp.T                            # (N, 4)
        lo[self.n_dyn: self.n_dyn + self.n_tq] = (-self.torque_max - M_bar).ravel()
        hi[self.n_dyn: self.n_dyn + self.n_tq] = (self.torque_max - M_bar).ravel()
        om_bar = x_bar[1:] @ self.Wv.T                       # (N, 4)
        lo[self.n_dyn + self.n_tq:] = (-self.wheel_speed_max - om_bar).ravel()
        hi[self.n_dyn + self.n_tq:] = (self.wheel_speed_max - om_bar).ravel()

        # linear cost: penalise x - x_ref = (x_bar - x_ref) + delta_x, same for w
        q = np.zeros(self.nz)
        for k in range(N + 1):
            d = x_bar[k] - x_ref[k]
            d[2] = wrap(d[2])
            q[self._ix(k)] = (self.Qf if k == N else self.Q) @ d
        for k in range(N):
            q[self._iu(k)] += self.R @ (w_bar[k] - w_ref[k])
        # rate penalty against the last applied input: |w_0 - w_applied|^2_Rd
        w_prev = w_bar[0] + self.prev_dw if self.prev_sol is None else self.prev_sol[1][0]
        q[self._iu(0)] += self.Rd @ (w_bar[0] - w_prev)
        for k in range(1, N):                                # w_bar changes along the horizon
            dwb = w_bar[k] - w_bar[k - 1]
            q[self._iu(k)] += self.Rd @ dwb
            q[self._iu(k - 1)] -= self.Rd @ dwb

        if self.solver is None:
            self.A.data = Ax
            self.solver = osqp.OSQP()
            self.solver.setup(self.P, q, self.A, lo, hi, verbose=False, warm_starting=True,
                              eps_abs=1e-4, eps_rel=1e-4, max_iter=4000, polishing=False)
        else:
            self.solver.update(q=q, l=lo, u=hi, Ax=Ax)
        res = self.solver.solve(raise_error=False)
        ok = res.info.status in ('solved', 'solved inaccurate')

        if ok:
            z = res.x
            dx = z[:NX * (N + 1)].reshape(N + 1, NX)
            dw = z[NX * (N + 1):].reshape(N, NU)
            xs, ws = x_bar + dx, w_bar + dw
        else:                                                # fall back: feedforward + shift
            if self.prev_sol is not None:
                xs, ws = self.prev_sol[0], np.vstack([self.prev_sol[1][1:], self.prev_sol[1][-1:]])
            else:
                xs, ws = x_ref, w_ref
        w0 = ws[0].copy()
        M = np.clip(self.Gp @ w0, -self.torque_max, self.torque_max)
        self.prev_sol = (xs, ws)
        self.prev_dw = w0 - w_bar[0]
        return w0, dict(torques=M, predicted=xs, plan_w=ws, w_bar0=w_bar[0], ok=ok,
                        cost=float(res.info.obj_val) if ok else float('nan'),
                        status=res.info.status, iters=res.info.iter,
                        solve_ms=res.info.run_time * 1e3,
                        total_ms=(time.perf_counter() - t0) * 1e3)
