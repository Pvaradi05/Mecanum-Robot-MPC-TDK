"""Nonlinear MPC after Kuehne, Gomes da Silva Jr., Lages (2005), section 3 (eq. 7-10), solved by
acados (NMPC plan: "The nonlinear optimal control problem", "How it is solved", "One control step,
end to end").

Since 2026-10-30 everything but the optimiser is the latest LTV-MPC's (LinearMPCv2 with
mecanum_lmpc_v2.LATEST = Kemenylegeny/Mecanum-tracking-tdk @ 411436a): the model (the Chaplygin
model with wheel-speed states, RK4), the states and inputs, the cost, the DARE terminal weight
and the constraints (the torque box only). Except, for now, the weights: Q defaults to Bryson's
rule (weights.Q_BRYSON), as the LTV-MPC's Q_BO diverges in Gazebo; Q=Q_BO gives the LTV-MPC's
problem exactly (decided 2026-10-30). The NMPC's own parts: acados, and the
problem it solves, the nonlinear model with states and torques as decision variables (multiple
shooting) instead of the LTV-MPC's model linearised along the reference in a condensed QP.

Decisions: N1 acados (SQP, Gauss-Newton, HPIPM); N2 torques and states as decision variables
(multiple shooting); N3 iterate to convergence (KKT residuals <= kkt_tol), capped at max_iter
iterations and time_budget; N4 the LTV-MPC's cost; N5 the LTV-MPC's constraints, the torque box
(no torque-rate limit and no soft wheel-speed limit since 2026-10-30); N6 every step starts from
the reference, so the first SQP iteration solves the LTV-MPC's QP; N7 model, reference and limits
from mecanum_mpc_common.

acados problem (stage j = 0..N-1, terminal N):
    z_j = [X, Y, psi, w1, w2, w3]   pose and the speeds of wheels 1-3 (wheel_dynamics.py)
    z_{j+1} = F(z_j, u_j)            F = RK4 step of the Chaplygin model over T
    cost  sum_j z~_j^T Q z~_j + u~_j^T R u~_j + z~_N^T P z~_N
          Q = diag(q) with the velocity block J3^T diag(q_v) J3, R = diag(r); P = solution of the
          DARE at (A_N-1, B_N-1), the RK4 Jacobians at (z_r,N-1, u_r,N-1), warm-started with the
          previous step's P (the LTV-MPC's terminal='dare')
          (acados: 1/2 |y - y_ref|^2_W with W = 2 diag(Q, R), W_e = 2 P; cost_scaling = 1)
    z_0 = z_meas,  |u_j| <= u_max
"""
import ctypes
import hashlib
import json
import os
import time

# acados built from source in ~/acados (README): find it without exported variables, and load its
# libraries globally so the generated solver resolves them (what LD_LIBRARY_PATH would do)
ACADOS_DIR = os.environ.setdefault('ACADOS_SOURCE_DIR', os.path.expanduser('~/acados'))
for _lib in ('libblasfeo.so', 'libhpipm.so', 'libacados.so'):
    ctypes.CDLL(os.path.join(ACADOS_DIR, 'lib', _lib), mode=ctypes.RTLD_GLOBAL)

import casadi as ca  # noqa: E402
import numpy as np  # noqa: E402
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver  # noqa: E402
from mecanum_mpc_common import motor, wheel_dynamics, wheel_dynamics_casadi
from mecanum_mpc_common.conventions import (STATUS_FALLBACK, STATUS_NOT_CONVERGED, STATUS_SOLVED,
                                            STATUS_STOPPED)
from mecanum_mpc_common.dare import solve_dare
from mecanum_mpc_common.params import RobotParams
from mecanum_mpc_common.weights import Q_BRYSON, R_BO

NX, NU = 6, 4
ACADOS_SUCCESS, ACADOS_MAXITER, ACADOS_MINSTEP, ACADOS_TIMEOUT = 0, 2, 3, 7
USABLE = (ACADOS_MAXITER, ACADOS_MINSTEP, ACADOS_TIMEOUT)    # caps: last iterate is applied
DEFAULT_CODE_DIR = os.path.expanduser('~/ros2_ws/build/mecanum_nmpc/acados_generated')


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class NonlinearMPC:
    def __init__(self, p: RobotParams, T=0.05, N=10, Q=Q_BRYSON, R=R_BO, u_max=motor.U_MAX,
                 max_failures=3, dare_iters=500, dare_tol=1e-9, max_iter=10, time_budget=0.04,
                 kkt_tol=1e-6, code_dir=DEFAULT_CODE_DIR, verbose=False):
        self.p, self.T, self.N = p, T, N
        self.state = 'wheel'         # x_meas, x_r are z = [X, Y, psi, w1, w2, w3], as the LTV-MPC's
        self.u_max, self.max_failures = u_max, max_failures
        self.dare_iters, self.dare_tol = dare_iters, dare_tol
        q = np.asarray(Q, dtype=float)
        self.Q, self.R = np.diag(q), np.diag(R).astype(float)
        self.Q[3:, 3:] = wheel_dynamics.velocity_weight(q[3:], p)
        settings = dict(T=T, N=N, Q=list(map(float, Q)), R=list(map(float, R)),
                        u_max=float(u_max), max_iter=int(max_iter),
                        time_budget=float(time_budget), kkt_tol=float(kkt_tol),
                        robot=[p.m_s, p.J_C, p.J1, p.R, p.rho, p.l])
        tag = hashlib.sha1(json.dumps(settings, sort_keys=True).encode()).hexdigest()[:10]
        ocp = self._build_ocp(settings, os.path.join(code_dir, tag))
        self.solver = AcadosOcpSolver(ocp, generate=False, build=False, check_reuse_possible=True,
                                      verbose=verbose)
        self.P_dare = None           # last DARE solution (warm start of the next step)
        self.dare_iterations = 0     # Riccati iterations of the last step
        self.prev_plan = None        # torques of the last converged plan (N x 4), for fallback
        self.failures = 0
        self.stopped = False

    def _build_ocp(self, s, out_dir):
        p, T, N = self.p, s['T'], s['N']
        model = AcadosModel()
        model.name = 'mecanum_nmpc'
        z = ca.SX.sym('z', NX)
        u = ca.SX.sym('u', NU)
        model.x, model.u = z, u
        model.disc_dyn_expr = wheel_dynamics_casadi.rk4_step(z, u, T, p)

        ocp = AcadosOcp()
        ocp.model = model
        ocp.solver_options.N_horizon = N
        ocp.solver_options.tf = N * T
        # cost: 1/2 |y - y_ref|^2_W = z~^T Q z~ + u~^T R u~ with W = 2 diag(Q, R); W_e = 2 P is
        # set every step (the DARE solution), 2 Q here only until the first step
        ocp.cost.cost_type = 'LINEAR_LS'
        ocp.cost.cost_type_e = 'LINEAR_LS'
        ocp.cost.Vx = np.vstack([np.eye(NX), np.zeros((NU, NX))])
        ocp.cost.Vu = np.vstack([np.zeros((NX, NU)), np.eye(NU)])
        ocp.cost.W = 2.0 * np.block([[self.Q, np.zeros((NX, NU))], [np.zeros((NU, NX)), self.R]])
        ocp.cost.yref = np.zeros(NX + NU)
        ocp.cost.Vx_e = np.eye(NX)
        ocp.cost.W_e = 2.0 * self.Q
        ocp.cost.yref_e = np.zeros(NX)
        ocp.solver_options.cost_scaling = np.ones(N + 1)     # no time-step scaling (as the LMPC)

        # constraints: initial state, torque box
        ocp.constraints.x0 = np.zeros(NX)
        ocp.constraints.idxbu = np.arange(NU)
        ocp.constraints.lbu = -s['u_max'] * np.ones(NU)
        ocp.constraints.ubu = s['u_max'] * np.ones(NU)

        o = ocp.solver_options
        o.integrator_type = 'DISCRETE'
        o.nlp_solver_type = 'SQP'
        o.hessian_approx = 'GAUSS_NEWTON'
        o.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        o.hpipm_mode = 'ROBUST'
        o.qp_solver_iter_max = 200
        o.qp_tol = 1e-10
        o.globalization = 'MERIT_BACKTRACKING'
        o.nlp_solver_max_iter = s['max_iter']
        o.tol = s['kkt_tol']
        o.timeout_max_time = s['time_budget']
        o.store_iterates = True                # first-iteration torques (= LMPC) for the log
        o.print_level = 0
        ocp.code_gen_options.code_export_directory = os.path.join(out_dir, 'c_generated_code')
        ocp.code_gen_options.json_file = 'acados_ocp.json'     # written into the export directory
        return ocp

    def step(self, x_meas, x_r, u_r, u_applied):
        """x_meas (6,), x_r (N+1 x 6) wheel-speed states, u_r (N x 4), u_applied (4,)
        -> (u (4,), info). u_applied is not used (no torque-rate limit), as in the LTV-MPC."""
        t0 = time.perf_counter()
        N, T, p, sv = self.N, self.T, self.p, self.solver
        nan4 = np.full(NU, np.nan)
        if self.stopped:
            return np.zeros(NU), dict(status=STATUS_STOPPED, qp_ms=0.0, step_ms=0.0,
                                      predicted=None, x_tilde0=np.zeros(NX), iterations=0,
                                      kkt=np.nan, acados_status=-1, u_first=nan4,
                                      alpha_first=np.nan, dare_iters=0)

        x = np.array(x_meas, dtype=float)
        x[2] = x_r[0, 2] + wrap(x[2] - x_r[0, 2])       # heading unwrapped onto the reference
        sv.set(0, 'lbx', x)
        sv.set(0, 'ubx', x)
        # terminal weight: the LTV-MPC's DARE solution at the end of the horizon
        A, B = wheel_dynamics.rk4_jacobians(x_r[N - 1], u_r[N - 1], T, p)
        self.P_dare, self.dare_iterations = solve_dare(A, B, self.Q, self.R, self.P_dare,
                                                       self.dare_iters, self.dare_tol)
        sv.cost_set(N, 'W', 2.0 * self.P_dare)
        # references and the starting point (N6): the reference trajectory itself
        for j in range(N):
            sv.set(j, 'yref', np.r_[x_r[j], u_r[j]])
            sv.set(j, 'x', x_r[j])
            sv.set(j, 'u', u_r[j])
        sv.set(N, 'yref', x_r[N])
        sv.set(N, 'x', x_r[N])

        status = sv.solve()
        solve_ms = 1e3 * sv.get_stats('time_tot')
        iterations = int(sv.get_stats('nlp_iter'))
        res = sv.get_stats('residuals')
        kkt = float(np.max(res)) if np.size(res) else np.nan
        if iterations >= 1:
            u_first = self._iterate_u0(1)
            alphas = np.atleast_1d(sv.get_stats('alpha'))
            alpha_first = float(alphas[1]) if alphas.size > 1 else np.nan
        else:
            u_first, alpha_first = sv.get(0, 'u'), np.nan

        if status == ACADOS_SUCCESS or status in USABLE:
            self.failures = 0
            plan = np.array([sv.get(j, 'u') for j in range(N)])
            u = plan[0]
            predicted = np.array([sv.get(j, 'x') for j in range(1, N + 1)])
            if status == ACADOS_SUCCESS:
                self.prev_plan = plan
                st = STATUS_SOLVED
            else:
                st = STATUS_NOT_CONVERGED
        else:
            self.failures += 1
            predicted = None
            if self.failures >= self.max_failures or self.prev_plan is None:
                self.stopped = self.failures >= self.max_failures
                u = np.zeros(NU)
                st = STATUS_STOPPED if self.stopped else STATUS_FALLBACK
            else:
                u = self.prev_plan[min(self.failures, N - 1)]   # last converged plan, shifted
                st = STATUS_FALLBACK
        u = np.clip(u, -self.u_max, self.u_max)
        return u, dict(status=st, qp_ms=solve_ms, step_ms=(time.perf_counter() - t0) * 1e3,
                       predicted=predicted, iterations=iterations, kkt=kkt,
                       acados_status=int(status), u_first=u_first, alpha_first=alpha_first,
                       x_tilde0=x - x_r[0], dare_iters=self.dare_iterations)

    def _iterate_u0(self, iteration):
        """u at stage 0 after the given SQP iteration (store_iterates). Reads one vector instead
        of the whole AcadosOcpIterate that get_iterate() assembles."""
        return self.solver._AcadosOcpSolver__ocp_nlp_get_from_iterate(iteration, 0, 'u')
