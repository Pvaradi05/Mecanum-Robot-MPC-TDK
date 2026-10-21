"""Nonlinear MPC after Kuehne, Gomes da Silva Jr., Lages (2005), section 3 (eq. 7-10), on the
RK4-discretised Chaplygin model, solved by acados (NMPC plan: "The nonlinear optimal control
problem", "How it is solved", "One control step, end to end").

Decisions: N1 acados (SQP, Gauss-Newton, HPIPM); N2 torques and states as decision variables
(multiple shooting); N3 iterate to convergence (KKT residuals <= kkt_tol), capped at max_iter
iterations and time_budget; N4 the LMPC's cost (D1); N5 the LMPC's box and rate limits plus a
soft wheel-speed limit; N6 every step starts from the reference, so the first SQP iteration
solves the LMPC's QP; N7 model, reference and limits from mecanum_mpc_common.

acados problem (stage j = 0..N-1, terminal N):
    z_j = [x_j; u_{j-1}]   (6 states + the previous torques, for the rate limit)
    z_{j+1} = [F(x_j, u_j); u_j]                 F = RK4 step of the Chaplygin model over T
    cost  sum_j x~_j^T Q x~_j + u~_j^T R u~_j + x~_N^T P x~_N + slack penalties
          (acados: 1/2 |y - y_ref|^2_W with W = 2 diag(Q, R), W_e = 2 P; cost_scaling = 1)
    z_0 = [x_meas; u_applied],  |u_j| <= u_max,  |u_j - u_{j-1}| <= du_max
    |omega(x_j)| <= omega_max + s_j  (j = 1..N),  penalty slack_lin s + slack_quad s^2
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
from mecanum_mpc_common import dynamics_casadi, motor
from mecanum_mpc_common.conventions import (STATUS_FALLBACK, STATUS_NOT_CONVERGED, STATUS_SOLVED,
                                            STATUS_STOPPED)
from mecanum_mpc_common.params import RobotParams
from mecanum_mpc_common.wheel_speed import SLACK_LIN, SLACK_QUAD

NX, NU = 6, 4
ACADOS_SUCCESS, ACADOS_MAXITER, ACADOS_MINSTEP, ACADOS_TIMEOUT = 0, 2, 3, 7
USABLE = (ACADOS_MAXITER, ACADOS_MINSTEP, ACADOS_TIMEOUT)    # caps: last iterate is applied
DEFAULT_CODE_DIR = os.path.expanduser('~/ros2_ws/build/mecanum_nmpc/acados_generated')


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class NonlinearMPC:
    def __init__(self, p: RobotParams, T=0.05, N=10, Q=(40000, 40000, 3283, 2500, 2500, 100),
                 R=(400, 400, 400, 400), P_scale=5.0, u_max=motor.U_MAX, du_max=motor.DU_MAX,
                 max_failures=3, omega_max=motor.OMEGA_MAX, wheel_speed_limit=True,
                 slack_lin=SLACK_LIN, slack_quad=SLACK_QUAD, max_iter=10, time_budget=0.04,
                 kkt_tol=1e-6, code_dir=DEFAULT_CODE_DIR, verbose=False):
        self.p, self.T, self.N = p, T, N
        self.u_max, self.du_max, self.max_failures = u_max, du_max, max_failures
        self.wheel_speed_limit = wheel_speed_limit
        settings = dict(T=T, N=N, Q=list(map(float, Q)), R=list(map(float, R)),
                        P_scale=float(P_scale), u_max=float(u_max), du_max=float(du_max),
                        omega_max=float(omega_max), wheel_speed_limit=bool(wheel_speed_limit),
                        slack_lin=float(slack_lin), slack_quad=float(slack_quad),
                        max_iter=int(max_iter), time_budget=float(time_budget),
                        kkt_tol=float(kkt_tol),
                        robot=[p.m_s, p.J_C, p.J1, p.R, p.rho, p.l])
        tag = hashlib.sha1(json.dumps(settings, sort_keys=True).encode()).hexdigest()[:10]
        ocp = self._build_ocp(settings, os.path.join(code_dir, tag))
        self.solver = AcadosOcpSolver(ocp, generate=False, build=False, check_reuse_possible=True,
                                      verbose=verbose)
        self.prev_plan = None        # torques of the last converged plan (N x 4), for fallback
        self.failures = 0
        self.stopped = False

    def _build_ocp(self, s, out_dir):
        p, T, N = self.p, s['T'], s['N']
        model = AcadosModel()
        model.name = 'mecanum_nmpc'
        x = ca.SX.sym('x', NX)
        u_prev = ca.SX.sym('u_prev', NU)
        u = ca.SX.sym('u', NU)
        model.x = ca.vertcat(x, u_prev)
        model.u = u
        model.disc_dyn_expr = ca.vertcat(dynamics_casadi.rk4_step(x, u, T, p), u)
        if s['wheel_speed_limit']:
            model.con_h_expr = dynamics_casadi.wheel_speeds(x, p)      # stages 1..N-1
            model.con_h_expr_e = dynamics_casadi.wheel_speeds(x, p)    # stage N (none at 0)

        ocp = AcadosOcp()
        ocp.model = model
        ocp.solver_options.N_horizon = N
        ocp.solver_options.tf = N * T
        Q, R = np.diag(s['Q']), np.diag(s['R'])
        # cost: 1/2 |y - y_ref|^2_W = x~^T Q x~ + u~^T R u~ with W = 2 diag(Q, R)
        ocp.cost.cost_type = 'LINEAR_LS'
        ocp.cost.cost_type_e = 'LINEAR_LS'
        Vx = np.zeros((NX + NU, NX + NU))
        Vx[:NX, :NX] = np.eye(NX)
        Vu = np.zeros((NX + NU, NU))
        Vu[NX:, :] = np.eye(NU)
        ocp.cost.Vx, ocp.cost.Vu = Vx, Vu
        ocp.cost.W = 2.0 * np.block([[Q, np.zeros((NX, NU))], [np.zeros((NU, NX)), R]])
        ocp.cost.yref = np.zeros(NX + NU)
        ocp.cost.Vx_e = np.hstack([np.eye(NX), np.zeros((NX, NU))])
        ocp.cost.W_e = 2.0 * s['P_scale'] * Q
        ocp.cost.yref_e = np.zeros(NX)
        ocp.solver_options.cost_scaling = np.ones(N + 1)     # no time-step scaling (as the LMPC)

        # constraints: initial state, torque box, torque-rate limit, soft wheel-speed limit
        ocp.constraints.x0 = np.zeros(NX + NU)
        ocp.constraints.idxbu = np.arange(NU)
        ocp.constraints.lbu = -s['u_max'] * np.ones(NU)
        ocp.constraints.ubu = s['u_max'] * np.ones(NU)
        ocp.constraints.C = np.hstack([np.zeros((NU, NX)), -np.eye(NU)])
        ocp.constraints.D = np.eye(NU)
        ocp.constraints.lg = -s['du_max'] * np.ones(NU)
        ocp.constraints.ug = s['du_max'] * np.ones(NU)
        if s['wheel_speed_limit']:
            w = s['omega_max'] * np.ones(4)
            ocp.constraints.lh, ocp.constraints.uh = -w, w
            ocp.constraints.lh_e, ocp.constraints.uh_e = -w, w
            ocp.constraints.idxsh = np.arange(4)
            ocp.constraints.idxsh_e = np.arange(4)
            z, Z = s['slack_lin'] * np.ones(4), 2.0 * s['slack_quad'] * np.ones(4)
            ocp.cost.zl, ocp.cost.zu, ocp.cost.Zl, ocp.cost.Zu = z, z, Z, Z
            ocp.cost.zl_e, ocp.cost.zu_e, ocp.cost.Zl_e, ocp.cost.Zu_e = z, z, Z, Z

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
        """x_meas (6,), x_r (N+1 x 6), u_r (N x 4), u_applied (4,) -> (u (4,), info)."""
        t0 = time.perf_counter()
        N, sv = self.N, self.solver
        nan4 = np.full(NU, np.nan)
        if self.stopped:
            return np.zeros(NU), dict(status=STATUS_STOPPED, qp_ms=0.0, step_ms=0.0,
                                      predicted=None, iterations=0, kkt=np.nan, slack_max=0.0,
                                      acados_status=-1, u_first=nan4, alpha_first=np.nan)

        x = np.array(x_meas, dtype=float)
        x[2] = x_r[0, 2] + wrap(x[2] - x_r[0, 2])       # heading unwrapped onto the reference
        z0 = np.r_[x, u_applied]
        sv.set(0, 'lbx', z0)
        sv.set(0, 'ubx', z0)
        # references and the starting point (N6): the reference trajectory itself
        for j in range(N):
            sv.set(j, 'yref', np.r_[x_r[j], u_r[j]])
            sv.set(j, 'x', np.r_[x_r[j], u_r[j - 1] if j > 0 else u_applied])
            sv.set(j, 'u', u_r[j])
        sv.set(N, 'yref', x_r[N])
        sv.set(N, 'x', np.r_[x_r[N], u_r[N - 1]])

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
            u = np.clip(plan[0], -self.u_max, self.u_max)
            u = np.clip(u, u_applied - self.du_max, u_applied + self.du_max)
            predicted = np.array([sv.get(j, 'x')[:NX] for j in range(1, N + 1)])
            slack_max = 0.0
            if self.wheel_speed_limit:
                slack_max = float(max(np.max(np.r_[sv.get(j, 'sl'), sv.get(j, 'su')])
                                      for j in range(1, N + 1)))
            if status == ACADOS_SUCCESS:
                self.prev_plan = plan
                st = STATUS_SOLVED
            else:
                st = STATUS_NOT_CONVERGED
        else:
            self.failures += 1
            predicted, slack_max = None, 0.0
            if self.failures >= self.max_failures or self.prev_plan is None:
                self.stopped = self.failures >= self.max_failures
                u = np.zeros(NU)
                st = STATUS_STOPPED if self.stopped else STATUS_FALLBACK
            else:
                u = self.prev_plan[min(self.failures, N - 1)]   # last converged plan, shifted
                st = STATUS_FALLBACK
            u = np.clip(u, -self.u_max, self.u_max)
        return u, dict(status=st, qp_ms=solve_ms, step_ms=(time.perf_counter() - t0) * 1e3,
                       predicted=predicted, iterations=iterations, kkt=kkt, slack_max=slack_max,
                       acados_status=int(status), u_first=u_first, alpha_first=alpha_first,
                       x_tilde0=x - x_r[0])

    def _iterate_u0(self, iteration):
        """u at stage 0 after the given SQP iteration (store_iterates). Reads one vector instead
        of the whole AcadosOcpIterate that get_iterate() assembles."""
        return self.solver._AcadosOcpSolver__ocp_nlp_get_from_iterate(iteration, 0, 'u')
