"""ROS 2 node of the nonlinear MPC (NMPC plan: "Software plan" and "One control step, end to end").

Measurement, reference, timer, topics (/nmpc/...) and logging are the shared ones in
mecanum_mpc_common.mpc_node, identical to the LMPC's; this node only supplies the controller.
Logged on top of the common columns (qp_ms = acados solve time):
    iterations     SQP iterations of the step
    kkt            largest KKT residual at exit
    slack_max      largest wheel-speed slack of the step (rad/s)
    acados_status  0 converged, 2 iteration cap, 3 minimum step, 7 time budget, else failed
    alpha_first    line-search step of the first iteration (1 = full step)
    M1_first..M4_first   torques after the first iteration (= the LMPC's step when alpha_first = 1)
"""
from mecanum_mpc_common.mpc_node import MPCNode, run_node

from .nmpc import NonlinearMPC


class NonlinearMPCNode(MPCNode):
    NAME = 'nmpc'
    EXTRA = ('iterations', 'kkt', 'slack_max', 'acados_status', 'alpha_first', 'u_first')

    def extra_names(self):
        return list(self.EXTRA[:-1]) + ['M1_first', 'M2_first', 'M3_first', 'M4_first']

    def make_controller(self, p, c):
        P = self.declare_parameter
        return NonlinearMPC(p, c['T'], c['N'], c['Q'], c['R'], c['P_scale'], c['u_max'],
                            c['du_max'], omega_max=c['omega_max'],
                            wheel_speed_limit=c['wheel_speed_limit'],
                            slack_lin=c['slack_lin'], slack_quad=c['slack_quad'],
                            max_iter=P('max_iter', 10).value,              # N3
                            time_budget=P('time_budget', 0.04).value,      # s, N3
                            kkt_tol=P('kkt_tol', 1e-6).value)              # N3


def main():
    run_node(NonlinearMPCNode)


if __name__ == '__main__':
    main()
