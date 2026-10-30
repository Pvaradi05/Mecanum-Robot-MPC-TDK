"""ROS 2 node of the linear MPC v2: the latest LTV-MPC (lmpc.py with mecanum_lmpc_v2.LATEST).

Measurement, reference, timer, topics (/lmpc_v2/...) and logging are the shared ones in
mecanum_mpc_common.mpc_node; this node only supplies the controller. Its switches, those of the
comparison repository's mpc_controller.py (defaults = LATEST):
    terminal            'dare' (default) or 'scaled' (P = P_scale Q)
    P_scale             5.0, the terminal factor of 'scaled' and of the Q_path stage weight
    S                   torque-change penalty S = s I (0: off, the default)
    Q_path              [qa, qc]: position weight in the path frame ([0, 0]: off, the default)
    wheel_speed_limit   soft wheel-speed limit |omega| <= omega_max (false: off, the default)
Logged on top of the common columns: slack_max, the largest wheel-speed slack of the step (rad/s),
and dare_iters, the Riccati iterations of the step's terminal weight.
"""
from mecanum_mpc_common.mpc_node import MPCNode, run_node

from . import LATEST
from .lmpc import LinearMPCv2


class LinearMPCv2Node(MPCNode):
    NAME = 'lmpc_v2'
    EXTRA = ('slack_max', 'dare_iters')

    def extra_values(self, info):
        return [info['slack_max'], self.mpc.dare_iterations]

    def make_controller(self, p, c):
        P = self.declare_parameter
        s = P('S', 0.0).value
        q_path = list(P('Q_path', [0.0, 0.0]).value)
        return LinearMPCv2(p, c['T'], c['N'], c['Q'], c['R'], P('P_scale', 5.0).value, c['u_max'],
                           omega_max=c['omega_max'],
                           wheel_speed_limit=P('wheel_speed_limit',
                                               LATEST['wheel_speed_limit']).value,
                           terminal=P('terminal', LATEST['terminal']).value,
                           S=[s] * 4 if s > 0 else None,
                           Q_path=q_path if any(q_path) else None, state=LATEST['state'])


def main():
    run_node(LinearMPCv2Node)


if __name__ == '__main__':
    main()
