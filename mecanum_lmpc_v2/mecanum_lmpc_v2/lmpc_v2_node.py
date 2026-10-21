"""ROS 2 node of the linear MPC v2 (plan: "Software plan" and "One control step, end to end").

Measurement, reference, timer, topics (/lmpc_v2/...) and logging are the shared ones in
mecanum_mpc_common.mpc_node; this node only supplies the controller. Logged on top of the
common columns: slack_max, the largest wheel-speed slack of the step (rad/s).
"""
from mecanum_mpc_common.mpc_node import MPCNode, run_node

from .lmpc import LinearMPCv2


class LinearMPCv2Node(MPCNode):
    NAME = 'lmpc_v2'
    EXTRA = ('slack_max',)

    def make_controller(self, p, c):
        return LinearMPCv2(p, c['T'], c['N'], c['Q'], c['R'], c['P_scale'], c['u_max'],
                           c['du_max'], omega_max=c['omega_max'],
                           wheel_speed_limit=c['wheel_speed_limit'],
                           slack_lin=c['slack_lin'], slack_quad=c['slack_quad'])


def main():
    run_node(LinearMPCv2Node)


if __name__ == '__main__':
    main()
