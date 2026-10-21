"""Gazebo sign test (plan test 8): checks the wheel numbering and signs against G.

    ros2 run mecanum_lmpc_v2 sign_test --ros-args -p use_sim_time:=true

Needs the sim in effort mode (own_mecanum_sim.launch.py drive:=effort). For each torque pattern
the wheels get +-M for `push` seconds of sim time; the body twist at the end must point where
G says; the change of the twist during the push is compared, so motion left over from the previous
pattern does not count. Then the pattern is reversed for the same time (brakes) and the wheels
are released.
"""
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64

from mecanum_mpc_common.params import WHEELS

PATTERNS = {                     # wheels 1..4 = FL, FR, RL, RR; expected sign of (V_xi, V_eta, psi_dot)
    'forward': ([1, 1, 1, 1], (1, 0, 0)),
    'strafe left': ([-1, 1, 1, -1], (0, 1, 0)),
    'turn left': ([-1, 1, -1, 1], (0, 0, 1)),
    'null (front vs rear)': ([1, 1, -1, -1], (0, 0, 0)),
}


class SignTest(Node):
    def __init__(self):
        super().__init__('lmpc_v2_sign_test')
        self.M = self.declare_parameter('torque', 0.02).value
        self.push = self.declare_parameter('push', 0.3).value
        model = self.declare_parameter('model_name', 'own_mecanum').value
        self.pubs = [self.create_publisher(Float64, f'/model/{model}/joint/{w}_wheel_joint/cmd_force', 10)
                     for w in WHEELS]
        self.twist = None
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)

    def _on_odom(self, msg):
        t = msg.twist.twist
        self.twist = np.array([t.linear.x, t.linear.y, t.angular.z])

    def apply(self, torques, duration):
        end = self.get_clock().now().nanoseconds * 1e-9 + duration
        while rclpy.ok() and self.get_clock().now().nanoseconds * 1e-9 < end:
            for p, m in zip(self.pubs, torques):
                p.publish(Float64(data=float(m)))
            rclpy.spin_once(self, timeout_sec=0.02)

    def run(self):
        while self.twist is None or min(p.get_subscription_count() for p in self.pubs) == 0:
            rclpy.spin_once(self, timeout_sec=0.1)
        ok_all = True
        ref_speed = None
        for name, (pattern, expect) in PATTERNS.items():
            u = self.M * np.array(pattern)
            tw0 = self.twist.copy()              # leftover motion from the previous pattern
            self.apply(u, self.push)
            tw = self.twist - tw0                # change of the body twist caused by the pattern
            self.apply(-u, self.push)
            self.apply(np.zeros(4), 1.0)
            lin = math.hypot(tw[0], tw[1])
            if name == 'forward':
                ref_speed = lin
            if any(expect):
                k = int(np.argmax(np.abs(expect)))
                scale = np.array([1.0, 1.0, 0.2])          # compare yaw rate in rad/s vs m/s loosely
                dom = abs(tw[k] * scale[k]) if k == 2 else abs(tw[k])
                others = [abs(tw[i]) * (scale[i] if i == 2 else 1.0) for i in range(3) if i != k]
                ok = np.sign(tw[k]) == expect[k] and all(o < 0.3 * dom for o in others)
            else:
                ok = lin < 0.1 * (ref_speed or 1.0) and abs(tw[2]) < 0.1
            ok_all &= bool(ok)
            self.get_logger().info(f'{name:22s} twist (V_xi, V_eta, psi_dot) = '
                                   f'({tw[0]:+.3f}, {tw[1]:+.3f}, {tw[2]:+.3f})  '
                                   f'{"PASS" if ok else "FAIL"}')
        self.get_logger().info('sign test ' + ('PASSED' if ok_all else 'FAILED'))
        return ok_all


def main():
    rclpy.init()
    node = SignTest()
    try:
        ok = node.run()
    finally:
        for p in node.pubs:
            p.publish(Float64(data=0.0))
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
