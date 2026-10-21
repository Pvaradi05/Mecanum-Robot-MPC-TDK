"""ROS 2 node shared by the wheel-torque controllers (LMPC plan: "Software plan" and "One control
step, end to end"; NMPC plan: "One control step, end to end").

A controller package subclasses MPCNode, sets NAME, and supplies make_controller() and the
info keys it logs on top of the common columns. Everything else is identical for every
controller, so measurement, reference, timing and logging cannot differ between them.

Subscribes  /odom          (nav_msgs/Odometry: pose in the world frame, twist in the body frame)
            /joint_states  (measured wheel speeds, diagnostics only)
Publishes   /model/<model>/joint/<corner>_wheel_joint/cmd_force   (std_msgs/Float64, N m)
            /<NAME>/reference, /<NAME>/predicted                   (nav_msgs/Path, frame odom)
            /<NAME>/status                                         (std_msgs/Float64MultiArray,
                                                                    field names in layout label)
Log         <log_dir>/<shape>_<heading>_<speed>_<time>.csv, one row per control step

Limits u_max, omega_max and du_max default to the DYNAMIXEL XM430-W210-T data at 12.0 V, see
motor.py (source: ROBOTIS e-manual).
"""
import csv
import math
import os
import signal
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, MultiArrayDimension

from . import motor, wheel_speed
from . import params as robot_params
from .conventions import STATUS_STOPPED
from .reference import SHAPES, Reference

STATUS_FIELDS = ['t', 'err_pos', 'err_psi', 'qp_ms', 'step_ms', 'status',
                 'M1', 'M2', 'M3', 'M4', 'w1', 'w2', 'w3', 'w4']


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class MPCNode(Node):
    NAME = ''          # node name, topic prefix and log folder, e.g. 'lmpc_v2'
    EXTRA = ()         # info keys logged after the common CSV columns and status fields

    def make_controller(self, p, c):
        """Return the controller; c holds the common settings (T, N, Q, R, P_scale, limits,
        wheel-speed limit and slack weights)."""
        raise NotImplementedError

    def extra_values(self, info):
        """Values of EXTRA for one step (vectors are flattened, in order)."""
        return [info[k] for k in self.EXTRA]

    def extra_names(self):
        return list(self.EXTRA)

    def __init__(self):
        super().__init__(self.NAME)
        P = self.declare_parameter
        default_urdf = os.path.join(get_package_share_directory('tb3_sim_bringup'),
                                    'models', 'own_mecanum', 'robot.urdf')
        shape = P('shape', 'circle').value
        size = P('size', 0.0).value                         # 0 = default size of the shape
        corner_radius = P('corner_radius', -1.0).value      # square only; < 0 = 15 % of the side
        speed = P('speed', 0.1).value          # m/s; feasible on every path with the XM430 limits
        heading = P('heading', 'fixed').value
        loop = P('loop', False).value
        self.start_offset = list(P('start_offset', [0.0, 0.0, 0.0]).value)
        c = dict(
            T=P('T', 0.05).value,
            N=P('N', 10).value,                 # D4: 10 steps = 0.5 s (was 20 until 2026-10-21)
            Q=list(P('Q', [40000.0, 40000.0, 3283.0, 2500.0, 2500.0, 100.0]).value),
            R=list(P('R', [400.0, 400.0, 400.0, 400.0]).value),
            P_scale=P('P_scale', 5.0).value,
            u_max=P('u_max', motor.U_MAX).value,                 # N m, XM430-W210-T graph maximum
            omega_max=P('omega_max', motor.OMEGA_MAX).value,     # rad/s, no-load speed at 12 V
            du_max=P('du_max', motor.DU_MAX).value,              # N m per step, = u_max
            wheel_speed_limit=P('wheel_speed_limit', True).value,   # soft limit (NMPC plan N5)
            slack_lin=P('slack_lin', wheel_speed.SLACK_LIN).value,     # per rad/s over omega_max
            slack_quad=P('slack_quad', wheel_speed.SLACK_QUAD).value,  # per (rad/s)^2 over it
        )
        urdf = P('urdf', default_urdf).value
        self.model = P('model_name', 'own_mecanum').value
        log_dir = P('log_dir', os.path.expanduser(f'~/ros2_ws/mpc_logs/{self.NAME}')).value
        if shape not in SHAPES:
            raise ValueError(f'shape must be one of {SHAPES}')
        self.omega_max = c['omega_max']

        self.p = robot_params.from_urdf(urdf)
        self.get_logger().info(
            f'model: m_s={self.p.m_s:.3f} kg, m_e={self.p.m_e:.3f} kg, J_e={self.p.J_e:.5f} kg m2, '
            f'kappa={self.p.kappa:.4f}, R={self.p.R:.5f} m, rho={self.p.rho} m, l={self.p.l} m, '
            f'wheel signs {self.p.wheel_sign.tolist()}')
        self.T, self.N = c['T'], c['N']
        self.ref = Reference(self.p, shape, size or None, speed, heading, loop,
                             corner_radius=None if corner_radius < 0 else corner_radius)
        maxima, violations = self.ref.check_limits(self.T, c['u_max'], c['du_max'], self.omega_max)
        self.get_logger().info(
            f'{shape}, {heading} heading, {speed} m/s, ' + ('loop' if loop else
                                                          f'{self.ref.T_end:.1f} s') +
            f'; reference needs max |u_r| {maxima["u"]:.4f} N m, max step change '
            f'{maxima["du"]:.4f} N m, max wheel speed {maxima["omega"]:.1f} rad/s')
        if violations:
            raise SystemExit('reference not feasible: ' + '; '.join(violations))
        self.mpc = self.make_controller(self.p, c)

        self.cmd_pubs = [self.create_publisher(
            Float64, f'/model/{self.model}/joint/{w}_wheel_joint/cmd_force', 10)
            for w in robot_params.WHEELS]
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ref_pub = self.create_publisher(Path, f'/{self.NAME}/reference', latched)
        self.pred_pub = self.create_publisher(Path, f'/{self.NAME}/predicted', 10)
        self.status_pub = self.create_publisher(Float64MultiArray, f'/{self.NAME}/status', 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_subscription(JointState, 'joint_states', self._on_joints, 10)

        self.x_meas = None
        self.wheel_speeds = np.full(4, np.nan)
        self.u_applied = np.zeros(4)
        self.t0 = None
        os.makedirs(log_dir, exist_ok=True)
        self.log_path = os.path.join(
            log_dir, f'{shape}_{heading}_{speed:.2f}_{time.strftime("%Y%m%d_%H%M%S")}.csv')
        self.log_file = open(self.log_path, 'w', newline='')
        self.log = csv.writer(self.log_file)
        self.log.writerow(['t', 'X', 'Y', 'psi', 'X_dot', 'Y_dot', 'psi_dot',
                           'X_r', 'Y_r', 'psi_r', 'X_dot_r', 'Y_dot_r', 'psi_dot_r',
                           'M1', 'M2', 'M3', 'M4', 'qp_ms', 'step_ms', 'status']
                          + self.extra_names())
        self.status_fields = STATUS_FIELDS + self.extra_names()
        self.timer = self.create_timer(self.T, self._step)     # sim time (use_sim_time:=true)
        self.get_logger().info(f'log: {self.log_path}')

    # ---- measurement -------------------------------------------------------------------------
    def _on_odom(self, msg):
        pp, tw = msg.pose.pose, msg.twist.twist
        psi = _yaw(pp.orientation)
        nu = np.array([tw.linear.x, tw.linear.y, tw.angular.z])      # body twist
        if self.x_meas is not None and np.abs(self.p.W @ nu).max() > 2 * self.omega_max:
            # implausible twist (gz yaw-rate glitch at +-180 deg): keep the previous velocities
            self.get_logger().warn(f'odometry twist rejected: {nu.round(3).tolist()}')
            vel = self.x_meas[3:]
        else:
            c, s = math.cos(psi), math.sin(psi)
            vel = np.array([c * nu[0] - s * nu[1], s * nu[0] + c * nu[1], nu[2]])
        self.x_meas = np.r_[pp.position.x, pp.position.y, psi, vel]
        if self.ref.anchor_pose is None:
            self.ref.anchor(self.x_meas[:3], self.start_offset)
            self._publish_path(self.ref_pub, self.ref.polyline(), latched=True)

    def _on_joints(self, msg):
        try:
            self.wheel_speeds = self.p.wheel_sign * np.array(
                [msg.velocity[msg.name.index(f'{w}_wheel_joint')] for w in robot_params.WHEELS])
        except (ValueError, IndexError):
            pass

    # ---- control step ---------------------------------------------------------------------------
    def _step(self):
        if self.x_meas is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            if min(p.get_subscription_count() for p in self.cmd_pubs) == 0:
                return                                   # torque bridge not connected yet
            self.t0 = now
        t = now - self.t0
        x_r, u_r = self.ref.horizon(t, self.T, self.N)
        x = self.x_meas.copy()
        u, info = self.mpc.step(x, x_r, u_r, self.u_applied)
        self._publish_torques(u)
        self.u_applied = u

        err_pos = math.hypot(x[0] - x_r[0, 0], x[1] - x_r[0, 1])
        err_psi = wrap(x[2] - x_r[0, 2])
        extra = [float(v) for v in np.hstack([np.ravel(v) for v in self.extra_values(info)])] \
            if self.EXTRA else []
        self.log.writerow([f'{t:.3f}', *np.round(x, 6), *np.round(x_r[0], 6), *np.round(u, 6),
                           f'{info["qp_ms"]:.3f}', f'{info["step_ms"]:.3f}', info['status']]
                          + [f'{v:.6g}' for v in extra])
        st = dict(t=t, err_pos=err_pos, err_psi=err_psi, qp_ms=info['qp_ms'],
                  step_ms=info['step_ms'], status=info['status'],
                  M1=u[0], M2=u[1], M3=u[2], M4=u[3], w1=self.wheel_speeds[0],
                  w2=self.wheel_speeds[1], w3=self.wheel_speeds[2], w4=self.wheel_speeds[3])
        msg = Float64MultiArray()
        msg.layout.dim.append(MultiArrayDimension(label=','.join(self.status_fields),
                                                  size=len(self.status_fields)))
        msg.data = [float(st[k]) for k in STATUS_FIELDS] + extra
        self.status_pub.publish(msg)
        if info['predicted'] is not None:
            self._publish_path(self.pred_pub, info['predicted'][:, :3])
        if info['status'] == STATUS_STOPPED:
            self.get_logger().error('optimiser failed 3 times in a row: zero torque, controller '
                                    'stopped')
            self._publish_torques(np.zeros(4))
            self.timer.cancel()
            self.log_file.flush()

    def _publish_torques(self, u):
        for pub, m, sign in zip(self.cmd_pubs, u, self.p.wheel_sign):
            pub.publish(Float64(data=float(sign * m)))

    def _publish_path(self, pub, pts, latched=False):
        path = Path()
        path.header.frame_id = 'odom'
        if not latched:
            path.header.stamp = self.get_clock().now().to_msg()
        for X, Y, psi in pts:
            ps = PoseStamped()
            ps.header.frame_id = 'odom'
            ps.pose.position.x, ps.pose.position.y = float(X), float(Y)
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(psi / 2), math.cos(psi / 2)
            path.poses.append(ps)
        pub.publish(path)

    def shutdown(self):
        self._publish_torques(np.zeros(4))
        self.log_file.close()


_stop_requested = False


def _request_stop(*_):
    global _stop_requested
    _stop_requested = True


def run_node(node_class):
    # SIGINT/SIGTERM only set a flag; the spin loop then ends and the zero torques are published
    # while the context is still up (ApplyJointForce keeps applying the last torque).
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)
    try:
        node = node_class()
    except SystemExit as e:                 # infeasible reference: refuse to start, name the limit
        print(f'[{node_class.NAME}] {e}', flush=True)
        rclpy.try_shutdown()
        raise
    try:
        while rclpy.ok() and not _stop_requested:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.shutdown()
        time.sleep(0.2)
        node.destroy_node()
        rclpy.try_shutdown()
