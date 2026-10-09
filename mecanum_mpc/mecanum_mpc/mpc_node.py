"""ROS 2 node: LTV-MPC path tracking for the own mecanum robot (sim: drive:=effort).

Subscribes  /odom (nav_msgs/Odometry; pose in odom, twist in base_footprint = body frame)
Publishes   /model/own_mecanum/joint/<corner>_wheel_joint/cmd_force  (std_msgs/Float64, N m)
            /mpc/reference  /mpc/predicted  (nav_msgs/Path, frame odom, for RViz)
            /mpc/tracking_error  (std_msgs/Float64, m)
            /mpc/status  (std_msgs/Float64MultiArray, all per-step statistics, see stats.py)
Gazebo      the reference path is drawn on the floor as model "mpc_reference_path" (draw_path)

On the first odometry message the chosen path is anchored at the robot's current pose (the path
starts there, heading along the robot's x axis), then tracked once (or `laps` times). Afterwards
the controller keeps holding the end pose until the node is stopped. With loop:=true the path is
driven over and over until the node is stopped, with a tracking report after every lap. On exit
the node sends zero torque.

Every step is logged to <log_dir>/<shape>_<heading>_<speed>_<time>.csv.
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

from . import gz_draw, model
from .stats import FIELDS, PredictionTracker, step_stats
from .ltv_mpc import LTVMPC, wrap
from .references import SHAPES, Reference


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class MPCNode(Node):
    def __init__(self):
        super().__init__('mecanum_mpc')
        default_urdf = os.path.join(get_package_share_directory('tb3_sim_bringup'),
                                    'models', 'own_mecanum', 'robot.urdf')
        P = self.declare_parameter
        self.shape = P('shape', 'circle').value
        self.heading = P('heading', 'fixed').value
        self.size = P('size', 0.0).value            # 0 = per-shape default
        self.speed = P('speed', 0.2).value
        self.laps = P('laps', 1.0).value              # ignored with loop:=true
        self.loop = P('loop', False).value            # drive the path until shutdown
        dt = P('dt', 0.05).value
        N = P('horizon', 20).value
        urdf = P('urdf', default_urdf).value
        extra_J1 = P('extra_wheel_inertia', 0.0).value
        self.friction = P('friction_torque', 0.003).value      # per-wheel Coulomb feedforward
        self.friction_eps = P('friction_speed_eps', 0.5).value  # rad/s, tanh smoothing
        torque_max = P('torque_max', 0.6).value
        wheel_speed_max = P('wheel_speed_max', 30.0).value
        lin = P('linearize_about', 'reference').value
        Q = P('Q', [1000.0, 1000.0, 250.0, 5.0, 5.0, 1.0]).value
        R = P('R', [0.01, 0.01, 1.0]).value
        Rd = P('Rd', [0.1, 0.1, 10.0]).value
        self.robot = P('model_name', 'own_mecanum').value
        self.draw_path = P('draw_path', True).value      # ribbon on the floor of the gz world
        self.world = P('world_name', 'default').value
        log_dir = P('log_dir', os.path.expanduser('~/ros2_ws/mpc_logs')).value
        if self.shape not in SHAPES:
            raise ValueError(f'shape must be one of {SHAPES}')

        self.p = model.params_from_urdf(urdf, extra_wheel_inertia=extra_J1)
        self.get_logger().info(
            f'model from {urdf}: ms={self.p.ms:.3f} kg, me={self.p.me:.3f} kg, Je={self.p.Je:.4f}, '
            f'R={self.p.R:.5f}, lx={self.p.lx}, ly={self.p.ly}, CoM offset '
            f'({self.p.com_offset[0]*1e3:.1f}, {self.p.com_offset[1]*1e3:.1f}) mm')
        self.mpc = LTVMPC(self.p, dt=dt, N=N, Q=Q, R=R, Rd=Rd, torque_max=torque_max,
                          wheel_speed_max=wheel_speed_max, linearize_about=lin)
        self.dt, self.N = dt, N
        sizes = {'circle': 0.75, 'ellipse': 1.0, 'figure8': 1.0, 'square': 1.5}
        self.ref = Reference(self.p, self.shape, self.size or sizes[self.shape], speed=self.speed,
                             laps=self.laps, heading=self.heading, loop=self.loop)
        if self.ref.max_wheel_speed > wheel_speed_max:
            self.get_logger().warn(
                f'reference needs {self.ref.max_wheel_speed:.1f} rad/s wheel speed > limit '
                f'{wheel_speed_max} rad/s: it is not feasible, expect tracking errors')
        self.get_logger().info(
            f'{self.shape} ({self.heading} heading), length {self.ref.length:.2f} m, '
            f'{self.speed} m/s, ' + ('looping until shutdown, ' if self.loop else
                                     f'duration {self.ref.duration:.1f} s, ') +
            f'max wheel speed {self.ref.max_wheel_speed:.1f} rad/s')

        self.pubs = [self.create_publisher(
            Float64, f'/model/{self.robot}/joint/{w}_wheel_joint/cmd_force', 10)
            for w in model.WHEELS]
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ref_pub = self.create_publisher(Path, '/mpc/reference', latched)
        self.pred_pub = self.create_publisher(Path, '/mpc/predicted', 10)
        self.err_pub = self.create_publisher(Float64, '/mpc/tracking_error', 10)
        self.status_pub = self.create_publisher(Float64MultiArray, '/mpc/status', 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_subscription(JointState, 'joint_states', self._on_joints, 10)
        self.joint_speeds = None                         # real wheel speeds (sim), for slip
        self.pred_tracker = PredictionTracker(N)
        self.last_step = None

        self.x = None
        self.rejected = 0
        self.anchor = None          # (X0, Y0, psi0) of the path frame in odom
        self.t0 = None
        self.done_reported = False
        self.lap = 0
        self.errs = []
        os.makedirs(log_dir, exist_ok=True)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        self.log_path = os.path.join(
            log_dir, f'{self.shape}_{self.heading}_{self.speed:.2f}_{stamp}.csv')
        self.log_file = open(self.log_path, 'w', newline='')
        self.log = csv.writer(self.log_file)
        self.log.writerow(['t', 'X', 'Y', 'psi', 'u', 'v', 'r',
                           'X_ref', 'Y_ref', 'psi_ref', 'u_ref', 'v_ref', 'r_ref',
                           'F_xi', 'F_eta', 'tau', 'M1', 'M2', 'M3', 'M4',
                           'qp_ms', 'step_ms', 'status'])
        self.timer = self.create_timer(dt, self._step)     # sim time (use_sim_time:=true)

    # ---- frames: the reference lives in the path frame, the robot state in odom ---------------
    def _to_odom(self, xr):
        X0, Y0, p0 = self.anchor
        c, s = math.cos(p0), math.sin(p0)
        out = xr.copy()
        out[..., 0] = X0 + c * xr[..., 0] - s * xr[..., 1]
        out[..., 1] = Y0 + s * xr[..., 0] + c * xr[..., 1]
        out[..., 2] = xr[..., 2] + p0           # body velocities are frame-independent
        return out

    def _on_odom(self, msg):
        pp, tw = msg.pose.pose, msg.twist.twist
        x = np.array([pp.position.x, pp.position.y, _yaw(pp.orientation),
                      tw.linear.x, tw.linear.y, tw.angular.z])
        # gz OdometryPublisher occasionally reports one yaw-rate sample of about +-2*pi/0.01 s
        # (an angle wrap in its differentiation). Reject velocities the wheels cannot produce
        # (> 2x the wheel-speed limit) and keep the previous ones.
        if self.x is not None and np.abs(self.p.wheel_jacobian() @ x[3:]).max() > \
                2 * self.mpc.wheel_speed_max:
            self.rejected += 1
            x[3:] = self.x[3:]
            if self.rejected <= 5:
                self.get_logger().warn(
                    f'implausible odometry twist ({tw.linear.x:.3f}, {tw.linear.y:.3f}, {tw.angular.z:.1f}) '
                    f'rejected ({self.rejected} so far)')
        self.x = x
        if self.anchor is None:
            self.anchor = tuple(self.x[:3])
            self._publish_reference()
            if self.draw_path:
                pts = self._to_odom(self.ref.polyline(1800 if self.loop else 600))[:, :2]
                gz_draw.spawn(gz_draw.path_sdf('mpc_reference_path', pts), 'mpc_reference_path',
                              self.world, self.get_logger())

    def _on_joints(self, msg):
        try:
            self.joint_speeds = np.array(
                [msg.velocity[msg.name.index(f'{w}_wheel_joint')] for w in model.WHEELS])
        except (ValueError, IndexError):
            pass

    def _publish_reference(self):
        path = Path()
        path.header.frame_id = 'odom'
        for X, Y, psi in self._to_odom(self.ref.polyline()):
            ps = PoseStamped()
            ps.header.frame_id = 'odom'
            ps.pose.position.x, ps.pose.position.y = float(X), float(Y)
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(psi / 2), math.cos(psi / 2)
            path.poses.append(ps)
        self.ref_pub.publish(path)

    # ---- control step --------------------------------------------------------------------------
    def _step(self):
        if self.x is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            if min(p.get_subscription_count() for p in self.pubs) == 0:
                return                                   # torque bridge not connected yet
            self.t0 = now
        t = now - self.t0
        xr, wr = self.ref.sample(t + self.dt * np.arange(self.N + 1))
        xr = self._to_odom(xr)
        x = self.x.copy()
        x[2] = xr[0, 2] + wrap(x[2] - xr[0, 2])         # unwrap measured heading onto the reference
        w, info = self.mpc.solve(x, xr, wr[:self.N])

        # Coulomb friction feedforward per wheel, smooth sign of the REFERENCE wheel speed (the
        # planned speed flips sign around standstill and makes this chatter)
        wheel_speed = self.p.wheel_jacobian() @ xr[1, 3:]
        M = info['torques'] + self.friction * np.tanh(wheel_speed / self.friction_eps)
        M = np.clip(M, -self.mpc.torque_max, self.mpc.torque_max)
        for pub, m in zip(self.pubs, M):
            pub.publish(Float64(data=float(m)))

        err = math.hypot(x[0] - xr[0, 0], x[1] - xr[0, 1])
        self.err_pub.publish(Float64(data=err))
        self._publish_status(t, now, x, xr[0], wr[0], info, M)
        self.log.writerow([f'{t:.3f}', *np.round(x, 5), *np.round(xr[0], 5), *np.round(w, 5),
                           *np.round(M, 5), f"{info['solve_ms']:.3f}", f"{info['total_ms']:.3f}",
                           info['status']])
        if self.loop:
            lap = self.ref.lap(t)
            if lap != self.lap:                          # report the lap just finished
                e = np.array(self.errs)
                self.get_logger().info(
                    f'lap {self.lap + 1} done: tracking error RMS {1e3*np.sqrt(np.mean(e**2)):.1f} mm, '
                    f'max {1e3*e.max():.1f} mm, {self.rejected} odometry samples rejected so far')
                self.lap, self.errs = lap, []
                self.log_file.flush()
            self.errs.append(err)
        elif t <= self.ref.duration:
            self.errs.append(err)
        elif not self.done_reported:
            self.done_reported = True
            e = np.array(self.errs)
            self.get_logger().info(
                f'path done: tracking error RMS {1e3*np.sqrt(np.mean(e**2)):.1f} mm, '
                f'max {1e3*e.max():.1f} mm, final {1e3*err:.1f} mm, '
                f'{self.rejected} odometry samples rejected. Holding the end pose; '
                f'log: {self.log_path}')
            self.log_file.flush()

        pred = Path()
        pred.header.frame_id = 'odom'
        pred.header.stamp = self.get_clock().now().to_msg()
        for X, Y, _psi, *_ in info['predicted']:
            ps = PoseStamped()
            ps.header.frame_id = 'odom'
            ps.pose.position.x, ps.pose.position.y = float(X), float(Y)
            pred.poses.append(ps)
        self.pred_pub.publish(pred)

    def _publish_status(self, t, now, x, xr0, wr0, info, M):
        info['M'] = M
        kin = self.p.wheel_jacobian() @ x[3:]           # wheel speeds the rolling constraint implies
        wheels = self.joint_speeds if self.joint_speeds is not None else kin
        st = step_stats(self.p, self.mpc, x, xr0, wr0, info, wheels)
        st.update(self.pred_tracker.update(x, info['predicted']))
        st.update(t=t, lap=self.ref.lap(t) if self.loop else 0,
                  period_ms=1e3 * (now - self.last_step) if self.last_step else np.nan,
                  slip=self.p.R * float(np.sqrt(np.mean((wheels - kin) ** 2))))
        self.last_step = now
        msg = Float64MultiArray()
        msg.layout.dim.append(MultiArrayDimension(label=','.join(FIELDS), size=len(FIELDS)))
        msg.data = [float(st[k]) for k in FIELDS]
        self.status_pub.publish(msg)

    def stop(self):
        for pub in self.pubs:
            pub.publish(Float64(data=0.0))
        self.log_file.close()


def _raise_keyboard_interrupt(*_):
    raise KeyboardInterrupt


def main():
    # Handle SIGINT/SIGTERM ourselves: with rclpy's handler the context is already shut down when
    # we get control back, and the zero torques could no longer be published. ApplyJointForce
    # keeps applying the last torque, so the robot would keep accelerating.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _raise_keyboard_interrupt)   # also if inherited as ignored
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    node = MPCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        time.sleep(0.2)          # let the zero torques go out
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
