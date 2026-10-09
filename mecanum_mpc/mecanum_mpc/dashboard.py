"""Live evaluation dashboard for mecanum_mpc (matplotlib window).

    ros2 run mecanum_mpc dashboard                       # or: mpc_sim.launch.py dashboard:=true
    ros2 run mecanum_mpc dashboard --ros-args -p window:=30.0
    ros2 run mecanum_mpc dashboard --ros-args -p snapshot:=/tmp/dash.png -p snapshot_after:=40.0

Subscribes /mpc/status (stats.py), /mpc/reference, /mpc/predicted. Time axes are the controller's
time since the path started (sim time). With `snapshot`, it renders off-screen, saves one PNG
after `snapshot_after` seconds of controller time and exits.
"""
from collections import deque
import math
import threading
import time

import numpy as np
import rclpy
import rclpy.executors
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64MultiArray

WHEEL_NAMES = ('FL', 'FR', 'RL', 'RR')
COLORS = ('#1f77b4', '#ff7f0e', '#2ca02c', '#d62728')
STATUS_TEXT = {0: 'solved', 1: 'inaccurate', 2: 'FALLBACK'}


class Dashboard(Node):
    def __init__(self):
        super().__init__('mpc_dashboard')
        self.window = self.declare_parameter('window', 20.0).value         # s of history shown
        self.snapshot = self.declare_parameter('snapshot', '').value
        self.snapshot_after = self.declare_parameter('snapshot_after', 30.0).value
        self.torque_max = self.declare_parameter('torque_max', 0.6).value
        self.wheel_speed_max = self.declare_parameter('wheel_speed_max', 30.0).value
        self.budget_ms = 1e3 * self.declare_parameter('dt', 0.05).value
        self.lock = threading.Lock()
        self._reset()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Float64MultiArray, '/mpc/status', self._on_status, 50)
        self.create_subscription(Path, '/mpc/reference', self._on_ref, latched)
        self.create_subscription(Path, '/mpc/predicted', self._on_pred, 10)

    def _reset(self):
        self.fields = None
        self.buf = {}                       # field -> deque of the last `window` seconds
        self.trail = deque(maxlen=20000)    # whole robot trail (x, y)
        self.all_err = []                   # (t, err) since start, for overall stats
        self.lap_rms = {}                   # lap -> (sum sq, n, max)
        self.counts = {0: 0, 1: 0, 2: 0}
        self.ref_xy = None
        self.pred_xy = None
        self.wall_t0 = None                 # for the real-time factor
        self.last_t = None

    # ---- ROS callbacks (executor thread) --------------------------------------------------------
    def _on_status(self, msg):
        names = msg.layout.dim[0].label.split(',') if msg.layout.dim else None
        if not names or len(names) != len(msg.data):
            return
        d = dict(zip(names, msg.data))
        with self.lock:
            if self.last_t is not None and d['t'] < self.last_t - 1.0:
                ref = self.ref_xy
                self._reset()                # controller restarted
                self.ref_xy = ref
            if self.fields is None:
                self.fields = names
                self.buf = {k: deque() for k in names}
                self.buf['wall'] = deque()
            wall = time.monotonic()
            for k in names:
                self.buf[k].append(d[k])
            self.buf['wall'].append(wall)
            while self.buf['t'] and self.buf['t'][0] < d['t'] - self.window:
                for q in self.buf.values():
                    q.popleft()
            self.trail.append((d['X'], d['Y']))
            self.all_err.append(d['err_pos'])
            lap = int(d['lap'])
            s, n, mx = self.lap_rms.get(lap, (0.0, 0, 0.0))
            self.lap_rms[lap] = (s + d['err_pos'] ** 2, n + 1, max(mx, d['err_pos']))
            self.counts[int(d['status'])] = self.counts.get(int(d['status']), 0) + 1
            self.last_t = d['t']

    def _on_ref(self, msg):
        with self.lock:
            self.ref_xy = np.array([(p.pose.position.x, p.pose.position.y) for p in msg.poses])

    def _on_pred(self, msg):
        with self.lock:
            self.pred_xy = np.array([(p.pose.position.x, p.pose.position.y) for p in msg.poses])

    # ---- drawing (main thread) -------------------------------------------------------------------
    def build_figure(self, plt):
        fig = plt.figure(figsize=(17, 9.5))
        fig.canvas.manager.set_window_title('mecanum_mpc live dashboard') \
            if hasattr(fig.canvas, 'manager') and fig.canvas.manager else None
        gs = fig.add_gridspec(3, 4, width_ratios=[1.25, 1, 1, 0.95], hspace=0.45, wspace=0.32,
                              left=0.04, right=0.99, top=0.95, bottom=0.06)
        ax = dict(
            map=fig.add_subplot(gs[0:2, 0]), err=fig.add_subplot(gs[0, 1]),
            head=fig.add_subplot(gs[0, 2]), force=fig.add_subplot(gs[1, 1]),
            pred=fig.add_subplot(gs[1, 2]), torque=fig.add_subplot(gs[2, 0]),
            speed=fig.add_subplot(gs[2, 1]), solver=fig.add_subplot(gs[2, 2]),
            text=fig.add_subplot(gs[:, 3]))
        ax['text'].axis('off')
        self.fig, self.ax = fig, ax
        return fig

    def draw(self):
        with self.lock:
            if self.fields is None or not self.buf['t']:
                return False
            b = {k: np.array(q) for k, q in self.buf.items()}
            trail = np.array(self.trail)
            ref, pred = self.ref_xy, self.pred_xy
            all_err = np.array(self.all_err)
            laps = dict(self.lap_rms)
            counts = dict(self.counts)
        t, ax = b['t'], self.ax
        for a in ax.values():
            if a is not ax['text']:
                a.cla()
                a.grid(True, alpha=0.3)

        # map: reference, trail, horizon, robot pose
        a = ax['map']
        if ref is not None:
            a.plot(ref[:, 0], ref[:, 1], '--', color='0.55', lw=1, label='reference')
        a.plot(trail[:, 0], trail[:, 1], color='#1f77b4', lw=1.2, label='robot')
        if pred is not None:
            a.plot(pred[:, 0], pred[:, 1], '.-', color='#ff7f0e', ms=3, lw=1, label='MPC prediction (1 s)')
        X, Y, psi = b['X'][-1], b['Y'][-1], b['psi'][-1]
        a.plot(b['X_ref'][-1], b['Y_ref'][-1], 'o', mfc='none', mec='k', ms=7, label='reference now')
        a.arrow(X, Y, 0.12 * math.cos(psi), 0.12 * math.sin(psi), width=0.012, color='#1f77b4')
        a.set_aspect('equal', adjustable='datalim')
        a.set_title('path (orange: what the MPC plans)')
        a.legend(fontsize=7, loc='upper right')

        # tracking error split along / across the path
        a = ax['err']
        a.plot(t, 1e3 * b['err_cross'], label='cross-track (left +)')
        a.plot(t, 1e3 * b['err_along'], label='along-track (ahead +)')
        a.plot(t, 1e3 * b['err_pos'], color='k', lw=0.5, alpha=0.6, label='|position error|')
        a.set_title('tracking error [mm]')
        a.legend(fontsize=7, loc='upper left')

        a = ax['head']
        a.plot(t, np.degrees(b['err_psi']), color='#9467bd', label='heading error [deg]')
        a.plot(t, 1e3 * b['err_speed'], color='#8c564b', lw=0.8, label='speed error [mm/s]')
        a.set_title('heading / speed error')
        a.legend(fontsize=7, loc='upper left')

        # forces: model feedforward vs MPC feedback
        a = ax['force']
        a.plot(t, b['F_xi_ff'], color='#1f77b4', label='F_xi feedforward')
        a.plot(t, b['F_xi_fb'], ':', color='#1f77b4', label='F_xi feedback')
        a.plot(t, b['F_eta_ff'], color='#ff7f0e', label='F_eta feedforward')
        a.plot(t, b['F_eta_fb'], ':', color='#ff7f0e', label='F_eta feedback')
        a.plot(t, b['coriolis'], color='#2ca02c', lw=0.9, label='|m_s v r, m_s u r| (non-holon.)')
        a.set_title('body forces [N]: model vs correction')
        a.legend(fontsize=6.5, loc='upper left', ncol=2)

        # model quality
        a = ax['pred']
        a.plot(t, 1e3 * b['pred1_pos'], label='1-step (50 ms) [mm]')
        a.plot(t, 1e3 * b['predN_pos'], label='1-horizon (1 s) [mm]')
        a.plot(t, 1e3 * b['slip'], color='0.4', lw=0.8, label='wheel slip [mm/s]')
        a.set_title('model vs reality: prediction error, slip')
        a.legend(fontsize=7, loc='upper left')

        # wheels
        a = ax['torque']
        for i in range(4):
            a.plot(t, b[f'M{i + 1}'], color=COLORS[i], lw=0.9, label=WHEEL_NAMES[i])
        lim = self.torque_max
        if np.abs([b[f'M{i + 1}'] for i in range(4)]).max() > 0.5 * lim:
            for s in (-1, 1):
                a.axhline(s * lim, color='r', ls='--', lw=1)
        a.set_title(f'wheel torques [N m] (limit ±{lim}; max use {100 * b["torque_util"].max():.1f} %)')
        a.legend(fontsize=7, ncol=4, loc='upper left')
        a.set_xlabel('t [s]')

        a = ax['speed']
        for i in range(4):
            a.plot(t, b[f'w{i + 1}'], color=COLORS[i], lw=0.9, label=WHEEL_NAMES[i])
        lim = self.wheel_speed_max
        if np.abs([b[f'w{i + 1}'] for i in range(4)]).max() > 0.5 * lim:
            for s in (-1, 1):
                a.axhline(s * lim, color='r', ls='--', lw=1)
        a.set_title(f'wheel speeds [rad/s] (limit ±{lim:g}; max use {100 * b["speed_util"].max():.0f} %)')
        a.legend(fontsize=7, ncol=4, loc='upper left')
        a.set_xlabel('t [s]')

        a = ax['solver']
        a.plot(t, b['qp_ms'], label='QP solve')
        a.plot(t, b['step_ms'], label='whole step')
        a.plot(t, b['period_ms'], color='0.6', lw=0.8, label='period (sim time)')
        bad = b['status'] > 0
        if bad.any():
            a.plot(t[bad], b['step_ms'][bad], 'rx', label='inaccurate / fallback')
        a.axhline(self.budget_ms, color='r', ls='--', lw=1, label=f'budget {self.budget_ms:.0f} ms')
        a.set_yscale('log')
        a.set_title('solver and timing [ms]')
        a.legend(fontsize=7, loc='upper left', ncol=2)
        a.set_xlabel('t [s]')

        # numbers
        def rms(e):
            return 1e3 * math.sqrt(np.mean(np.square(e))) if len(e) else float('nan')
        dt_wall = b['wall'][-1] - b['wall'][0]
        rtf = (t[-1] - t[0]) / dt_wall if dt_wall > 1.0 else float('nan')
        n = sum(counts.values())
        lines = [
            ('LIVE', None),
            ('controller time', f'{t[-1]:7.1f} s'), ('lap', f'{int(b["lap"][-1]) + 1}'),
            ('real-time factor', f'{rtf:.2f}'),
            ('', None), ('TRACKING', None),
            ('position error', f'{1e3 * b["err_pos"][-1]:6.1f} mm'),
            ('  cross / along', f'{1e3 * b["err_cross"][-1]:+5.1f} / {1e3 * b["err_along"][-1]:+5.1f} mm'),
            ('heading error', f'{np.degrees(b["err_psi"][-1]):+6.2f} deg'),
            (f'RMS last {self.window:g} s', f'{rms(b["err_pos"]):6.1f} mm'),
            (f'max last {self.window:g} s', f'{1e3 * b["err_pos"].max():6.1f} mm'),
            ('RMS since start', f'{rms(all_err):6.1f} mm'),
        ]
        for lap in sorted(laps)[-4:]:
            s, k, mx = laps[lap]
            lines.append((f'  lap {lap + 1}', f'{1e3 * math.sqrt(s / k):5.1f} RMS {1e3 * mx:5.1f} max'))
        lines += [
            ('', None), ('MODEL', None),
            ('1-step pred. error', f'{1e3 * np.nanmean(b["pred1_pos"]):6.2f} mm'),
            ('1-s pred. error', f'{1e3 * np.nanmean(b["predN_pos"]):6.1f} mm'),
            ('non-holonomic terms', f'{b["coriolis"].max():6.3f} N max'),
            ('feedforward |F|', f'{np.hypot(b["F_xi_ff"], b["F_eta_ff"]).max():6.3f} N max'),
            ('feedback |F|', f'{np.hypot(b["F_xi_fb"], b["F_eta_fb"]).max():6.3f} N max'),
            ('wheel slip', f'{1e3 * np.nanmean(b["slip"]):6.1f} mm/s'),
            ('', None), ('CONSTRAINTS', None),
            ('torque use', f'{100 * b["torque_util"][-1]:5.1f} %  (max {100 * b["torque_util"].max():.1f})'),
            ('wheel-speed use', f'{100 * b["speed_util"][-1]:5.1f} %  (max {100 * b["speed_util"].max():.1f})'),
            ('active in plan', f'{int(b["torque_active"][-1])} torque, {int(b["speed_active"][-1])} speed steps'),
            ('', None), ('SOLVER', None),
            ('QP p50 / p95', f'{np.percentile(b["qp_ms"], 50):.2f} / {np.percentile(b["qp_ms"], 95):.2f} ms'),
            ('step p95 / budget', f'{np.percentile(b["step_ms"], 95):.1f} / {self.budget_ms:.0f} ms'),
            ('iterations', f'{int(b["iters"][-1])}'),
            ('status now', STATUS_TEXT.get(int(b['status'][-1]), '?')),
            ('inaccurate / fallback', f'{counts.get(1, 0)} / {counts.get(2, 0)} of {n}'),
        ]
        a = ax['text']
        a.cla()
        a.axis('off')
        y = 1.0
        for k, v in lines:
            if v is None:
                a.text(0.0, y, k, fontsize=9.5, fontweight='bold', transform=a.transAxes, va='top')
            else:
                a.text(0.02, y, k, fontsize=8.5, transform=a.transAxes, va='top')
                a.text(0.98, y, v, fontsize=8.5, family='monospace', transform=a.transAxes,
                       va='top', ha='right')
            y -= 0.026
        return True


def main():
    rclpy.init()
    node = Dashboard()
    import matplotlib
    if node.snapshot:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    # ROS runs in a background thread, matplotlib needs the main thread
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    node.build_figure(plt)
    try:
        if node.snapshot:
            while node.last_t is None or node.last_t < node.snapshot_after:
                time.sleep(0.2)
            node.draw()
            node.fig.savefig(node.snapshot, dpi=80)
            node.get_logger().info(f'snapshot saved: {node.snapshot}')
            return
        plt.show(block=False)
        while plt.fignum_exists(node.fig.number):
            if node.draw():
                node.fig.canvas.draw_idle()
            plt.pause(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()          # stop spinning before tearing the node down
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
