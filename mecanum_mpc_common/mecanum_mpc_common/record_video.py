"""Overhead video of a Gazebo run (tooling only, not part of the controller).

    ros2 run mecanum_mpc_common record_video --ros-args -p use_sim_time:=true \\
        -p controller:=lmpc_v2 -p shape:=circle -p heading:=fixed -p frames_dir:=<dir>
                                                                       # start before the node
    ros2 run mecanum_mpc_common record_video encode <frames_dir> <run.csv> <out.mp4>

Record (ROS node, sim already running, controller not yet started):
  - waits for /odom and anchors the reference at the robot's pose, as the controller does;
  - spawns a static, visual-only line along the reference path and a static camera looking
    straight down; gz saves each camera frame as PNG (<save>) at FPS in sim time;
  - while the controller runs, pairs the number of saved frames with the path time t of
    /<controller>/status, so each frame can be given its path time; written to meta.json on exit.
Encode: frames -> H.264 mp4 at FPS (plays in real time of the sim), from 1 s before the path
  starts to 2 s after it ends, with a caption, the path time and the tracking error from the CSV.
"""
import json
import math
import os
import re
import signal
import subprocess
import sys

import numpy as np

from .conventions import TITLES

FPS = 30
WIDTH, HEIGHT = 1280, 720
CAM_HEIGHT = 20.0         # m above the ground; high and narrow, so the robot's height barely shifts it
MARGIN = 0.35             # m of floor around the path (robot half-size + some)
LINE_WIDTH = 0.01         # m, reference line on the floor
SEGMENT = 0.02            # m, length of one line piece
SHAPE_TITLES = {'circle': 'circle', 'ellipse': 'ellipse', 'figure8': 'figure-8',
                'square': 'square'}


# ---- SDF of the two spawned models -----------------------------------------------------------------
def path_sdf(P, name='mpc_reference_path'):
    """Static, visual-only model: thin boxes along the polyline P (n x 2), resampled by arc length."""
    s = np.r_[0.0, np.cumsum(np.hypot(*np.diff(P, axis=0).T))]
    q = np.c_[np.interp(np.arange(0.0, s[-1], SEGMENT), s, P[:, 0]),
              np.interp(np.arange(0.0, s[-1], SEGMENT), s, P[:, 1])]
    q = np.vstack([q, P[-1]])
    vis = []
    for i, (a, b) in enumerate(zip(q[:-1], q[1:])):
        m, d = (a + b) / 2, b - a
        vis.append(
            f'<visual name="s{i}"><pose>{m[0]:.4f} {m[1]:.4f} 0.0005 0 0 {math.atan2(d[1], d[0]):.4f}'
            f'</pose><geometry><box><size>{np.hypot(*d) + 0.002:.4f} {LINE_WIDTH} 0.001</size></box>'
            '</geometry><material><ambient>0.9 0.35 0.05 1</ambient><diffuse>0.9 0.35 0.05 1</diffuse>'
            '<emissive>0.6 0.2 0.02 1</emissive></material><cast_shadows>false</cast_shadows></visual>')
    return (f'<?xml version="1.0"?><sdf version="1.9"><model name="{name}"><static>true</static>'
            f'<link name="link">{"".join(vis)}</link></model></sdf>')


def camera_sdf(P, frames_dir, name='mpc_overhead_camera'):
    """Static camera above the middle of P, looking down; image right = +X, image up = +Y.
    Returns the SDF and the spawn pose (x, y, z, roll, pitch, yaw): `create` overrides the
    pose in the SDF with its own arguments."""
    lo, hi = P.min(axis=0) - MARGIN, P.max(axis=0) + MARGIN
    c = (lo + hi) / 2
    w = max(hi[0] - lo[0], (hi[1] - lo[1]) * WIDTH / HEIGHT)      # floor width in the image
    hfov = 2 * math.atan(w / 2 / CAM_HEIGHT)
    pose = (c[0], c[1], CAM_HEIGHT, 0.0, math.pi / 2, math.pi / 2)
    return (f'<?xml version="1.0"?><sdf version="1.9"><model name="{name}"><static>true</static>'
            '<link name="link"><sensor name="overhead" type="camera"><always_on>1</always_on>'
            f'<update_rate>{FPS}</update_rate><topic>mpc_record/overhead_camera</topic>'
            f'<camera><horizontal_fov>{hfov:.6f}</horizontal_fov><image><width>{WIDTH}</width>'
            f'<height>{HEIGHT}</height><format>R8G8B8</format></image>'
            '<clip><near>15</near><far>25</far></clip>'
            f'<save enabled="true"><path>{frames_dir}</path></save></camera></sensor></link>'
            '</model></sdf>'), pose


def frame_files(frames_dir):
    """Saved PNG frames in capture order."""
    f = [n for n in os.listdir(frames_dir) if n.endswith('.png')]
    return sorted(f, key=lambda n: int(re.findall(r'\d+', n)[-1]))


# ---- record --------------------------------------------------------------------------------------
_stop_requested = False


def _request_stop(*_):
    global _stop_requested
    _stop_requested = True


def record():
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import Float64MultiArray

    from . import params
    from .mpc_node import _yaw
    from .reference import Reference

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)
    node = Node('mpc_record_video')
    P_ = node.declare_parameter
    controller = P_('controller', 'lmpc_v2').value    # NAME of the controller node
    shape, heading = P_('shape', 'circle').value, P_('heading', 'fixed').value
    speed, size = P_('speed', 0.1).value, P_('size', 0.0).value
    frames_dir = os.path.abspath(P_('frames_dir', '').value)
    os.makedirs(frames_dir, exist_ok=True)
    urdf = os.path.join(get_package_share_directory('tb3_sim_bringup'),
                        'models', 'own_mecanum', 'robot.urdf')

    pose, pairs = [], []
    node.create_subscription(Odometry, 'odom', lambda m: pose.append(m.pose.pose) if not pose
                             else None, 10)
    while rclpy.ok() and not pose and not _stop_requested:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not pose:
        return
    pp = pose[0]
    ref = Reference(params.from_urdf(urdf), shape, size or None, speed, heading)
    ref.anchor((pp.position.x, pp.position.y, _yaw(pp.orientation)))
    P = ref.polyline(2000)[:, :2]
    for model, (sdf, pose6) in (('path', (path_sdf(P), (0,) * 6)),
                                ('camera', camera_sdf(P, frames_dir))):
        path = os.path.join(frames_dir, f'{model}.sdf')
        with open(path, 'w') as f:
            f.write(sdf)
        args = [a for k, v in zip(('-x', '-y', '-z', '-R', '-P', '-Y'), pose6) for a in (k, f'{v:.6f}')]
        subprocess.run(['ros2', 'run', 'ros_gz_sim', 'create', '-world', 'default', '-file', path,
                        *args, '--ros-args', '--log-level', 'warn'], check=True)
    # gz renders (and saves) a camera only while its topic has a subscriber: keep one open
    bridge = subprocess.Popen(
        ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
         '/mpc_record/overhead_camera@sensor_msgs/msg/Image[gz.msgs.Image',
         '--ros-args', '-r', '__node:=mpc_record_camera_bridge', '--log-level', 'warn'],
        start_new_session=True)
    node.get_logger().info(f'path line and camera spawned, frames in {frames_dir}')
    with open(os.path.join(frames_dir, 'spawned'), 'w'):
        pass

    def on_status(msg):          # path time t of the controller vs frames saved so far
        if len(pairs) == 0 or msg.data[0] - pairs[-1][1] >= 0.5:
            pairs.append((len(frame_files(frames_dir)), float(msg.data[0])))
    node.create_subscription(Float64MultiArray, f'/{controller}/status', on_status, 10)
    try:
        while rclpy.ok() and not _stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        with open(os.path.join(frames_dir, 'meta.json'), 'w') as f:
            json.dump(dict(controller=controller, shape=shape, heading=heading, speed=speed,
                           fps=FPS, pairs=pairs, T_end=ref.T_end), f)
        try:
            os.killpg(bridge.pid, signal.SIGINT)
            bridge.wait(timeout=5)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            os.killpg(bridge.pid, signal.SIGKILL)
        node.destroy_node()
        rclpy.try_shutdown()


# ---- encode --------------------------------------------------------------------------------------
def encode(frames_dir, csv_path, out_path, before=1.0, after=2.0):
    import cv2
    from PIL import Image, ImageDraw, ImageFont

    from .evaluate import load, wrap

    with open(os.path.join(frames_dir, 'meta.json')) as f:
        meta = json.load(f)
    fps = meta['fps']
    pairs = np.array(meta['pairs'])
    if len(pairs) < 2:
        raise SystemExit('no controller status was recorded, cannot place the frames in time')
    # frame k shows path time (k - k0) / fps; k0 from all (frames saved, t) pairs
    k0 = float(np.median(pairs[:, 0] - pairs[:, 1] * fps))
    d = load(csv_path)
    err = 1e3 * np.hypot(d['X'] - d['X_r'], d['Y'] - d['Y_r'])
    epsi = np.degrees(wrap(d['psi'] - d['psi_r']))
    t_end = meta['T_end']

    font_dir = '/usr/share/fonts/truetype/dejavu'
    big = ImageFont.truetype(os.path.join(font_dir, 'DejaVuSans-Bold.ttf'), 26)
    small = ImageFont.truetype(os.path.join(font_dir, 'DejaVuSans.ttf'), 22)
    mono = ImageFont.truetype(os.path.join(font_dir, 'DejaVuSansMono.ttf'), 22)
    title = (f'{TITLES.get(meta.get("controller", "lmpc_v2"))}  ·  '
             f'{SHAPE_TITLES.get(meta["shape"], meta["shape"])}  ·  '
             f'{meta["heading"]} heading  ·  {meta["speed"]:g} m/s')

    def box(draw, xy, text, font):
        l, t, r, b = draw.textbbox(xy, text, font=font)
        draw.rounded_rectangle((l - 10, t - 7, r + 10, b + 7), 6, fill=(20, 20, 20, 170))
        draw.text(xy, text, font=font, fill=(255, 255, 255, 255))

    files = frame_files(frames_dir)
    writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*'avc1'), fps, (WIDTH, HEIGHT))
    n = 0
    for k, name in enumerate(files):
        t = (k - k0) / fps
        if t < -before or t > t_end + after:
            continue
        im = Image.open(os.path.join(frames_dir, name)).convert('RGBA')
        ov = Image.new('RGBA', im.size, (0, 0, 0, 0))
        dr = ImageDraw.Draw(ov)
        box(dr, (24, 20), title, big)
        tt = min(max(t, 0.0), d['t'][-1])
        box(dr, (WIDTH - 190, 20), f't = {max(t, 0.0):5.1f} s', mono)
        box(dr, (24, HEIGHT - 48),
            f'tracking error {np.interp(tt, d["t"], err):5.2f} mm   '
            f'heading error {np.interp(tt, d["t"], epsi):+5.2f}°', small)
        box(dr, (WIDTH - 330, HEIGHT - 48), 'orange line: reference path', small)
        frame = Image.alpha_composite(im, ov).convert('RGB')
        writer.write(cv2.cvtColor(np.asarray(frame), cv2.COLOR_RGB2BGR))
        n += 1
    writer.release()
    print(f'{out_path}: {n} frames, {n / fps:.1f} s')


def main():
    if len(sys.argv) > 1 and sys.argv[1] == 'encode':
        encode(*sys.argv[2:5])
    else:
        record()


if __name__ == '__main__':
    main()
