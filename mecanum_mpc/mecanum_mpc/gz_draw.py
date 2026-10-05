"""Draw a path on the floor of the running Gazebo world.

The path becomes a static model with only visuals: a ribbon of thin flat boxes lying on the
ground, plus a disc at the start. It has no collisions, so it does not affect the robot. Gazebo's
world frame equals the robot's /odom frame in this sim (gz OdometryPublisher reports world poses),
so the points are used as they are.
"""
import os
import subprocess
import tempfile
import threading

import numpy as np


def path_sdf(name, points, width=0.02, color=(1.0, 0.45, 0.0), height=0.002):
    """SDF of a static, visual-only model tracing the (n, 2) world points."""
    rgb = ' '.join(f'{c:.3f}' for c in color)
    material = (f'<material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse>'
                f'<emissive>{" ".join(f"{0.5 * c:.3f}" for c in color)} 1</emissive></material>')
    visuals = []
    for i, (a, b) in enumerate(zip(points[:-1], points[1:])):
        d = b - a
        length = float(np.hypot(*d))
        if length < 1e-6:
            continue
        mid = (a + b) / 2
        yaw = float(np.arctan2(d[1], d[0]))
        visuals.append(
            f'<visual name="seg{i}"><pose>{mid[0]:.4f} {mid[1]:.4f} {height / 2:.4f} 0 0 {yaw:.4f}</pose>'
            f'<geometry><box><size>{length + width:.4f} {width} {height}</size></box></geometry>'
            f'<cast_shadows>false</cast_shadows>{material}</visual>')
    sx, sy = points[0]
    visuals.append(   # start marker: green disc
        f'<visual name="start"><pose>{sx:.4f} {sy:.4f} {height:.4f} 0 0 0</pose>'
        f'<geometry><cylinder><radius>{2.5 * width}</radius><length>{2 * height}</length></cylinder></geometry>'
        f'<cast_shadows>false</cast_shadows><material><ambient>0 0.8 0 1</ambient>'
        f'<diffuse>0 0.8 0 1</diffuse><emissive>0 0.4 0 1</emissive></material></visual>')
    return (f'<?xml version="1.0"?><sdf version="1.8"><model name="{name}"><static>true</static>'
            f'<link name="path">{"".join(visuals)}</link></model></sdf>')


def spawn(sdf, name, world='default', logger=None):
    """Replace model `name` in the running gz world with `sdf`, in a background thread."""
    fd, path = tempfile.mkstemp(prefix=f'{name}_', suffix='.sdf')
    with os.fdopen(fd, 'w') as f:
        f.write(sdf)

    def work():
        try:
            # drop a path left over from an earlier run in the same sim (fails harmlessly if none)
            subprocess.run(['gz', 'service', '-s', f'/world/{world}/remove',
                            '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
                            '--timeout', '2000', '--req', f'name: "{name}" type: MODEL'],
                           capture_output=True, timeout=10)
            r = subprocess.run(['ros2', 'run', 'ros_gz_sim', 'create', '-world', world,
                                '-file', path, '-name', name],
                               capture_output=True, text=True, timeout=60)
            if logger is not None:
                if r.returncode == 0:
                    logger.info(f'path drawn in Gazebo as model "{name}"')
                else:
                    logger.warn(f'could not draw the path in Gazebo: {r.stderr.strip()[-300:]}')
        except Exception as e:  # never let drawing take the controller down
            if logger is not None:
                logger.warn(f'could not draw the path in Gazebo: {e}')
        finally:
            os.unlink(path)

    threading.Thread(target=work, daemon=True).start()
