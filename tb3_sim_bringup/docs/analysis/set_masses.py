# Set the weighed masses in models/own_mecanum/robot.urdf, every component homogeneous (2026-10-16).
#
#   python3 set_masses.py ../../models/own_mecanum/robot.urdf            # print only
#   python3 set_masses.py ../../models/own_mecanum/robot.urdf --write    # rewrite the <inertial>s
#
# Weighed: body without wheels 1927.4 g, one wheel (hub + 9 rollers) 51.65 g.
# - base_link: uniform density over meshes/chassis_visual.stl -> CoM = mesh centroid, inertia from
#   the mesh. The mesh has small gaps (4351 open edges), so it is integrated about its bbox centre;
#   volume, centroid and inertia change by < 0.5 % / 1 mm / 0.5 % with that reference point.
# - each wheel: one uniform density over the hub cylinder (r 21 mm, 33.6 mm long, the hub's only
#   geometry in the URDF) and the 9 rollers (meshes/roller.stl, closed) -> mass split by volume.
# Re-run with --write after copying a newer robot.urdf from Mecanum-TDK.
import argparse
import os
import re
import struct

import numpy as np

M_BODY, M_WHEEL = 1.9274, 0.05165
HUB_R, HUB_L = 0.021, 0.0336


def read_stl(path):
    data = open(path, 'rb').read()
    if data[:5] == b'solid' and b'facet' in data[:300]:
        v = [list(map(float, l.split()[1:4])) for l in data.decode().splitlines()
             if l.strip().startswith('vertex')]
        return np.array(v).reshape(-1, 3, 3)
    n = struct.unpack('<I', data[80:84])[0]
    rec = np.frombuffer(data[84:84 + 50 * n], dtype=np.dtype(
        [('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
    return rec['v'].astype(float)


def mass_properties(tris):
    """Volume, centroid and inertia about the centroid of a closed mesh at unit density
    (sum over signed tetrahedra origin-a-b-c)."""
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    vol6 = np.einsum('ij,ij->i', a, np.cross(b, c))
    V = vol6.sum() / 6
    com = (vol6[:, None] * (a + b + c)).sum(0) / 24 / V
    S = np.zeros((3, 3))                      # second moments int x_i x_j dV
    for i in range(3):
        for j in range(3):
            S[i, j] = (vol6 * (a[:, i] * a[:, j] + b[:, i] * b[:, j] + c[:, i] * c[:, j]
                               + (a[:, i] + b[:, i] + c[:, i]) * (a[:, j] + b[:, j] + c[:, j]))
                       ).sum() / 120
    S -= V * np.outer(com, com)
    return V, com, np.trace(S) * np.eye(3) - S


def g(x):
    return '0' if abs(x) < 1e-15 else f'{x:.6g}'    # drop round-off of exactly-zero terms


def inertial(m, I, origin=None):
    o = f'<origin xyz="{" ".join(g(v) for v in origin)}"/>' if origin is not None else ''
    return (f'<inertial>{o}<mass value="{g(m)}"/><inertia ixx="{g(I[0, 0])}" iyy="{g(I[1, 1])}" '
            f'izz="{g(I[2, 2])}" ixy="{g(I[0, 1])}" ixz="{g(I[0, 2])}" iyz="{g(I[1, 2])}"/>'
            f'</inertial>')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('urdf')
    ap.add_argument('--write', action='store_true')
    a = ap.parse_args()
    meshes = os.path.join(os.path.dirname(a.urdf), 'meshes')
    src = open(a.urdf).read()

    # body, in the mesh frame, then into base_link via the visual origin of base_link
    t = read_stl(os.path.join(meshes, 'chassis_visual.stl'))
    ref = (t.reshape(-1, 3).min(0) + t.reshape(-1, 3).max(0)) / 2
    V, com, I = mass_properties(t - ref)
    vis = re.search(r'<link name="base_link">\s*<visual><origin xyz="([^"]+)" rpy="([^"]+)"/>', src)
    xyz, (r, p, y) = np.array(vis.group(1).split(), float), np.array(vis.group(2).split(), float)
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    R = np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                  [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                  [-sp, cp * sr, cp * cr]])
    com_b = R @ (com + ref) + xyz
    body = inertial(M_BODY, R @ (I / V * M_BODY) @ R.T, com_b)

    # wheel: hub + 9 rollers, one density
    Vr, _, Ir = mass_properties(read_stl(os.path.join(meshes, 'roller.stl')))
    Vh = np.pi * HUB_R**2 * HUB_L
    rho = M_WHEEL / (Vh + 9 * Vr)
    mh = rho * Vh
    Ih = mh * np.diag([(3 * HUB_R**2 + HUB_L**2) / 12, HUB_R**2 / 2, (3 * HUB_R**2 + HUB_L**2) / 12])
    hub = inertial(mh, Ih)                    # wheel axis = link y
    roller = inertial(rho * Vr, rho * Ir)     # roller axis = link x, mesh centred on the link

    print(f'body: mesh volume {V * 1e6:.1f} cm^3, CoM in base_link {np.round(com_b * 1e3, 2)} mm')
    print(f'wheel: density {rho:.1f} kg/m^3, hub {mh * 1e3:.3f} g, roller {rho * Vr * 1e3:.4f} g x 9')
    print(f'total {M_BODY + 4 * M_WHEEL:.4f} kg\n{body}\n{hub}\n{roller}')

    counts = {}

    def repl(m):
        name = m.group(1)
        kind = ('body' if name == 'base_link' else 'roller' if '_roller_' in name
                else 'hub' if name.endswith('_wheel') else None)
        if kind is None:
            return m.group(0)
        counts[kind] = counts.get(kind, 0) + 1
        return re.sub(r'<inertial>.*?</inertial>', {'body': body, 'hub': hub, 'roller': roller}[kind],
                      m.group(0), flags=re.S)

    out = re.sub(r'<link name="([^"]+)">.*?</link>', repl, src, flags=re.S)
    assert counts == {'body': 1, 'hub': 4, 'roller': 36}, counts
    if a.write:
        open(a.urdf, 'w').write(out)
        print(f'written: {a.urdf}')
    elif out != src:
        print('(differs from the file: run with --write to apply)')
    else:
        print('(file already has these values)')


if __name__ == '__main__':
    main()
