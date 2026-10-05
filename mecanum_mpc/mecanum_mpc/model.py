"""Exact (Chaplygin) dynamics of a 4-wheel mecanum robot, after
I. Zeidis, K. Zimmermann, "Dynamics of a four-wheeled mobile robot with Mecanum wheels",
ZAMM 2019, doi:10.1002/zamm.201900173.

The paper's exact model, eq. (64), is written in wheel angular velocities. In body-frame
velocities it is equivalent (checked numerically, tb3_sim_bringup/docs/analysis/chaplygin_check.py) to

    me * du/dt - ms * v * r = F_xi
    me * dv/dt + ms * u * r = F_eta
    Je * dr/dt              = tau

with me = ms + 4 J1 / R^2 and Je = JC + 4 J1 L^2 / R^2, L = rho + l. Note the Coriolis terms use
the body mass ms, not me: that difference is the non-holonomic part.

State  x = [X, Y, psi, u, v, r]  (world pose, body-frame velocities)
Input  w = [F_xi, F_eta, tau]    (body wrench)

Wheel order and signs follow the paper (eqs. 10-11): 1 = front left, 2 = front right,
3 = rear left, 4 = rear right; R * dphi_i/dt = u + sv_i * v + sr_i * L * r.
"""
from dataclasses import dataclass
import xml.etree.ElementTree as ET

import numpy as np
from scipy.linalg import expm

WHEELS = ('front_left', 'front_right', 'rear_left', 'rear_right')
SV = np.array([-1.0, 1.0, 1.0, -1.0])   # sign of v in each wheel's rolling constraint
SR = np.array([-1.0, 1.0, -1.0, 1.0])   # sign of L*r


@dataclass
class Params:
    ms: float        # total mass [kg]
    JC: float        # yaw inertia of the whole robot about the wheel centre, without wheel spin [kg m^2]
    J1: float        # spin inertia of one wheel incl. rollers (and reflected motor inertia) [kg m^2]
    R: float         # effective rolling radius [m]
    lx: float        # half wheelbase (paper: rho) [m]
    ly: float        # half track (paper: l) [m]
    com_offset: tuple = (0.0, 0.0)   # CoM in base_link x/y; the model assumes (0, 0)

    @property
    def L(self):
        return self.lx + self.ly

    @property
    def me(self):
        return self.ms + 4.0 * self.J1 / self.R**2

    @property
    def Je(self):
        return self.JC + 4.0 * self.J1 * self.L**2 / self.R**2

    def wheel_jacobian(self):
        """dphi/dt = W @ [u, v, r]  (4x3)."""
        return np.c_[np.ones(4), SV, SR * self.L] / self.R

    def torque_to_wrench(self):
        """w = G @ M  (3x4); G = W^T by the principle of virtual power."""
        return self.wheel_jacobian().T

    def wrench_to_torque(self):
        """Minimum-norm wheel torques producing a given wrench: M = pinv(G) @ w  (4x3)."""
        return np.linalg.pinv(self.torque_to_wrench())


def f(x, w, p: Params):
    """Continuous-time dynamics dx/dt."""
    _, _, psi, u, v, r = x
    c, s = np.cos(psi), np.sin(psi)
    return np.array([
        c * u - s * v,
        s * u + c * v,
        r,
        (w[0] + p.ms * v * r) / p.me,
        (w[1] - p.ms * u * r) / p.me,
        w[2] / p.Je,
    ])


def jacobians(x, p: Params):
    """A = df/dx, B = df/dw of the continuous dynamics."""
    _, _, psi, u, v, r = x
    c, s = np.cos(psi), np.sin(psi)
    k = p.ms / p.me
    A = np.zeros((6, 6))
    A[0, 2], A[0, 3], A[0, 4] = -s * u - c * v, c, -s
    A[1, 2], A[1, 3], A[1, 4] = c * u - s * v, s, c
    A[2, 5] = 1.0
    A[3, 4], A[3, 5] = k * r, k * v
    A[4, 3], A[4, 5] = -k * r, -k * u
    B = np.zeros((6, 3))
    B[3, 0] = B[4, 1] = 1.0 / p.me
    B[5, 2] = 1.0 / p.Je
    return A, B


def rk4(x, w, dt, p: Params, substeps=1):
    """Discrete nonlinear step x_{k+1} = f_d(x_k, w_k), input held constant."""
    h = dt / substeps
    for _ in range(substeps):
        k1 = f(x, w, p)
        k2 = f(x + h / 2 * k1, w, p)
        k3 = f(x + h / 2 * k2, w, p)
        k4 = f(x + h * k3, w, p)
        x = x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return x


def discretize(x, p: Params, dt):
    """Zero-order-hold discretisation of the dynamics linearised at x: (Ad, Bd)."""
    A, B = jacobians(x, p)
    M = np.zeros((9, 9))
    M[:6, :6], M[:6, 6:] = A, B
    E = expm(M * dt)
    return E[:6, :6], E[:6, 6:]


# --------------------------------------------------------------------------------------------
# Parameters from the URDF


def _rpy_matrix(rpy):
    r, p_, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p_), np.sin(p_), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _origin(el):
    o = el.find('origin') if el is not None else None
    xyz = np.array([float(v) for v in o.get('xyz', '0 0 0').split()]) if o is not None else np.zeros(3)
    rpy = np.array([float(v) for v in o.get('rpy', '0 0 0').split()]) if o is not None else np.zeros(3)
    return xyz, _rpy_matrix(rpy)


def params_from_urdf(path, extra_wheel_inertia=0.0) -> Params:
    """Mass, inertias, wheel geometry and rolling radius of the own mecanum robot's URDF
    (models/own_mecanum/robot.urdf: base_link, <corner>_wheel, <corner>_roller_NN).

    extra_wheel_inertia: added to J1, e.g. a gearmotor's reflected rotor inertia
    (not in the URDF / simulation)."""
    root = ET.parse(path).getroot()
    links = {lk.get('name'): lk for lk in root.findall('link')}
    joints = {j.find('child').get('link'): j for j in root.findall('joint')}

    def pose(link):          # pose of a link in base_link at zero joint angles
        if link == 'base_link':
            return np.zeros(3), np.eye(3)
        j = joints[link]
        pt, pR = pose(j.find('parent').get('link'))
        t, R = _origin(j)
        return pt + pR @ t, pR @ R

    def inertial(link):      # mass, CoM and inertia tensor about the CoM, all in base_link
        el = links[link].find('inertial')
        if el is None:
            return 0.0, np.zeros(3), np.zeros((3, 3))
        m = float(el.find('mass').get('value'))
        a = el.find('inertia').attrib
        I = np.array([[float(a['ixx']), float(a.get('ixy', 0)), float(a.get('ixz', 0))],
                      [float(a.get('ixy', 0)), float(a['iyy']), float(a.get('iyz', 0))],
                      [float(a.get('ixz', 0)), float(a.get('iyz', 0)), float(a['izz'])]])
        lt, lR = pose(link)
        ct, cR = _origin(el)
        Rw = lR @ cR
        return m, lt + lR @ ct, Rw @ I @ Rw.T

    ms, mc, JC = 0.0, np.zeros(3), 0.0
    for name in links:
        m, c, I = inertial(name)
        ms += m
        mc += m * c
        JC += I[2, 2] + m * (c[0]**2 + c[1]**2)   # about the vertical axis through the origin

    J1s, radii, lxs, lys = [], [], [], []
    for wheel in WHEELS:
        jt = joints[f'{wheel}_wheel']
        wt, wR = pose(f'{wheel}_wheel')
        axis = wR @ np.array([float(v) for v in jt.find('axis').get('xyz').split()])
        axis /= np.linalg.norm(axis)
        lxs.append(abs(wt[0]))
        lys.append(abs(wt[1]))
        J1 = 0.0
        for name in [f'{wheel}_wheel'] + [n for n in links if n.startswith(f'{wheel}_roller_')]:
            m, c, I = inertial(name)
            d = c - wt
            J1 += axis @ I @ axis + m * (d @ d - (d @ axis)**2)
            if name == f'{wheel}_wheel':
                continue
            lt, lR = pose(name)
            for col in links[name].findall('collision'):
                sph = col.find('geometry/sphere')
                if sph is None:
                    continue
                ct, _ = _origin(col)
                d = lt + lR @ ct - wt
                radii.append(np.sqrt(d @ d - (d @ axis)**2) + float(sph.get('radius')))
        J1s.append(J1)

    return Params(ms=ms, JC=JC, J1=float(np.mean(J1s)) + extra_wheel_inertia,
                  R=float(max(radii)), lx=float(np.mean(lxs)), ly=float(np.mean(lys)),
                  com_offset=tuple((mc / ms)[:2]))
