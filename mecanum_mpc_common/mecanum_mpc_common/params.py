"""Robot parameters of the own mecanum robot, read from its URDF (plan: "Robot parameters and
conventions").

    m_s  total mass (body + 4 wheels)                       paper: m0 + 4 m1
    J_C  yaw inertia of the whole robot about C, no spin    paper: J0 + 4 (J2 + m1 (rho^2 + l^2))
    J1   spin inertia of one wheel incl. its rollers        paper: J1
    R    rolling radius (envelope of the roller spheres)    paper: R
    rho  half wheelbase, C to each axle                     paper: rho
    l    half track                                         paper: l

Point C = base_link origin in x/y (base_footprint, under the centre of the four wheels).
Wheel numbering (paper Fig. 1, eq. 10): 1 front-left, 2 front-right, 3 rear-left, 4 rear-right.
A positive wheel speed rolls the robot forward; a wheel whose joint axis points the other way
gets sign -1 (applied to its torque command and its measured speed).
"""
from dataclasses import dataclass, field
import xml.etree.ElementTree as ET

import numpy as np

WHEELS = ('front_left', 'front_right', 'rear_left', 'rear_right')   # paper wheels 1, 2, 3, 4
S_V = np.array([-1.0, 1.0, 1.0, -1.0])   # sign of V_Ceta in each wheel's rolling constraint (eq. 11)
S_R = np.array([-1.0, 1.0, -1.0, 1.0])   # sign of L * psi_dot
NULL_TORQUE = np.array([1.0, 1.0, -1.0, -1.0])   # n with G n = 0 (front pair against rear pair)


@dataclass
class RobotParams:
    m_s: float
    J_C: float
    J1: float
    R: float
    rho: float
    l: float
    wheel_sign: np.ndarray = field(default_factory=lambda: np.ones(4))

    @property
    def L(self):
        return self.rho + self.l

    @property
    def m_e(self):
        return self.m_s + 4.0 * self.J1 / self.R ** 2

    @property
    def J_e(self):
        return self.J_C + 4.0 * self.J1 * self.L ** 2 / self.R ** 2

    @property
    def kappa(self):
        return 1.0 - self.m_s / self.m_e

    @property
    def W(self):
        """Rolling constraints (eq. 11): wheel speeds = W @ [V_Cxi, V_Ceta, psi_dot]  (4x3)."""
        return np.c_[np.ones(4), S_V, S_R * self.L] / self.R

    @property
    def G(self):
        """Body wrench [F_xi, F_eta, tau] = G @ u, G = W^T  (3x4)."""
        return self.W.T

    @property
    def G_pinv(self):
        """Minimum-norm allocation u = G+ w  (4x3); G G^T = (4/R^2) diag(1, 1, L^2)."""
        return self.G.T @ np.diag([self.R ** 2 / 4, self.R ** 2 / 4, self.R ** 2 / (4 * self.L ** 2)])


def _rpy(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _origin(el):
    o = el.find('origin') if el is not None else None
    if o is None:
        return np.zeros(3), np.eye(3)
    xyz = np.array([float(v) for v in o.get('xyz', '0 0 0').split()])
    rpy = np.array([float(v) for v in o.get('rpy', '0 0 0').split()])
    return xyz, _rpy(rpy)


def from_urdf(path) -> RobotParams:
    """Parameters of models/own_mecanum/robot.urdf (links base_link, <corner>_wheel,
    <corner>_roller_NN; joints <corner>_wheel_joint)."""
    root = ET.parse(path).getroot()
    links = {lk.get('name'): lk for lk in root.findall('link')}
    joint_of = {j.find('child').get('link'): j for j in root.findall('joint')}

    def pose(link):                  # position and rotation of a link frame in base_link
        if link == 'base_link':
            return np.zeros(3), np.eye(3)
        j = joint_of[link]
        p_t, p_R = pose(j.find('parent').get('link'))
        t, R = _origin(j)
        return p_t + p_R @ t, p_R @ R

    def inertial(link):              # mass, CoM and inertia tensor about the CoM, in base_link
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

    m_s, J_C = 0.0, 0.0
    for name in links:               # every link, about the vertical axis through C
        m, c, I = inertial(name)
        m_s += m
        J_C += I[2, 2] + m * (c[0] ** 2 + c[1] ** 2)

    J1s, radii, rhos, ls, signs = [], [], [], [], []
    for wheel in WHEELS:
        wt, wR = pose(f'{wheel}_wheel')
        axis = wR @ np.array([float(v) for v in joint_of[f'{wheel}_wheel'].find('axis').get('xyz').split()])
        axis /= np.linalg.norm(axis)
        signs.append(1.0 if axis[1] > 0 else -1.0)   # +y axis: positive rotation rolls forward
        rhos.append(abs(wt[0]))
        ls.append(abs(wt[1]))
        J1 = 0.0
        for name in [f'{wheel}_wheel'] + [n for n in links if n.startswith(f'{wheel}_roller_')]:
            m, c, I = inertial(name)
            d = c - wt
            J1 += axis @ I @ axis + m * (d @ d - (d @ axis) ** 2)
            if name == f'{wheel}_wheel':
                continue
            lt, lR = pose(name)
            for col in links[name].findall('collision'):
                sph = col.find('geometry/sphere')
                if sph is not None:
                    ct, _ = _origin(col)
                    d = lt + lR @ ct - wt
                    radii.append(np.sqrt(d @ d - (d @ axis) ** 2) + float(sph.get('radius')))
        J1s.append(J1)

    return RobotParams(m_s=m_s, J_C=J_C, J1=float(np.mean(J1s)), R=float(max(radii)),
                       rho=float(np.mean(rhos)), l=float(np.mean(ls)), wheel_sign=np.array(signs))
