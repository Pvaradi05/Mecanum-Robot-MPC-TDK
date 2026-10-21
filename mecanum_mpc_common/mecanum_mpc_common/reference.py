"""Reference trajectory and reference torques: the virtual reference robot of Kuehne et al. (eq. 4)
with the model of dynamics.py (plan: "Reference trajectory and reference torques").

Paths (local path frame: start at the origin heading along +x, counter-clockwise):
    circle    radius 0.75 m
    ellipse   semi-axes 1.0 x 0.5 m
    figure8   lemniscate of Gerono, 2 x 1 m, rotated to start along +x
    square    side 1.5 m, corner radius r_c > 0 (default 15 % of the side)
Timing: cosine ramp up over ramp_time, constant speed, cosine ramp down; then the end pose is
held. loop=True: ramp up, then the path repeats at constant speed forever.
Heading: 'fixed' (psi stays at its start value) or 'tangent' (psi = direction of travel, raw,
not smoothed - decided 2026-10-21).

Reference torques: inverse dynamics, w_r = [F_xi, F_eta, tau]
    [F_xi, F_eta] = m_e R(psi)^T (a - kappa psi_dot S v),   tau = J_e psi_ddot
and minimum-norm allocation u_r = G+ w_r. Only step averages of u_r are used (decision D3):
    u_r,j = (U(t + (j+1)T) - U(t + jT)) / T,     U(t) = G+ int_0^t w_r
The force part of U is integrated on a 2 ms grid. The torque part is exact,
int J_e psi_ddot = J_e (psi_dot(t) - psi_dot(0)), so the turn-rate jumps of the raw tangent
heading (square line/arc joints) give finite step averages.
"""
import numpy as np

from .params import RobotParams

SHAPES = ('circle', 'ellipse', 'figure8', 'square')
DEFAULT_SIZE = {'circle': 0.75, 'ellipse': 1.0, 'figure8': 1.0, 'square': 1.5}
S_MAT = np.array([[0.0, -1.0], [1.0, 0.0]])


def _rot(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s], [s, c]])


class _ThetaCurve:
    """Closed curve p(theta), theta in [0, 2 pi], with analytic first and second derivatives;
    evaluated by arc length through a dense theta(s) table."""

    def __init__(self, p, dp, ddp, n=200001):
        self.p, self.dp, self.ddp = p, dp, ddp
        th = np.linspace(0.0, 2 * np.pi, n)
        speed = np.linalg.norm(dp(th), axis=1)
        s = np.r_[0.0, np.cumsum((speed[1:] + speed[:-1]) / 2 * np.diff(th))]
        self.th_tab, self.s_tab = th, s
        self.length = float(s[-1])

    def at(self, s):
        """Position, unit tangent, curvature at arc lengths s in [0, length]."""
        th = np.interp(s, self.s_tab, self.th_tab)
        d1, d2 = self.dp(th), self.ddp(th)
        n1 = np.linalg.norm(d1, axis=1)
        tang = d1 / n1[:, None]
        curv = (d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) / n1 ** 3
        return self.p(th), tang, curv


class _Square:
    """Square of side a with rounded corners of radius rc, by arc length; starts at the middle of
    the bottom side heading +x, counter-clockwise."""

    def __init__(self, a, rc):
        if not 0.0 < rc <= a / 2:
            raise ValueError(f'corner radius must be in (0, {a / 2}] m, got {rc}')
        self.rc = rc
        e = a - 2 * rc
        self.segs = []                       # (s_start, length, kind, start point, heading)
        s, pos, head = 0.0, np.zeros(2), 0.0
        for length, kind in [(e / 2, 'line')] + [(np.pi / 2 * rc, 'arc'), (e, 'line')] * 3 + \
                [(np.pi / 2 * rc, 'arc'), (e / 2, 'line')]:
            if length > 0:
                self.segs.append((s, length, kind, pos.copy(), head))
            if kind == 'line':
                pos = pos + length * np.array([np.cos(head), np.sin(head)])
            else:
                ctr = pos + rc * np.array([-np.sin(head), np.cos(head)])
                head += np.pi / 2
                pos = ctr + rc * np.array([np.sin(head), -np.cos(head)])
            s += length
        self.length = s

    def at(self, s):
        s = np.atleast_1d(s)
        P, Tg, K = np.zeros((len(s), 2)), np.zeros((len(s), 2)), np.zeros(len(s))
        for s0, length, kind, p0, h0 in self.segs:
            m = (s >= s0) & (s <= s0 + length)
            q = s[m] - s0
            if kind == 'line':
                P[m] = p0 + np.outer(q, [np.cos(h0), np.sin(h0)])
                Tg[m] = [np.cos(h0), np.sin(h0)]
                K[m] = 0.0
            else:
                ctr = p0 + self.rc * np.array([-np.sin(h0), np.cos(h0)])
                h = h0 + q / self.rc
                P[m] = ctr + self.rc * np.c_[np.sin(h), -np.cos(h)]
                Tg[m] = np.c_[np.cos(h), np.sin(h)]
                K[m] = 1.0 / self.rc
        return P, Tg, K


def make_curve(shape, size, corner_radius=None):
    if shape == 'circle':
        r = size
        return _ThetaCurve(lambda t: np.c_[r * np.sin(t), r * (1 - np.cos(t))],
                           lambda t: np.c_[r * np.cos(t), r * np.sin(t)],
                           lambda t: np.c_[-r * np.sin(t), r * np.cos(t)])
    if shape == 'ellipse':
        a, b = size, size / 2
        return _ThetaCurve(lambda t: np.c_[a * np.sin(t), b * (1 - np.cos(t))],
                           lambda t: np.c_[a * np.cos(t), b * np.sin(t)],
                           lambda t: np.c_[-a * np.sin(t), b * np.cos(t)])
    if shape == 'figure8':
        a = size
        Rm = _rot(-np.pi / 4)                 # start tangent of (a sin t, a sin t cos t) is 45 deg
        return _ThetaCurve(lambda t: np.c_[a * np.sin(t), a * np.sin(t) * np.cos(t)] @ Rm.T,
                           lambda t: np.c_[a * np.cos(t), a * np.cos(2 * t)] @ Rm.T,
                           lambda t: np.c_[-a * np.sin(t), -2 * a * np.sin(2 * t)] @ Rm.T)
    if shape == 'square':
        return _Square(size, 0.15 * size if corner_radius is None else corner_radius)
    raise ValueError(f'unknown shape {shape!r}, expected one of {SHAPES}')


class Reference:
    """Timed reference x_r(t) (world frame once anchored) and step-averaged reference torques."""

    def __init__(self, p: RobotParams, shape='circle', size=None, speed=0.1, heading='fixed',
                 loop=False, ramp_time=2.0, corner_radius=None, grid_dt=0.002):
        if heading not in ('fixed', 'tangent'):
            raise ValueError("heading must be 'fixed' or 'tangent'")
        self.p, self.shape, self.heading, self.loop = p, shape, heading, loop
        self.V, self.Tr = float(speed), float(ramp_time)
        self.curve = make_curve(shape, size or DEFAULT_SIZE[shape], corner_radius)
        self.length = self.curve.length
        if not loop and self.length < self.V * self.Tr:
            raise ValueError('path too short for the speed ramps')
        self.T_lap = self.length / self.V
        self.Tc = (self.length - self.V * self.Tr) / self.V        # cruise time, one lap
        self.T_end = np.inf if loop else 2 * self.Tr + self.Tc

        # unwrapped tangent heading along one lap (dense in s), and its gain per lap
        s_tab = np.linspace(0.0, self.length, 100001)
        _, tg, _ = self.curve.at(s_tab)
        self._s_tab = s_tab
        self._psi_tab = np.unwrap(np.arctan2(tg[:, 1], tg[:, 0]))
        self.dpsi_lap = float(self._psi_tab[-1] - self._psi_tab[0]) if heading == 'tangent' else 0.0

        # running integral of the force part of the reference wrench, on the grid
        t_tab = np.arange(0.0, (self.Tr + self.T_lap if loop else self.T_end) + grid_dt, grid_dt)
        F = self._force(t_tab)
        self._t_tab = t_tab
        self._UF_tab = np.r_[np.zeros((1, 2)), np.cumsum((F[1:] + F[:-1]) / 2 * grid_dt, axis=0)]
        if loop:
            self._UF_lap = self._UF_interp(self.Tr + self.T_lap) - self._UF_interp(self.Tr)
        self._psidot0 = self._path_state(np.array([0.0]))[0][0, 5]
        self.anchor_pose = None

    # ---- time law s(t) ------------------------------------------------------------------------
    def _s(self, t):
        t = np.asarray(t, dtype=float)
        V, Tr = self.V, self.Tr
        s, sd, sdd = np.zeros_like(t), np.zeros_like(t), np.zeros_like(t)
        up = t < Tr
        s[up] = V / 2 * (t[up] - Tr / np.pi * np.sin(np.pi * t[up] / Tr))
        sd[up] = V / 2 * (1 - np.cos(np.pi * t[up] / Tr))
        sdd[up] = V * np.pi / (2 * Tr) * np.sin(np.pi * t[up] / Tr)
        cruise = (t >= Tr) & (self.loop | (t < Tr + self.Tc))
        s[cruise] = V * Tr / 2 + V * (t[cruise] - Tr)
        sd[cruise] = V
        if not self.loop:
            down = (t >= Tr + self.Tc) & (t < self.T_end)
            q = t[down] - Tr - self.Tc
            s[down] = V * Tr / 2 + V * self.Tc + V / 2 * (q + Tr / np.pi * np.sin(np.pi * q / Tr))
            sd[down] = V / 2 * (1 + np.cos(np.pi * q / Tr))
            sdd[down] = -V * np.pi / (2 * Tr) * np.sin(np.pi * q / Tr)
            end = t >= self.T_end
            s[end] = self.length
        return s, sd, sdd

    # ---- state and acceleration in the path frame -----------------------------------------------
    def _path_state(self, t):
        s, sd, sdd = self._s(t)
        lap = np.floor(s / self.length)
        sm = s - lap * self.length
        lap[(sm <= 0) & (lap > 0)] -= 1                     # s exactly at a lap end: stay on it
        sm = s - lap * self.length
        P, tg, curv = self.curve.at(sm)
        nrm = tg @ S_MAT.T
        vel = tg * sd[:, None]
        acc = tg * sdd[:, None] + nrm * (curv * sd ** 2)[:, None]
        if self.heading == 'tangent':
            psi_i = np.interp(sm, self._s_tab, self._psi_tab)
            psi = psi_i + (np.arctan2(tg[:, 1], tg[:, 0]) - psi_i + np.pi) % (2 * np.pi) - np.pi
            psi = psi + lap * self.dpsi_lap
            psid = curv * sd
        else:
            psi, psid = np.zeros_like(s), np.zeros_like(s)
        x = np.c_[P, psi, vel, psid]
        return x, acc

    def _force(self, t):
        """[F_xi, F_eta] of the inverse dynamics (body frame, independent of the anchor)."""
        x, acc = self._path_state(t)
        psi, v, psid = x[:, 2], x[:, 3:5], x[:, 5]
        rhs = acc - self.p.kappa * psid[:, None] * (v @ S_MAT.T)
        c, s = np.cos(psi), np.sin(psi)
        return self.p.m_e * np.c_[c * rhs[:, 0] + s * rhs[:, 1], -s * rhs[:, 0] + c * rhs[:, 1]]

    def _UF_interp(self, t):
        return np.c_[np.interp(t, self._t_tab, self._UF_tab[:, 0]),
                     np.interp(t, self._t_tab, self._UF_tab[:, 1])]

    def _UF(self, t):
        t = np.atleast_1d(np.asarray(t, dtype=float))
        if not self.loop:
            return self._UF_interp(np.minimum(t, self._t_tab[-1]))
        k = np.maximum(0.0, np.floor((t - self.Tr) / self.T_lap))
        return self._UF_interp(t - k * self.T_lap) + k[:, None] * self._UF_lap

    def _U(self, t):
        """U(t) = G+ int_0^t w_r  (n x 4)."""
        t = np.atleast_1d(np.asarray(t, dtype=float))
        UF = self._UF(t)
        Utau = self.p.J_e * (self._path_state(t)[0][:, 5] - self._psidot0)
        return np.c_[UF, Utau] @ self.p.G_pinv.T

    # ---- public --------------------------------------------------------------------------------
    def anchor(self, robot_pose, start_offset=(0.0, 0.0, 0.0)):
        """Place the path frame so that the robot's pose = path start (+) start_offset."""
        X, Y, psi = robot_pose
        dx, dy, dpsi = start_offset
        psi0 = psi - dpsi
        X0, Y0 = np.array([X, Y]) - _rot(psi0) @ np.array([dx, dy])
        self.anchor_pose = (float(X0), float(Y0), float(psi0))

    def states(self, t):
        """x_r at times t (n x 6), world frame (anchored)."""
        x, _ = self._path_state(np.atleast_1d(np.asarray(t, dtype=float)))
        X0, Y0, psi0 = self.anchor_pose
        Rm = _rot(psi0)
        out = x.copy()
        out[:, 0:2] = np.array([X0, Y0]) + x[:, 0:2] @ Rm.T
        out[:, 2] = x[:, 2] + psi0
        out[:, 3:5] = x[:, 3:5] @ Rm.T
        return out

    def step_torques(self, t, T, N):
        """u_r,j = mean of u_r over [t + jT, t + (j+1)T], j = 0..N-1  (N x 4)."""
        U = self._U(t + T * np.arange(N + 1))
        return np.diff(U, axis=0) / T

    def horizon(self, t, T, N):
        """x_r,j (j = 0..N) and u_r,j (j = 0..N-1) for the step starting at t."""
        return self.states(t + T * np.arange(N + 1)), self.step_torques(t, T, N)

    def check_limits(self, T, u_max, du_max, omega_max):
        """Largest |u_r|, step-to-step change of u_r, and wheel speed over the whole path (or the
        ramp + 2 laps in loop mode), on the control grid; returns (dict of maxima, list of
        violations)."""
        t_end = self.Tr + 2 * self.T_lap if self.loop else self.T_end + T
        tk = np.arange(0.0, t_end, T)
        ur = self.step_torques_at(tk, T)
        du = np.abs(np.diff(np.r_[np.zeros((1, 4)), ur], axis=0))   # first step from rest
        x, _ = self._path_state(tk)
        c, s = np.cos(x[:, 2]), np.sin(x[:, 2])
        nu = np.c_[c * x[:, 3] + s * x[:, 4], -s * x[:, 3] + c * x[:, 4], x[:, 5]]
        wheel = np.abs(nu @ self.p.W.T)
        m = dict(u=float(np.abs(ur).max()), du=float(du.max()), omega=float(wheel.max()))
        bad = []
        if m['u'] > u_max:
            bad.append(f'reference torque {m["u"]:.3f} N m > u_max {u_max} N m')
        if m['du'] > du_max:
            bad.append(f'reference torque change {m["du"]:.3f} N m/step > du_max {du_max} N m/step')
        if m['omega'] > omega_max:
            bad.append(f'reference wheel speed {m["omega"]:.1f} rad/s > omega_max {omega_max} rad/s')
        return m, bad

    def step_torques_at(self, tk, T):
        """Step-averaged u_r for the steps starting at each time in tk."""
        tk = np.asarray(tk, dtype=float)
        return (self._U(tk + T) - self._U(tk)) / T

    def polyline(self, n=600):
        """Anchored path points (n x 3: X, Y, psi) over one lap (loop) or the whole path."""
        t = np.linspace(0.0, self.Tr + self.T_lap if self.loop else self.T_end, n)
        return self.states(t)[:, :3]
