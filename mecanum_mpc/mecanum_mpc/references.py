"""Reference trajectories for the path-tracking MPC: circle, ellipse, figure-8 and square.

Every path starts at the origin heading along +x of a local frame. The caller places that frame
at the robot's pose when tracking starts. The path is driven at constant speed with smooth
(cosine) ramps at start and end, then the reference holds the final pose.

Heading modes:
  fixed    psi stays at its start value. The robot never rotates, so the bilinear terms of the
           dynamics stay 0 along the reference.
  tangent  psi follows the path tangent, like a car, smoothed over ~heading_smoothing seconds.

The feedforward wrench w_ref comes from inverse dynamics of model.f along the reference.
"""
import numpy as np
from scipy.ndimage import gaussian_filter1d

from .model import Params

SHAPES = ('circle', 'ellipse', 'figure8', 'square')


def _curve(shape, size, corner_radius=None):
    """Return c(theta) -> (n, 2) points, theta in [0, 2*pi), plus the period.
    size = radius (circle) / semi-major axis (ellipse) / half-width (figure-8) / side (square).
    corner_radius (square only): None = 15 % of the side, 0 = sharp corners."""
    if shape == 'circle':
        return lambda t: np.c_[size * np.sin(t), size * (1 - np.cos(t))]
    if shape == 'ellipse':
        b = size / 2
        return lambda t: np.c_[size * np.sin(t), b * (1 - np.cos(t))]
    if shape == 'figure8':                     # lemniscate of Gerono, rotated to start along +x
        a = np.arctan2(size, size)             # start tangent of (a sin t, a sin t cos t) is 45 deg
        Rm = np.array([[np.cos(-a), -np.sin(-a)], [np.sin(-a), np.cos(-a)]])
        return lambda t: np.c_[size * np.sin(t), size * np.sin(t) * np.cos(t)] @ Rm.T
    if shape == 'square':                      # rounded corners, default radius 15 % of the side
        rc = 0.15 * size if corner_radius is None else float(corner_radius)
        if not 0 <= rc <= size / 2:
            raise ValueError(f'corner_radius must be in [0, {size / 2}], got {rc}')
        e = size - 2 * rc                      # straight part of each side
        per = 4 * e + 2 * np.pi * rc
        segs = []                              # start at the middle of the bottom side, CCW
        pos, head = np.array([0.0, 0.0]), 0.0
        for length, kind in [(e / 2, 'line')] + [(np.pi / 2 * rc, 'arc'), (e, 'line')] * 3 + \
                [(np.pi / 2 * rc, 'arc'), (e / 2, 'line')]:
            if length > 0:                     # corner_radius 0: no arcs, the heading just jumps
                segs.append((length, kind, pos.copy(), head))
            if kind == 'line':
                pos = pos + length * np.array([np.cos(head), np.sin(head)])
            else:
                ctr = pos + rc * np.array([-np.sin(head), np.cos(head)])
                head += np.pi / 2
                pos = ctr + rc * np.array([np.sin(head), -np.cos(head)])
        bounds = np.cumsum([0] + [sg[0] for sg in segs])

        def c(t):
            d = np.mod(t, 2 * np.pi) / (2 * np.pi) * per
            out = np.zeros((len(d), 2))
            for i, (length, kind, p0, h0) in enumerate(segs):
                m = (d >= bounds[i]) & (d <= bounds[i + 1])
                s = d[m] - bounds[i]
                if kind == 'line':
                    out[m] = p0 + np.outer(s, [np.cos(h0), np.sin(h0)])
                else:
                    ctr = p0 + rc * np.array([-np.sin(h0), np.cos(h0)])
                    h = h0 + s / rc
                    out[m] = ctr + rc * np.c_[np.sin(h), -np.cos(h)]
            return out
        return c
    raise ValueError(f'unknown shape {shape!r}, expected one of {SHAPES}')


class Reference:
    """Time-parameterised reference x_ref(t) = [X, Y, psi, u, v, r] and w_ref(t)."""

    def __init__(self, p: Params, shape='circle', size=0.75, speed=0.2, laps=1.0,
                 heading='fixed', ramp_time=2.0, heading_smoothing=0.25, dt=0.002,
                 loop=False, corner_radius=None):
        """loop=False: drive `laps` laps, ramp down, then hold the end pose.
        loop=True: ramp up once, then drive the path forever (`laps` is ignored).
        corner_radius: square only, see _curve."""
        if heading not in ('fixed', 'tangent'):
            raise ValueError("heading must be 'fixed' or 'tangent'")
        c = _curve(shape, size, corner_radius)

        # arc-length table of the curve
        th = np.linspace(0, 2 * np.pi, 200001)
        pts = c(th)
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        arclen = np.r_[0, np.cumsum(seg)]
        self.length = arclen[-1]

        # distance travelled vs time: cosine ramp up, cruise, cosine ramp down
        if loop:
            # ramp up, then cruise; the table covers 3 laps and the 2nd lap (all cruise, with
            # laps before and after it, so heading smoothing is periodic) repeats forever
            D = 3 * self.length
            Tr = min(ramp_time, self.length / speed)
            Tc = (D - speed * Tr / 2) / speed
            T = Tr + Tc
        else:
            D = laps * self.length
            Tr = min(ramp_time, D / speed)      # ramps cover speed*Tr/2 each
            Tc = (D - speed * Tr) / speed
            T = 2 * Tr + Tc
        t = np.arange(0, T + dt, dt)
        vel = np.where(t < Tr, speed * (1 - np.cos(np.pi * t / Tr)) / 2,
                       np.where((t < Tr + Tc) | loop, speed,
                                speed * (1 + np.cos(np.pi * np.clip(t - Tr - Tc, 0, Tr) / Tr)) / 2))
        dist = np.r_[0, np.cumsum((vel[1:] + vel[:-1]) / 2 * dt)]
        dist *= D / dist[-1]
        self.loop = loop
        self.duration = np.inf if loop else T
        if loop:
            self.lap_start = float(np.interp(self.length, dist, t))   # start of the 2nd lap
            self.lap_time = self.length / speed

        lap, rem = np.divmod(dist, self.length)
        theta = np.interp(rem, arclen, th) + 2 * np.pi * lap
        P = c(theta)
        h = 1e-6
        tang = c(theta + h) - c(theta - h)
        if heading == 'tangent':
            psi = np.unwrap(np.arctan2(tang[:, 1], tang[:, 0]))
            # Curvature jumps (square corners, start of an arc) would demand infinite angular
            # acceleration. A mecanum robot can turn independently of its direction of travel,
            # so let it start turning a little before the corner (Gaussian, sigma in seconds).
            if heading_smoothing > 0:
                psi = gaussian_filter1d(psi, heading_smoothing / dt, mode='nearest')
        else:
            psi = np.zeros_like(t)

        vw = np.gradient(P, dt, axis=0)
        cp, sp = np.cos(psi), np.sin(psi)
        u = cp * vw[:, 0] + sp * vw[:, 1]
        v = -sp * vw[:, 0] + cp * vw[:, 1]
        r = np.gradient(psi, dt)
        du, dv, dr = np.gradient(u, dt), np.gradient(v, dt), np.gradient(r, dt)

        self.t = t
        self.x = np.c_[P, psi, u, v, r]
        self.w = np.c_[p.me * du - p.ms * v * r, p.me * dv + p.ms * u * r, p.Je * dr]
        self.shape, self.heading, self.speed = shape, heading, speed
        # largest wheel speed the reference needs [rad/s]; compare with the motor limit
        self.max_wheel_speed = float(np.abs(self.x[:, 3:] @ p.wheel_jacobian().T).max())
        if loop:   # heading gained per lap (2*pi on a circle with tangent heading, else 0)
            self.lap_dpsi = float(np.interp(self.lap_start + self.lap_time, t, psi)
                                  - np.interp(self.lap_start, t, psi))

    def lap(self, time):
        """Loop mode: number of the lap being driven at `time` (0 = first lap, incl. the
        ramp up). Always 0 without loop."""
        if self.loop:
            return 0 if time < self.lap_start else 1 + int((time - self.lap_start) // self.lap_time)
        return 0

    def sample(self, times, w_window=0.0):
        """x_ref, w_ref at the given times (s since start). Without loop the end pose is held
        afterwards; with loop the 2nd lap of the table repeats forever.

        w_window > 0: w_ref is its mean over [t, t + w_window] instead of its value at t. Over
        one control period that is what a zero-order hold has to deliver. It stays finite where
        the reference velocity jumps (sharp corners): the exact w_ref is an impulse there, and
        its point value is either 0 or huge."""
        times = np.asarray(times, dtype=float)
        x, w = self._sample(times)
        if w_window > 0:
            n = max(1, round(w_window / (self.t[1] - self.t[0])))
            w = np.mean([self._sample(times + (j + 0.5) * w_window / n)[1] for j in range(n)],
                        axis=0)
        return x, w

    def _sample(self, times):
        if self.loop:
            k = np.maximum(0, np.floor((times - self.lap_start) / self.lap_time))
            times = times - k * self.lap_time
            x = np.stack([np.interp(times, self.t, self.x[:, i]) for i in range(6)], axis=-1)
            x[..., 2] += k * self.lap_dpsi
            w = np.stack([np.interp(times, self.t, self.w[:, i]) for i in range(3)], axis=-1)
            return x, w
        times = np.clip(times, 0, self.t[-1])
        x = np.stack([np.interp(times, self.t, self.x[:, i]) for i in range(6)], axis=-1)
        w = np.stack([np.interp(times, self.t, self.w[:, i]) for i in range(3)], axis=-1)
        after = times >= self.t[-1]
        x[after, 3:] = 0.0
        w[after] = 0.0
        return x, w

    def polyline(self, n=400):
        """Path points (for plotting / RViz)."""
        idx = np.linspace(0, len(self.t) - 1, n).astype(int)
        return self.x[idx, :3]
