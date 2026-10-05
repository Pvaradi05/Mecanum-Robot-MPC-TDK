# Own robot, Chaplygin model in body form. For each path & heading mode:
#  (a) size of the bilinear terms vs the total force needed along the reference
#  (b) 1 s open-loop prediction error, starting off the reference (5 cm, 10 deg, 0.05 m/s),
#      of: LTV (linearised along the reference) and LTI (linearised once at the start state)
#      vs the nonlinear model, all fed the same feedforward torques.
import numpy as np
R, rho, l = 0.03028, 0.064, 0.1526; L = rho + l
m0, m1 = 2.6, 0.1; ms = m0 + 4*m1
J0 = 0.028361; J2 = 1e-5; JC = J0 + 4*(J2 + m1*(rho**2 + l**2))
V = 0.2                                     # path speed m/s
def model(J1):
    me = ms + 4*J1/R**2; Je = JC + 4*J1*L**2/R**2
    def f(x, F):                            # x = [X, Y, psi, u, v, r], F = body wrench (Fxi, Feta, tau)
        X, Y, p, u, v, r = x; c, s = np.cos(p), np.sin(p)
        return np.array([c*u - s*v, s*u + c*v, r, (F[0] + ms*v*r)/me, (F[1] - ms*u*r)/me, F[2]/Je])
    def jac(x):                             # df/dx (input enters linearly with constant B)
        X, Y, p, u, v, r = x; c, s = np.cos(p), np.sin(p); A = np.zeros((6, 6))
        A[0, 2] = -s*u - c*v; A[0, 3] = c; A[0, 4] = -s
        A[1, 2] = c*u - s*v;  A[1, 3] = s; A[1, 4] = c
        A[2, 5] = 1
        A[3, 4] = ms*r/me; A[3, 5] = ms*v/me
        A[4, 3] = -ms*r/me; A[4, 5] = -ms*u/me
        return A
    return f, jac, me, Je
def path(name, tangent, n=4000):
    s = np.linspace(0, 2*np.pi, n, endpoint=False)
    P = {'circle':   np.c_[np.cos(s), np.sin(s)],
         'ellipse':  np.c_[1.5*np.cos(s), 0.75*np.sin(s)],
         'figure-8': np.c_[np.sin(s), np.sin(s)*np.cos(s)]}[name]
    d = np.gradient(P, axis=0); d[0] = P[1]-P[-1]; seg = np.linalg.norm(np.roll(P, -1, 0) - P, axis=1)
    t = np.r_[0, np.cumsum(seg)[:-1]] / V   # constant speed V
    psi = np.unwrap(np.arctan2(d[:, 1], d[:, 0])) if tangent else np.zeros(n)
    tu = np.linspace(0, t[-1], n)              # resample on a uniform time grid
    P = np.c_[np.interp(tu, t, P[:, 0]), np.interp(tu, t, P[:, 1])]
    return tu, P, np.interp(tu, t, psi)
rows = []
for J1, jl in [(3.7e-5, 'wheel only'), (3.7e-5 + 9e-4, 'wheel + gearmotor (robot_config guess)')]:
    f, jac, me, Je = model(J1)
    print(f'\n=== J1 = {J1:.1e} ({jl}):  ms = {ms:.2f} kg, me = {me:.2f} kg, Je = {Je:.4f} kg m^2')
    print(f'{"path":9} {"heading":8} | {"max |ms*u*r|":>12} {"max |F| needed":>14} | 1 s prediction error from off-ref start:  {"LTV":>8} {"LTI":>8}')
    for name in ['circle', 'ellipse', 'figure-8']:
        for tangent in [False, True]:
            t, P, psi = path(name, tangent); dt = np.diff(t).mean()
            vw = np.gradient(P, t, axis=0); r = np.gradient(psi, t)
            c, s_ = np.cos(psi), np.sin(psi)
            u = c*vw[:, 0] + s_*vw[:, 1]; v = -s_*vw[:, 0] + c*vw[:, 1]
            du, dv, dr = np.gradient(u, t), np.gradient(v, t), np.gradient(r, t)
            Fx = me*du - ms*v*r; Fy = me*dv + ms*u*r; tau = Je*dr
            cor = np.abs(np.c_[ms*v*r, ms*u*r]).max(); Fmax = np.abs(np.c_[Fx, Fy]).max()
            # prediction test, worst case over 8 start points, horizon 1 s
            H = int(1.0/dt); eL, eI = 0, 0
            for k0 in range(0, len(t) - H, len(t)//8):
                xr = np.c_[P, psi, u, v, r]; Fr = np.c_[Fx, Fy, tau]
                x0 = xr[k0] + np.array([0.05, 0, np.radians(10), 0.05, 0, 0])
                xn = x0.copy(); dxL = x0 - xr[k0]; xI = x0.copy(); A0 = jac(x0); f0 = f(x0, Fr[k0])
                for k in range(k0, k0 + H):
                    xn = xn + dt*f(xn, Fr[k])                        # nonlinear (Euler, small dt)
                    dxL = dxL + dt*(jac(xr[k]) @ dxL)                # LTV error dynamics
                    xI = xI + dt*(f0 + A0 @ (xI - x0) + (f(x0, Fr[k]) - f0))   # LTI about x0
                xL = xr[k0 + H] + dxL
                eL = max(eL, np.linalg.norm(xL[:2] - xn[:2])); eI = max(eI, np.linalg.norm(xI[:2] - xn[:2]))
            print(f'{name:9} {"tangent" if tangent else "fixed":8} | {cor:9.3f} N {Fmax:11.3f} N    | {"":40} {eL*1000:6.1f}mm {eI*1000:6.1f}mm')
