# 1) Integrate the paper's exact Chaplygin model, eq. (64) with (39), (62), (12),
#    and compare with a body-frame rigid-body model:
#       me*(du/dt) - ms*v*r = Fxi,  me*(dv/dt) + ms*u*r = Feta,  Je*(dr/dt) = tau
#    me = ms + 4*J1/R^2, Je = JC + 4*J1*L^2/R^2, wrench = J^T M  (J from eq. 32).
import numpy as np
from scipy.integrate import solve_ivp
def params(m0, m1, R, rho, l, J0, J1, J2):
    L = rho + l; ms = m0 + 4*m1; JC = J0 + 4*(J2 + m1*(rho**2 + l**2))
    A = ms*R**2/8 + JC*R**2/(16*L**2) + J1; B = JC*R**2/(16*L**2); C = ms*R**2/8 - JC*R**2/(16*L**2)
    return dict(L=L, ms=ms, JC=JC, A=A, B=B, C=C, R=R, J1=J1)
def exact_rhs(t, w, p, M):
    A,B,C,R,L = p['A'],p['B'],p['C'],p['R'],p['L']; w1,w2,w3 = w; k = R*(B+C)/L
    P = np.array([-k*(w2+w3)*(w2-w3), k*(w1-w3)*(w2-w3), k*(w1+w2)*(w2-w3)])
    Mm = np.array([[2*(A+C), A+C, -(A+C)], [A+C, 2*(A+B), -(A+2*B-C)], [-(A+C), -(A+2*B-C), 2*(A+B)]])
    Q = np.array([M[0]+M[3], M[1]+M[3], M[2]-M[3]])
    return np.linalg.solve(Mm, Q - P)
def body_rhs(t, s, p, M, ms_coriolis=True):
    u, v, r = s; R, L, ms, J1 = p['R'], p['L'], p['ms'], p['J1']
    me = ms + 4*J1/R**2; Je = p['JC'] + 4*J1*L**2/R**2; mc = ms if ms_coriolis else me
    F = np.array([sum(M), -M[0]+M[1]+M[2]-M[3], L*(-M[0]+M[1]-M[2]+M[3])])/R
    return [(F[0] + mc*v*r)/me, (F[1] - mc*u*r)/me, F[2]/Je]
p = params(3.1, 0.35, 0.05, 0.15, 0.15, 0.032, 6.25e-4, 3.13e-4)   # paper eq. (101)
M = [0.05, 0.25, -0.05, 0.20]                                        # paper eq. (102)
T = np.linspace(0, 3, 31)
ex = solve_ivp(exact_rhs, (0, 3), [0, 0, 0], args=(p, M), t_eval=T, rtol=1e-10, atol=1e-12)
w1, w2, w3 = ex.y; R, L = p['R'], p['L']
uvr_exact = np.vstack([R/2*(w1+w2), R/2*(w3-w1), R/(2*L)*(w2-w3)])   # eq. (12)
for name, flag in [('Coriolis with ms (body only)', True), ('Coriolis with me (incl. wheels)', False)]:
    b = solve_ivp(body_rhs, (0, 3), [0, 0, 0], args=(p, M, flag), t_eval=T, rtol=1e-10, atol=1e-12)
    print(f'{name:34s} max |diff| vs exact eq.(64): {np.abs(b.y - uvr_exact).max():.2e}   (max |u,v| {np.abs(uvr_exact[:2]).max():.2f} m/s)')
