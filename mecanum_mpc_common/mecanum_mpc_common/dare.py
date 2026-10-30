"""Terminal weight of both controllers: the solution of the discrete algebraic Riccati equation (DARE)
at the end of the horizon, the LTV-MPC's terminal='dare'. solve_dare is the function of
Kemenylegeny/Mecanum-tracking-tdk @ 411436a, mpc/lmpc.py, moved here unchanged so that the LMPC
(mecanum_lmpc_v2/lmpc.py) and the NMPC compute the same P.
"""
import numpy as np


def solve_dare(A, B, Q, R, P0=None, max_iter=500, tol=1e-9):
    """P = Q + A^T P (A - B K),  K = (R + B^T P B)^-1 B^T P A, by Riccati iteration from P0 (default Q).
    Returns (P, number of iterations). Stops at max |P_new - P| < tol * max |P|."""
    P = Q.copy() if P0 is None else P0
    for i in range(1, max_iter + 1):
        K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
        P_new = Q + A.T @ P @ (A - B @ K)
        P_new = (P_new + P_new.T) / 2
        if np.abs(P_new - P).max() < tol * np.abs(P_new).max():
            return P_new, i
        P = P_new
    return P, max_iter
