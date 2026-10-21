"""Conventions shared by every controller: step status codes and display names."""

# status of one control step (logged in the CSV and on /<controller>/status)
STATUS_SOLVED = 0          # optimiser finished normally
STATUS_FALLBACK = 1        # optimiser failed: previous plan, shifted, applied
STATUS_STOPPED = 2         # failed max_failures times in a row: zero torque, controller stopped
STATUS_NOT_CONVERGED = 3   # NMPC only: iteration or time cap reached, last iterate applied

FAILED = (STATUS_FALLBACK, STATUS_STOPPED)

TITLES = {'lmpc_v2': 'Linear MPC v2', 'nmpc': 'Nonlinear MPC'}
