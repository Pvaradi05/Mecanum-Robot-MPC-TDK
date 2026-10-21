"""Wheel motor limits: ROBOTIS DYNAMIXEL XM430-W210-T, one per wheel, driving it directly.

Source: ROBOTIS e-manual, https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/
(specification table and performance graph, read 2026-10-21). Supply voltage 12.0 V (the
recommended voltage; decided 2026-10-21).

    stall torque         3.0 N m at 12.0 V (2.3 A): momentary maximum, not used as the limit
    performance graph    highest measured torque ~1.95 N m (~2.05 A)            -> U_MAX
    no-load speed        77 rev/min at 12.0 V = 8.06 rad/s                       -> OMEGA_MAX
    torque rate          no limit in Current Control Mode (Profile Acceleration does not apply)
                         -> DU_MAX = U_MAX per control step (decided 2026-10-21)
Not in the datasheet: continuous torque rating, current slew rate, rotor inertia.
"""
import math

U_MAX = 1.95                           # N m, wheel torque limit (performance graph maximum)
OMEGA_MAX = 77.0 * 2.0 * math.pi / 60.0   # rad/s, no-load speed at 12.0 V
DU_MAX = U_MAX                         # N m per control step, torque-rate limit (D6)
