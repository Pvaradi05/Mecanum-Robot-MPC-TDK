"""Closed loop of the linear MPC v2 against the model, without ROS or Gazebo (plan test 7).
The plant, reference and metrics are the shared ones (mecanum_mpc_common.offline_sim).

    python3 -m mecanum_lmpc_v2.offline_sim                       # all paths x headings, nominal
    python3 -m mecanum_lmpc_v2.offline_sim --plant perturbed     # mass +20 %, J1 x 2, friction
"""
from mecanum_mpc_common import offline_sim as _sim

from .lmpc import LinearMPCv2


def run(shape='circle', heading='fixed', speed=0.1, plant='nominal', start_offset=(0, 0, 0),
        urdf=None, T=0.05, N=10, mass_scale=1.2, J1_scale=2.0, friction=0.003, extra=(),
        **mpc_kw):
    return _sim.run(lambda p, T, N: LinearMPCv2(p, T, N, **mpc_kw), shape, heading, speed, plant,
                    start_offset, urdf, T, N, mass_scale, J1_scale, friction, extra)


def main():
    _sim.main(lambda p, T, N: LinearMPCv2(p, T, N), __doc__)


if __name__ == '__main__':
    main()
