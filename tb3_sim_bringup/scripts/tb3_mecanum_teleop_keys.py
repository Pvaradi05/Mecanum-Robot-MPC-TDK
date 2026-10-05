#!/usr/bin/env python3
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Darby Lim
#
# Adapted for tb3_sim_bringup from ros-jazzy-turtlebot3-teleop's
# teleop_keyboard.py, via scripts/tb3_teleop_keys.py, for the mecanum
# Waffle Pi variant (models/turtlebot3_waffle_pi_mecanum): adds a third,
# lateral (strafe) axis on linear.y, moves rotation to q/e so a/d can
# strafe, keeps a per-axis zero hotkey for every axis, and always
# publishes zero on exit.

import select
import sys
import termios
import tty

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.clock import Clock
from rclpy.qos import QoSProfile

MAX_LIN_VEL = 0.26  # m/s, forward and strafe (Waffle Pi limit)
MAX_ANG_VEL = 1.82  # rad/s

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your Mecanum TurtleBot3!
---------------------------
Moving around:
   q    w    e
   a    s    d
        x

w/x : increase/decrease forward velocity  (linear.x, ~ 0.26)
a/d : increase/decrease strafe velocity   (linear.y, a = left, ~ 0.26)
q/e : increase/decrease rotation velocity (angular.z, q = counter-clockwise, ~ 1.82)

1 : zero FORWARD velocity only
2 : zero STRAFE velocity only
3 : zero ROTATION velocity only
space key, s : force stop (zero all)

CTRL-C to quit (always stops the robot)
"""


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_vels(target_x, target_y, target_angular):
    print('currently:\tforward {0}\t strafe {1}\t rotation {2} '.format(
        target_x, target_y, target_angular))


def make_simple_profile(output_vel, input_vel, slop):
    if input_vel > output_vel:
        output_vel = min(input_vel, output_vel + slop)
    elif input_vel < output_vel:
        output_vel = max(input_vel, output_vel - slop)
    else:
        output_vel = input_vel

    return output_vel


def constrain(input_vel, bound):
    return max(-bound, min(bound, input_vel))


def main():
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('tb3_mecanum_teleop_keys')
    pub = node.create_publisher(TwistStamped, 'cmd_vel', qos)

    def publish(x, y, angular):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = Clock().now().to_msg()
        twist_stamped.header.frame_id = ''
        twist_stamped.twist.linear.x = x
        twist_stamped.twist.linear.y = y
        twist_stamped.twist.angular.z = angular
        pub.publish(twist_stamped)

    status = 0
    # [forward, strafe, rotation]
    target = [0.0, 0.0, 0.0]
    control = [0.0, 0.0, 0.0]
    bounds = [MAX_LIN_VEL, MAX_LIN_VEL, MAX_ANG_VEL]
    steps = [LIN_VEL_STEP_SIZE, LIN_VEL_STEP_SIZE, ANG_VEL_STEP_SIZE]
    # key -> (axis, direction)
    step_keys = {
        'w': (0, 1), 'x': (0, -1),
        'a': (1, 1), 'd': (1, -1),
        'q': (2, 1), 'e': (2, -1),
    }
    zero_keys = {'1': 0, '2': 1, '3': 2}

    try:
        print(msg)
        while (1):
            key = get_key(settings)
            if key in step_keys:
                axis, direction = step_keys[key]
                target[axis] = constrain(
                    target[axis] + direction * steps[axis], bounds[axis])
                status += 1
                print_vels(*target)
            elif key in zero_keys:
                axis = zero_keys[key]
                target[axis] = 0.0
                control[axis] = 0.0
                print_vels(*target)
            elif key == ' ' or key == 's':
                target = [0.0, 0.0, 0.0]
                control = [0.0, 0.0, 0.0]
                print_vels(*target)
            else:
                if key == '\x03':
                    break

            if status == 20:
                print(msg)
                status = 0

            for axis in range(3):
                control[axis] = make_simple_profile(
                    control[axis], target[axis], steps[axis] / 2.0)

            publish(*control)

    except Exception as e:
        print(e)

    finally:
        # Always stop on exit, regardless of what was last commanded.
        publish(0.0, 0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
