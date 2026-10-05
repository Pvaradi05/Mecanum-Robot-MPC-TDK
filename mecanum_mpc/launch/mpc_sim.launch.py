"""Own mecanum robot in gz sim with torque-driven wheels + the LTV-MPC tracking a path.

    ros2 launch mecanum_mpc mpc_sim.launch.py shape:=figure8 heading:=tangent
    ros2 launch mecanum_mpc mpc_sim.launch.py shape:=square loop:=true     # until Ctrl-C
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('shape', default_value='circle',
                              description='circle | ellipse | figure8 | square'),
        DeclareLaunchArgument('heading', default_value='fixed', description='fixed | tangent'),
        DeclareLaunchArgument('speed', default_value='0.2', description='path speed [m/s]'),
        DeclareLaunchArgument('loop', default_value='false',
                              description='true: drive the path over and over until shutdown; '
                                          'false: drive it `laps` times, then hold the end pose'),
        DeclareLaunchArgument('laps', default_value='1.0', description='ignored with loop:=true'),
        DeclareLaunchArgument('size', default_value='0.0',
                              description='radius / semi-axis / half-width / side [m]; 0 = default'),
        DeclareLaunchArgument('linearize_about', default_value='reference',
                              description='reference (LTV-MPC) | prediction (one SQP-RTI step)'),
        DeclareLaunchArgument('world', default_value='empty_world'),
        DeclareLaunchArgument('gui', default_value='true'),
    ]
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('tb3_sim_bringup'), 'launch', 'own_mecanum_sim.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world'), 'gui': LaunchConfiguration('gui'),
                          'x_pose': '0.0', 'y_pose': '0.0', 'drive': 'effort'}.items())
    mpc = Node(
        package='mecanum_mpc', executable='mpc_node', name='mecanum_mpc', output='screen',
        parameters=[{
            'use_sim_time': True,
            'shape': LaunchConfiguration('shape'),
            'heading': LaunchConfiguration('heading'),
            'speed': LaunchConfiguration('speed'),
            'laps': LaunchConfiguration('laps'),
            'loop': LaunchConfiguration('loop'),
            'size': LaunchConfiguration('size'),
            'linearize_about': LaunchConfiguration('linearize_about'),
        }])
    # give the robot a few seconds to spawn and settle before the path starts
    return LaunchDescription(args + [sim, TimerAction(period=8.0, actions=[mpc])])
