"""Own mecanum robot in gz sim with torque-driven wheels + the linear MPC v2.

    ros2 launch mecanum_lmpc_v2 lmpc_v2_sim.launch.py shape:=square heading:=tangent
    ros2 launch mecanum_lmpc_v2 lmpc_v2_sim.launch.py shape:=figure8 loop:=true
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
        DeclareLaunchArgument('speed', default_value='0.1',
                              description='path speed [m/s]; 0.1 is feasible on every path with '
                                          'the XM430-W210-T speed limit'),
        DeclareLaunchArgument('loop', default_value='false',
                              description='true: repeat the path until shutdown'),
        DeclareLaunchArgument('size', default_value='0.0',
                              description='radius / semi-axis / half-width / side [m]; 0 = default'),
        DeclareLaunchArgument('corner_radius', default_value='-1.0',
                              description='square only [m]; < 0 = 15 % of the side'),
        DeclareLaunchArgument('world', default_value='empty_world'),
        DeclareLaunchArgument('gui', default_value='true'),
    ]
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('tb3_sim_bringup'), 'launch', 'own_mecanum_sim.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world'), 'gui': LaunchConfiguration('gui'),
                          'x_pose': '0.0', 'y_pose': '0.0', 'drive': 'effort'}.items())
    controller = Node(
        package='mecanum_lmpc_v2', executable='lmpc_v2_node', name='lmpc_v2', output='screen',
        parameters=[{
            'use_sim_time': True,
            'shape': LaunchConfiguration('shape'),
            'heading': LaunchConfiguration('heading'),
            'speed': LaunchConfiguration('speed'),
            'loop': LaunchConfiguration('loop'),
            'size': LaunchConfiguration('size'),
            'corner_radius': LaunchConfiguration('corner_radius'),
        }])
    # the robot needs a few seconds to spawn and settle before the path starts
    return LaunchDescription(args + [sim, TimerAction(period=8.0, actions=[controller])])
