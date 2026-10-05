import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include_upstream_world(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    launch_file = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'launch', f'{world}.launch.py')
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('turtlebot3_model', default_value='waffle_pi'),
        DeclareLaunchArgument(
            'world', default_value='turtlebot3_world',
            description="Base name of the upstream turtlebot3_gazebo launch file to include "
                        "(e.g. 'turtlebot3_world' or 'empty_world')"),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', LaunchConfiguration('turtlebot3_model')),
        OpaqueFunction(function=_include_upstream_world),
    ])
