import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Mirrors upstream turtlebot3_gazebo's <world>.launch.py + spawn_turtlebot3.launch.py,
# but spawns this package's mecanum model instead of models/turtlebot3_$TURTLEBOT3_MODEL.
# The upstream waffle_pi bridge config is reused as-is: the mecanum model publishes and
# subscribes on the same topics (cmd_vel now also honours linear.y).


def _gz_server(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    world_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'worlds', f'{world}.world')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r -s -v2 {world_path}', 'on_exit_shutdown': 'true'}.items())]


def generate_launch_description():
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    this_pkg = get_package_share_directory('tb3_sim_bringup')

    model_sdf = os.path.join(this_pkg, 'models', 'turtlebot3_waffle_pi_mecanum', 'model.sdf')
    with open(os.path.join(this_pkg, 'urdf', 'turtlebot3_waffle_pi_mecanum.urdf')) as f:
        robot_desc = f.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='turtlebot3_world',
            description="Base name of a world file in turtlebot3_gazebo/worlds "
                        "(e.g. 'turtlebot3_world', 'empty_world', 'turtlebot3_house')"),
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='-0.5'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Start the Gazebo GUI client'),

        # Meshes are referenced as model://turtlebot3_common/...
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(tb3_gazebo, 'models')),

        OpaqueFunction(function=_gz_server),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items(),
            condition=IfCondition(LaunchConfiguration('gui'))),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True, 'robot_description': robot_desc}],
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'turtlebot3_waffle_pi_mecanum',
                '-file', model_sdf,
                '-x', LaunchConfiguration('x_pose'),
                '-y', LaunchConfiguration('y_pose'),
                '-z', '0.01',
            ],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '--ros-args', '-p',
                'config_file:=' + os.path.join(
                    tb3_gazebo, 'params', 'turtlebot3_waffle_pi_bridge.yaml'),
            ],
            output='screen',
        ),
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['/camera/image_raw'],
            output='screen',
        ),
    ])
