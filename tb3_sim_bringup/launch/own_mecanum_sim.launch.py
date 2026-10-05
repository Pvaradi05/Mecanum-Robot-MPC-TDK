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
import xacro

WHEELS = ('front_left', 'front_right', 'rear_left', 'rear_right')

# Same structure as tb3_waffle_pi_mecanum_sim.launch.py, but spawns the own mecanum robot
# (urdf/own_mecanum.urdf.xacro) from its URDF instead of an SDF model. Sensors and drive use
# the TurtleBot3 topics, so the upstream waffle_pi bridge config is reused as-is (the robot
# has no camera, so its camera topics simply stay silent). drive:=effort swaps the velocity
# drive for per-wheel torque inputs, see urdf/own_mecanum.urdf.xacro.


def _gz_server(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    world_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'worlds', f'{world}.world')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r -s -v2 {world_path}', 'on_exit_shutdown': 'true'}.items())]


def _robot(context, *args, **kwargs):
    drive = LaunchConfiguration('drive').perform(context)
    if drive not in ('velocity', 'effort'):
        raise ValueError(f"drive must be 'velocity' or 'effort', got {drive!r}")
    this_pkg = get_package_share_directory('tb3_sim_bringup')
    # robot.urdf (copied unchanged from the Mecanum-TDK repo) uses mesh paths relative to
    # its own folder; make them resolvable for RViz and Gazebo.
    robot_desc = xacro.process_file(
        os.path.join(this_pkg, 'urdf', 'own_mecanum.urdf.xacro'),
        mappings={'drive': drive}).toxml().replace(
        'filename="meshes/', 'filename="package://tb3_sim_bringup/models/own_mecanum/meshes/')
    actions = [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_desc}],
    )]
    if drive == 'effort':
        # wheel torque inputs (ApplyJointForce), ROS std_msgs/Float64 -> gz.msgs.Double
        actions.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='wheel_torque_bridge',
            arguments=[f'/model/own_mecanum/joint/{w}_wheel_joint/cmd_force'
                       '@std_msgs/msg/Float64]gz.msgs.Double' for w in WHEELS],
            output='screen',
        ))
    return actions


def generate_launch_description():
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    this_pkg = get_package_share_directory('tb3_sim_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='turtlebot3_world',
            description="Base name of a world file in turtlebot3_gazebo/worlds "
                        "(e.g. 'turtlebot3_world', 'empty_world', 'turtlebot3_house')"),
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='-0.5'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Start the Gazebo GUI client'),
        DeclareLaunchArgument(
            'drive', default_value='velocity',
            description="'velocity': MecanumDrive on /cmd_vel. 'effort': wheel torques on "
                        "/model/own_mecanum/joint/<corner>_wheel_joint/cmd_force and ground-truth "
                        "/odom (for mecanum_mpc)"),

        # package://tb3_sim_bringup/... is looked up under <prefix>/share by Gazebo
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(this_pkg)),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(tb3_gazebo, 'models')),

        OpaqueFunction(function=_gz_server),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items(),
            condition=IfCondition(LaunchConfiguration('gui'))),

        OpaqueFunction(function=_robot),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'own_mecanum',
                '-topic', 'robot_description',
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
    ])
