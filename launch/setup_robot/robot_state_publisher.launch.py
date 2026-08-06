import os

from ergocub_navigation.launch_utils import bool_param
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _state_publisher(context, *args, **kwargs):
    model = LaunchConfiguration('model').perform(context)
    superbuild = LaunchConfiguration('superbuild_src').perform(context)
    robot_urdf = os.path.join(
        superbuild, 'src', 'ergocub-software', 'urdf', 'ergoCub', 'robots',
        model, 'model.urdf')
    if not os.path.isfile(robot_urdf):
        raise RuntimeError(
            f'URDF not found: {robot_urdf}\n'
            'Set model:=<robot> and superbuild_src:=<path>, or export '
            'YARP_ROBOT_NAME and ROBOTOLOGY_SUPERBUILD_SOURCE_DIR.')
    with open(robot_urdf, 'r') as infp:
        robot_desc = infp.read()

    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'publish_frequency': 200.0,
            'use_sim_time': bool_param(LaunchConfiguration('use_sim_time')),
        }],
        arguments=[robot_urdf])]


def generate_launch_description():
    """
    robot_state_publisher fed from the robotology-superbuild URDF.

    Replaces robot_state_publisher.launch.py + robot_state_publisher_sim.launch.py;
    the sim copy hardcoded an absolute path that is exactly what these two
    environment variables already resolve to.
    """
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'model', default_value=os.environ.get('YARP_ROBOT_NAME', ''),
            description='Robot model directory under ergocub-software/urdf/ergoCub/robots'),
        DeclareLaunchArgument(
            'superbuild_src',
            default_value=os.environ.get('ROBOTOLOGY_SUPERBUILD_SOURCE_DIR', ''),
            description='Path to the robotology-superbuild source directory'),
        OpaqueFunction(function=_state_publisher),
    ])
