from ergocub_navigation.launch_utils import bool_param
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Footstep marker visualiser.

    The use_sim_time default was 'true' here even though this is a real-robot
    launch file; it now defaults to false like every other leaf, and the world is
    selected by bringup.launch.py.
    """
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='ergocub_navigation',
            executable='footsteps_viewer',
            output='screen',
            parameters=[{
                'use_sim_time': bool_param(LaunchConfiguration('use_sim_time'))}]
        )
    ])
