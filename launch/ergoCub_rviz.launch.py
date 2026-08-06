from ergocub_navigation.launch_utils import bool_param, pkg_share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Start RViz.

    Replaces ergoCub_rviz.launch.py + ergoCub_rviz_sim.launch.py, which already
    loaded the same rviz/nav2.rviz and differed only in use_sim_time.
    """
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'rviz_config', default_value=pkg_share('rviz', 'nav2.rviz'),
            description='Full path to the RViz configuration file'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            output={'both': 'log'},
            parameters=[{'use_sim_time': bool_param(LaunchConfiguration('use_sim_time'))}],
        )
    ])
