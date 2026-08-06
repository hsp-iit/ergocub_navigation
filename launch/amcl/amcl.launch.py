from ergocub_navigation.launch_utils import bool_param, pkg_share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """AMCL plus its lifecycle manager. Replaces amcl.launch.py + amcl_sim.launch.py."""
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'params_file', default_value=pkg_share('param', 'ergoCub_amcl_2.yaml'),
            description='Full path to the AMCL parameters file'),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[params_file, {'use_sim_time': bool_param(use_sim_time)}]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': bool_param(use_sim_time)},
                        {'autostart': True},
                        {'node_names': ['amcl']}]
        )
    ])
