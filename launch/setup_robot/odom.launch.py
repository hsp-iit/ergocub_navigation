from ergocub_navigation.launch_utils import bool_param, lifecycle_bringup, pkg_share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """YARP unicycle odometry publisher. Replaces odom.launch.py + odom_sim.launch.py."""
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'params_file', default_value=pkg_share('param', 'odom.yaml'),
            description='Full path to the odom_node parameters file'),
        *lifecycle_bringup(
            name='odom_node',
            executable='odom_node',
            param_file=LaunchConfiguration('params_file'),
            extra_parameters={'use_sim_time': bool_param(use_sim_time)},
        ),
    ])
