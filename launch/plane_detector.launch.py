from ergocub_navigation.launch_utils import bool_param, lifecycle_bringup, pkg_share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Floor-plane detector.

    The referenced param/plane_detector.yaml did not exist until now, so the node
    silently ran on the defaults compiled into plane_detector.cpp; the YAML now
    reproduces those defaults.
    """
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'params_file', default_value=pkg_share('param', 'plane_detector.yaml'),
            description='Full path to the plane_detector parameters file'),
        *lifecycle_bringup(
            name='plane_detector',
            executable='plane_detector',
            param_file=LaunchConfiguration('params_file'),
            extra_parameters={'use_sim_time': bool_param(use_sim_time)},
        ),
    ])
