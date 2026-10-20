from ergocub_navigation.launch_utils import bool_param, lifecycle_bringup, pkg_share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    IMU-gated depth pointcloud filter.

    Replaces pointcloud_filter.launch.py + pointcloud_filter_sim.launch.py. Both
    passed the LaunchConfiguration object instead of the string 'use_sim_time' as
    the DeclareLaunchArgument name, so the argument was never really declared.
    """
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'params_file', default_value=pkg_share('param', 'depth_filter.yaml'),
            description='Full path to the pointcloud_filter parameters file'),
        *lifecycle_bringup(
            name='pointcloud_filter_node',
            executable='pointcloud_filter',
            param_file=LaunchConfiguration('params_file'),
            extra_parameters={'use_sim_time': bool_param(use_sim_time)},
        ),
    ])
