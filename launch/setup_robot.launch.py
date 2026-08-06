from ergocub_navigation.launch_utils import include, pkg_share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Robot-side sensing and state: URDF, scan filtering, odometry.

    Replaces setup_robot / setup_robot_sim / setup_robot_odom_only /
    setup_robot_odom_only_sim / setup_robot_slam, which differed only in which
    leaves they pulled in and whether those leaves were the _sim copies.
    Localization and RViz are not started here; bringup.launch.py owns them.
    """
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'projectors', default_value='robot',
            description="pointcloud_to_laserscan projector set: 'robot' or 'sim'"),
        DeclareLaunchArgument(
            'scan_params', default_value=pkg_share('param', 'scan.yaml'),
            description='Full path to the scan_node parameters file'),
        DeclareLaunchArgument(
            'odom_params', default_value=pkg_share('param', 'odom.yaml'),
            description='Full path to the odom_node parameters file'),
        DeclareLaunchArgument(
            'use_pointcloud_filter', default_value='false',
            description='Run the IMU-gated depth pointcloud filter'),
        DeclareLaunchArgument(
            'depth_params', default_value=pkg_share('param', 'depth_filter.yaml'),
            description='Full path to the pointcloud_filter parameters file'),

        include('setup_robot/robot_state_publisher.launch.py', {
            'use_sim_time': use_sim_time,
        }),
        include('setup_robot/scan_filtering.launch.py', {
            'use_sim_time': use_sim_time,
            'params_file': LaunchConfiguration('scan_params'),
            'projectors': LaunchConfiguration('projectors'),
        }),
        include('setup_robot/odom.launch.py', {
            'use_sim_time': use_sim_time,
            'params_file': LaunchConfiguration('odom_params'),
        }),
        include('setup_robot/pointcloud_filter.launch.py', {
            'use_sim_time': use_sim_time,
            'params_file': LaunchConfiguration('depth_params'),
        }, condition=IfCondition(LaunchConfiguration('use_pointcloud_filter'))),
    ])
