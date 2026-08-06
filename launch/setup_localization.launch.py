from ergocub_navigation.launch_utils import include, pkg_share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Map server, optionally AMCL, optionally an identity map->odom transform.

    Replaces setup_localization / setup_localization_vicon / setup_localization_sim
    / odom_only/localization_odom_only / simulation/localization_odom_only_sim.
    RViz is no longer started here; bringup.launch.py owns it so it is started once.
    """
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_amcl = LaunchConfiguration('use_amcl')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'map', default_value='floor0_ergoCub_modded.yaml',
            description='Map YAML, either a name under maps/ or an absolute path'),
        DeclareLaunchArgument(
            'amcl_params', default_value=pkg_share('param', 'ergoCub_amcl_2.yaml'),
            description='Full path to the AMCL parameters file'),
        DeclareLaunchArgument(
            'use_amcl', default_value='true',
            description='Run AMCL against the served map'),
        DeclareLaunchArgument(
            'static_map_odom_tf', default_value='false',
            description='Publish an identity map->odom transform instead of localizing. '
                        'Used by the odometry-only profile.'),

        include('amcl/map_server.launch.py', {
            'use_sim_time': use_sim_time,
            'map': LaunchConfiguration('map'),
        }),
        include('amcl/amcl.launch.py', {
            'use_sim_time': use_sim_time,
            'params_file': LaunchConfiguration('amcl_params'),
        }, condition=IfCondition(use_amcl)),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_static_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            condition=IfCondition(LaunchConfiguration('static_map_odom_tf')),
        ),
    ])
