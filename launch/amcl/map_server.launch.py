from ergocub_navigation.launch_utils import bool_param, pkg_share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """
    Serve a single map.

    Replaces map_server / map_server_vicon / map_server_sim / empty_map_server
    (x3), which differed only in the map file they pointed at.
    """
    use_sim_time = LaunchConfiguration('use_sim_time')
    # os.path.join semantics: an absolute 'map' overrides the maps/ prefix, so
    # both 'empty_map.yaml' and '/abs/path/to/map.yaml' work.
    map_yaml = PathJoinSubstitution([pkg_share('maps'), LaunchConfiguration('map')])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'map', default_value='floor0_ergoCub_modded.yaml',
            description='Map YAML, either a name under maps/ or an absolute path'),
        Node(
            package='nav2_map_server',
            executable='map_server',
            parameters=[
                {'yaml_filename': map_yaml},
                {'use_sim_time': bool_param(use_sim_time)}]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='map_server_lifecycle_manager',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': bool_param(use_sim_time)},
                        {'autostart': True},
                        {'node_names': ['map_server']}]
        )
    ])
