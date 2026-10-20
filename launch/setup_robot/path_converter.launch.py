from ergocub_navigation.launch_utils import bool_param, lifecycle_bringup, pkg_share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Bridges the nav2 /plan to the YARP walking coordinator.

    Replaces path_converter.launch.py + path_converter_sim.launch.py, which
    differed only in trailing whitespace and a shadowed local default.

    NOTE: config/param/path_converter.yaml is keyed 'path_converter_v2_node' while
    this node is named 'path_converter_node', so none of its values currently
    reach the node -- it runs on the defaults in
    src/navigation/walking_planning/path_converter.cpp. Left as-is on purpose:
    correcting the key would newly apply previously inert config to the walking
    bridge, which is a behaviour change, not a reorganisation.
    """
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'params_file', default_value=pkg_share('param', 'path_converter.yaml'),
            description='Full path to the path_converter parameters file'),
        *lifecycle_bringup(
            name='path_converter_node',
            executable='path_converter',
            param_file=LaunchConfiguration('params_file'),
            extra_parameters={'use_sim_time': bool_param(use_sim_time)},
        ),
    ])
