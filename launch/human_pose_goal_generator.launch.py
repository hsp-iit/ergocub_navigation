from ergocub_navigation.launch_utils import bool_param, lifecycle_bringup, pkg_share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Turn a tracked human pose into a nav2 goal."""
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=pkg_share('param', 'human_pose_goal_generator.yaml'),
            description='Full path to the human_pose_goal_generator parameters file'),
        *lifecycle_bringup(
            name='human_pose_goal_generator',
            executable='human_pose_goal_generator',
            param_file=LaunchConfiguration('params_file'),
            extra_parameters={'use_sim_time': bool_param(use_sim_time)},
        ),
    ])
