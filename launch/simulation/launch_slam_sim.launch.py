import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    setup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ergocub_navigation'), 'launch'),
            '/simulation/setup_robot_sim.launch.py'])
        )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ergocub_navigation'), 'launch'),
            '/slam/nav2_stack_slam_sim.launch.py'])
        )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ergocub_navigation'), 'launch'),
            '/slam/slam_online_async.launch.py'])
        )
    path_converter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ergocub_navigation'), 'launch'),
            'path_converter.launch.py'])
        )

    return LaunchDescription([
        Node(
            package='ergocub_navigation',
            executable='planner_trigger_server',
            output='screen'
            ),
        setup,
        slam,
        navigation,
        #path_converter
    ])
