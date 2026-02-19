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
            '/setup_robot.launch.py'])
        )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ergocub_navigation'), 'launch'),
            '/nav2_stack.launch.py'])
        )
    plane_detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ergocub_navigation'), 'launch'),
            '/plane_detector.launch.py'])
        )

    return LaunchDescription([
        plane_detector,
        Node(
            package='ergocub_navigation',
            executable='planner_trigger_server',
            output='screen'
            ),
        setup,
        navigation
    ])