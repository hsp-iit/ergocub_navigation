import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ergocub_navigation'), 'launch'),
            '/simulation/setup_robot/robot_state_publisher_sim.launch.py'])
        )
    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ergocub_navigation'), 'launch'),
            '/simulation/ergoCub_rviz_sim.launch.py'])
        )
    scan_filtering_compensated = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ergocub_navigation'), 'launch'),
            '/simulation/setup_robot/scan_filtering_sim.launch.py'])
        )
    virtual_unicycle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ergocub_navigation'), 'launch'),
            '/simulation/setup_robot/odom_sim.launch.py'])
        )
    return LaunchDescription([
        state_publisher,
        rviz_node,
        scan_filtering_compensated,
        virtual_unicycle
    ])