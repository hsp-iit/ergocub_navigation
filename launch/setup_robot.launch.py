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
            '/setup_robot/robot_state_publisher.launch.py'])
        )
    scan_filtering_compensated = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ergocub_navigation'), 'launch'),
            '/setup_robot/scan_filtering_compensated.launch.py'])
        )
    depth_to_pointcloud = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ergocub_navigation'), 'launch'),
            '/setup_robot/point_cloud_xyz.launch.py'])
        )
    virtual_unicycle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ergocub_navigation'), 'launch'),
            '/setup_robot/odom.launch.py'])
        )
    amcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ergocub_navigation'), 'launch'),
            '/setup_localization.launch.py'])
        )
    footprints_viewer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ergocub_navigation'), 'launch'),
            '/footprints_viewer.launch.py'])
        )
    pointcloud_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ergocub_navigation'), 'launch'),
            '/setup_robot/pointcloud_filter.launch.py'])
        )
    return LaunchDescription([
        state_publisher,
        scan_filtering_compensated,
        #depth_to_pointcloud,
        virtual_unicycle,
        amcl,
        pointcloud_filter
        #footprints_viewer,
    ])