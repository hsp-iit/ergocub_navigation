import os

from ament_index_python.packages import get_package_share_directory

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
            '/simulation/nav2_stack_sim.launch.py'])
        )
    

    return LaunchDescription([
        setup,
        navigation
    ])