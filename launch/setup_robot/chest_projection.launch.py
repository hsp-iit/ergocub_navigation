import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    config = os.path.join(
        get_package_share_directory('ergocub_navigation'),
        'config/param',
        'chest_projection_params.yaml'
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='ergocub_navigation',
            executable='chest_projection',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])