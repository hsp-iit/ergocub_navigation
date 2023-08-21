from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    parms_path = os.path.join(get_package_share_directory('ergocub_navigation'), 'param', "imu_filter.yaml")
    return LaunchDescription(
        [
            Node(
                package='imu_complementary_filter',
                remappings=[('imu/data_raw', 'head_imu'),
                            ('imu/data', 'head_imu/filtered')],
                executable='complementary_filter_node',
                name='complementary_filter_gain_node',
                output='screen',
                parameters=[parms_path],
            )
        ]
    )
    