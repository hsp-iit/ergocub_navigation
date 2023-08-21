import os
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    parms_path = os.path.join(get_package_share_directory('ergocub_navigation'), 'param', "imu_filter.yaml")

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package='imu_filter_madgwick',
                remappings=[('imu/data_raw', 'head_imu'),
                            ('imu/data', 'head_imu/filtered')],
                executable='imu_filter_madgwick_node',
                name='imu_filter',
                output='screen',
                parameters=[parms_path]
            )
        ]
    )