from ergocub_navigation.launch_utils import bool_param, pkg_share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Replaces imu_madgwick_filter.launch.py + imu_complementary_filter.launch.py,
# which already shared param/imu_filter.yaml.
#
# The node names are kept exactly as they were. Note the YAML is keyed
# 'imu_filter', so it only reaches the madgwick node; the complementary node runs
# on its own defaults. Left as-is because renaming it would newly apply config
# that has never been in effect.
IMU_FILTERS = {
    'madgwick': {
        'package': 'imu_filter_madgwick',
        'executable': 'imu_filter_madgwick_node',
        'name': 'imu_filter',
    },
    'complementary': {
        'package': 'imu_complementary_filter',
        'executable': 'complementary_filter_node',
        'name': 'complementary_filter_gain_node',
    },
}


def _filter_node(context, *args, **kwargs):
    choice = LaunchConfiguration('filter').perform(context)
    spec = IMU_FILTERS[choice]
    return [Node(
        package=spec['package'],
        executable=spec['executable'],
        name=spec['name'],
        output='screen',
        remappings=[('imu/data_raw', 'head_imu'),
                    ('imu/data', 'head_imu/filtered')],
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': bool_param(LaunchConfiguration('use_sim_time'))},
        ],
    )]


def generate_launch_description():
    """Orientation filter for the head IMU."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'filter', default_value='madgwick', choices=sorted(IMU_FILTERS),
            description='Which orientation filter implementation to run'),
        DeclareLaunchArgument(
            'params_file', default_value=pkg_share('param', 'imu_filter.yaml'),
            description='Full path to the IMU filter parameters file'),
        OpaqueFunction(function=_filter_node),
    ])
