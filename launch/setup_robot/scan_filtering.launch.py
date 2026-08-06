from ergocub_navigation.launch_utils import (
    bool_param,
    lifecycle_bringup,
    pkg_share,
    scan_projector_nodes,
    SCAN_PROJECTORS,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _projectors(context, *args, **kwargs):
    profile = LaunchConfiguration('projectors').perform(context)
    if profile not in SCAN_PROJECTORS:
        raise RuntimeError(
            f'projectors:={profile} is not one of {sorted(SCAN_PROJECTORS)}')
    return scan_projector_nodes(profile, LaunchConfiguration('use_sim_time'))


def generate_launch_description():
    """
    IMU-compensated scan filter plus its pointcloud_to_laserscan projectors.

    Replaces scan_filtering_compensated.launch.py (4 inline projector dicts) and
    scan_filtering_sim.launch.py (1). The projector configurations now live in
    ergocub_navigation.launch_utils.SCAN_PROJECTORS; 'projectors' selects a set.
    """
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'params_file', default_value=pkg_share('param', 'scan.yaml'),
            description='Full path to the scan_node parameters file'),
        DeclareLaunchArgument(
            'projectors', default_value='robot',
            choices=sorted(SCAN_PROJECTORS),
            description='Which set of pointcloud_to_laserscan projectors to start. '
                        "'robot' is the wide scan plus front and two rear wedges; "
                        "'sim' is a single full-circle projector."),
        *lifecycle_bringup(
            name='scan_node',
            executable='scan_filter',
            param_file=LaunchConfiguration('params_file'),
            extra_parameters={'use_sim_time': bool_param(use_sim_time)},
        ),
        OpaqueFunction(function=_projectors),
    ])
