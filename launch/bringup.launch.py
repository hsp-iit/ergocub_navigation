"""
Single entry point for the ergoCub navigation stack.

Selects a profile along two axes instead of shipping one launch file per
combination:

    ros2 launch ergocub_navigation bringup.launch.py world:=sim localization:=slam

The legacy launch_all*/launch_sim entry points are thin wrappers around this file
and pass the same arguments those files used to hardcode.
"""

from ergocub_navigation.launch_utils import include, pkg_share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WORLDS = ('robot', 'sim')
LOCALIZATIONS = ('amcl', 'slam', 'odom_only', 'none')

# Per-profile defaults. Anything here can still be overridden on the command line.
_NAV2_PARAMS = {
    ('robot', 'slam'): ('param', 'slam', 'robot', 'ergoCub_nav2_slam.yaml'),
    ('robot', 'odom_only'): ('param', 'ergoCub_nav2_odom_only.yaml'),
    ('robot', None): ('param', 'ergoCub_nav2.yaml'),
    ('sim', 'slam'): ('param', 'slam', 'sim', 'ergoCub_nav2_slam_sim.yaml'),
    ('sim', None): ('param', 'simulation', 'ergoCub_nav2_sim.yaml'),
}

_WORLD_DEFAULTS = {
    'robot': {
        'map': 'floor0_ergoCub_modded.yaml',
        'amcl_params': ('param', 'ergoCub_amcl_2.yaml'),
        'scan_params': ('param', 'scan.yaml'),
        'projectors': 'robot',
    },
    'sim': {
        'map': 'map_gz_sim_ionic.yaml',
        'amcl_params': ('param', 'simulation', 'ergoCub_amcl_sim.yaml'),
        'scan_params': ('param', 'simulation', 'scan.yaml'),
        'projectors': 'sim',
    },
}

# Optional components, defaulted per profile so that a bare `bringup.launch.py`
# reproduces how the real robot is actually run.
#
# The keepout filters, the plane detector and the depth pointcloud filter are all
# real-robot + static-map features: the keepout mask is a map artifact, and the
# other two need the RealSense. They are on for robot+amcl and off everywhere
# else, which is exactly what the pre-refactor entry points did.
_FEATURE_DEFAULTS = {
    ('robot', 'amcl'): {
        'use_keepout': 'true',
        'use_plane_detector': 'true',
        'use_pointcloud_filter': 'true',
    },
}
_FEATURE_FALLBACK = {
    'use_keepout': 'false',
    'use_plane_detector': 'false',
    'use_pointcloud_filter': 'false',
}


def _resolve(context, name, fallback):
    """Value of an argument, or the profile default when it was left empty."""
    value = LaunchConfiguration(name).perform(context)
    return value if value else fallback


def _bringup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    localization = LaunchConfiguration('localization').perform(context)
    if world not in WORLDS:
        raise RuntimeError(f'world:={world} is not one of {list(WORLDS)}')
    if localization not in LOCALIZATIONS:
        raise RuntimeError(
            f'localization:={localization} is not one of {list(LOCALIZATIONS)}')

    world_defaults = _WORLD_DEFAULTS[world]
    use_sim_time = 'true' if world == 'sim' else 'false'

    nav2_params = _resolve(
        context, 'nav2_params',
        pkg_share(*_NAV2_PARAMS.get((world, localization),
                                    _NAV2_PARAMS[(world, None)])))
    # odom-only serves a blank map rather than the world map
    default_map = ('empty_map.yaml' if localization in ('odom_only', 'none')
                   else world_defaults['map'])

    features = _FEATURE_DEFAULTS.get((world, localization), _FEATURE_FALLBACK)
    use_keepout = _resolve(context, 'use_keepout',
                           features.get('use_keepout', 'false'))
    use_plane_detector = _resolve(context, 'use_plane_detector',
                                  features.get('use_plane_detector', 'false'))
    use_pointcloud_filter = _resolve(context, 'use_pointcloud_filter',
                                     features.get('use_pointcloud_filter', 'false'))

    actions = [
        include('setup_robot.launch.py', {
            'use_sim_time': use_sim_time,
            'projectors': _resolve(context, 'projectors', world_defaults['projectors']),
            'scan_params': _resolve(context, 'scan_params',
                                    pkg_share(*world_defaults['scan_params'])),
            'odom_params': _resolve(context, 'odom_params',
                                    pkg_share('param', 'odom.yaml')),
            'use_pointcloud_filter': use_pointcloud_filter,
        }),
    ]

    if localization in ('amcl', 'odom_only'):
        actions.append(include('setup_localization.launch.py', {
            'use_sim_time': use_sim_time,
            'map': _resolve(context, 'map', default_map),
            'amcl_params': _resolve(context, 'amcl_params',
                                    pkg_share(*world_defaults['amcl_params'])),
            'use_amcl': 'true' if localization == 'amcl' else 'false',
            'static_map_odom_tf': 'true' if localization == 'odom_only' else 'false',
        }))
    elif localization == 'slam':
        actions.append(include('slam/slam_online_async.launch.py', {
            'use_sim_time': use_sim_time,
            'slam_params_file': _resolve(
                context, 'slam_params',
                pkg_share('param', 'slam', 'robot', 'slam_online_async.yaml')),
        }))

    actions.append(include('nav2_stack.launch.py', {
        'use_sim_time': use_sim_time,
        'params_file': nav2_params,
        'use_keepout': use_keepout,
        'keepout_mask': _resolve(
            context, 'keepout_mask',
            pkg_share('maps', 'floor0_ergoCub_modded_keepout_full.yaml')),
    }, condition=IfCondition(LaunchConfiguration('use_nav2'))))

    actions.append(include('plane_detector.launch.py', {
        'use_sim_time': use_sim_time,
    }, condition=IfCondition(use_plane_detector)))

    actions.append(include('ergoCub_rviz.launch.py', {
        'use_sim_time': use_sim_time,
    }, condition=IfCondition(LaunchConfiguration('rviz'))))

    actions.append(Node(
        package='ergocub_navigation',
        executable='planner_trigger_server',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_planner_trigger')),
    ))

    return actions


def generate_launch_description():
    # Arguments default to '' and are filled in per profile by _bringup, so that
    # "unset" is distinguishable from "explicitly set to the robot default".
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='robot', choices=list(WORLDS),
            description='Real robot or Gazebo simulation; also sets use_sim_time'),
        DeclareLaunchArgument(
            'localization', default_value='amcl', choices=list(LOCALIZATIONS),
            description='amcl against a map, slam_toolbox mapping, odom_only '
                        '(blank map + identity map->odom), or none'),
        DeclareLaunchArgument(
            'map', default_value='',
            description='Map YAML under maps/ or an absolute path (default: per world)'),
        DeclareLaunchArgument(
            'nav2_params', default_value='',
            description='nav2 parameters file (default: per world and localization)'),
        DeclareLaunchArgument(
            'amcl_params', default_value='',
            description='AMCL parameters file (default: per world)'),
        DeclareLaunchArgument(
            'slam_params', default_value='',
            description='slam_toolbox parameters file'),
        DeclareLaunchArgument(
            'scan_params', default_value='',
            description='scan_node parameters file (default: per world)'),
        DeclareLaunchArgument(
            'odom_params', default_value='',
            description='odom_node parameters file'),
        DeclareLaunchArgument(
            'projectors', default_value='',
            description='pointcloud_to_laserscan projector set (default: per world)'),
        DeclareLaunchArgument(
            'rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument(
            'use_nav2', default_value='true', description='Start the nav2 stack'),
        DeclareLaunchArgument(
            'use_keepout', default_value='',
            description='Start the costmap filter servers that serve the keepout '
                        'mask (default: true on robot+amcl, false otherwise)'),
        DeclareLaunchArgument(
            'keepout_mask', default_value='',
            description='Map YAML served as the keepout filter mask'),
        DeclareLaunchArgument(
            'use_plane_detector', default_value='',
            description='Start the floor-plane detector '
                        '(default: true on robot+amcl, false otherwise)'),
        DeclareLaunchArgument(
            'use_pointcloud_filter', default_value='',
            description='Start the IMU-gated depth pointcloud filter '
                        '(default: true on robot+amcl, false otherwise)'),
        DeclareLaunchArgument(
            'use_planner_trigger', default_value='true',
            description='Start the YARP planner trigger server'),
        OpaqueFunction(function=_bringup),
    ])
