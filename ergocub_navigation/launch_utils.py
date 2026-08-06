"""
Shared helpers for the ergocub_navigation launch files.

Installed to site-packages by ``ament_python_install_package`` so that launch
files can ``from ergocub_navigation.launch_utils import ...`` from both the
source and the install space.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
import lifecycle_msgs.msg

PACKAGE = 'ergocub_navigation'


def pkg_share(*parts):
    """Return an absolute path inside share/ergocub_navigation."""
    return os.path.join(get_package_share_directory(PACKAGE), *parts)


def include(rel_path, launch_arguments=None, condition=None):
    """
    Include a launch file addressed relative to share/ergocub_navigation/launch.

    Paths are joined with os.path.join rather than string concatenation, which is
    what previously produced targets such as 'launchpath_converter.launch.py'.
    """
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg_share('launch', *rel_path.split('/'))),
        launch_arguments=list((launch_arguments or {}).items()),
        condition=condition,
    )


def bool_param(value):
    """Wrap a LaunchConfiguration so it reaches the node as a bool, not a string."""
    return ParameterValue(value, value_type=bool)


def lifecycle_bringup(name, executable, param_file, extra_parameters=None,
                      package=PACKAGE, namespace='', output='screen'):
    """
    Build a LifecycleNode plus the configure/activate event handlers.

    Replaces the block that was copy-pasted into every lifecycle launch file.
    Returns the actions in the same order the originals added them, so the
    startup sequence is unchanged.
    """
    parameters = [param_file]
    if extra_parameters:
        parameters.append(extra_parameters)

    node = LifecycleNode(
        name=name,
        namespace=namespace,
        package=package,
        executable=executable,
        output=output,
        parameters=parameters,
    )

    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    on_unconfigured = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            goal_state='unconfigured',
            entities=[
                LogInfo(msg=f'-- {name}: Unconfigured --'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )

    on_inactive = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg=f'-- {name}: Inactive --'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    return [on_unconfigured, on_inactive, node, configure]


# --- pointcloud_to_laserscan projectors -------------------------------------
#
# Values transcribed verbatim from the four inline dicts that used to live in
# setup_robot/scan_filtering_compensated.launch.py and the single one in
# simulation/setup_robot/scan_filtering_sim.launch.py. Only the keys that
# actually varied between them are listed per projector; the rest are shared.

SCAN_PROJECTOR_COMMON = {
    'target_frame': 'geometric_unicycle',
    'transform_tolerance': 0.03,
    'min_height': -0.2,
    'max_height': 3.0,
    'angle_increment': 0.003926991,  # 2*M_PI/1600
    'scan_time': 0.05,
    'range_max': 30.0,
    'use_inf': True,
    'inf_epsilon': 1.0,
}

SCAN_PROJECTORS = {
    # Real robot: one wide scan for the costmap, a narrow front scan, and two
    # rear wedges fed by the second and third compensated clouds.
    'robot': [
        {
            'name': 'pointcloud_to_laserscan',
            'cloud_in': '/compensated_pc2',
            'scan': '/filtered_scan_compensated',
            'angle_min': -2.7,
            'angle_max': 2.7,
            'range_min': 0.5,
        },
        {
            'name': 'rear_pointcloud_to_laserscan_right',
            'cloud_in': '/compensated_pc2_2',
            'scan': '/rear_scan_compensated_right',
            'angle_min': -2.61799,
            'angle_max': -1.0,
            'range_min': 0.2,
        },
        {
            'name': 'rear_pointcloud_to_laserscan_left',
            'cloud_in': '/compensated_pc2_3',
            'scan': '/rear_scan_compensated_left',
            'angle_min': 1.0,
            'angle_max': 2.61799,
            'range_min': 0.2,
        },
        {
            # Was also named 'rear_pointcloud_to_laserscan_left', colliding with
            # the projector above; renamed to match the topic it publishes.
            'name': 'front_pointcloud_to_laserscan',
            'cloud_in': '/compensated_pc2',
            'scan': '/scan_compensated_front',
            'angle_min': -1.4,
            'angle_max': 1.4,
            'range_min': 0.5,
        },
    ],
    # Simulation: a single full-circle projector, no rear wedges.
    'sim': [
        {
            'name': 'pointcloud_to_laserscan',
            'cloud_in': '/compensated_pc2',
            'scan': '/filtered_scan_compensated',
            'angle_min': -3.141592653,
            'angle_max': 3.141592653,
            'range_min': 0.2,
        },
    ],
}


def scan_projector_nodes(profile, use_sim_time):
    """Node actions for every pointcloud_to_laserscan projector in a profile."""
    from launch_ros.actions import Node

    nodes = []
    for spec in SCAN_PROJECTORS[profile]:
        params = dict(SCAN_PROJECTOR_COMMON)
        params.update({k: v for k, v in spec.items()
                       if k not in ('name', 'cloud_in', 'scan')})
        params['use_sim_time'] = bool_param(use_sim_time)
        nodes.append(Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name=spec['name'],
            remappings=[('cloud_in', spec['cloud_in']), ('scan', spec['scan'])],
            parameters=[params],
        ))
    return nodes
