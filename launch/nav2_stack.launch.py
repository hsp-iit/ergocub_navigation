# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ergocub_navigation.launch_utils import bool_param, pkg_share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    bt_xml = LaunchConfiguration('bt_xml')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    use_respawn = LaunchConfiguration('use_respawn')
    use_keepout = LaunchConfiguration('use_keepout')
    keepout_mask = LaunchConfiguration('keepout_mask')
    keepout_topic = LaunchConfiguration('keepout_topic')
    global_frame = LaunchConfiguration('global_frame')

    lifecycle_nodes = ['controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower']

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions.
    #
    # default_nav_to_pose_bt_xml is injected here rather than stored in the params
    # YAML, which is what used to force an absolute source-tree path. It is given
    # as a full dotted path on purpose: RewrittenYaml only *inserts* a missing key
    # when the rewrite path contains 'ros__parameters' (see add_params), whereas a
    # bare leaf name can only replace a key that already exists.
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': bt_xml}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_cmds = [
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='False',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'params_file', default_value=pkg_share('param', 'ergoCub_nav2.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes'),
        DeclareLaunchArgument(
            'bt_xml',
            default_value=pkg_share(
                'behavior_trees',
                'navigate_to_pose_w_replanning_and_recovery_decorator.xml'),
            description='Behavior tree XML used by bt_navigator'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),
        DeclareLaunchArgument(
            'use_composition', default_value='False',
            description='Use composed bringup if True'),
        DeclareLaunchArgument(
            'container_name', default_value='nav2_container',
            description='Name of the container that nodes will load in if use_composition'),
        DeclareLaunchArgument(
            'use_respawn', default_value='False',
            description='Whether to respawn if a node crashes. '
                        'Applied when composition is disabled.'),
        DeclareLaunchArgument(
            'use_keepout', default_value='False',
            description='Start the costmap filter servers that serve the keepout mask'),
        DeclareLaunchArgument(
            'keepout_mask',
            default_value=pkg_share('maps', 'floor0_ergoCub_modded_keepout_full.yaml'),
            description='Map YAML served as the keepout filter mask'),
        DeclareLaunchArgument(
            'keepout_topic', default_value='/keepout_filter_mask',
            description='Topic the keepout filter mask is published on'),
        DeclareLaunchArgument(
            'global_frame', default_value='map',
            description='Frame the keepout filter mask is published in'),
    ]

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'use_sim_time': bool_param(use_sim_time)},
                            {'autostart': bool_param(autostart)},
                            {'node_names': lifecycle_nodes}])
        ]
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_waypoint_follower',
                plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{'use_sim_time': bool_param(use_sim_time),
                             'autostart': bool_param(autostart),
                             'node_names': lifecycle_nodes}]),
        ],
    )

    # Costmap filters serving the keepout mask. Previously this block was live in
    # nav2_stack.launch.py, commented out in three of its copies and deleted from a
    # fourth; it is now a single conditional group.
    load_keepout_filters = GroupAction(
        condition=IfCondition(use_keepout),
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='filter_mask_server',
                output='screen',
                emulate_tty=True,
                parameters=[{'use_sim_time': bool_param(use_sim_time)},
                            {'frame_id': global_frame},
                            {'topic_name': keepout_topic},
                            {'yaml_filename': keepout_mask}]),
            Node(
                package='nav2_map_server',
                executable='costmap_filter_info_server',
                name='costmap_filter_info_server',
                output='screen',
                emulate_tty=True,
                parameters=[configured_params]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_costmap_filters',
                output='screen',
                emulate_tty=True,
                parameters=[{'use_sim_time': bool_param(use_sim_time)},
                            {'autostart': True},
                            {'node_names': ['filter_mask_server',
                                            'costmap_filter_info_server']}]),
        ]
    )

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    for cmd in declare_cmds:
        ld.add_action(cmd)
    ld.add_action(load_keepout_filters)
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld
