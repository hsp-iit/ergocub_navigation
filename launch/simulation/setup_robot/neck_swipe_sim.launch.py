import os

import launch
import launch.actions
import launch.events

import launch_ros
import launch_ros.actions
import launch_ros.events

from launch import LaunchDescription

import lifecycle_msgs.msg

from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    ld = launch.LaunchDescription()
    params_file = LaunchConfiguration('params_file')
    params_file_arg = DeclareLaunchArgument('params_file',
                                            default_value=str(
                                                os.path.join(
                                                    get_package_share_directory('ergocub_navigation'),
                                                    'param',
                                                    'neck_controller.yaml')),
                                            description='name or path to the parameters file to use.')

    neck_controller_node = launch_ros.actions.LifecycleNode(
            name = 'neck_controller_node',
            namespace='',
            package='ergocub_navigation',
            executable='phase_detector',
            parameters=[params_file],
            output='screen'
        )
    
    sensor_configure_event = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(neck_controller_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    sensor_activate_event  = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=neck_controller_node, 
            #start_state = 'configuring',
            goal_state='inactive',
            entities=[
                launch.actions.LogInfo(msg="-- Activating --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(neck_controller_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
            handle_once=True
        )
    )
    
    sensor_unconfigure_event  = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=neck_controller_node, 
            goal_state='unconfigured',
            entities=[
                launch.actions.LogInfo(msg="-- Unconfigured --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(neck_controller_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )
    
    sensor_finalized_event = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node = neck_controller_node, 
            goal_state = 'finalized',
            entities=[
                launch.actions.LogInfo(
                    msg="--Finalizing--"),
                launch.actions.EmitEvent(event=launch.events.Shutdown(
                    reason="Could not start properly"))
            ],
        )
    )

    ld.add_action(params_file_arg)
    ld.add_action(neck_controller_node)
    ld.add_action(sensor_configure_event)
    ld.add_action(sensor_activate_event)
    #ld.add_action(sensor_unconfigure_event)
    ld.add_action(sensor_finalized_event)
    
    return ld