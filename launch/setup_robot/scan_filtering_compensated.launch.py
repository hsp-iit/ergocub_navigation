import os

import launch
import launch.actions
import launch.events

import launch_ros
import launch_ros.actions
import launch_ros.events

from launch import LaunchDescription
from launch_ros.actions import Node

import lifecycle_msgs.msg

from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    ld = launch.LaunchDescription()

    scan_param_dir = launch.substitutions.LaunchConfiguration(
        'scan_param_dir',
        default=os.path.join(
            get_package_share_directory('ergocub_navigation'),
            'param',
            'scan.yaml'))

    scan_node = launch_ros.actions.LifecycleNode(
            name = 'scan_node',
            namespace='',
            package='ergocub_navigation',
            executable='scan_filter',
            output='screen',
            parameters=[scan_param_dir]
        )
    
    to_inactive = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(scan_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    
    from_unconfigured_to_inactive = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=scan_node, 
            goal_state='unconfigured',
            entities=[
                launch.actions.LogInfo(msg="-- Unconfigured --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(scan_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )

    from_inactive_to_active = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=scan_node, 
            start_state = 'configuring',
            goal_state='inactive',
            entities=[
                launch.actions.LogInfo(msg="-- Inactive --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(scan_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )


    ld.add_action(from_unconfigured_to_inactive)
    ld.add_action(from_inactive_to_active)
    
    ld.add_action(scan_node)
    ld.add_action(to_inactive)
    

    return LaunchDescription([
        #DeclareLaunchArgument(
        #    'use_sim_time',
        #    default_value='false',
        #    description='Use simulation (Gazebo) clock if true'
        #),
        DeclareLaunchArgument(
            name='scanner', 
            default_value='scanner',
            description='Namespace for sample topics'
        ),
        ld,
        
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/compensated_pc2'),
                        ('scan', '/filtered_scan_compensated')],
            parameters=[{
                'target_frame': 'geometric_unicycle',    #virtual_unicycle_base
                'transform_tolerance': 0.03,        #0.01
                'min_height': -0.2,  #-300
                'max_height': 3.0,  #300
                'angle_min': -2.7,   #-2.61799,  # -M_PI
                'angle_max': 2.7,    #2.61799,  # M_PI
                'angle_increment': 0.003926991,  # 2M_PI/360.0
                'scan_time': 0.05,
                'range_min': 0.5,
                'range_max': 30.0,
                'use_inf': True,
                'inf_epsilon': 1.0
                #'concurrency_level': 2
            }],
            name='pointcloud_to_laserscan'
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/compensated_pc2_2'),
                        ('scan', '/rear_scan_compensated_right')],
            parameters=[{
                'target_frame': 'geometric_unicycle',    #virtual_unicycle_base
                'transform_tolerance': 0.03,        #0.01
                'min_height': -0.2,  #-300
                'max_height': 3.0,  #300
                'angle_min': -2.61799,   #-3.141592653,  # -M_PI
                'angle_max': -1.0,    #3.141592653,  # M_PI
                'angle_increment': 0.003926991,  # 2M_PI/360.0
                'scan_time': 0.05,
                'range_min': 0.2,
                'range_max': 30.0,
                'use_inf': True,
                'inf_epsilon': 1.0
                #'concurrency_level': 2
            }],
            name='rear_pointcloud_to_laserscan_right'
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/compensated_pc2_3'),
                        ('scan', '/rear_scan_compensated_left')],
            parameters=[{
                'target_frame': 'geometric_unicycle',    #virtual_unicycle_base
                'transform_tolerance': 0.03,        #0.01
                'min_height': -0.2,  #-300
                'max_height': 3.0,  #300
                'angle_min': 1.0,   #-3.141592653,  # -M_PI
                'angle_max': 2.61799,    #3.141592653,  # M_PI
                'angle_increment': 0.003926991,  # 2M_PI/360.0
                'scan_time': 0.05,
                'range_min': 0.2,
                'range_max': 30.0,
                'use_inf': True,
                'inf_epsilon': 1.0
                #'concurrency_level': 2
            }],
            name='rear_pointcloud_to_laserscan_left'
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/compensated_pc2'),
                        ('scan', '/scan_compensated_front')],
            parameters=[{
                'target_frame': 'geometric_unicycle',    #virtual_unicycle_base
                'transform_tolerance': 0.03,        #0.01
                'min_height': -0.2,  #-300
                'max_height': 3.0,  #300
                'angle_min': -1.4,   #-3.141592653,  # -M_PI
                'angle_max': 1.4,    #3.141592653,  # M_PI
                'angle_increment': 0.003926991,  # 2M_PI/360.0
                'scan_time': 0.05,
                'range_min': 0.5,
                'range_max': 30.0,
                'use_inf': True,
                'inf_epsilon': 1.0
                #'concurrency_level': 2
            }],
            name='rear_pointcloud_to_laserscan_left'
        )
    ])
    
