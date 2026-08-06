"""
Bring up gz-sim with ergoCub spawned and its YARP sensor modules running.

Paths are resolved from the package share directory and the robotology-superbuild
environment variables; they used to be absolute /home/ecub_docker/... literals.
"""

import os

from ergocub_navigation.launch_utils import pkg_share
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# yarprobotinterface device configs started once the robot is in the world, in
# order, one second apart. The original file staggered them 1.5s..5.5s.
YARP_MODULES = ('head_imu_ros2.xml',
                'depth_compressed_ros2.xml',
                'lidar_compressed_ros2.xml')


def _robot_urdf(context):
    model = LaunchConfiguration('model').perform(context)
    superbuild = LaunchConfiguration('superbuild_src').perform(context)
    return os.path.join(superbuild, 'src', 'ergocub-software', 'urdf', 'ergoCub',
                        'robots', model, 'model.urdf')


def _simulation(context, *args, **kwargs):
    superbuild = LaunchConfiguration('superbuild_src').perform(context)
    robot_urdf = _robot_urdf(context)
    world = LaunchConfiguration('world_sdf').perform(context)

    for path in (robot_urdf, world):
        if not os.path.isfile(path):
            raise RuntimeError(f'File not found: {path}')

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world, '--verbose', '-r'],
        output='screen',
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    spawn_robot_after_clock = ExecuteProcess(
        cmd=['bash', '-lc', f"""
            until timeout 2 ros2 topic echo /clock --once > /dev/null 2>&1; do
                echo "Waiting for /clock before spawning ergoCub..."
                sleep 0.5
            done

            ros2 run ros_gz_sim create \
              -file {robot_urdf} \
              -name ergocub \
              -z {LaunchConfiguration('spawn_height').perform(context)}
            """],
        output='screen',
    )

    wholebodydynamics = ExecuteProcess(
        cmd=['yarprobotinterface', '--config',
             os.path.join(superbuild, 'src', 'ergocub-software', 'urdf', 'ergoCub',
                          'conf', 'launch_wholebodydynamics_ecub.xml')],
        output='screen',
    )

    staged = [TimerAction(period=1.5, actions=[wholebodydynamics])]
    for i, module in enumerate(YARP_MODULES):
        staged.append(TimerAction(period=2.5 + i, actions=[ExecuteProcess(
            cmd=['yarprobotinterface', '--config',
                 pkg_share('yarp', 'simulation', module)],
            output='screen')]))
    staged.append(TimerAction(period=2.5 + len(YARP_MODULES), actions=[ExecuteProcess(
        cmd=['python3', pkg_share('sim', 'joint_states_republisher.py')],
        output='screen')]))

    return [
        gazebo,
        clock_bridge,
        spawn_robot_after_clock,
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_robot_after_clock,
            on_exit=staged,
        )),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model', default_value=os.environ.get('YARP_ROBOT_NAME', ''),
            description='Robot model directory under ergocub-software/urdf/ergoCub/robots'),
        DeclareLaunchArgument(
            'superbuild_src',
            default_value=os.environ.get('ROBOTOLOGY_SUPERBUILD_SOURCE_DIR', ''),
            description='Path to the robotology-superbuild source directory'),
        DeclareLaunchArgument(
            'world_sdf', default_value=pkg_share('sim', 'ionic.sdf'),
            description='Gazebo world SDF to load'),
        DeclareLaunchArgument(
            'spawn_height', default_value='0.8',
            description='Z offset the robot is spawned at'),
        SetEnvironmentVariable(name='YARP_CLOCK', value='/clock'),
        OpaqueFunction(function=_simulation),
    ])
