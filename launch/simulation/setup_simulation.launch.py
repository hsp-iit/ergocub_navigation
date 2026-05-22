from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    set_yarp_clock = SetEnvironmentVariable(
        name="YARP_CLOCK",
        value="/clock"
    )

    gazebo = ExecuteProcess(
        cmd=[
            "gz", "sim",
            "/home/ecub_docker/ros2_workspace/src/ergocub_navigation/sim/ionic.sdf",
            "--verbose",
            "-r"
        ],
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    spawn_robot_after_clock = ExecuteProcess(
        cmd=[
            "bash", "-lc",
            """
            until timeout 2 ros2 topic echo /clock --once > /dev/null 2>&1; do
                echo "Waiting for /clock before spawning ergoCub..."
                sleep 0.5
            done

            ros2 run ros_gz_sim create \
              -file /home/ecub_docker/robotology-superbuild/src/ergocub-software/urdf/ergoCub/robots/ergoCubGazeboSN001_minContacts/model.urdf \
              -name ergocub \
              -z 0.8
            """
        ],
        output="screen",
    )

    wholebodydynamics = ExecuteProcess(
        cmd=[
            "yarprobotinterface",
            "--config",
            "/home/ecub_docker/robotology-superbuild/src/ergocub-software/urdf/ergoCub/conf/launch_wholebodydynamics_ecub.xml"
        ],
        output="screen",
    )

    head_imu = ExecuteProcess(
        cmd=[
            "yarprobotinterface",
            "--config",
            "/home/ecub_docker/ros2_workspace/src/ergocub_navigation/config/yarp/simulation/head_imu_ros2.xml"
        ],
        output="screen",
    )

    depth_compressed_ros2 = ExecuteProcess(
        cmd=[
            "yarprobotinterface",
            "--config",
            "/home/ecub_docker/ros2_workspace/src/ergocub_navigation/config/yarp/simulation/depth_compressed_ros2.xml"
        ],
        output="screen",
    )

    lidar_compressed_ros2 = ExecuteProcess(
        cmd=[
            "yarprobotinterface",
            "--config",
            "/home/ecub_docker/ros2_workspace/src/ergocub_navigation/config/yarp/simulation/lidar_compressed_ros2.xml"
        ],
        output="screen",
    )

    joint_states_republisher = ExecuteProcess(
        cmd=[
            "python3",
            "/home/ecub_docker/ros2_workspace/src/ergocub_navigation/src/sim/joint_states_republisher.py",
        ],
        output="screen",
    )

    start_modules_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot_after_clock,
            on_exit=[
                TimerAction(period=1.5, actions=[wholebodydynamics]),
                TimerAction(period=2.5, actions=[head_imu]),
                TimerAction(period=3.5, actions=[depth_compressed_ros2]),
                TimerAction(period=4.5, actions=[lidar_compressed_ros2]),
                TimerAction(period=5.5, actions=[joint_states_republisher]),
            ],
        )
    )

    return LaunchDescription([
        set_yarp_clock,
        gazebo,
        clock_bridge,
        spawn_robot_after_clock,
        start_modules_after_spawn
    ])
