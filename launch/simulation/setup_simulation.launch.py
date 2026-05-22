from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    set_yarp_clock = SetEnvironmentVariable(
        name="YARP_CLOCK",
        value="/clock"
    )

    gazebo = ExecuteProcess(
        cmd=[
            "gz", "sim",
            "/home/ecub_docker/ros2_workspace/src/ergocub_navigation/sim/warehouse.sdf",
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

            sleep 1.5

            yarprobotinterface --config /home/ecub_docker/robotology-superbuild/src/ergocub-software/urdf/ergoCub/conf/launch_wholebodydynamics_ecub.xml
            """
        ],
        output="screen",
    )

    return LaunchDescription([
        set_yarp_clock,
        gazebo,
        clock_bridge,
        spawn_robot_after_clock,
    ])
