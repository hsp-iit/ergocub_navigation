from ast import arg
import imp
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    #use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    robot_urdf = robot_urdf="/home/ecub_docker/robotology-superbuild/src/ergocub-software/urdf/ergoCub/robots/ergoCubGazeboV1/model.urdf"
    with open(robot_urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
            
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc , 'publish_frequency': 200.0, 'use_sim_time': True}],
            arguments=[robot_urdf])
    ])

