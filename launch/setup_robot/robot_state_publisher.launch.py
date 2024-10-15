from ast import arg
import imp
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    #use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    #robot_urdf = '/usr/local/src/robot/robotology-superbuild/src/ergocub-software/urdf/ergoCub/robots/ergoCubSN001/model.urdf'
    #robot_urdf = '/home/ecub_docker/ergocub-software/urdf/ergoCub/robots/ergoCubSN002/model.urdf'
    ROBOT_NAME = os.getenv('YARP_ROBOT_NAME')
    if ROBOT_NAME == None:
        robot_urdf = '/home/ecub_docker/ergocub-software/urdf/ergoCub/robots/ergoCubSN002/model.urdf'
    else: 
        robot_urdf = f'/usr/local/src/robot/robotology-superbuild/src/ergocub-software/urdf/ergoCub/robots/{ROBOT_NAME}/model.urdf'

    with open(robot_urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
            
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc , 'publish_frequency': 200.0, 'use_sim_time': False}],
            arguments=[robot_urdf])
    ])

