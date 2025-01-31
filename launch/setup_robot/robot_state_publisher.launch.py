from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    #use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    robotname = os.environ['YARP_ROBOT_NAME']
    if robotname == 'ergoCubSN002':
        robot_urdf = '/home/ecub_docker/ergocub-software/urdf/ergoCub/robots/ergoCubSN002/model.urdf'
    else:
        robot_urdf = f'/usr/local/src/robot/robotology-superbuild/src/ergocub-software/urdf/ergoCub/robots/{robotname}/model.urdf'

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

