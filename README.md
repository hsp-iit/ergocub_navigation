# ergocub_navigation
Run these following commands on the robot laptop, if not explicitily expressed to SSH to other locations.

## How to run on ergoCubSN000

0- Turn on the robot and use the `yarpmanager` to run the yarpserver while the robot is suspended on the crane.

1- SSH to the robot torso: `ssh -X ergocub-torso` from the laptop. Set up the robot using the command `yarprobotinterface --config ergocub_wbd_ros2.xml` in the ergoCubSN000 torso `robot_configuration` folder (you can go there using the alias `gotoRobotConfigurationFolder`)

2- Run the merging of the yarp ports for the navigation, using the alias `feet_ports_merge` on another terminal.

3- Calibrate the FT sensors by:

```
yarp rpc /wholeBodyDynamics/rpc
>> calib all 300
```
/walking-coordinator/rpc
4- SSH to the torso and launch the `WalkingModue` (after checking out to the proper branch for navigation: simonemic/ergoCubSN000 and making and installing it).
For further instruction on how to use the walking-controllers, look at this link: https://github.com/robotology/walking-controllers

5- While the robot is still suspended on the crane, type the following command:

```
yarp rpc /walking-coordinator/rpc
>> prepareRobot
```

6- After the robot is prepared correctly, lower the robot on the ground and activate the walking mode (in the same terminal as the one in the previous point):
```
yarp rpc /walking-coordinator/rpc
>> startWalking
```
## Sensors Setup

1- SSH to the robot head: `ssh -X ergocub-head` from the laptop. Set up the robot using the command `yarprobotinterface --config sensors.xml` in the ergoCubSN000 head `robot_configuration` folder (you can go there using the alias `gotoRobotConfigurationFolder`)

2- Launch the ROS2 yarp devices by typing each of the following commands inside the folder: `cd $ROBOT_CODE && cd /ros2_ws/src/ergocub_navigation/config/yarp` and typing each of the following commands in a separate terminal:

`yarprobotinterface --config depth_compressed_ros2.xml`

`yarprobotinterface --config head_imu_ros2.xml`  

`yarprobotinterface --config lidar_compressed_ros2.xml`


## ROS2 setup

1- Run the BT interface with the walking-controllers by: `ros2 run ergocub_navigation planner_trigger_server`

2- Launch the navigation stack using: `ros2 launch ergocub_navigation launch_all.launch.py`

3- Run the navigation interface with the walking-controller by: `ros2 run ergocub_navigation path_converter`

4- (optional) To display planned footsteps on RViz: `ros2 launch ergocub_navigation footsteps_planner_viewer`


