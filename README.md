# ergocub_navigation
Run these following commands on the robot laptop, if not explicitily expressed to SSH to other locations.

## Before Starting
Be assured that ROS2 is properly set up and all the PCs clocks are synchronized (use `ntpdate -b <IP_TO_NTP_SERVER>` on ergoCub robots is 10.0.2.1).

The ROS2 setup must use `cyclone_dds` as DDS, instead of the ROS2 default, and use the `cyclonedds.xml` config file on each PC, with the proper IPs set (guide here: https://cyclonedds.io/docs/cyclonedds/latest/config/index.html)

## Install on ergoCub Laptop
0) Install ROS2 following [this guide](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debs.html)
1) Install Nav2 following [this guide](https://docs.nav2.org/development_guides/build_docs/index.html)
2) Create your workspace under `$ROBOT_CODE/hsp/ros2_workspace/src`
3) In /src clone this repo `git clone https://github.com/hsp-iit/ergocub_navigation` and these ones: `https://github.com/hsp-iit/bt_nav2_ergocub`, `https://github.com/ros-perception/pointcloud_to_laserscan` (switch to branch humble)
4) in the folder `ros2_workspace` execute the command: ```colcon build --symlink-install```


## How to run on ergoCubSN000

0- Turn on the robot and use the `yarpmanager` to run the yarpserver and its nodes while the robot is suspended on the crane.

1- SSH to the robot torso: `ssh -X ergocub-torso` from the laptop. Set up the robot using the command `yarprobotinterface --config ergocub_wbd_ros2.xml` in the ergoCub torso `robot_configuration` folder (you can go there using the alias `gotoRobotConfigurationFolder`)

2- Calibrate the FT sensors by:

```
yarp rpc /wholeBodyDynamics/rpc
>> calib all 300
```

3- SSH to the torso go to the walking-controllers folder and switch to the proper branch for navigation:
```
cd /usr/local/src/robot/robotology-superbuild/src/walking-controllers && git switch nav_integration && goToBuildSuperbuild && make install -j4
```
then launch the following command:
```
WalkingModule --from /usr/local/src/robot/robotology-superbuild/src/walking-controllers/src/WalkingModule/app/robots/ergoCubSN001/dcm_walking_iFeel_joint_retargeting_navigation_strict.ini
```
For further instruction on how to use the walking-controllers, look at this link: https://github.com/robotology/walking-controllers

4- While the robot is still suspended on the crane, type the following command:

```
yarp rpc /walking-coordinator/rpc
>> prepareRobot
```

5- After the robot is prepared correctly, lower the robot on the ground and activate the walking mode (in the same terminal as the one in the previous point):
```
yarp rpc /walking-coordinator/rpc
>> startWalking
```
## Sensors Setup

1- SSH to the robot head: `ssh -X ergocub-head` from the laptop. Set up the robot using the command `yarprobotinterface --config sensors.xml` in the ergoCub head `robot_configuration` folder (you can go there using the alias `gotoRobotConfigurationFolder`)

2- *ON the LAPTOP* Launch the ROS2 yarp devices by typing each of the following commands inside the folder: `cd $ROBOT_CODE && cd /ros2_ws/src/ergocub_navigation/config/yarp` and typing each of the following commands in a separate terminal:

```
yarprobotinterface --config depth_compressed_ros2.xml
```

```
yarprobotinterface --config head_imu_ros2.xml
```

```
yarprobotinterface --config lidar_compressed_ros2.xml
```


## ROS2 application launch

1- Run the BT interface with the walking-controllers by: 
  ```
  ros2 run ergocub_navigation planner_trigger_server
  ```

2- Launch the navigation stack using: 
  ```
  ros2 launch ergocub_navigation launch_all.launch.py
  ```

3- Run the navigation interface with the walking-controller by: 
  ```
  ros2 launch ergocub_navigation path_converter.launch.py
  ```

4- (optional) To display planned footsteps on RViz: 
  ```
  ros2 launch ergocub_navigation footsteps_planner_viewer.launch.py
  ```


