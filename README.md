# ergocub_navigation

## How to run on ergoCubSN000

0- Turn on the robot on with yarpmanager running the yarpserver, while placed on the crane.

1- SSH to the robot torso. Set up the robot using the command `yarprobotinterface --config ergocub_wbd_ros2.xml` in the ergoCubSN000 `robot_configuration` folder (you can go there using the alias `gotoRobotConfigurationFolder`)

2- Run the merging of the yarp ports for the navigation using the alias `feet_ports_merge`

3- Calibrate the FT sensors:

```
yarp rpc /wholeBodyDynamics/rpc
>> calib all 300
```

4- SSH to the torso and launch the WalkingModue (after checking out to the proper branch for navigation: simonemic/ergoCubSN000 and making and installing).

5- While the robot is on the crane, type the following command:

```

```

## ROS2 setup


