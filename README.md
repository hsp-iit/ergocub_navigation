# ergocub_navigation

ROS2 navigation stack for ergoCub humanoid robots, bridging Nav2 with the YARP-based walking controllers.

Run the following commands on the robot laptop unless explicitly stated otherwise.

## Before Starting

Ensure ROS2 is properly set up and all PC clocks are synchronized (`ntpdate -b <IP_TO_NTP_SERVER>`; on ergoCub robots this is `10.0.2.1`).

The ROS2 setup must use `cyclone_dds` as the DDS middleware. Configure each PC with the `cyclonedds.xml` file in this repo, with the proper IPs set ([guide](https://cyclonedds.io/docs/cyclonedds/latest/config/index.html)).

## Install on ergoCub Laptop

0. Install ROS2 following [this guide](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debs.html)
1. Install Nav2 following [this guide](https://docs.nav2.org/development_guides/build_docs/index.html)
2. Create your workspace under `$ROBOT_CODE/hsp/ros2_workspace/src`
3. Clone this repo and its dependencies into `/src`:
   - `git clone https://github.com/hsp-iit/ergocub_navigation`
   - `https://github.com/hsp-iit/bt_nav2_ergocub`
   - `https://github.com/ros-perception/pointcloud_to_laserscan` (branch `humble`)
4. From the `ros2_workspace` folder: `colcon build --symlink-install`


## Modules

### odom\_node

Bridges the YARP virtual-unicycle state (from the walking controller) to a ROS2 `nav_msgs/Odometry` topic. This is the robot's odometry source for Nav2.

**Launch:** `ros2 launch ergocub_navigation setup_robot/odom.launch.py`
**Params:** `config/param/odom.yaml`

---

### scan\_filter

Compensates and filters the 2D lidar scan, removing points that hit the robot's own body based on TF transforms.

**Launch:** `ros2 launch ergocub_navigation setup_robot/scan_filtering_compensated.launch.py`
**Params:** `config/param/scan.yaml`

---

### pointcloud\_filter

Filters the depth camera pointcloud, cropping out the robot's own body to prevent self-collisions in the costmap.

**Launch:** `ros2 launch ergocub_navigation setup_robot/pointcloud_filter.launch.py`
**Params:** `config/param/depth_filter.yaml`

---

### path\_converter

Converts the Nav2 global plan into walking-controller commands and sends them via YARP to the walking-coordinator. This is the main bridge between Nav2 and the bipedal walking controller.

**Launch:** `ros2 launch ergocub_navigation setup_robot/path_converter.launch.py`
**Params:** `config/param/path_converter.yaml`

---

### planner\_trigger\_server

Exposes a ROS2 action server that the Behavior Tree (bt\_nav2\_ergocub) calls. Forwards navigation triggers to the walking-controller via YARP.

**Run:** `ros2 run ergocub_navigation planner_trigger_server`

---

### head\_orientation\_controller

Controls the robot's head/neck gaze direction along the Nav2 planned path. Subscribes to `/plan` and sends joint commands to the ergocub-head-controller via YARP RPC, keeping the robot looking ahead toward the next waypoint. Returns the head to a neutral pose when navigation ends.

**Launch:** `ros2 launch ergocub_navigation setup_robot/head_orientation_controller.launch.py`
**Params:** `config/param/head_orientation_controller.yaml`

---

### human\_pose\_goal\_generator

Generates Nav2 navigation goals based on detected human poses received via YARP. Useful for human-following scenarios.

**Launch:** `ros2 launch ergocub_navigation human_pose_goal_generator.launch.py`
**Params:** `config/param/human_pose_goal_generator.yaml`

---

### plane\_detector

Detects ground-plane and performs a camera orientation calibration based on floor RANSAC. Republishes a TF frame called `compensated_realsense_frame` and re-publishes the pointcloud oriented properly to `/adjusted_depth_pc`

**Launch:** `ros2 launch ergocub_navigation plane_detector.launch.py`

---

### footsteps\_viewer

Subscribes to planned footstep sequences via YARP and publishes them as RViz `visualization_msgs/MarkerArray` for debugging the walking planning.

**Launch:** `ros2 launch ergocub_navigation footprints_viewer.launch.py`


## How to Run on ergoCubSN000

### 1 — Start the robot

Turn on the robot and use `yarpmanager` to start the yarp server and its nodes while the robot is suspended on the crane.

### 2 — Configure the robot torso

SSH to the robot torso: `ssh -X ergocub-torso`. In the `robot_configuration` folder (alias: `gotoRobotConfigurationFolder`):

```bash
yarprobotinterface --config ergocub.xml --enable_tags "(enable_ros2)"
```

### 3 — Calibrate FT sensors

```bash
yarp rpc /wholeBodyDynamics/rpc
>> calib all 300
```

### 4 — Start the walking controller

SSH to the torso, switch to the navigation branch and rebuild:

```bash
cd /usr/local/src/robot/robotology-superbuild/src/walking-controllers
git switch nav_integration
goToBuildSuperbuild && make install -j4
```

Launch the walking module:

```bash
WalkingModule --from /usr/local/src/robot/robotology-superbuild/src/walking-controllers/src/WalkingModule/app/robots/ergoCubSN001/dcm_walking_iFeel_joint_retargeting_navigation_strict.ini
```

See also: [walking-controllers docs](https://github.com/robotology/walking-controllers)

### 5 — Prepare and start walking (while still on crane)

```bash
yarp rpc /walking-coordinator/rpc
>> prepareRobot
```

Lower the robot to the ground, then:

```bash
>> startWalking
```

### 6 — Sensors setup

SSH to the robot head: `ssh -X ergocub-head`. Run `yarprobotinterface --config sensors.xml` in the head `robot_configuration` folder.

On the **laptop**, from `$ROBOT_CODE/ros2_ws/src/ergocub_navigation/config/yarp`, launch each in a separate terminal:

```bash
yarprobotinterface --config depth_compressed_ros2.xml
yarprobotinterface --config head_imu_ros2.xml
yarprobotinterface --config lidar_compressed_ros2.xml
```

### 7 — Launch the ROS2 navigation stack

```bash
# Full stack (setup + AMCL + Nav2 + plane detector)
ros2 launch ergocub_navigation launch_all.launch.py

# Path converter (YARP bridge to walking-controller)
ros2 launch ergocub_navigation setup_robot/path_converter.launch.py

# Head gaze controller (optional)
ros2 launch ergocub_navigation setup_robot/head_orientation_controller.launch.py

# Footstep visualizer (optional)
ros2 launch ergocub_navigation footprints_viewer.launch.py
```

## Simulation (gz-sim)
Inside the docker container do:

```bash
yarpserver --write
```

```bash
# Launches all the gz sim simulation with the robot spawned and the sensor modules started
ros2 launch ergocub_navigation setup_simulation.launch.py
```

```bash
# Start the walking controller
export YARP_CLOCK=/clock && WalkingModule --from /home/$USERNAME/robotology-superbuild/src/walking-controllers/src/WalkingModule/app/robots/ergoCubGazeboV1/dcm_walking_iFeel_joint_retargeting.ini
```

```bash
# Full simulation stack (setup + localization + Nav2)
ros2 launch ergocub_navigation launch_sim.launch.py
```

Map and params for simulation are in `config/param/simulation/` and `config/maps/`.

## Launch Variants

| Launch file | Description |
| --- | --- |
| `launch_all.launch.py` | Robot — AMCL localization |
| `launch_all_slam.launch.py` | Robot — SLAM-based localization |
| `launch_all_vicon.launch.py` | Robot — Vicon motion-capture localization |
| `launch_all_odom_only.launch.py` | Robot — Odometry-only (no map) |
| `launch_all_human_avoidance.launch.py` | Robot — AMCL + dynamic human obstacle layer |
| `launch_sim.launch.py` | Simulation — AMCL localization |
| `launch_slam_sim.launch.py` | Simulation — SLAM |
| `launch_all_odom_only_sim.launch.py` | Simulation — Odometry-only |
