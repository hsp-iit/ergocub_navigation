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


## Launching

Normally you do not launch nodes one at a time — [`bringup.launch.py`](#bringup)
starts the whole stack for a chosen profile. The per-node launch files below are
for bringing up or debugging a single component.

Two rules apply to every command in this README:

1. **`ros2 launch <pkg> <file>` matches on the file name only, never on a path
   inside the package.** `setup_robot/odom.launch.py` does **not** resolve;
   `odom.launch.py` does. Launch file names are unique across the package, and a
   test enforces that.
2. **Every launch file takes `use_sim_time` (default `false`), and every one that
   loads a node takes `params_file`.** So you can point any node at your own YAML
   without editing the repo:

   ```bash
   ros2 launch ergocub_navigation odom.launch.py \
       params_file:=/abs/path/to/my_odom.yaml use_sim_time:=true
   ```

   Add `--show-args` to any launch file to list what it accepts.

## Modules

### odom\_node

Bridges the YARP virtual-unicycle state (from the walking controller) to a ROS2 `nav_msgs/Odometry` topic. This is the robot's odometry source for Nav2.

**Launch:** `ros2 launch ergocub_navigation odom.launch.py`
**Params:** `config/param/odom.yaml`

---

### scan\_filter

Compensates and filters the 2D lidar scan, removing points that hit the robot's own body based on TF transforms. Also starts the `pointcloud_to_laserscan` projectors that turn the compensated cloud into the scans the costmaps consume.

**Launch:** `ros2 launch ergocub_navigation scan_filtering.launch.py`
**Params:** `config/param/scan.yaml` (sim: `config/param/simulation/scan.yaml`)
**Extra arg:** `projectors:=robot|sim` — `robot` starts four projectors (wide,
front, and two rear wedges); `sim` starts a single full-circle one. The
configurations live in `ergocub_navigation/launch_utils.py` as `SCAN_PROJECTORS`.

---

### pointcloud\_filter

Filters the depth camera pointcloud, cropping out the robot's own body to prevent self-collisions in the costmap.

**Launch:** `ros2 launch ergocub_navigation pointcloud_filter.launch.py`
**Params:** `config/param/depth_filter.yaml`

---

### path\_converter

Converts the Nav2 global plan into walking-controller commands and sends them via YARP to the walking-coordinator. This is the main bridge between Nav2 and the bipedal walking controller.

**Launch:** `ros2 launch ergocub_navigation path_converter.launch.py`
**Params:** `config/param/path_converter.yaml`

> Known issue: that YAML is keyed `path_converter_v2_node` while the node is named
> `path_converter_node`, so none of its values currently reach the node — it runs
> on the defaults in `src/navigation/walking_planning/path_converter.cpp`. Fixing
> the key would change runtime behaviour, so it has been left alone deliberately.

---

### planner\_trigger\_server

Exposes a ROS2 action server that the Behavior Tree (bt\_nav2\_ergocub) calls. Forwards navigation triggers to the walking-controller via YARP.

**Run:** `ros2 run ergocub_navigation planner_trigger_server`
Started automatically by `bringup.launch.py` unless `use_planner_trigger:=false`.

---

### head\_orientation\_controller

Controls the robot's head/neck gaze direction along the Nav2 planned path. Subscribes to `/plan` and sends joint commands to the ergocub-head-controller via YARP RPC, keeping the robot looking ahead toward the next waypoint. Returns the head to a neutral pose when navigation ends.

**Launch:** `ros2 launch ergocub_navigation head_orientation_controller.launch.py`
**Params:** `config/param/head_orientation_controller.yaml`
Not part of `bringup.launch.py`; run it alongside.

---

### human\_pose\_goal\_generator

Generates Nav2 navigation goals based on detected human poses received via YARP. Useful for human-following scenarios.

**Launch:** `ros2 launch ergocub_navigation human_pose_goal_generator.launch.py`
**Params:** `config/param/human_pose_goal_generator.yaml`
Not part of `bringup.launch.py`; run it alongside.

---

### plane\_detector

Detects ground-plane and performs a camera orientation calibration based on floor RANSAC. Republishes a TF frame called `compensated_realsense_frame` and re-publishes the pointcloud oriented properly to `/adjusted_depth_pc`

**Launch:** `ros2 launch ergocub_navigation plane_detector.launch.py`
**Params:** `config/param/plane_detector.yaml`
Or via `bringup.launch.py use_plane_detector:=true`.

---

### footsteps\_viewer

Subscribes to planned footstep sequences via YARP and publishes them as RViz `visualization_msgs/MarkerArray` for debugging the walking planning.

**Launch:** `ros2 launch ergocub_navigation footprints_viewer.launch.py`

---

### imu\_filter

Orientation filter for the head IMU, remapping `imu/data_raw` → `head_imu` and `imu/data` → `head_imu/filtered`.

**Launch:** `ros2 launch ergocub_navigation imu_filter.launch.py filter:=madgwick`
**Params:** `config/param/imu_filter.yaml`
**Extra arg:** `filter:=madgwick|complementary`.

> Known issue: the YAML is keyed `imu_filter`, which matches the madgwick node
> only. The complementary node is named `complementary_filter_gain_node` and runs
> on its own defaults.

---

### Supporting launch files

| File | Purpose | Notable args |
| --- | --- | --- |
| `robot_state_publisher.launch.py` | Publishes the URDF from robotology-superbuild | `model` (default `$YARP_ROBOT_NAME`), `superbuild_src` (default `$ROBOTOLOGY_SUPERBUILD_SOURCE_DIR`) |
| `setup_robot.launch.py` | URDF + scan filtering + odometry, optionally the pointcloud filter | `projectors`, `scan_params`, `odom_params`, `use_pointcloud_filter` |
| `setup_localization.launch.py` | Map server, optionally AMCL, optionally an identity `map`→`odom` TF | `map`, `amcl_params`, `use_amcl`, `static_map_odom_tf` |
| `map_server.launch.py` | Serves one map | `map` (name under `maps/`, or an absolute path) |
| `amcl.launch.py` | AMCL + its lifecycle manager | `params_file` |
| `nav2_stack.launch.py` | The nav2 servers | `params_file`, `bt_xml`, `use_keepout`, `keepout_mask` |
| `slam_online_async.launch.py` | slam\_toolbox in async mapping mode | `slam_params_file`, `autostart` |
| `ergoCub_rviz.launch.py` | RViz | `rviz_config` |
| `setup_simulation.launch.py` | gz-sim world + robot spawn + YARP sensor modules | `model`, `superbuild_src`, `world_sdf`, `spawn_height` |
| `point_cloud_xyz.launch.py` | depth\_image\_proc container (not started by bringup) | — |


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
# Full stack (setup + AMCL + Nav2 + plane detector + keepout filters)
ros2 launch ergocub_navigation bringup.launch.py

# Path converter (YARP bridge to walking-controller)
ros2 launch ergocub_navigation path_converter.launch.py

# Head gaze controller (optional)
ros2 launch ergocub_navigation head_orientation_controller.launch.py

# Footstep visualizer (optional)
ros2 launch ergocub_navigation footprints_viewer.launch.py
```

> `ros2 launch <pkg> <file>` matches on the **file name only**, never on a path
> inside the package. `setup_robot/path_converter.launch.py` does not resolve;
> `path_converter.launch.py` does.

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
ros2 launch ergocub_navigation bringup.launch.py world:=sim
```

Map and params for simulation are in `config/param/simulation/` and `config/maps/`.

## Bringup

`bringup.launch.py` is the single entry point. It selects a profile rather than
having one launch file per combination:

```bash
ros2 launch ergocub_navigation bringup.launch.py world:=sim localization:=slam
ros2 launch ergocub_navigation bringup.launch.py localization:=odom_only rviz:=false
ros2 launch ergocub_navigation bringup.launch.py map:=floor1_ergoCub_2.yaml
```

Run `ros2 launch ergocub_navigation bringup.launch.py --show-args` for the full
list. The main ones:

| Argument | Values | Default | Effect |
| --- | --- | --- | --- |
| `world` | `robot`, `sim` | `robot` | Sets `use_sim_time` and the per-world map, AMCL params, scan params and scan projector set |
| `localization` | `amcl`, `slam`, `odom_only`, `none` | `amcl` | `odom_only` serves a blank map plus an identity `map`→`odom` transform |
| `map` | name under `maps/`, or an absolute path | per world | `empty_map.yaml` for `odom_only`/`none` |
| `nav2_params` | path | per world + localization | |
| `rviz` | `true`, `false` | `true` | |
| `use_nav2` | `true`, `false` | `true` | |
| `use_keepout` | `true`, `false` | **per profile** | Costmap filter servers for the keepout mask |
| `keepout_mask` | path | `maps/floor0_ergoCub_modded_keepout_full.yaml` | |
| `use_plane_detector` | `true`, `false` | **per profile** | |
| `use_pointcloud_filter` | `true`, `false` | **per profile** | |
| `use_planner_trigger` | `true`, `false` | `true` | |

Every argument left unset falls back to the profile default, so overriding one
does not force you to restate the others.

**Per-profile defaults.** The keepout filters, the plane detector and the depth
pointcloud filter are real-robot + static-map features — the keepout mask is a map
artifact, and the other two need the RealSense. They default to **on for
`robot` + `amcl`** and off for every other combination:

| Profile | keepout | plane detector | pointcloud filter |
| --- | --- | --- | --- |
| `robot` + `amcl` (the default) | on | on | on |
| everything else | off | off | off |

Override any of them explicitly, e.g. to run the robot without the keepout zones:

```bash
ros2 launch ergocub_navigation bringup.launch.py use_keepout:=false
```

### Legacy entry points

These still exist as thin wrappers, and start exactly what they used to:

| Launch file | Equivalent |
| --- | --- |
| `launch_all.launch.py` | `bringup.launch.py` (the default profile) |
| `launch_all_slam.launch.py` | `bringup.launch.py localization:=slam use_planner_trigger:=false` |
| `launch_all_odom_only.launch.py` | `bringup.launch.py localization:=odom_only use_planner_trigger:=false` |
| `launch_sim.launch.py` | `bringup.launch.py world:=sim localization:=amcl` |
| `launch_slam_sim.launch.py` | `bringup.launch.py world:=sim localization:=slam` |
| `launch_all_odom_only_sim.launch.py` | `bringup.launch.py world:=sim localization:=none use_planner_trigger:=false` |

The `vicon` and `human_avoidance` variants have been removed.

## Layout

```
launch/
  bringup.launch.py          single entry point
  setup_robot.launch.py      URDF + scan filtering + odometry
  setup_localization.launch.py   map server, optional AMCL, optional map->odom TF
  nav2_stack.launch.py       the nav2 servers
  setup_robot/               one leaf launch per node
  amcl/  slam/  imu/         localization and sensor leaves
  simulation/                gz-sim bringup and the sim entry-point wrappers
ergocub_navigation/
  launch_utils.py            pkg_share(), include(), lifecycle_bringup(),
                             SCAN_PROJECTORS -- shared by all launch files
```

`ergocub_navigation/launch_utils.py` is installed to site-packages via
`ament_python_install_package`, which is what lets the launch files share code
instead of copy-pasting lifecycle boilerplate.

## Tests

```bash
colcon test --packages-select ergocub_navigation && colcon test-result --verbose
```

`test/test_launch_files.py` resolves every launch file's include graph and checks
that the params, maps and behavior trees they name are actually installed. It
needs no robot and no simulator.
