# Docker preliminaries to build docker containing the navigation and manipulation stacks of the ergoCub project

After building the docker environment will be setup as follows to be capable of running ergoCub simulations. 
- Ubuntu 24.04
- Gazebo Sim (gz-sim)
- ROS2 Jazzy
- Simulated ergoCub robot with necessary controllers and interfaces from Yarp and ROS2 from robotology-superbuild to perform simulated robot walking experiments.
- The ROS2 ergoCub Navigation Stack, built from the checked-out repository (not cloned from GitHub), so local changes end up in the image.

Two scripts to build and run the docker image are included and can be used as below. The build script uses the repository root as build context, so it can be launched from any directory. The run script must have the correct image name and tag before being executed.
```
./build_docker.sh "User Name" "User Git Email"
./run.sh
```
