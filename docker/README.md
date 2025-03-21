# Docker preliminaries to build docker containing the navigation and manipulation stacks of the ergoCub project

After building the docker environment will be setup as follows to be capable of running ergoCub simulations. 
- Ubuntu 22.04
- Gazebo 11
- ROS2 Iron
- Simulated ergoCub robot with necessary controllers and interfaces from Yarp and ROS2 from robotology-superbuild to perform simulated robot walking experiments.
- The ROS2 ergoCub Navigation Stack.
- ergoCub Bimanual manipulation

Two scripts to build and run the docker image are included and can be used as below. The run script must have the correct image name and tag before being executed.
```
./build_docker.sh "User Name" "User Git Email"
./run.sh
```

The pre-built docker image capable of running the full stack out of the box can also be pulled using the following command.
```
docker pull vraghavan913/ecub_nav_tests:test_v1
```