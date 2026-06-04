#!/bin/bash
# Aliases and environment setup for ergocub development

export PATH=$PATH:/home/$USERNAME/robotology-superbuild/build/install/bin

alias goToBuildSuperbuild='cd ../../build/src/${PWD##*/}'
alias 0_yarpserver='yarpserver --write'
alias 1_gazebo_warehouse='export YARP_CLOCK=/clock && gz sim /home/ecub_docker/ros2_workspace/src/ergocub_navigation/sim/warehouse.sdf --verbose'
alias 2_spawn_robot='ros2 run ros_gz_sim create -file /home/ecub_docker/robotology-superbuild/src/ergocub-software/urdf/ergoCub/robots/ergoCubGazeboSN001_minContacts/model.urdf -name ergocub -z 0.8'
alias walking_retargeting='export YARP_CLOCK=/clock WalkingModule --from /home/$USERNAME/robotology-superbuild/src/walking-controllers/src/WalkingModule/app/robots/ergoCubGazeboV1/dcm_walking_iFeel_joint_retargeting.ini'
alias launch_wbd_interface='yarprobotinterface --config /home/$USERNAME/robotology-superbuild/src/ergocub-software/urdf/ergoCub/conf/launch_wholebodydynamics_ecub.xml'
alias merge_ports='yarp merge --input /wholeBodyDynamics/right_foot_front/cartesianEndEffectorWrench:o /wholeBodyDynamics/left_foot_front/cartesianEndEffectorWrench:o --output /feetWrenches'
alias col_build='colcon build --symlink-install'

source /opt/ros/jazzy/setup.bash
source /home/$USERNAME/ros2_workspace/install/setup.bash

export LD_LIBRARY_PATH=/home/ecub_docker/robotology-superbuild/build/install/lib:$LD_LIBRARY_PATH
