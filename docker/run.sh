#!/bin/bash
NAME=vraghavan913/ecub_nav_tests
TAG=test_v1

sudo xhost +
sudo docker run \
     --network=host --privileged \
     -it \
     --rm \
     -e DISPLAY=unix${DISPLAY} \
     -e ROS_DOMAIN_ID=11 \
     --device /dev/dri/card0:/dev/dri/card0 \
     ${NAME}:${TAG} bash

     #-v /home/simomic/rosbag2_2024_01_11-16_35_16:/home/ecub_docker/rosbags \
