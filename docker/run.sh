#!/bin/bash
NAME=ergocub_navigation
TAG=jazzy

sudo xhost +
sudo docker run \
     --network=host --privileged \
     -it \
     --rm \
     --gpus all \
     -e DISPLAY=unix${DISPLAY} \
     -e ROS_DOMAIN_ID=11 \
     --device /dev/dri/card0:/dev/dri/card0 \
     -v /tmp/.X11-unix:/tmp/.X11-unix \
     -e QT_X11_NO_MITSHM=1 \
     -e NVIDIA_DRIVER_CAPABILITIES=all \
     -e __NV_PRIME_RENDER_OFFLOAD=1 \
     -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
     -e __VK_LAYER_NV_optimus=NVIDIA_only \
     ${NAME}:${TAG} bash

#     --group-add video \
#     --group-add 110 \
