# syntax=docker/dockerfile:1
FROM ubuntu:22.04

ARG RELEASE

ARG LAUNCHPAD_BUILD_ARCH


CMD ["/bin/bash"]

# Linux stuff
RUN dpkg --add-architecture i386 && \
    apt-get update && \
    apt-get install -y --no-install-recommends libxau6 libxau6:i386 libxdmcp6 libxdmcp6:i386  libxcb1 libxcb1:i386 libxext6 libxext6:i386 libx11-6 libx11-6:i386 && rm -rf /var/lib/apt/lists/*

# Nvidia Graphics
ENV NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all}

ENV NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,compat32,utility

RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf &&     echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

ENV LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/usr/lib/i386-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}:/usr/local/nvidia/lib:/usr/local/nvidia/lib64

RUN apt-get update  && apt-get install -y --no-install-recommends         pkg-config         libglvnd-dev libglvnd-dev:i386         libgl1-mesa-dev libgl1-mesa-dev:i386         libegl1-mesa-dev libegl1-mesa-dev:i386         libgles2-mesa-dev libgles2-mesa-dev:i386 &&     rm -rf /var/lib/apt/lists/*

ARG DEBIAN_FRONTEND=noninteractive

# Essentials pkgs setup
RUN apt-get update  && apt-get install -y build-essential         cmake         cppcheck         curl         doxygen         gdb         git         gnupg2         libbluetooth-dev         libcwiid-dev         libgoogle-glog-dev         libspnav-dev         libusb-dev         locales         lsb-release         mercurial         python3-dbg         python3-empy        python3-psutil         python3-venv         software-properties-common         sudo         tzdata         vim         wget         curl	 tmux   psmisc  firefox && \
   sudo add-apt-repository ppa:gnome-terminator && sudo apt install -y terminator && \
   apt-get clean

# Xorg setup
RUN apt update && \
    apt install -y xfce4 xfce4-goodies xserver-xorg-video-dummy xserver-xorg-legacy && \
    sed -i 's/allowed_users=console/allowed_users=anybody/' /etc/X11/Xwrapper.config
COPY ./xorg.conf /etc/X11/xorg.conf
# Latest x11vnc
RUN sed -Ei 's/^# deb-src /deb-src /' /etc/apt/sources.list && \
    apt update && \
    git clone https://github.com/LibVNC/x11vnc.git /opt/x11vnc && \
    cd /opt/x11vnc && \
    apt build-dep -y x11vnc && \
    autoreconf -fiv && \
    ./configure && \
    make -j && \
    make install && \
    rm /opt/x11vnc -Rf

# ROS2 preparation
RUN locale-gen en_US en_US.UTF-8 &&   update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 &&   export LANG=en_US.UTF-8 && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null


# ROS2 install
RUN apt update && apt install -y python3-rosdep python3-vcstool python3-colcon-common-extensions  python3-colcon-mixin \
    ros-iron-control-msgs     ros-iron-controller-manager     ros-iron-desktop     ros-iron-generate-parameter-library \
    ros-iron-geometric-shapes     ros-iron-gripper-controllers     ros-iron-joint-state-broadcaster     ros-iron-joint-state-publisher \
    ros-iron-joint-trajectory-controller     ros-iron-moveit-common     ros-iron-moveit-configs-utils     ros-iron-moveit-core  \
    ros-iron-moveit-hybrid-planning     ros-iron-moveit-msgs     ros-iron-moveit-resources-panda-moveit-config    \
    ros-iron-moveit-ros-move-group     ros-iron-moveit-ros-perception     ros-iron-moveit-ros-planning   \
    ros-iron-moveit-ros-planning-interface     ros-iron-moveit-ros-visualization     ros-iron-moveit-servo     ros-iron-moveit-visual-tools   \
    ros-iron-moveit     ros-iron-rmw-cyclonedds-cpp     ros-iron-ros2-control     ros-iron-rviz-visual-tools     ros-iron-xacro \
    ros-iron-test-msgs   ros-iron-rqt* && apt clean

ENV ROS_DISTRO=iron

#USER root 

RUN useradd -l -u 33334 -G sudo -md /home/ecub_docker -s /bin/bash -p ecub_docker ecub_docker &&     sed -i.bkp -e 's/%sudo\s\+ALL=(ALL\(:ALL\)\?)\s\+ALL/%sudo ALL=NOPASSWD:ALL/g' /etc/sudoers

ENV USERNAME=ecub_docker

USER $USERNAME

WORKDIR /home/$USERNAME

RUN sudo rosdep init && rosdep update

RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && colcon mixin update default && rm -rf log

RUN sudo apt install software-properties-common apt-transport-https wget -y

RUN wget -O- https://packages.microsoft.com/keys/microsoft.asc | sudo gpg --dearmor | sudo tee /usr/share/keyrings/vscode.gpg

RUN echo deb [arch=amd64 signed-by=/usr/share/keyrings/vscode.gpg] https://packages.microsoft.com/repos/vscode stable main | sudo tee /etc/apt/sources.list.d/vscode.list

RUN sudo apt update

RUN sudo apt install -y code

# Git Setup
ARG GIT_USERNAME
ARG GIT_USER_EMAIL
RUN git config --global user.name "${GIT_USERNAME}" && git config --global user.email ${GIT_USER_EMAIL}

### Robotology Superbuild Install Section
ARG BUILD_TYPE=Release

RUN sudo apt-get install -y build-essential git cmake cmake-curses-gui \
  ycm-cmake-modules \
  libeigen3-dev \
  libace-dev \
  libedit-dev \
  libsqlite3-dev \
  libtinyxml-dev \
  qtbase5-dev qtdeclarative5-dev qtmultimedia5-dev \
  qml-module-qtquick2 qml-module-qtquick-window2 \
  qml-module-qtmultimedia qml-module-qtquick-dialogs \
  qml-module-qtquick-controls qml-module-qt-labs-folderlistmodel \
  qml-module-qt-labs-settings \
  libqcustomplot-dev \
  libgraphviz-dev \
  libjpeg-dev \
  gedit \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-libav

  RUN curl -sSL http://get.gazebosim.org | sh



RUN sudo ln -s /usr/local/share/bash-completion/completions/yarp /usr/share/bash-completion/completions && \
    sudo apt install -y glpk-doc glpk-utils libglpk-dev libglpk40

# Superbuild cloning and installing
RUN git clone https://github.com/robotology/robotology-superbuild && \
    cd robotology-superbuild && git checkout v2024.11.0  && \
    sudo chmod +x scripts/install_apt_dependencies.sh && \
    sudo chmod +x scripts/install_apt_python_dependencies.sh && \
    sudo bash ./scripts/install_apt_dependencies.sh 
    
RUN cd robotology-superbuild && mkdir build && cd build && \
    export OpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 && cmake -DROBOTOLOGY_ENABLE_CORE=ON -DROBOTOLOGY_ENABLE_DYNAMICS=ON -DROBOTOLOGY_ENABLE_DYNAMICS_FULL_DEPS=ON .. && \
    make -j8 && make install -j8

# Gazebo Yarp Plugins
RUN git clone https://github.com/robotology/gazebo-yarp-plugins.git && cd gazebo-yarp-plugins && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE="Release "../ -DCMAKE_INSTALL_PREFIX=/home/$USERNAME/robotology-superbuild/build/install .. && \
    cmake --build . --target install
    
# Adding custom worlds to gazebo
COPY worlds /usr/share/gazebo-11/worlds


# Environment setup for simulation
ENV YARP_COLORED_OUTPUT=1
ENV WalkingControllers_INSTALL_DIR=/home/$USERNAME/robotology-superbuild/build/install
ENV YARP_DATA_DIRS=$YARP_DATA_DIRS:$WalkingControllers_INSTALL_DIR/share/yarp
ENV YARP_DATA_DIRS=${YARP_DATA_DIRS}:/home/$USERNAME/robotology-superbuild/build/install/share/iCub
ENV YARP_DATA_DIRS=${YARP_DATA_DIRS}:/home/$USERNAME/robotology-superbuild/build/install/share/ergoCub
ENV GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/home/$USERNAME/robotology-superbuild/build/install/share/iCub/robots:/home/$USERNAME/robotology-superbuild/build/install/share/ergoCub/robots
ENV GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/home/$USERNAME/robotology-superbuild/build/install/share:/usr/share/gazebo-11/models
ENV GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/home/ecub_docker/ergocub-bimanual/gazebo/worlds
ENV GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11/worlds:/usr/share/gazebo-11
ENV YARP_DATA_DIRS=${YARP_DATA_DIRS}:/home/$USERNAME/robotology-superbuild/build/install/share/ICUBcontrib
ENV PATH=${PATH}:/home/$USERNAME/robotology-superbuild/build/install/bin
ENV GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/home/$USERNAME/robotology-superbuild/build/install/lib
ENV YARP_ROBOT_NAME=ergoCubGazeboV1
ENV PYTHONPATH=${PYTHONPATH}:/home/$USERNAME/$USERNAME/robotology-superbuild/build/install/lib/python3

# Bimanual
RUN git clone https://github.com/hsp-iit/ergocub-bimanual.git && cd ergocub-bimanual && cd include && wget https://raw.githubusercontent.com/Woolfrey/software_simple_qp/924a01ddea41c48d9a12b5081f78873b932c7b3e/include/QPSolver.h && cd ../ && mkdir build && cd build && \
    cmake .. && make -j


# Bashrc setup
RUN echo "export PATH=$PATH:/home/$USERNAME/robotology-superbuild/build/install/bin" >> ~/.bashrc && \
    echo "alias goToBuildSuperbuild='cd ../../build/src/${PWD##*/}'" >> ~/.bashrc && \
    echo "alias 0_yarpserver='yarpserver --write'" >> ~/.bashrc && \
    echo "alias 1_clock_export='export YARP_CLOCK=/clock'" >> ~/.bashrc && \
    echo "alias 2_gazebo='export YARP_CLOCK=/clock && gazebo -s libgazebo_yarp_clock.so -s libgazebo_ros_init.so'" >> ~/.bashrc && \
    echo "alias 3_gazebo_warehouse='export YARP_CLOCK=/clock && gazebo worlds/SmallWarehouseScen3.world -s libgazebo_yarp_clock.so -s libgazebo_ros_init.so'" >> ~/.bashrc && \
    echo "alias walking_retargeting='YARP_CLOCK=/clock WalkingModule --from /home/$USERNAME/robotology-superbuild/src/walking-controllers/src/WalkingModule/app/robots/ergoCubGazeboV1/dcm_walking_iFeel_joint_retargeting.ini'" >> ~/.bashrc && \
    echo "alias launch_wbd_interface='yarprobotinterface --config /home/$USERNAME/robotology-superbuild/src/ergocub-software/urdf/ergoCub/conf/launch_wholebodydynamics_ecub.xml'" >> ~/.bashrc && \
    echo "alias merge_ports='yarp merge --input /wholeBodyDynamics/right_foot_front/cartesianEndEffectorWrench:o /wholeBodyDynamics/left_foot_front/cartesianEndEffectorWrench:o --output /feetWrenches'" >> ~/.bashrc && \
    echo "alias col_build='colcon build --symlink-install'" >> ~/.bashrc && \
    echo "source /opt/ros/iron/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "source /home/$USERNAME/ros2_workspace/install/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "source /home/$USERNAME/robotology-superbuild/build/install/share/robotology-superbuild/setup.sh" >> ~/.bashrc

EXPOSE 8080
EXPOSE 8888
EXPOSE 6080
EXPOSE 10000/tcp 10000/udp


# Nav2
RUN sudo apt update && sudo apt install -y ros-iron-navigation2 ros-iron-nav2-bringup ros-iron-perception ros-iron-slam-toolbox ros-iron-gazebo-ros
# Adding pointcloud to laserscan and ergocub_navigation on ROS2 WS
SHELL ["/bin/bash", "-c"]
RUN mkdir -p /home/$USERNAME/ros2_workspace/src && cd /home/$USERNAME/ros2_workspace && \
    /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.sh && colcon build"  && \
    cd src && \
    git clone https://github.com/hsp-iit/ergocub_navigation.git && \
    git clone https://github.com/hsp-iit/bt_nav2_ergocub.git && \
    git clone -b humble https://github.com/ros-perception/pointcloud_to_laserscan && \
    cd .. && source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_CXX_FLAGS=-w
ENV AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH:/opt/ros/iron
    
# Install BT Groot
RUN sudo apt install -y qtbase5-dev libqt5svg5-dev libzmq3-dev libdw-dev && \
    git clone --recurse-submodules https://github.com/BehaviorTree/Groot.git && \
    cd Groot && \
    cmake -S . -B build && \
    cmake --build build
    
# Update walking-comtrollers in robotology superbuild to work in navigation
RUN cd robotology-superbuild/src/walking-controllers && git remote add SimoneMic https://github.com/SimoneMic/walking-controllers && \
    git fetch SimoneMic && \
    git switch ergoCub_SN001_master_merged && \
    cd ../../build/src/walking-controllers && make install -j && cd .

# Install VisualStudio Code extensions
RUN code --install-extension ms-vscode.cpptools \
		--install-extension ms-vscode.cpptools-themes \
		--install-extension ms-vscode.cmake-tools \
                --install-extension ms-python.python \
                --install-extension eamodio.gitlens

# yarp-devices-ros2
CMD ["bash"]
ENV AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH:/home/$USERNAME/robotology-superbuild/build/install
RUN git clone https://github.com/robotology/yarp-devices-ros2 && \
    cd yarp-devices-ros2/ros2_interfaces_ws && \
    git checkout 073d7dbf08306746c416ed77269ee0cc73986891 && \
    source /opt/ros/iron/setup.sh && colcon build && \
    cd .. && mkdir build && cd build && \
    source /opt/ros/$ROS_DISTRO/setup.bash && source /home/$USERNAME/yarp-devices-ros2/ros2_interfaces_ws/install/setup.bash && cmake .. -DYARP_ROS2_USE_SYSTEM_map2d_nws_ros2_msgs=ON -DYARP_ROS2_USE_SYSTEM_yarp_control_msgs=ON && make -j11 && \
    echo "source /home/$USERNAME/yarp-devices-ros2/ros2_interfaces_ws/install/local_setup.bash" >> ~/.bashrc
    #cmake -S. -Bbuild -DCMAKE_INSTALL_PREFIX=/home/$USERNAME/robotology-superbuild/build/install -DBUILD_TESTING=OFF && \
    #cmake --build build && \
    #cmake --build build --target install
ENV YARP_DATA_DIRS=${YARP_DATA_DIRS}:/home/$USERNAME/yarp-devices-ros2/build/share/yarp:/home/$USERNAME/yarp-devices-ros2/build/share/yarp-devices-ros2   
#:/home/$USERNAME/robotology-superbuild/build/install/share/yarp-devices-ros2

# ergocub-software for ros2
RUN cd robotology-superbuild/src/ergocub-software &&\
    git remote add simomic https://github.com/SimoneMic/ergocub-software && \
    git fetch simomic && \
    git switch ros2 && \
    cd ../../build/src/ergocub-software && \
    cmake . && make install

# ergocub-gaze-controller and rpc interfaces
RUN git clone https://github.com/hsp-iit/ergocub-rpc-interfaces && \
    cd ergocub-rpc-interfaces/ecub_gaze_controller/cpp_library && mkdir build && cd build && \
    cmake .. && \
    sudo make install -j6 && \
    cd && \
    git clone https://github.com/SimoneMic/ergocub-gaze-control && \
    cd ergocub-gaze-control && mkdir build && cd build && \
    cmake -DCMAKE_INSTALL_PREFIX=/home/$USERNAME/robotology-superbuild/build/install .. && \
    make install -j5 && \
    cd && echo "alias run_gaze_controller='cd /home/ecub_docker/ergocub-gaze-control/build/bin && ./ergocub-gaze-control /home/ecub_docker/ergocub-gaze-control/ecub_config.ini'" >> ~/.bashrc

# Hospital world for gazebo
RUN git clone https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps && cd Dataset-of-Gazebo-Worlds-Models-and-Maps/worlds/hospital && \
    unzip models_part1.zip && unzip models_part2.zip && unzip models_part3.zip && unzip models_part4.zip && \
    mkdir models && \
    cp -a models_part1/. models && cp -a models_part2/. models && cp -a models_part3/. models && cp -a models_part4/. models && \
    rm -rf models_part1 && rm -rf models_part2 && rm -rf models_part3 && rm -rf models_part4 && \
    echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/ecub_docker/Dataset-of-Gazebo-Worlds-Models-and-Maps/worlds/hospital/models" >> ~/.bashrc && \
    echo "export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/home/ecub_docker/Dataset-of-Gazebo-Worlds-Models-and-Maps/worlds/hospital" >> ~/.bashrc && \
    echo "alias 9_gazebo_hospital='export YARP_CLOCK=/clock && gazebo /home/ecub_docker/Dataset-of-Gazebo-Worlds-Models-and-Maps/worlds/hospital/hospital.world -s libgazebo_yarp_clock.so -s libgazebo_ros_init.so'" >> ~/.bashrc

RUN sudo apt install -y mlocate && sudo apt clean && sudo rm -rf /var/lib/apt/lists/* && sudo updatedb


