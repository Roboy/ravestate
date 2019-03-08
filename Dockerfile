FROM missxa/bouncy-roboy


# ros melodic
RUN apt-get update && apt-get install -q -y \
    dirmngr \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# install ros packages
ENV ROS_DISTRO melodic
RUN apt-get update && apt-get install -y \
    ros-melodic-ros-base=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*

# install ros packages
ENV ROS_DISTRO melodic
RUN apt-get update && apt-get install -y \
    ros-melodic-ros-base=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*


# fix weird cmake issue
RUN apt-get update && apt-get install -y python-pip && \
    pip install cmake

# roboy_communication for melodic
RUN cd ~ && mkdir -p ~/melodic_ws/src && \
    cd ~/melodic_ws/src && git clone https://github.com/Roboy/roboy_communication.git -b melodic && \
    cd ~/melodic_ws && . /opt/ros/melodic/setup.sh && catkin_make


# roboy_communication fix for bouncy
RUN cd ~/ros2_ws/src/roboy_communication && \
    git pull && git checkout ros1bridge && \
    . /opt/ros/bouncy/setup.sh &&  export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH:$CMAKE_PREFIX_PATH && \
    cd ~/ros2_ws && colcon build --symlink-install

# ros1_bridge
RUN mkdir -p ~/ros1_bridge_ws/src && cd ~/ros1_bridge_ws/src && \
    git clone https://github.com/Roboy/ros1_bridge.git -b bouncy

RUN . ~/melodic_ws/devel/setup.sh && \
    . /opt/ros/bouncy/setup.sh && . ~/ros2_ws/install/setup.sh && \
    export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH:$CMAKE_PREFIX_PATH && \
    cd ~/ros1_bridge_ws && colcon build --symlink-install

# install ravestate dependencies
ADD requirements.txt /tmp/requirements.txt
ADD requirements-dev.txt /tmp/requirements-dev.txt
RUN pip3 install -r /tmp/requirements.txt
RUN pip3 install -r /tmp/requirements-dev.txt
