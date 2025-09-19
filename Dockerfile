FROM ros:humble-ros-base

# Install system dependencies
RUN apt-get update && apt-get install -y \
    libglew-dev libgl1-mesa-dev libegl1-mesa-dev \
    libwayland-dev libxkbcommon-dev wayland-protocols \
    libavcodec-dev libavformat-dev libavutil-dev libswscale-dev libavdevice-dev \
    libjpeg-dev libtiff-dev libopenexr-dev libpng-dev \
    libeigen3-dev pkg-config \
    ninja-build libepoxy-dev \
    libopencv-dev python3-pip \
    libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

# Build Pangolin from source

RUN pip3 install --no-cache-dir wheel

WORKDIR /opt
RUN git clone --recursive https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -GNinja && \
    ninja && ninja install

# Clone ORB_SLAM3 source
WORKDIR /opt
RUN git clone https://github.com/zang09/ORB-SLAM3-STEREO-FIXED.git ORB_SLAM3 && \
    cd ORB_SLAM3 && \
    chmod +x build.sh && \
    ./build.sh

RUN apt-get update && apt-get install -y curl gnupg2 lsb-release
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list
RUN apt-get update

RUN apt-get install -y \
    ros-humble-vision-opencv \
    ros-humble-message-filters \
    ros-humble-rosbag2-storage-mcap

RUN apt-get update && apt-get install -y \
    ros-humble-rviz2 \
    ros-humble-rviz-common \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-rosbag2-transport \
    ros-humble-rosbag2-storage-default-plugins

RUN apt-get install -y tmux

RUN mkdir -p /colcon_ws/src

WORKDIR /colcon_ws/src
RUN git clone -b humble https://github.com/norlab-ulaval/ORB_SLAM3_ROS2.git

WORKDIR /colcon_ws
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install --packages-select orbslam3 

RUN echo "\n\
    source /opt/ros/humble/setup.bash\n\
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash\n\
    source /colcon_ws/install/local_setup.bash\n\
    source /colcon_ws/install/setup.bash" >> /root/.bashrc

# ROS2 workspace
WORKDIR /ros2_ws/src

