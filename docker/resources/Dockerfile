#
# this dockerfile roughly follows the 'Install ROS From Source' procedures from:
#   https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html
#

FROM dustynv/ros:noetic-pytorch-l4t-r32.6.1

ENV L4T_MAJOR_VERSION=32
ENV L4T_MINOR_VERSION=6
ENV L4T_PATCH_VERSION=1

ENV ZED_SDK_MAJOR=3
ENV ZED_SDK_MINOR=7

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"] 

WORKDIR /

RUN apt-get update && \
    apt-get install -y --no-install-recommends apt-utils
RUN apt-get install -y nano tmux curl htop net-tools iproute2

# ---
# ZED
# ---

#This environment variable is needed to use the streaming features on Jetson inside a container
ENV LOGNAME root
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update || true
RUN apt-get install --no-install-recommends lsb-release wget less udev sudo apt-transport-https build-essential cmake -y&& \
    echo "# R${L4T_MAJOR_VERSION} (release), REVISION: ${L4T_MINOR_VERSION}.${L4T_PATCH_VERSION}" > /etc/nv_tegra_release ; \
    wget -q --no-check-certificate -O ZED_SDK_Linux.run https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/l4t${L4T_MAJOR_VERSION}.${L4T_MINOR_VERSION}/jetsons && \
    chmod +x ZED_SDK_Linux.run ; ./ZED_SDK_Linux.run silent skip_tools && \
    rm -rf /usr/local/zed/resources/* \
    rm -rf ZED_SDK_Linux.run && \
    rm -rf /var/lib/apt/lists/*

#This symbolic link is needed to use the streaming features on Jetson inside a container
RUN ln -sf /usr/lib/aarch64-linux-gnu/tegra/libv4l2.so.0 /usr/lib/aarch64-linux-gnu/libv4l2.so

# ---
# tj2_ros dependencies
# ---

WORKDIR /tmp

RUN git clone https://github.com/AprilRobotics/apriltag.git && \
    cd apriltag && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j4 && \
    make install && \
    cd ../.. && \
    rm -r apriltag

RUN apt-get update && sudo apt-get install -y \
    libbullet-dev \
    libeigen3-dev \
    libsdl-image1.2-dev \
    libsdl-dev \
    libyaml-cpp-dev

RUN git clone https://github.com/orocos/orocos_kinematics_dynamics.git && \
    cd orocos_kinematics_dynamics && \
    git submodule update --init && \
    cd orocos_kdl && \
    mkdir build && cd build && \
    cmake .. && make -j4 && \
    sudo make install && \
    python3 -m pip install psutil && \
    cd ../../python_orocos_kdl && \
    mkdir build && cd build && \
    cmake -D PYTHON_EXECUTABLE=/usr/bin/python3 \
        -D PYTHON_INCLUDE_DIR=/usr/include/python3.6 \
        -D PYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.6m.so \
        -D PYBIND11_PYTHON_VERSION=3 .. && \
    make -j4 && \
    make install && \
    python3 -m pip install psutil && \
    ldconfig


RUN cd /tmp && \
    git clone https://github.com/FRC-Utilities/LibDS.git && \
    cd LibDS && \
    qmake -qt=qt5 && \
    make -j5 && \
    make install && \
    cd ../ && \
    rm -r LibDS


RUN cd /tmp && \
    git clone --recursive https://github.com/ThadHouse/CmakeWpilib.git && \
    cd CmakeWpilib && \
    mkdir build && cd build && \
    cmake .. -DWITHOUT_CSCORE=ON -DWITHOUT_JAVA=ON && \
    make -j4 && \
    make install && \
    cd ../.. && \
    rm -r CmakeWpilib

RUN apt-get install -y libpcl-dev \
    libvtk6-qt-dev \
    libspnav-dev \
    joystick \
    liborocos-kdl-dev \
    liborocos-kdl1.3 \
    libnetpbm10-dev \
    libogg-dev \
    libtheora-dev \
    graphviz \
    libgeographic-dev \
    python3-pyproj \
    ffmpeg \
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libswscale-dev \
    v4l-utils \
    liburdfdom-headers-dev \
    libtinyxml-dev \
    liburdfdom-dev \
    hddtemp \
    lm-sensors \
    python3-psutil

# ---
# tj2_ros ROS dependencies
# ---

ENV USER=root
ENV HOME=/${USER}

ENV ROS_WS_ROOT=${HOME}/ros_ws
ENV ROS_WS_SRC=${HOME}/ros_ws/src

RUN mkdir -p ${ROS_WS_SRC}
WORKDIR ${ROS_WS_SRC}

RUN mkdir /tmp/scripts
COPY ./scripts/clone_ros_packages.sh /tmp/scripts
RUN /tmp/scripts/clone_ros_packages.sh ${ROS_WS_SRC}
COPY ./scripts/fix-image-pipeline.patch /tmp/scripts
COPY ./scripts/fix-imu-tools.patch /tmp/scripts
COPY ./scripts/apply_patches.sh /tmp/scripts
RUN /tmp/scripts/apply_patches.sh ${ROS_WS_SRC}
RUN rm -r /tmp/scripts

COPY ./scripts/preferences /tmp
RUN sudo mv /tmp/preferences /etc/apt/preferences

# ---
# build dependencies
# ---

WORKDIR ${ROS_WS_ROOT}

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && rosdep install --from-paths src --ignore-src --rosdistro=noetic -y -r; exit 0

ENV LD_LIBRARY_PATH=/usr/local/lib/python3.6/dist-packages/torch/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
ENV CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/usr/local/lib/python3.6/dist-packages/torch/share/cmake/Torch

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && catkin_make

# ---
# setup entry point
# ---

ENV FLASK_ENV=development

RUN rm /ros_entrypoint.sh
COPY ./scripts/ros_entrypoint.sh /ros_entrypoint.sh
COPY ./scripts/launch.sh /launch.sh

WORKDIR ${ROS_WS_ROOT}

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/launch.sh"]
