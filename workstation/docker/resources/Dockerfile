FROM osrf/ros:noetic-desktop-full

RUN apt-get update && \
    apt-get install -y --no-install-recommends apt-utils
RUN apt-get update && \
    apt-get install -y git nano tmux curl htop net-tools iproute2 iputils-ping dumb-init \
    libsdl-image1.2-dev libsdl-dev python3-pip wget python3-tk rsync sudo

RUN ln -s /usr/bin/python3 /usr/bin/python

RUN python -m pip install -U setuptools
RUN python -m pip install -U scipy==1.5.4 \
    shapely==1.6.4 \
    mido \
    pyperclip \
    dataclasses \
    pynetworktables==2021.0.0 \
    numpy==1.22.4 \
    ezdxf \
    jsonlines \
    PyQt5==5.10.1 \
    mypy

RUN apt-get install -y llvm-7*
RUN ln -s /usr/lib/llvm-7/bin/llvm-config /usr/bin
RUN python -m pip install Cython
RUN python -m pip install llvmlite==0.32.0
RUN python -m pip install numba==0.49.0



# ---
# tj2_ros dependencies
# ---

RUN sh -c 'echo "deb http://packages.ros.org/ros-testing/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# generated using rosdep install --from-paths . -r --reinstall --simulate
RUN apt-get update && apt-get install -y --ignore-missing ros-noetic-rospy \
    ros-noetic-catkin \
    ros-noetic-tf \
    ros-noetic-rviz \
    ros-noetic-apriltag-ros \
    ros-noetic-topic-tools \
    ros-noetic-joy \
    ros-noetic-roscpp \
    ros-noetic-std-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-geometry2 \
    ros-noetic-tf2 \
    ros-noetic-nav-msgs \
    ros-noetic-message-generation \
    ros-noetic-roslaunch \
    ros-noetic-message-runtime \
    ros-noetic-message-filters \
    ros-noetic-visualization-msgs \
    ros-noetic-image-transport \
    ros-noetic-vision-msgs \
    ros-noetic-pcl-msgs \
    ros-noetic-camera-info-manager \
    ros-noetic-dynamic-reconfigure \
    ros-noetic-image-transport-plugins \
    ros-noetic-image-pipeline \
    ros-noetic-image-common \
    ros-noetic-marker-msgs

RUN sh -c 'echo "deb http://packages.ros.org/ros-testing/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-get update && apt-get install -y \
    python3-catkin-tools \
    python3-osrf-pycommon \
    python3-wstool


WORKDIR /tmp
RUN git clone https://github.com/FRC-Utilities/LibDS.git
WORKDIR /tmp/LibDS
RUN qmake -qt=qt5
RUN make -j4 && make install

WORKDIR /tmp
RUN git clone --recursive https://github.com/ThadHouse/CmakeWpilib.git && \
    cd CmakeWpilib && \
    mkdir build
RUN sed -i "16i #define __PTHREAD_SPINS         0, 0" CmakeWpilib/libraries/wpiutil/wpiutil/src/main/native/include/support/priority_mutex.h
WORKDIR /tmp/CmakeWpilib/build
RUN cmake .. -DWITHOUT_CSCORE=ON -DWITHOUT_JAVA=ON
RUN make -j4 && make install

WORKDIR /
RUN ldconfig
RUN rm -r /tmp/*

# ---
# User setup
# ---

ENV USER=tj2


RUN groupadd -g 1000 ${USER} && \
    useradd -r -u 1000 -m -s /bin/bash -g ${USER} \
    -G dialout,plugdev,video,audio,sudo ${USER}

ENV HOME=/home/${USER}
RUN mkdir -p ${HOME}
WORKDIR ${HOME}

RUN chown -R ${USER} ${HOME} && \
  adduser ${USER} sudo && \
  echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER ${USER}

# ---
# Build temporary workspace
# ---

ENV DEP_ROS_WS_ROOT=${HOME}/dep_ws
ENV DEP_ROS_WS_SRC=${HOME}/dep_ws/src

COPY --chown=1000:1000 ./workstation/docker/resources/install ${HOME}/install

WORKDIR ${DEP_ROS_WS_SRC}
RUN wstool init .
RUN wstool merge -t . ${HOME}/install/tj2_ros.rosinstall
RUN wstool update -t .

RUN rosdep update
RUN rosdep install -y --from-paths . --ignore-src --rosdistro noetic

RUN rm -r ${DEP_ROS_WS_SRC}/zed-ros-wrapper/zed_nodelets

WORKDIR ${DEP_ROS_WS_ROOT}
# https://github.com/ros/catkin/issues/863
RUN catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release -DSETUPTOOLS_DEB_LAYOUT=OFF
RUN catkin config --install
RUN catkin build

# ---
# ROS build environment
# ---

ENV ROS_WS_ROOT=${HOME}/ros_ws
ENV ROS_WS_SRC=${ROS_WS_ROOT}/src

ENV FLASK_ENV=development
ENV PYTHONPATH=${ROS_WS_SRC}/tj2_ros/tj2_tools:/opt/yolov5${PYTHONPATH:+:${PYTHONPATH}}
ENV PATH=${HOME}/.local/bin${PATH:+:${PATH}}


# ---
# tj2_ros launch environment
# ---

RUN cat ${HOME}/install/bashrc >> ${HOME}/.bashrc
RUN rm ${HOME}/install/bashrc

COPY --chown=1000:1000 ./workstation/docker/resources/launch/entrypoint.sh /entrypoint.sh
COPY --chown=1000:1000 ./workstation/docker/resources/launch/launch.sh /launch.sh
COPY --chown=1000:1000 ./workstation/docker/resources/launch/roscore.sh /roscore.sh

WORKDIR ${HOME}
RUN sudo chown -R 1000:1000 ${HOME}

ENTRYPOINT ["/usr/bin/dumb-init", "--"]
