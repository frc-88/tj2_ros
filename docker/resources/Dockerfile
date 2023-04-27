#
# this dockerfile roughly follows the 'Install ROS From Source' procedures from:
#   https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html
#

FROM dustynv/ros:noetic-pytorch-l4t-r32.6.1

ENV L4T_MAJOR_VERSION=32 \
    L4T_MINOR_VERSION=6 \
    L4T_PATCH_VERSION=1 \
    ZED_SDK_MAJOR=3 \
    ZED_SDK_MINOR=8 \
    DEBIAN_FRONTEND=noninteractive \
    SHELL=/bin/bash
SHELL ["/bin/bash", "-c"] 

RUN apt-get update && \
    apt-get install -y apt-utils \
    nano tmux curl htop net-tools iproute2 iputils-ping gdb dumb-init rsync sudo

# ---
# User setup
# ---

ENV USER=tj2
ENV HOME=/home/${USER}

RUN mkdir -p /root/install
COPY ./install/setup_user.sh /root/install
RUN bash /root/install/setup_user.sh

USER root

# ---
# CMake
# ---

COPY ./install/upgrade_cmake.sh /root/install
RUN bash /root/install/upgrade_cmake.sh

# ---
# PyTorch CMake
# ---

ENV LD_LIBRARY_PATH=/usr/local/lib/python3.6/dist-packages/torch/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}} \
    CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/usr/local/lib/python3.6/dist-packages/torch/share/cmake/Torch

# ---
# ZED
# ---

WORKDIR /

# This environment variable is needed to use the streaming features on Jetson inside a container
ENV LOGNAME=root \
    DEBIAN_FRONTEND=noninteractive
COPY ./install/zed_install.sh /root/install
RUN bash /root/install/zed_install.sh

# ---
# tj2_ros dependencies
# ---

COPY ./install/basic /root/install/basic
RUN bash /root/install/basic/install_apt_packages.sh && \
    bash /root/install/basic/install_python_dependencies.sh && \
    bash /root/install/basic/install_libraries.sh

# ---
# ROS evergreen dependency workspace
# ---

ENV DEP_ROS_WS_ROOT=${HOME}/dep_ws \
    DEP_ROS_WS_SRC=${HOME}/dep_ws/src

COPY ./install/preferences /etc/apt/preferences
COPY ./install/evergreen /root/install/evergreen
COPY ./install/rosdep_install.sh /root/install
RUN bash /root/install/evergreen/install_evergreen_packages.sh

# ---
# ROS overlay dependency workspace
# ---

COPY ./install/overlay /root/install/overlay
RUN bash /root/install/overlay/install_overlay_packages.sh

# ---
# Environment setup
# ---

ENV ROS_WS_ROOT=${HOME}/ros_ws
ENV ROS_WS_SRC=${ROS_WS_ROOT}/src
ENV FLASK_ENV=development \
    PYTHONPATH=${ROS_WS_SRC}/tj2_ros/tj2_tools${PYTHONPATH:+:${PYTHONPATH}} \
    PYTHONIOENCODING=utf-8

COPY --chown=1000:1000 ./install/bashrc ${HOME}/.bashrc

COPY --chown=1000:1000 \
    ./launch/entrypoint.sh \
    ./launch/launch.sh \
    ./launch/roscore.sh \
    /

WORKDIR ${HOME}

USER ${USER}

ENTRYPOINT ["/usr/bin/dumb-init", "--"]
