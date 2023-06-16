FROM dustynv/ros:noetic-pytorch-l4t-r32.6.1

ENV L4T_MAJOR_VERSION=32 \
    L4T_MINOR_VERSION=6 \
    L4T_PATCH_VERSION=1 \
    ZED_SDK_MAJOR=3 \
    ZED_SDK_MINOR=8 \
    DEBIAN_FRONTEND=noninteractive \
    SHELL=/bin/bash
SHELL ["/bin/bash", "-c"] 

# ---
# User setup
# ---

ENV USER=tj2
ENV HOME=/home/${USER}

RUN sudo mkdir -p /opt/tj2/install && sudo chown -R 1000:1000 /opt/tj2/
COPY --chown=1000:1000 ./install/setup_user.sh /opt/tj2/install
RUN bash /opt/tj2/install/setup_user.sh

USER ${USER}

# ---
# Basic tools
# ---

RUN sudo apt-get update && \
    sudo apt-get install -y apt-utils \
         git nano tmux curl wget htop net-tools iproute2 iputils-ping gdb dumb-init rsync

# ---
# CMake
# ---

COPY --chown=1000:1000 ./install/upgrade_cmake_aarch64.sh /opt/tj2/install
RUN bash /opt/tj2/install/upgrade_cmake_aarch64.sh

# ---
# PyTorch CMake
# ---

ENV LD_LIBRARY_PATH=/usr/local/lib/python3.6/dist-packages/torch/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}} \
    CMAKE_PREFIX_PATH=/usr/local/lib/python3.6/dist-packages/torch/share/cmake/Torch${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}

# ---
# ZED
# ---

WORKDIR /

# This environment variable is needed to use the streaming features on Jetson inside a container
ENV LOGNAME=root
COPY --chown=1000:1000 ./install/zed_jetson_install.sh /opt/tj2/install
RUN bash /opt/tj2/install/zed_jetson_install.sh

# ---
# Basic dependencies
# ---

COPY --chown=1000:1000 \
    ./install/jetson/install_apt_packages.sh \
    ./install/jetson/install_python_dependencies.sh \
    ./install/jetson/install_libraries.sh \
    /opt/tj2/install/basic/
RUN bash /opt/tj2/install/basic/install_apt_packages.sh && \
    bash /opt/tj2/install/basic/install_python_dependencies.sh && \
    bash /opt/tj2/install/basic/install_libraries.sh

# ---
# ROS dependency workspace
# ---

ENV DEP_ROS_WS_ROOT=${HOME}/dep_ws \
    DEP_ROS_WS_SRC=${HOME}/dep_ws/src

COPY --chown=1000:1000 ./install/preferences /etc/apt/preferences
COPY --chown=1000:1000 ./install/rosdep_install.sh /opt/tj2/install
COPY --chown=1000:1000 ./install/ros /opt/tj2/install/ros
RUN bash /opt/tj2/install/ros/install_ros_packages.sh

# ---
# Python extra packages
# ---

COPY --chown=1000:1000 \
    ./install/jetson/requirements.txt \
    ./install/jetson/install_python_extras.sh \
    /opt/tj2/install/basic/
RUN cd /opt/tj2/install/basic/ && bash ./install_python_extras.sh

# ---
# Environment setup
# ---

ENV ROS_WS_ROOT=${HOME}/ros_ws
ENV ROS_WS_SRC=${ROS_WS_ROOT}/src
ENV FLASK_ENV=development \
    PATH=${HOME}/.local/bin:/opt/tj2/scripts${PATH:+:${PATH}} \
    PYTHONPATH=${ROS_WS_SRC}/tj2_ros/tj2_tools${PYTHONPATH:+:${PYTHONPATH}} \
    PYTHONIOENCODING=utf-8

COPY --chown=1000:1000 ./install/host_bashrc ${HOME}/.bashrc

COPY --chown=1000:1000 \
    ./launch/entrypoint.sh \
    ./launch/launch.sh \
    ./launch/roscore.sh \
    /opt/tj2/
RUN ln -s /opt/tj2/tj2_ros ${HOME}/tj2_ros

RUN chown 1000:1000 ${HOME}

WORKDIR ${HOME}

ENTRYPOINT ["/usr/bin/dumb-init", "--"]
