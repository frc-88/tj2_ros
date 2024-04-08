ARG L4T_MAJOR_VERSION
ARG L4T_MINOR_VERSION
ARG L4T_PATCH_VERSION
ARG ZED_SDK_MAJOR
ARG ZED_SDK_MINOR
ARG PYTHON_VERSION_MAJOR
ARG PYTHON_VERSION_MINOR

FROM dustynv/ros:noetic-pytorch-l4t-r${L4T_MAJOR_VERSION}.${L4T_MINOR_VERSION}.${L4T_PATCH_VERSION}

ARG L4T_MAJOR_VERSION
ARG L4T_MINOR_VERSION
ARG L4T_PATCH_VERSION
ARG ZED_SDK_MAJOR
ARG ZED_SDK_MINOR
ARG PYTHON_VERSION_MAJOR
ARG PYTHON_VERSION_MINOR

ENV L4T_MAJOR_VERSION=${L4T_MAJOR_VERSION} \
    L4T_MINOR_VERSION=${L4T_MINOR_VERSION} \
    L4T_PATCH_VERSION=${L4T_PATCH_VERSION} \
    ZED_SDK_MAJOR=${ZED_SDK_MAJOR} \
    ZED_SDK_MINOR=${ZED_SDK_MINOR} \
    PYTHON_VERSION_MAJOR=${PYTHON_VERSION_MAJOR} \
    PYTHON_VERSION_MINOR=${PYTHON_VERSION_MINOR} \
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

ENV LD_LIBRARY_PATH=/usr/local/lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/dist-packages/torch/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}} \
    CMAKE_PREFIX_PATH=/usr/local/lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/dist-packages/torch/share/cmake/Torch${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}

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

COPY --chown=1000:1000 ./install/jetson/install_apt_packages.sh /opt/tj2/install/basic/
RUN bash /opt/tj2/install/basic/install_apt_packages.sh
COPY --chown=1000:1000 ./install/jetson/install_python_dependencies.sh /opt/tj2/install/basic/
RUN bash /opt/tj2/install/basic/install_python_dependencies.sh
COPY --chown=1000:1000 ./install/jetson/install_libraries.sh /opt/tj2/install/basic/
RUN bash /opt/tj2/install/basic/install_libraries.sh

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
COPY --chown=1000:1000 ./install/jetson/make_symlinks.sh /opt/tj2/install
RUN bash /opt/tj2/install/make_symlinks.sh

RUN chown 1000:1000 ${HOME}

WORKDIR ${HOME}

ENTRYPOINT ["/usr/bin/dumb-init", "--"]
