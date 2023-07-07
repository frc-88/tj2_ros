ARG SOURCE_IMAGE

FROM $SOURCE_IMAGE as workstation_tj2_ros

COPY --chown=1000:1000 \
    ./install/training/install_apt_packages.sh \
    ./install/training/install_python_dependencies.sh \
    /opt/tj2/install/training/
RUN bash /opt/tj2/install/training/install_apt_packages.sh && \
    bash /opt/tj2/install/training/install_python_dependencies.sh

FROM nvidia/cuda:12.1.1-devel-ubuntu20.04

ENV L4T_MAJOR_VERSION=32 \
    L4T_MINOR_VERSION=6 \
    L4T_PATCH_VERSION=1 \
    ZED_SDK_MAJOR=3 \
    ZED_SDK_MINOR=8 \
    DEBIAN_FRONTEND=noninteractive \
    SHELL=/bin/bash
SHELL ["/bin/bash", "-c"] 

# ---
# Basic tools
# ---

RUN apt-get update && \
    apt-get install -y apt-utils \
        git nano tmux curl wget htop net-tools iproute2 iputils-ping gdb dumb-init rsync sudo

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
# Multistage copy
# ---

COPY --from=workstation_tj2_ros / /tmp/workstation_tj2_ros
COPY --chown=1000:1000 ./install/training/copy_over_multistage.sh /opt/tj2/install/training/
RUN bash /opt/tj2/install/training/copy_over_multistage.sh

# ---
# Environment variables
# ---

ENV DEP_ROS_WS_ROOT=${HOME}/dep_ws
ENV DEP_ROS_WS_SRC=${HOME}/dep_ws/src

ENV ROS_DISTRO=noetic
ENV ROS_WS_ROOT=${HOME}/ros_ws
ENV ROS_WS_SRC=${ROS_WS_ROOT}/src

ENV FLASK_ENV=development \
    PATH=${HOME}/.local/bin:/opt/tj2/scripts${PATH:+:${PATH}} \
    PYTHONPATH=${ROS_WS_SRC}/tj2_ros/tj2_tools${PYTHONPATH:+:${PYTHONPATH}} \
    PYTHONIOENCODING=utf-8

# ---
# tj2_ros launch environment
# ---

COPY --chown=1000:1000 ./install/client_bashrc ${HOME}/.bashrc

RUN chown 1000:1000 ${HOME} && \
    chown -R 1000:1000 ${HOME}/.ros

WORKDIR ${HOME}
USER ${USER}

ENTRYPOINT ["/usr/bin/dumb-init", "--"]
