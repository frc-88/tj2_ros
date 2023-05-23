FROM tj2_ros_workstation:latest as tj2_ros_workstation

USER root

RUN apt-get update && apt-get install -y --ignore-missing \
    ros-noetic-robot-localization \
    ros-noetic-xacro \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-serial \
    ros-noetic-twist-mux \
    ros-noetic-cv-bridge \
    ros-noetic-image-geometry \
    ros-noetic-perception-pcl \
    ros-noetic-amcl \
    ros-noetic-map-server \
    ros-noetic-gmapping \
    ros-noetic-laser-filters \
    ros-noetic-move-base \
    ros-noetic-teb-local-planner \
    ros-noetic-global-planner \
    ros-noetic-dwa-local-planner \
    ros-noetic-base-local-planner \
    ros-noetic-costmap-converter \
    ros-noetic-py-trees-ros \
    ros-noetic-py-trees-msgs \
    ros-noetic-py-trees \
    ros-noetic-rqt-py-trees \
    ros-noetic-rosbridge-suite \
    ros-noetic-ros-numpy

RUN python -m pip install torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cu116

RUN python -m pip install wandb
RUN mkdir -p /opt/yolov5
WORKDIR /opt/yolov5
RUN git clone https://github.com/ultralytics/yolov5.git
RUN cd yolov5 && python -m pip install -r requirements.txt


WORKDIR /tmp
RUN git clone https://github.com/wkentaro/labelme.git
WORKDIR /tmp/labelme
RUN python3 setup.py install

RUN rm -r /tmp/*

FROM nvidia/cuda:12.1.1-devel-ubuntu20.04
COPY --from=tj2_ros_workstation / /tmp/tj2_ros_workstation
RUN apt-get update && apt-get install rsync -y
RUN rm -r /tmp/tj2_ros_workstation/sys && \
    rsync -rtul --ignore-existing /tmp/tj2_ros_workstation/* /
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

RUN adduser ${USER} sudo && \
  echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

RUN chown -R 1000:1000 ${HOME}

# ---
# ROS build environment
# ---

ENV ROS_DISTRO=noetic
ENV ROS_WS_ROOT=${HOME}/ros_ws
ENV ROS_WS_SRC=${ROS_WS_ROOT}/src

ENV DEP_ROS_WS_ROOT=${HOME}/dep_ws
ENV DEP_ROS_WS_SRC=${HOME}/dep_ws/src

ENV FLASK_ENV=development
ENV PYTHONPATH=${ROS_WS_SRC}/tj2_ros/tj2_tools:/opt/yolov5${PYTHONPATH:+:${PYTHONPATH}}


# ---
# tj2_ros launch environment
# ---
USER root
RUN chown root:root /usr/bin/sudo && \
    chmod 4755 /usr/bin/sudo

WORKDIR ${HOME}
USER ${USER}

ENTRYPOINT ["/usr/bin/dumb-init", "--"]
