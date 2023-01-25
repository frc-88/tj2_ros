FROM osrf/ros:noetic-desktop-full

RUN apt-get update && \
    apt-get install -y --no-install-recommends apt-utils
RUN apt-get install -y git nano tmux curl htop net-tools iproute2 iputils-ping dumb-init
RUN apt-get install -y libsdl-image1.2-dev libsdl-dev python3-pip wget python3-tk

RUN ln -s /usr/bin/python3 /usr/bin/python

RUN python -m pip install -U setuptools
RUN python -m pip install -U scipy==1.5.4 \
    shapely==1.6.4 \
    mido \
    pyperclip \
    dataclasses \
    pynetworktables==2021.0.0 \
    numpy==1.22.4 \
    open3d \
    ezdxf \
    jsonlines

RUN apt-get install -y llvm-7*
RUN ln -s /usr/lib/llvm-7/bin/llvm-config /usr/bin
RUN python -m pip install Cython
RUN python -m pip install llvmlite==0.32.0
RUN python -m pip install numba==0.49.0

RUN python -m pip install torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cu116


# ---
# tj2_ros dependencies
# ---

# generated using rosdep install --from-paths . -r --reinstall --simulate
RUN apt-get install -y --ignore-missing ros-noetic-rospy \
    ros-noetic-catkin \
    ros-noetic-tf \
    ros-noetic-robot-localization \
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
    ros-noetic-xacro \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-serial \
    ros-noetic-tf2 \
    ros-noetic-nav-msgs \
    ros-noetic-twist-mux \
    ros-noetic-message-generation \
    ros-noetic-roslaunch \
    ros-noetic-message-runtime \
    ros-noetic-message-filters \
    ros-noetic-visualization-msgs \
    ros-noetic-image-transport \
    ros-noetic-cv-bridge \
    ros-noetic-vision-msgs \
    ros-noetic-image-geometry \
    ros-noetic-camera-info-manager \
    ros-noetic-dynamic-reconfigure \
    ros-noetic-image-transport-plugins \
    ros-noetic-perception-pcl \
    ros-noetic-pcl-msgs \
    ros-noetic-image-pipeline \
    ros-noetic-image-common \
    ros-noetic-amcl \
    ros-noetic-map-server \
    ros-noetic-gmapping \
    ros-noetic-rplidar-ros \
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
    ros-noetic-marker-msgs \
    ros-noetic-ros-numpy

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

WORKDIR /tmp
RUN git clone https://github.com/wkentaro/labelme.git
WORKDIR /tmp/labelme
RUN python3 setup.py install

RUN python -m pip install wandb
RUN mkdir -p /opt/yolov5
WORKDIR /opt/yolov5
RUN git clone https://github.com/ultralytics/yolov5.git
RUN cd yolov5 && python -m pip install -r requirements.txt

WORKDIR /
RUN ldconfig
RUN rm -r /tmp/*

# ---
# tj2_ros build environment
# ---

ENV USER=root
ENV HOME=/${USER}
WORKDIR ${HOME}

ENV ROS_WS_ROOT=${HOME}/ros_ws
ENV ROS_WS_SRC=${HOME}/ros_ws/src

ENV FLASK_ENV=development
ENV PYTHONPATH="${PYTHONPATH}:${ROS_WS_SRC}/tj2_ros/tj2_tools:/opt/yolov5"

COPY ./workstation/docker/resources/install ${HOME}/install

# ---
# tj2_ros launch environment
# ---

RUN cat ${HOME}/install/bashrc >> ${HOME}/.bashrc
RUN rm ${HOME}/install/bashrc

COPY ./workstation/docker/resources/install/entrypoint.sh /entrypoint.sh
COPY ./workstation/docker/resources/install/launch.sh /launch.sh
COPY ./workstation/docker/resources/install/roscore.sh /roscore.sh

ENTRYPOINT ["/usr/bin/dumb-init", "--"]
