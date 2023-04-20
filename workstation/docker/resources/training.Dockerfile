FROM tj2_ros_workstation:latest

RUN apt-get update && apt-get install -y --ignore-missing ros-noetic-rospy \
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
