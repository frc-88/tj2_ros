FROM tj2_ros_workstation:latest

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

USER ${USER}
RUN sudo chown -R 1000:1000 ${HOME}
WORKDIR ${HOME}
