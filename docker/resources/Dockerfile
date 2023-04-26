#
# this dockerfile roughly follows the 'Install ROS From Source' procedures from:
#   https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html
#

FROM dustynv/ros:noetic-pytorch-l4t-r32.6.1

ENV L4T_MAJOR_VERSION=32
ENV L4T_MINOR_VERSION=6
ENV L4T_PATCH_VERSION=1

ENV ZED_SDK_MAJOR=3
ENV ZED_SDK_MINOR=8

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"] 

RUN apt-get update && \
    apt-get install -y --no-install-recommends apt-utils
RUN apt-get update && \
    apt-get install -y nano tmux curl htop net-tools iproute2 iputils-ping gdb dumb-init rsync sudo

# ---
# CMake
# ---

WORKDIR /tmp

RUN apt-get update && \
    apt-get install -y build-essential libtool autoconf zip unzip wget zstd
RUN wget -O cmake.sh https://cmake.org/files/v3.23/cmake-3.23.1-linux-aarch64.sh
RUN mkdir /opt/cmake
RUN sh cmake.sh --prefix=/opt/cmake --exclude-subdir --skip-license
RUN mv /usr/bin/cmake /usr/bin/cmake-old
RUN ln -s /opt/cmake/bin/cmake /usr/bin/cmake

# ---
# PyTorch CMake
# ---

ENV LD_LIBRARY_PATH=/usr/local/lib/python3.6/dist-packages/torch/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
ENV CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/usr/local/lib/python3.6/dist-packages/torch/share/cmake/Torch

# ---
# ZED
# ---

WORKDIR /

# This environment variable is needed to use the streaming features on Jetson inside a container
ENV LOGNAME root
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update && \
    apt-get install --no-install-recommends lsb-release less udev sudo apt-transport-https -y && \
    echo "# R${L4T_MAJOR_VERSION} (release), REVISION: ${L4T_MINOR_VERSION}.${L4T_PATCH_VERSION}" > /etc/nv_tegra_release ; \
    wget -q --no-check-certificate -O ZED_SDK_Linux.run https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/l4t${L4T_MAJOR_VERSION}.${L4T_MINOR_VERSION}/jetsons && \
    chmod +x ZED_SDK_Linux.run ; ./ZED_SDK_Linux.run silent skip_tools && \
    rm -rf /usr/local/zed/resources/* \
    rm -rf ZED_SDK_Linux.run

# This symbolic link is needed to use the streaming features on Jetson inside a container
RUN ln -sf /usr/lib/aarch64-linux-gnu/tegra/libv4l2.so.0 /usr/lib/aarch64-linux-gnu/libv4l2.so

# ---
# tj2_ros dependencies
# ---

RUN rm /usr/bin/python
RUN ln -s /usr/bin/python3 /usr/bin/python

RUN apt-get update && sudo apt-get install -y \
    libbullet-dev \
    libeigen3-dev \
    libsdl-image1.2-dev \
    libsdl-dev \
    libyaml-cpp-dev \
    libpcl-dev  \
    libvtk6-qt-dev  \
    libspnav-dev  \
    joystick  \
    liborocos-kdl-dev  \
    liborocos-kdl1.3  \
    libnetpbm10-dev  \
    libogg-dev  \
    libtheora-dev  \
    graphviz  \
    libgeographic-dev  \
    python3-pyproj  \
    ffmpeg  \
    libavcodec-dev  \
    libavformat-dev  \
    libavutil-dev  \
    libswscale-dev  \
    v4l-utils  \
    liburdfdom-headers-dev  \
    libtinyxml-dev  \
    liburdfdom-dev  \
    hddtemp  \
    lm-sensors  \
    python3-psutil  \
    libusb-1.0-0*  \
    libswscale-dev \
    libsuitesparse-dev \
    python3-termcolor \
    i2c-tools \
    python3-smbus \
    libceres-dev \
    libturbojpeg0-dev \
    python3-twisted \
    python3-tornado \
    python3-autobahn \
    python3-bson

RUN apt-get upgrade -y

RUN python -m pip install --upgrade pip
RUN python -m pip install --upgrade setuptools
RUN python -m pip install scipy==1.5.4 \
    shapely==1.6.4 \
    dataclasses \
    pynetworktables==2021.0.0 \
    pynmcli==1.0.5 \
    flask==2.0.3 \
    py-trees==0.7.6 \
    open3d==0.15.1 \
    psutil

RUN apt-get update && apt-get install -y llvm-7*
RUN ln -s /usr/lib/llvm-7/bin/llvm-config /usr/bin
RUN python -m pip install Cython
RUN python -m pip install llvmlite==0.32.0
RUN python -m pip install numba==0.49.0

WORKDIR /tmp
RUN git clone https://github.com/wjakob/tbb.git && \
    cd tbb && \
    git checkout 9e219e24fe223b299783200f217e9d27790a87b0
WORKDIR /tmp/tbb/build
RUN cmake ..
RUN make -j4 && make install

WORKDIR /tmp
RUN git clone https://github.com/AprilRobotics/apriltag.git && \
    cd apriltag && \
    git checkout 3e8e974d0d8d6ab318abf56d87506d15d7f2cc35 && \
    mkdir build
WORKDIR /tmp/apriltag/build
RUN cmake ..
RUN make -j4 && make install

WORKDIR /tmp
RUN git clone https://github.com/orocos/orocos_kinematics_dynamics.git && \
    cd orocos_kinematics_dynamics && \
    git checkout 5541147d4a220cab97d0ae1efa1aa860557d5c32 && \
    git submodule update --init && \
    cd orocos_kdl && \
    mkdir build
WORKDIR /tmp/orocos_kinematics_dynamics/orocos_kdl/build
RUN cmake ..
RUN make -j4 && make install
RUN mkdir ../../python_orocos_kdl/build
WORKDIR /tmp/orocos_kinematics_dynamics/python_orocos_kdl/build
RUN cmake -D PYTHON_EXECUTABLE=/usr/bin/python3 \
    -D PYTHON_INCLUDE_DIR=/usr/include/python3.6 \
    -D PYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.6m.so \
    -D PYBIND11_PYTHON_VERSION=3 ..
RUN make -j4 && make install

WORKDIR /tmp
RUN git clone https://github.com/FRC-Utilities/LibDS.git && \
    cd LibDS && \
    git checkout 14cac0a7f3b911b3f1c661c3b5f455522ae6638b
WORKDIR /tmp/LibDS
RUN qmake -qt=qt5
RUN make -j4 && make install

WORKDIR /tmp
RUN git clone --recursive https://github.com/ThadHouse/CmakeWpilib.git && \
    cd CmakeWpilib && \
    git checkout d5bbeb949de35a4c576838e598493779bcf6a328 && \
    mkdir build
WORKDIR /tmp/CmakeWpilib/build
RUN cmake .. -DWITHOUT_CSCORE=ON -DWITHOUT_JAVA=ON
RUN make -j4 && make install

WORKDIR /tmp
RUN git clone https://github.com/RainerKuemmerle/g2o.git
WORKDIR /tmp/g2o
RUN git checkout 20201223_git
RUN mkdir build
WORKDIR /tmp/g2o/build
RUN cmake ..
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
# ROS evergreen dependency workspace
# ---

RUN sh -c 'echo "deb http://packages.ros.org/ros-testing/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-get update && apt-get install -y \
    python3-osrf-pycommon \
    python3-wstool \
    python3-catkin-pkg

ENV DEP_ROS_WS_ROOT=${HOME}/dep_ws
ENV DEP_ROS_WS_SRC=${HOME}/dep_ws/src

WORKDIR ${DEP_ROS_WS_SRC}
RUN wstool init .

COPY --chown=1000:1000 ./install/preferences /etc/apt/preferences
COPY --chown=1000:1000 ./install/evergreen ${HOME}/install/evergreen

WORKDIR ${DEP_ROS_WS_SRC}
RUN wstool merge -t . ${HOME}/install/evergreen/tj2_ros_evergreen.rosinstall
RUN wstool update -t .
RUN ${HOME}/install/evergreen/patch_evergreen_packages.sh

WORKDIR ${DEP_ROS_WS_ROOT}
COPY --chown=1000:1000 ./install/rosdep_install.sh ${HOME}/install
RUN ${HOME}/install/rosdep_install.sh

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin_make -DCMAKE_BUILD_TYPE=Release -DSETUPTOOLS_DEB_LAYOUT=OFF

# ---
# ROS overlay dependency workspace
# ---

COPY --chown=1000:1000 ./install/overlay ${HOME}/install/overlay

WORKDIR ${DEP_ROS_WS_SRC}
RUN wstool merge -t . ${HOME}/install/overlay/tj2_ros_overlay.rosinstall
RUN wstool update -t .

WORKDIR ${DEP_ROS_WS_ROOT}
RUN ${HOME}/install/rosdep_install.sh
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin_make -DCMAKE_BUILD_TYPE=Release -DSETUPTOOLS_DEB_LAYOUT=OFF

RUN apt-get clean

# ---
# Environment setup
# ---

ENV ROS_WS_ROOT=${HOME}/ros_ws
ENV ROS_WS_SRC=${ROS_WS_ROOT}/src

ENV FLASK_ENV=development
ENV PYTHONPATH=${ROS_WS_SRC}/tj2_tools${PYTHONPATH:+:${PYTHONPATH}}
ENV PYTHONIOENCODING=utf-8

COPY --chown=1000:1000 ./install/enable_tmux_logger.sh ${HOME}/install
COPY --chown=1000:1000 ./install/bashrc ${HOME}/.bashrc

COPY --chown=1000:1000 ./launch/entrypoint.sh /entrypoint.sh
COPY --chown=1000:1000 ./launch/launch.sh /launch.sh
COPY --chown=1000:1000 ./launch/roscore.sh /roscore.sh

WORKDIR ${ROS_WS_ROOT}
RUN sudo chown -R 1000:1000 ${HOME}

ENTRYPOINT ["/usr/bin/dumb-init", "--"]
