ARG BASE_IMAGE_NAME=ubuntu:22.04
FROM nvidia/cuda:12.3.0-devel-ubuntu22.04 as base
# FROM $BASE_IMAGE_NAME AS BASE
LABEL maintainer="Andrew Messing"\
      description="Base image for GRSTAPSE. Contains all the third-party libraries needed."\
      version="0.0.25"
SHELL ["/bin/bash", "-c"]
ARG NUM_BUILD_CORES=30

#
ENV TZ=America/New_York
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

ENV DEBIAN_FRONTEND noninteractive

# Install APT packages
RUN apt update --fix-missing &&\
    apt upgrade -y &&\
    apt-get install -y --no-install-recommends software-properties-common lsb-release
# For gcc-11 & g++-11
RUN add-apt-repository -y ppa:ubuntu-toolchain-r/test
RUN apt update --fix-missing &&\
    apt upgrade -y &&\
    apt-get install -y --no-install-recommends \
        bison \
        build-essential \
        castxml \
        clang-format-12 \
        clang-tidy-12 \
        cmake \
        curl \
        doxygen \
        libeigen3-dev \
        flex \
        g++-10 \
        gcc-10 \
        g++-11 \
        gcc-11 \
        git \
        git-lfs \
        gnupg \
        graphviz \
        graphviz-dev \
        keyboard-configuration \
        lcov \
        libassimp-dev \
        libboost-all-dev \
        libccd-dev \
        libfcl-dev \
        libgeos++-dev \
        libglu1-mesa-dev \
        libode-dev \
        libopencv-dev \
        libssl-dev \
        libtbb-dev \
        libtinyxml2-dev \
        libyaml-cpp-dev \
        lsb-core \
        python3 \
        pypy3 \
        python3-apt \
        python3-celery \
        python3-dev \
        python3-distutils \
        python3-flask \
        python3-opencv \
        python3-opengl \
        python3-numpy \
        python3-pip \
        python3-pybind11 \
        python3-tk \
        tmux \
        valgrind \
        wget \
        xorg-dev \
        zlib1g-dev

RUN KERNEL_VERSION=$(uname -r) &&\
    apt-get install -y --no-install-recommends "linux-tools-${KERNEL_VERSION}" &&\
    sh -c 'echo kernel.perf_event_paranoid=1 >> /etc/sysctl.d/99-perf.conf' &&\
    sh -c 'echo kernel.kptr_restrict=0 >> /etc/sysctl.d/99-perf.conf' &&\
    sh -c 'sysctl --system'

RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 3
# RUN update-alternatives --install /usr/bin/python python /usr/bin/python2 2

# Setup alternatives
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 10 &&\
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-10 10 &&\
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 11 &&\
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 11 &&\
    update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-12 12 &&\
    update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-12 12

# Install pip packages
RUN pip3 install \
    dgl \
    gcovr \
    gitpython \
    igraph \
    ipython \
    matplotlib \
    networkx \
    numpy \
    pandas \
    pygraphviz \
    pygtrie \
    torch \
    scipy \
    scikit-learn

## For cmake 3.20
#RUN apt purge --auto-remove -y cmake &&\
#    wget https://github.com/Kitware/CMake/releases/download/v3.20.5/cmake-3.20.5.tar.gz &&\
#    tar -zxvf cmake-3.20.5.tar.gz &&\
#    cd cmake-3.20.5 &&\
#    ./bootstrap &&\
#    make -j${NUM_BUILD_CORES} &&\
#    make install

# Include OMPL (python bindings are off)
RUN wget -O - https://github.com/ompl/ompl/archive/1.5.2.tar.gz | tar zxf - &&\
    mkdir -p ompl-1.5.2/build &&\
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DOMPL_BUILD_DEMOS=OFF \
          -DOMPL_BUILD_PYBINDINGS=OFF \
          -DOMPL_BUILD_PYTESTS=OFF \
          -DOMPL_BUILD_TESTS=OFF \
          -DOMPL_REGISTRATION=OFF\
          -DOMPL_VERSIONED_INSTALL=OFF\
          -S ompl-1.5.2 -B ompl-1.5.2/build &&\
    cmake --build ompl-1.5.2/build --parallel ${NUM_BUILD_CORES} &&\
    cmake --install ompl-1.5.2/build &&\
    rm -rf ompl-1.5.2

RUN apt autoclean -y &&\
    apt autoremove -y &&\
    apt clean -y &&\
    rm -rf /var/lib/apt/lists/*

# fix eigen
RUN ln -s /usr/include/eigen3/Eigen /usr/include/Eigen

## Google Test
RUN git clone https://github.com/amessing/googletest.git &&\
    export CXXFLAGS="-Ofast" &&\
    mkdir -p googletest/build &&\
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DBUILD_GMOCK=ON \
          -DINSTALL_GTEST=ON \
          -S googletest -B googletest/build &&\
    cmake --build googletest/build --parallel ${NUM_BUILD_CORES} &&\
    cmake --install googletest/build &&\
    rm -rf googletest

## Google benchmark
RUN git clone https://github.com/amessing/benchmark.git &&\
    export CXXFLAGS="-Ofast" &&\
    mkdir -p benchmark/build &&\
    cmake  -DCMAKE_BUILD_TYPE=Release \
           -DBENCHMARK_ENABLE_TESTING=OFF \
           -S benchmark -B benchmark/build &&\
    cmake --build benchmark/build  --parallel ${NUM_BUILD_CORES} &&\
    cmake --install benchmark/build &&\
    rm -rf benchmark

## nlohmann json
RUN git clone https://github.com/amessing/json.git &&\
    export CXXFLAGS="-Ofast" &&\
    mkdir -p json/build &&\
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DJSON_BuildTests=OFF \
          -S json -B json/build &&\
    cmake --build json/build --parallel ${NUM_BUILD_CORES} &&\
    cmake --install json/build &&\
    rm -rf json

## fmt
RUN git clone https://github.com/amessing/fmt.git &&\
    export CXXFLAGS="-Ofast" &&\
    mkdir -p fmt/build &&\
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DFMT_DOC=OFF \
          -DFMT_TEST=OFF \
          -DCMAKE_POSITION_INDEPENDENT_CODE=TRUE\
          -S fmt -B fmt/build &&\
    cmake --build fmt/build --parallel ${NUM_BUILD_CORES} &&\
    cmake --install fmt/build &&\
    rm -rf fmt

## spdog
RUN git clone -b v1.x https://github.com/amessing/spdlog.git &&\
    export CXXFLAGS="-Ofast" &&\
    mkdir -p spdlog/build &&\
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_CXX_FLAGS="-fpic" \
          -DSPDLOG_FMT_EXTERNAL=ON \
          -DSPDLOG_FMT_EXTERNAL_HO=OFF \
          -DSPDLOG_BUILD_TESTS=OFF \
          -DSPDLOG_BUILD_EXAMPLE=OFF \
          -S spdlog -B spdlog/build &&\
    cmake --build spdlog/build --parallel ${NUM_BUILD_CORES} &&\
    cmake --install spdlog/build &&\
    rm -rf spdlog

## range-v3
RUN git clone https://github.com/amessing/range-v3.git range &&\
    export CXXFLAGS="-Ofast" &&\
    mkdir -p range/build &&\
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DRANGE_V3_TESTS=OFF \
          -DRANGE_V3_EXAMPLES=OFF \
          -DRANGE_V3_PERF=OFF \
          -DRANGE_V3_DOCS=Off \
          -S range -B range/build &&\
    cmake --build range/build --parallel ${NUM_BUILD_CORES} &&\
    cmake --install range/build &&\
    rm -rf range

## cppcoro
RUN git clone https://github.com/amessing/cppcoro.git &&\
    export CXXFLAGS="-Ofast -fcoroutines" &&\
    mkdir -p cppcoro/build &&\
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DBUILD_TESTING=OFF \
          -S cppcoro -B cppcoro/build &&\
    cmake --build cppcoro/build --parallel ${NUM_BUILD_CORES} &&\
    cmake --install cppcoro/build &&\
    rm -rf cppcoro

## concurrencpp
RUN git clone -b develop https://github.com/amessing/concurrencpp.git &&\
    export CXXFLAGS="-Ofast -fcoroutines" &&\
    mkdir -p concurrencpp/build &&\
    cmake -DCMAKE_BUILD_TYPE=Release \
          -S concurrencpp -B concurrencpp/build &&\
    cmake --build concurrencpp/build --parallel ${NUM_BUILD_CORES} &&\
    cmake --install concurrencpp/build &&\
    rm -rf concurrencpp

ENV GUROBI_INSTALL /opt/gurobi
ENV GUROBI_HOME $GUROBI_INSTALL/linux64
ENV PATH $PATH:$GUROBI_HOME/bin
ENV GUROBI_LIB_PATH $GUROBI_HOME/lib

ARG GUROBI_MAJOR_VERSION=9.5
ARG GUROBI_VERSION=9.5.0
ARG GUROBI_FOLDER_VERSION=950

# Install third party libs
## Install gurobi
RUN mkdir -p ${GUROBI_INSTALL} &&\
    wget http://packages.gurobi.com/${GUROBI_MAJOR_VERSION}/gurobi${GUROBI_VERSION}_linux64.tar.gz &&\
    tar xvfz gurobi${GUROBI_VERSION}_linux64.tar.gz &&\
    mv gurobi${GUROBI_FOLDER_VERSION}/linux64/ ${GUROBI_INSTALL}
RUN export CXXFLAGS="-Ofast" && cd ${GUROBI_HOME}/src/build && make -j ${NUM_BUILD_CORES}
RUN mv ${GUROBI_HOME}/src/build/libgurobi_c++.a ${GUROBI_LIB_PATH}/libgurobi_g++10.3.a
RUN cd ${GUROBI_LIB_PATH} && ln -sf libgurobi_g++10.3.a libgurobi_c++.a
RUN rm -r ${GUROBI_HOME}/docs &&\
    rm -r ${GUROBI_HOME}/examples &&\
    rm -r ${GUROBI_HOME}/src &&\
    rm -r gurobi${GUROBI_FOLDER_VERSION}/ &&\
    rm gurobi${GUROBI_VERSION}_linux64.tar.gz

## OR-TOOLS
#RUN git clone https://github.com/amessing/or-tools.git ortools &&\
#    mkdir -p ortools/build &&\
#    cmake -DCMAKE_BUILD_TYPE=Release \
#          -DBUILD_CXX=ON \
#          -DBUILD_DEPS=ON \
#          -DBUILD_PYTHON=ON \
#          -BUILD_EXAMPLES=OFF \
#          -DBUILD_SAMPLES=OFF \
#          -S ortools \
#          -B ortools/build &&\
#    cmake --build ortools/build --parallel ${NUM_BUILD_CORES} &&\
#    cmake --install ortools/build &&\
#    rm -rf ortools


#### REMOTE ####
FROM base AS remote
LABEL maintainer="Andrew Messing"\
      description="Creates a remote environment that can be connected to through ssh."\
      version="0.0.6"
ARG CUSTOM_USER=debugger
# ARG host_uid
# ARG host_gid
ARG home=/home/$CUSTOM_USER

RUN apt update --fix-missing &&\
    apt-get install -y --no-install-recommends \
        apt-utils \
        dbus-x11 \
        openssh-server \
        gdb \
        gdbserver \
        gosu \
        mesa-utils \
        rsync \
        vim \
        xauth
# Check that gosu works for the entrypoint
RUN gosu nobody true
RUN apt autoclean -y &&\
    apt autoremove -y &&\
    apt clean -y &&\
    rm -rf /var/lib/apt/lists/*

RUN mkdir /var/run/sshd &&\
    echo 'root:root' | chpasswd &&\
    sed -i 's/PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config &&\
    sed -i 's/#AddressFamily any/AddressFamily inet/' /etc/ssh/sshd_config

# SSH login fix. Otherwise user is kicked off after login
RUN sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd

ENV NOTVISIBLE "in users profile"
RUN echo "export VISIBLE=now" >> /etc/profile

RUN sed -i 's/Port 22/Port 7776/' /etc/ssh/sshd_config
# 22 for ssh server. 7777 for gdb server.
#EXPOSE 22 7777

RUN useradd -ms /bin/bash $CUSTOM_USER
RUN echo $CUSTOM_USER:pwd | chpasswd
# RUN usermod --uid $host_uid $user
# RUN groupmod --gid $host_gid $user

USER $CUSTOM_USER
ENV HOME $home

USER root
CMD ["/usr/sbin/sshd", "-D", "-p", "7776"]


#### REMOTE with NVIDIA ####
#FROM REMOTE AS REMOTE_NVIDIA
#LABEL maintainer="Andrew Messing" \
#      description="Upgrades the remote environment to have ros and interface with nvidia" \
#      version="0.0.1"

USER root

ARG CUSTOM_USER=debugger

# Install Nvidia Driver
RUN add-apt-repository ppa:graphics-drivers &&\
    apt update --fix-missing &&\
    apt install -y --no-install-recommends \
        nvidia-driver-515

# Install CUDA
#RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin &&\
#    mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600 &&\
#    wget https://developer.download.nvidia.com/compute/cuda/12.3.0/local_installers/cuda-repo-ubuntu2204-12-3-local_12.3.0-515.43.04-1_amd64.deb &&\
#    dpkg -i cuda-repo-ubuntu2204-11-7-local_11.7.0-515.43.04-1_amd64.deb &&\
#    cp /var/cuda-repo-ubuntu2204-11-7-local/cuda-*-keyring.gpg /usr/share/keyrings/ &&\
#    apt update --fix-missing &&\
#    apt install -y --no-install-recommends \
#        cuda


RUN pip3 uninstall -y torch dgl &&\
    pip3 install torch --extra-index-url https://download.pytorch.org/whl/cu121 \
                 dgl-cu113 dglgo -f https://data.dgl.ai/wheels/repo.html

RUN apt autoclean -y &&\
    apt autoremove -y &&\
    apt clean -y


CMD ["/usr/sbin/sshd", "-D", "-p", "7776"]

##### REMOTE with ROS ####
#FROM REMOTE AS REMOTE_ROS
#LABEL maintainer="Andrew Messing" \
#      description="Upgrades the remote environment to have ros" \
#      version="0.0.13"
#
#ARG CUSTOM_USER=debugger
#ARG ROS_DISTRO=noetic
#
## Install ROS
#RUN echo "deb [trusted=yes] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list &&\
#    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - &&\
#    apt update --fix-missing &&\
#    apt-get install -y --no-install-recommends \
#        ros-${ROS_DISTRO}-ros-base \
#        ros-${ROS_DISTRO}-gazebo-ros-control \
#        ros-${ROS_DISTRO}-gmapping \
#        ros-${ROS_DISTRO}-hector-gazebo \
#        ros-${ROS_DISTRO}-hector-slam \
#        ros-${ROS_DISTRO}-interactive-markers \
#        ros-${ROS_DISTRO}-joy \
#        ros-${ROS_DISTRO}-lms1xx \
#        ros-${ROS_DISTRO}-navigation \
#        ros-${ROS_DISTRO}-position-controllers\
#        ros-${ROS_DISTRO}-robot-localization \
#        ros-${ROS_DISTRO}-robot-state-publisher \
#        ros-${ROS_DISTRO}-ros-control \
#        ros-${ROS_DISTRO}-ros-controllers \
#        ros-${ROS_DISTRO}-roslint \
#        ros-${ROS_DISTRO}-rqt-console \
#        ros-${ROS_DISTRO}-rqt-graph \
#        ros-${ROS_DISTRO}-rqt-tf-tree \
#        ros-${ROS_DISTRO}-rqt-topic \
#        ros-${ROS_DISTRO}-rviz \
#        ros-${ROS_DISTRO}-slam-toolbox \
#        ros-${ROS_DISTRO}-twist-mux \
#        ros-${ROS_DISTRO}-xacro &&\
#    apt-get install -y --no-install-recommends \
#        python3-rosdep \
#        python3-rosinstall \
#        python3-rosinstall-generator \
#        python3-wstool
#
#RUN rosdep init
#
## Back to our stuff
#RUN apt autoclean &&\
#    apt autoremove &&\
#    apt clean &&\
#    rm -rf /var/lib/apt/lists/*
#
## Update gencpp to work with C++20
#RUN git clone https://github.com/amessing/gencpp.git &&\
#    cp gencpp/scripts/msg.h.template /opt/ros/${ROS_DISTRO}/share/gencpp/ &&\
#    cp gencpp/src/gencpp/__init__.py /opt/ros/${ROS_DISTRO}/lib/python3/dist-packages/gencpp/ &&\
#    rm -rf gencpp
#
## Fix previously created headers from msgs
#COPY fix_msgs.py /
#RUN python fix_msgs.py
#
#USER $CUSTOM_USER
#RUN rosdep update
#ENV ROS_DISTRO $ROS_DISTRO
#RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
#RUN mkdir -p ~/catkin_ws/src
#
## fkie
## RUN git clone https://github.com/fkie/multimaster_fkie.git ~/catkin_ws/src/multimaster
#
## Clearpath
### Husky (Outdoor UGV)
#RUN git clone https://github.com/husky/husky.git ~/catkin_ws/src/husky
## ## Clone Warthog (Outdoor UGV)
## RUN git clone https://github.com/warthog-cpr/warthog.git ~/catkin_ws/src/warthog
## RUN git clone https://github.com/warthog-cpr/warthog_simulator.git ~/catkin_ws/src/warthog_simulator
## ## Jackal (Outdoor UGV)
## RUN git clone https://github.com/jackal/jackal.git ~/catkin_ws/src/jackal
## RUN git clone https://github.com/jackal/jackal_simulator.git ~/catkin_ws/src/jackal_simulator
## ## Ridgeback (Omnidirectional Platform)
## RUN git clone https://github.com/ridgeback/ridgeback.git ~/catkin_ws/src/ridgeback
## RUN git clone https://github.com/ridgeback/ridgeback_simulator.git ~/catkin_ws/src/ridgeback_simulator
### CPR Multimaster Tools
## RUN git clone https://github.com/clearpathrobotics/cpr_multimaster_tools.git ~/catkin_ws/src/cpr_multimaster_tools
#
## Fetch
## RUN git clone https://github.com/fetchrobotics/fetch_ros.git ~/catkin_ws/src/fetch
#
## Universal Robotics
## RUN git clone https://github.com/ros-industrial/universal_robot.git ~/catkin_ws/src/universal_robot
#
## Clone hector gazebo
## RUN git clone https://github.com/tu-darmstadt-ros-pkg/hector_gazebo.git ~/catkin_ws/src/hector_gazebo
## RUN git clone https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor.git ~/catkin_ws/src/hector_quadrotor
## RUN git clone https://github.com/tu-darmstadt-ros-pkg/hector_localization.git ~/catkin_ws/src/hector_localization
#
## ROBOTIS
### turtlebot2
#RUN git clone https://github.com/turtlebot/turtlebot.git ~/catkin_ws/src/turtlebot2 &&\
#    git clone https://github.com/turtlebot/turtlebot_msgs.git ~/catkin_ws/src/turtlebot2_msgs &&\
#    git clone https://github.com/turtlebot/turtlebot_simulator.git ~/catkin_ws/src/turtlebot2_simulator
### turtlebot 3
#RUN git clone https://github.com/ROBOTIS-GIT/turtlebot3.git ~/catkin_ws/src/turtlebot3 &&\
#    git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs ~/catkin_ws/src/turtlebot3_msgs &&\
#    git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations ~/catkin_ws/src/turtlebot3_simulations &&\
#    echo 'export TURTLEBOT3_MODEL="burger"' >> ~/.bashrc
### open manipulator
## RUN git clone https://github.com/ROBOTIS-GIT/robotis_manipulator.git ~/catkin_ws/src/robotis_manipulator
## RUN git clone https://github.com/ROBOTIS-GIT/open_manipulator.git ~/catkin_ws/src/open_manipulator
## RUN git clone https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git ~/catkin_ws/src/open_manipulator_msgs
## RUN git clone https://github.com/ROBOTIS-GIT/open_manipulator_simulations.git ~/catkin_ws/src/open_manipulator_simulations
#
## Clone interactive marker twist server
#RUN git clone https://github.com/ros-visualization/interactive_marker_twist_server.git ~/catkin_ws/src/interactive_marker_twist_server
#
#RUN source /opt/ros/${ROS_DISTRO}/setup.bash &&\
#    cd ~/catkin_ws &&\
#    catkin_make_isolated &&\
#    source devel/setup.bash
#RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc  &&\
#    printf 'export HUSKY_GAZEBO_DESCRIPTION="$(rospack find husky_gazebo)/urdf/description.gazebo.xacro"\n' >> ~/.bashrc
#
#RUN roscore &
#
#USER root
#CMD ["/usr/sbin/sshd", "-D", "-p", "7776"]