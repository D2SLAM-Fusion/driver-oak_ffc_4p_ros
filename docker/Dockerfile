FROM ros:noetic-robot-focal
ARG OAK_WS=/root/oak_ffc_ws/
ARG DEPTH_CORE=/root/3rd_party/
ARG ROS_VERSION=noetic
ARG DEPTH_AI_CORE=2.28.0
ARG SPDLOG=1.8.2
ARG FMT=7.1.3
SHELL ["/bin/bash", "-c"] 

WORKDIR ${DEPTH_CORE}

RUN apt-get update &&\
    apt-get install -y \
    vim wget pip clang-format gdb xterm\
    ros-noetic-depthai-bridge \
    ros-noetic-depthai-ros-msgs \
    ros-noetic-image-transport \
    ros-noetic-cv-bridge \
    python3-catkin-tools

# install depth-ai core   
RUN wget https://github.com/luxonis/depthai-core/releases/download/v${DEPTH_AI_CORE}/depthai-core-v${DEPTH_AI_CORE}.tar.gz  &&\
    tar -xvf depthai-core-v${DEPTH_AI_CORE}.tar.gz &&\
    mv depthai-core-v${DEPTH_AI_CORE} depthai-core &&\
    rm depthai-core-v${DEPTH_AI_CORE}.tar.gz &&\
    cd depthai-core &&\
    mkdir build &&\
    cd build &&\
    cmake .. &&\
    make -j$(nproc) &&\
    make install

RUN cd ${DEPTH_CORE}/depthai-core/build/install/include &&\
    mv ./depthai /usr/local/include/depthai &&\
    mv ./depthai-shared /usr/local/include/depthai-shared &&\
    mv ./depthai-bootloader-shared /usr/local/include/depthai-bootloader-shared

# install spdlog
RUN wget https://github.com/gabime/spdlog/archive/refs/tags/v${SPDLOG}.tar.gz &&\
    tar -xvf v${SPDLOG}.tar.gz &&\
    mv spdlog-${SPDLOG} spdlog &&\
    rm v${SPDLOG}.tar.gz &&\
    cd spdlog &&\
    mkdir build &&\
    cd build &&\
    cmake .. &&\
    make -j$(nproc) &&\
    make install

# install fmt
RUN wget https://github.com/fmtlib/fmt/archive/refs/tags/${FMT}.tar.gz &&\
    tar -xvf ${FMT}.tar.gz &&\
    rm ${FMT}.tar.gz &&\
    mv fmt-${FMT} fmt &&\
    cd fmt &&\
    mkdir build &&\
    cd build &&\
    cmake .. &&\
    make -j$(nproc) &&\
    make install

# ROS ENV setup
RUN echo "source /opt/ros/${ROS_VERSION}/setup.bash" >> ~/.bashrc &&\
    source ~/.bashrc

# Build oak_ffc_4p_ros
COPY . ${OAK_WS}/src/oak_ffc_4p_ros

RUN source "/opt/ros/${ROS_VERSION}/setup.bash" &&\
    cd ${OAK_WS} &&\
    catkin build &&\
    echo "source ${OAK_WS}/devel/setup.bash" >> ~/.bashrc &&\
    source ~/.bashrc

WORKDIR ${OAK_WS}

RUN source "/opt/ros/noetic/setup.bash" &&\
    cd ${OAK_WS} &&\
    catkin_make
