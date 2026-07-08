FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic

# Base build tools + ROS/catkin deps
RUN apt-get update && apt-get install -y \
    git \
    wget \
    curl \
    vim \
    build-essential \
    cmake \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    liburdfdom-dev \
    liboctomap-dev \
    libassimp-dev \
    ros-noetic-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

# MuJoCo C++ SDK (used by legged_mujoco, hardcoded to /usr/local/mujoco
# in legged_mujoco/CMakeLists.txt). Requires C++17 (mjspec.h uses std::byte),
# already set via CMAKE_CXX_STANDARD in that CMakeLists.
ARG MUJOCO_VERSION=3.2.3
RUN wget -q https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz -O /tmp/mujoco.tar.gz \
    && mkdir -p /usr/local/mujoco \
    && tar -xzf /tmp/mujoco.tar.gz -C /usr/local/mujoco --strip-components=1 \
    && rm /tmp/mujoco.tar.gz

# MuJoCo Python bindings, used by the debug viewer (legged_mujoco/scripts/mujoco_viewer.py)
RUN pip3 install mujoco==${MUJOCO_VERSION} glfw

# Create catkin workspace. src/ is meant to be bind-mounted at `docker run`
# time (see docker/README.md) with this repo plus its sibling OCS2
# dependencies (ocs2, ocs2_robotic_assets, hpp-fcl, pinocchio) side by side.
RUN mkdir -p /home/catkin_ws/src
WORKDIR /home/catkin_ws

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo"

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /home/catkin_ws/devel/setup.bash 2>/dev/null || true" >> ~/.bashrc

CMD ["/bin/bash"]
