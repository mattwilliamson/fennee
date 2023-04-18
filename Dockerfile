# https://github.com/dusty-nv/jetson-containers
# https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-tensorflow
# https://github.com/dusty-nv/ros_deep_learning

# https://github.com/dusty-nv/jetson-containers/blob/master/Dockerfile.ros.noetic

#
# this dockerfile roughly follows the 'Installing from source' from:
#   http://wiki.ros.org/noetic/Installation/Source
#
ARG BASE_IMAGE=nvcr.io/nvidia/l4t-base:r32.7.1
FROM ${BASE_IMAGE}

ARG ROS_PKG=ros_base
ENV ROS_DISTRO=noetic
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /workspace


#
# add the ROS deb repo to the apt sources list
#
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        git \
		cmake \
		build-essential \
		curl \
		wget \
		gnupg2 \
		lsb-release \
		ca-certificates \
    && rm -rf /var/lib/apt/lists/*

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -


#
# install bootstrap dependencies
#
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-pip \
        libpython3-dev \
        python3-rosdep \
        python3-rosinstall-generator \
        python3-vcstool \
        build-essential && \
    rosdep init && \
    rosdep update && \
    pip3 install importlib-metadata
# && rm -rf /var/lib/apt/lists/*

#
# download/build the ROS source
#
ARG ROS_DEPS="vision_msgs image_transport tf urdf ecl_threads robot_state_publisher robot_localization controller_manager"
RUN mkdir ros_catkin_ws && \
    cd ros_catkin_ws && \
    rosinstall_generator ${ROS_PKG} ${ROS_DEPS} --rosdistro ${ROS_DISTRO} --deps --tar > ${ROS_DISTRO}-${ROS_PKG}.rosinstall && \
    mkdir src && \
    vcs import --input ${ROS_DISTRO}-${ROS_PKG}.rosinstall ./src && \
    apt-get update && \
    rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro ${ROS_DISTRO} --skip-keys python3-pykdl -y && \
    python3 ./src/catkin/bin/catkin_make_isolated --install --install-space ${ROS_ROOT} -DCMAKE_BUILD_TYPE=Release && \
    rm -rf /var/lib/apt/lists/*


# -------

ENV WS /fennee_ws

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source $WS/devel/setup.bash" >> ~/.bashrc
CMD ["bash"]

# # Install dependencies first to cache docker layer
WORKDIR $WS/src

ADD fennee_ros/fennee.rosinstall fennee/fennee_ros/
RUN vcs import < fennee/fennee_ros/fennee.rosinstall --recursive
WORKDIR $WS

ARG ROSDEP_SKIP="python3-pykdl"
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -r --rosdistro ${ROS_DISTRO} --skip-keys ${ROSDEP_SKIP} -y && \
    rm -rf /var/lib/apt/lists/*

COPY . src/fennee

# RUN apt-get update  && \
#     apt-get install -y ros-${ROS_DISTRO}-gazebo-ros && \
#     rm -rf /var/lib/apt/lists/*

COPY fennee_ros/CATKIN_IGNORE src/champ/champ/champ_gazebo/
# RUN rm -rf src/champ/champ/champ_gazebo/

RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
            rosdep install --from-paths src --ignore-src -r -y"

RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
            catkin_make install"

# RUN source devel/setup.bash
