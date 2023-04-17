# https://github.com/dusty-nv/jetson-containers
# https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-tensorflow
# https://github.com/dusty-nv/ros_deep_learning

# Much is from https://github.com/dusty-nv/jetson-containers/blob/master/Dockerfile.ros.melodic

ARG BASE_IMAGE=nvcr.io/nvidia/l4t-base:r32.5.0
FROM ${BASE_IMAGE}

ARG ROS_PKG=ros_base
ENV ROS_DISTRO=melodic
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /workspace

# add the ROS deb repo to the apt sources list
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        git \
		cmake \
		build-essential \
		curl \
		wget \
		gnupg2 \
		lsb-release \
		ca-certificates

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -


# 
# install ROS packages
#
# Desktop because we need rviz and such
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
		ros-melodic-`echo "${ROS_PKG}" | tr '_' '-'` \
		ros-melodic-image-transport \
		ros-melodic-vision-msgs \
        python-rosdep \
        python-rosinstall \
        python-rosinstall-generator \
        python-wstool \
        python3-vcstool \
        ros-${ROS_DISTRO}-desktop
    
# init/update rosdep
RUN apt-get update && \
    cd ${ROS_ROOT} && \
    rosdep init && \
    rosdep update

ENV WS /fennee_ws

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source $WS/devel/setup.bash" >> ~/.bashrc
CMD ["bash"]

# # Install dependencies first to cache docker layer
WORKDIR $WS/src

ADD fennee_ros/fennee.rosinstall fennee/fennee_ros/
RUN vcs import < fennee/fennee_ros/fennee.rosinstall --recursive
WORKDIR $WS

RUN apt-get update && rosdep install --from-paths src --ignore-src -r -y


COPY . src/fennee

RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
            rosdep install --from-paths src --ignore-src -r -y && \
            catkin_make install"

# RUN source devel/setup.bash
