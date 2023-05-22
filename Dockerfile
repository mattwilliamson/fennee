# TODO: inlcude gazebo if we are launching with GUI
# TODO: handle BUILD_SEQUENTIAL=1 to build on jetson/pi - catkin build -j1 -l1

ARG ROS_DISTRO=noetic
FROM ros:${ROS_DISTRO}-ros-base
ARG USE_RVIZ
ARG BUILD_SEQUENTIAL=1
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends \
      software-properties-common \
      git \
      libusb-1.0-0-dev \
      wget \
      python3-catkin-tools \
      python3-pip \
      libopencv-dev \
      python3-vcstool \
      clang-format-10 && \
   rm -rf /var/lib/apt/lists/*

# RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-tf

ENV DEBIAN_FRONTEND=dialog

# depthai + champ
ENV UNDERLAY_WS=/underlay
WORKDIR $UNDERLAY_WS
COPY fennee_config/fennee_config.rosinstall src/fennee_config.rosinstall
COPY fennee_camera/fennee_camera.rosinstall src/fennee_camera.rosinstall
RUN catkin config --extend /opt/ros/noetic && \
   cd src && \
   vcs import < fennee_config.rosinstall --recursive && \
   # vcs import < fennee_camera.rosinstall --recursive && \
   git clone https://github.com/luxonis/depthai-ros.git -b ${ROS_DISTRO} && \
   catkin config --skiplist champ_gazebo
RUN apt-get update && \
   rosdep install --from-paths src --ignore-src -r -y && \
   rm -rf /var/lib/apt/lists/*

# must run build twice for some reason
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
   catkin build || \
   catkin build
# RUN . /opt/ros/${ROS_DISTRO}/setup.sh && catkin build -j1 -l1 



# Fennee
ENV WS=/ws
WORKDIR $WS
COPY entrypoint.sh .
RUN chmod +x entrypoint.sh
ENTRYPOINT [ "/ws/entrypoint.sh" ]
CMD ["bash"]

RUN echo "if [ -f ${WS}/devel/setup.bash ]; then source ${WS}/devel/setup.bash; fi" >> $HOME/.bashrc

# fennee_control
COPY fennee_control/requirements.txt src/
RUN pip3 install -r src/requirements.txt

COPY . $WS/src
WORKDIR $WS
   
RUN catkin config --extend ${UNDERLAY_WS}/devel && \
   rosdep install --from-paths src --ignore-src -r -y && \
   catkin build

# # RUN if [ "$USE_RVIZ" = "1" ] ; then echo "RVIZ ENABLED" && sudo apt install -y ros-${ROS_DISTRO}-rviz ros-${ROS_DISTRO}-rviz-imu-plugin ; else echo "RVIZ NOT ENABLED"; fi


