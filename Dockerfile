# FROM fennee_base
ARG ROS_DISTRO=noetic
FROM ros:${ROS_DISTRO}-ros-base
# FROM dustynv/ros:${ROS_DISTRO}-pytorch-l4t-r32.7.1
ARG USE_RVIZ
ARG BUILD_SEQUENTIAL=1
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends \
   software-properties-common \
   git \
   libusb-1.0-0-dev \
   wget \
   zsh \
   python3-catkin-tools \
   python3-pip \
   libpython3-dev \
   python3-rosdep \
   python3-rosinstall-generator \
   python3-vcstool \
   build-essential

# RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-tf

ENV DEBIAN_FRONTEND=dialog
RUN sh -c "$(wget https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)"

ENV WS=/ws
WORKDIR $WS
RUN mkdir src
COPY entrypoint.sh .
RUN chmod +x entrypoint.sh
ENTRYPOINT [ "/ws/entrypoint.sh" ]
CMD ["zsh"]

RUN pip3 install adafruit_blinka adafruit-circuitpython-servokit pyyaml Jetson.GPIO

COPY ./fennee/fennee_ros/fennee.rosinstall src/
RUN cd src && vcs import < fennee.rosinstall --recursive
RUN rosdep install --from-paths src --ignore-src -r -y

# Build multiple times to cache layers
# RUN if [ "$BUILD_SEQUENTIAL" = "1" ] ; then cd $WS/ && . /opt/ros/noetic/setup.sh && catkin build -j1 -l1; else cd .$WS/ && . /opt/ros/noetic/setup.sh && catkin build; fi 
ENV CATKIN_FLAGS "-DCATKIN_BLACKLIST_PACKAGES='champ_gazebo' -j1 -l1"
RUN mkdir /ws/devel/ && touch /ws/devel/setup.bash
RUN . /opt/ros/noetic/setup.sh && catkin_make $CATKIN_FLAGS

COPY depthai ./src/
RUN rosdep install --from-paths src --ignore-src -r -y
RUN . /opt/ros/noetic/setup.sh && catkin_make $CATKIN_FLAGS

COPY fennee ./src/
RUN rosdep install --from-paths src --ignore-src -r -y
RUN . /opt/ros/noetic/setup.sh && catkin_make $CATKIN_FLAGS


RUN  . devel/setup.sh && rosdep install --from-paths src --ignore-src -y && catkin_make $CATKIN_FLAGS

# RUN cd $WS/ && . /opt/ros/noetic/setup.sh && . devel/setup.sh && catkin build $CATKIN_FLAGS
RUN if [ "$USE_RVIZ" = "1" ] ; then echo "RVIZ ENABLED" && sudo apt install -y ros-noetic-rviz ros-noetic-rviz-imu-plugin ; else echo "RVIZ NOT ENABLED"; fi
RUN echo "if [ -f ${WS}/devel/setup.zsh ]; then source ${WS}/devel/setup.zsh; fi" >> $HOME/.zshrc
RUN echo "if [ -f ${WS}/devel/setup.bash ]; then source ${WS}/devel/setup.bash; fi" >> $HOME/.bashrc



# # ------


