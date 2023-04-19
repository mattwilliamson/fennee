# Install ROS Noetic on Ubuntu 20.04

From http://wiki.ros.org/noetic/Installation/Ubuntu

```sh
ssh jetson@nano

# Let's update and upgrade everything first
sudo apt-get update
sudo apt-get upgrade -y

# Expand root partition to use up entire SD card
sudo apt-get install -y gparted
gparted

# Install ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-get install -y curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update
# sudo apt-get install -y ros-noetic-desktop-full
sudo apt-get install -y \
        ros-noetic-desktop \
        python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool \
        python3-vcstool \
        build-essential \
        ros-noetic-ecl-threads \
        ros-noetic-vision-msgs \
        ros-noetic-image-transport \
        ros-noetic-tf \
        ros-noetic-urdf \
        ros-noetic-ecl-threads \
        ros-noetic-robot-state-publisher \
        ros-noetic-robot-localization \
        ros-noetic-controller-manager \
        ros-noetic-joy
        

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo rosdep init
rosdep update

```