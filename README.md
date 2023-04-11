# Fennee Quadruped Robot

Hardware based on SpotMicro. 

Software based on Champ.
https://github.com/chvmp/champ

TODO: right now the 200hz loop is too fast for the micro to subscribe to. Need some sort of reducer node.

## Requirements
- Ubuntu 20.04
- ROS 1 Noetic http://wiki.ros.org/noetic/Installation/Ubuntu

## Building

```sh
# Source your ROS installation
source /opt/ros/noetic/setup.bash

# CD to your workspace source, e.g. ~/ros_ws and clone the repo
cd $YOUR_WS/src
git clone https://github.com/mattwilliamson/fennee.git
vcs import < fennee/fennee_ros/.fennee.repos --recursive

# CD to your workspace and build
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make
catkin_make install

# Source your build
source devel/setup.bash
```

## Running


### Simulator

```sh
source devel/setup.bash
roslaunch fennee_config bringup.launch rviz:=true
```

```sh
source devel/setup.bash
roslaunch champ_teleop teleop.launch joy:=true
```

With a PS4/DS4 controller:

| Input | Output |
| --- | --- |
| Left Stick | Walk/Yaw |
| Right Stick | Pitch/Roll |
| L1 (hold) | Holonomic (strafing) |
| R2 | Sit |


### Hardware

```
roslaunch fennee_config bringup.launch hardware_connected:=true

roslaunch champ_teleop teleop.launch joy:=true

rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200

rostopic echo joint_states
```

```
roslaunch fennee_config bringup.launch hardware_connected:=true joy:=true
```


## TODO

- [ ] IMU Publisher
- [ ] Calibration
- [ ] Faster baud rate