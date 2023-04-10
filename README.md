# Fennee Quadruped Robot

Hardware based on SpotMicro. 

Software based on Champ.
https://github.com/chvmp/champ

## Building

```sh
cd $YOUR_WS/src
vcs import < fennee/fennee_ros/.fennee.repos --recursive

cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make
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
