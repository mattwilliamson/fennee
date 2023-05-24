# Fennee Quadruped Robot

**F**oundation for **E**xperimental **N**eural **N**etwork **E**ndeavors

---

# Hardware
Check the [Hardware & Parts Details docs](./docs/hardware.md)

# Software

- [Champ](https://github.com/chvmp/champ)
- [Ubuntu 20.04 Jetson Image](https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image) flashed on SD card with [Balena Etcher](https://www.balena.io/etcher)
- [ROS 1 Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
- [Adafruit CircuitPython libraries](https://docs.circuitpython.org/projects/bundle/en/latest/drivers.html)


## Install Fennee Packages

First [Install ROS Noetic](./docs/install_ros.md)

**I'd like to switch to ROS2 when CHAMP finishes porting over**

```sh
# Source your ROS installation
source /opt/ros/noetic/setup.bash

# CD to your workspace source, e.g. ~/ros_ws and clone the repo
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
git clone https://github.com/mattwilliamson/fennee.git
vcs import < fennee/fennee_ros/fennee.rosinstall --recursive

# CD to your workspace and build
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make

# Source your build
source devel/setup.bash
```

## Running

TODO: Need to specify dependencies correctly

```sh

sudo pip3 install -t /opt/ros/noetic/lib/python3/dist-packages adafruit_blinka adafruit-circuitpython-servokit pyyaml
```

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

| Input       | Output               |
| ----------- | -------------------- |
| Left Stick  | Walk/Yaw             |
| Right Stick | Pitch/Roll           |
| L1 (hold)   | Holonomic (strafing) |
| R2          | Sit                  |

### Jetson Nano
```sh
ssh user@jetson
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src

git clone https://github.com/mattwilliamson/fennee.git
cd fennee

sudo docker build -t fennee .
sudo docker run -it --rm --runtime nvidia --network host fennee
sudo docker run -it --rm --runtime nvidia -v $HOME/ros_ws/src/fennee:/fennee_ws/src/fennee --network host fennee

# Source your build
source devel/setup.bash
roslaunch fennee_config bringup.launch hardware_connected:=true
```


### Hardware

```sh
roslaunch fennee_config bringup.launch hardware_connected:=true

roslaunch champ_teleop teleop.launch joy:=true

rostopic echo joint_states
```

```sh
roslaunch fennee_config bringup.launch hardware_connected:=true
```

### Calibration

```
roslaunch fennee_config bringup.launch hardware_connected:=true publish_joint_control:=false
```

```
roslaunch champ_bringup joints_gui.launch
```

```
rostopic echo servo_duty_cycles
```

Put duty cycles into `fennee_control/scripts/servo_interface.py` `pwm_map`.

fennee_config/config/gait/gait.yaml has `com_x_translation` to shift the center of balance from the front to the back. 
For example `-0.01` if the robot is leaning backwards a bit, keeping it from walking.

### Watching logs

GUI

```
rqt_console
```

### Credits
https://github.com/chvmp
https://github.com/michaelkubina/SpotMicroESP32
https://spotmicroai.readthedocs.io/en/latest/
https://www.thingiverse.com/thing:4937631

## TODO

- [ ] Make calibration launch file and parse yaml for servo_controller
- [ ] Update URDF with longer body
- [ ] Package requirements are a mess. pip vs rosdep etc.
- [ ] Charging connector
- [ ] Add clips to battery clips for wires
- [ ] Rear cover hole for voltmeter
- [ ] Oak D Lite Camera mount
- [ ] LED Ring sticking out under lidar?
- [ ] Separate mount for IMU
- [ ] Supports around holes for covers
- [ ] Custom top and bottom cover, speakers, mic, fan
- [x] Lengthen chassis for more electronics room
- [ ] Removable electronicts mount
- [x] Custom hip for metal servo arm
- [ ] Custom mounting board for electronics
- [ ] Custom nose for camera and LED ring
- [x] Lengthen body
- [ ] New cover with lidar mount and fan
- [ ] Front camera
- [ ] IMU Publisher
- [x] Faster baud rate
- [ ] Calibration config file
- [ ] Wiring diagrams
- [ ] 3d print instructions/list
- [ ] Show loading screen while robot is waiting for ROS
- [ ] LED ring
- [ ] LIDAR
- [ ] Relay for servos
- [ ] Some kind of quick release for covers - removing battery is annoying
- [ ] Speakers/mic



---

# Docker

https://hub.docker.com/_/ros/

Building the image
`BUILD_SEQUENTIAL` is for lower memory systems, but takes longer.

```sh
sudo docker build --build-arg BUILD_SEQUENTIAL=1 -t fennee .
```

ROS uses the ~/.ros/ directory for storing logs, and debugging info
```sh
docker run -v "/home/ubuntu/.ros/:/root/.ros/" -it --rm --network=host fennee
```

```sh
sudo docker run --rm -it -e ROS_IP=192.168.50.83 --privileged -v /dev/:/dev/ --network host fennee roslaunch fennee_config bringup.launch hardware_connected:=true
```

RViz/Gazebo
```sh
sudo docker run --rm -it -v /dev/:/dev/ --network host -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --privileged depthai_ros roslaunch depthai_filters example_seg_overlay.launch
```

Camera publisher with overlay for object detection
```sh
sudo docker run --rm -it -e ROS_IP=192.168.50.83 --privileged -v /dev/:/dev/ --network host fennee roslaunch depthai_filters example_det2d_overlay.launch camera_model:=OAK-D-LITE
```

Camera publisher with stereo, RGB and IMU
```sh
sudo docker run --rm -it -e ROS_IP=192.168.50.83 --privileged -v /dev/:/dev/ --network host fennee roslaunch depthai_examples stereo_inertial_node.launch enableRviz:=false
```



View Camera
```sh
ROS_MASTER_URI=http://192.168.50.83:11311
rqt_image_view
```

Record Video to output.avi
```sh
fennee
ROS_MASTER_URI=http://127.0.0.1:11311 rosrun image_view video_recorder image:=/mobilenet_publisher/color/image
vlc output.avi
```