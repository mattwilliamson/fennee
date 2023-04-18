# Fennee Quadruped Robot

# Hardware

## Parts List

- [SpotMicro](https://spotmicroai.readthedocs.io/en/latest/)
- [NVIDIA Jetson Nano](https://www.amazon.com/NVIDIA-Jetson-Nano-Developer-945-13450-0000-100/dp/B084DSDDLT/ref=sr_1_3?crid=K3EEP2QYGRKR&keywords=NVIDIA+Jetson+Nano&qid=1681300762&s=electronics&sprefix=nvidia+jetson+nano%252Celectronics%252C161&sr=1-3&ufe=app_do%253Aamzn1.fos.f5122f16-c3e8-4386-bf32-63e904010ad0&_encoding=UTF8&tag=aimatt02-20&linkCode=ur2&linkId=7d3968aff925cd432b2f563af53e1284&camp=1789&creative=9325) for high-level control
- [SD Card](https://www.amazon.com/dp/B07FCMBLV6?psc=1&ref=ppx_yo2ov_dt_b_product_details&_encoding=UTF8&tag=aimatt02-20&linkCode=ur2&linkId=d9ee3513e15091796e6be4dad591e27c&camp=1789&creative=9325)
- [Freenove ESP32 Cam](https://www.amazon.com/Freenove-ESP32-WROVER-Bluetooth-Compatible-Tutorials/dp/B09BC5CNHM?&_encoding=UTF8&tag=aimatt02-20&linkCode=ur2&linkId=ae7b14c22685cad2407abdaab565e8d2&camp=1789&creative=9325) for low-level interfacing. Can probably use just about any [Arduino](https://www.amazon.com/gp/search?ie=UTF8&tag=aimatt02-20&linkCode=ur2&linkId=e289d4057da769f17dc1e7b3221a4050&camp=1789&creative=9325&index=electronics&keywords=Arduino) or other [ESP32](https://www.amazon.com/gp/search?ie=UTF8&tag=aimatt02-20&linkCode=ur2&linkId=2d1d81042a386509499b0f72ca5084ee&camp=1789&creative=9325&index=electronics&keywords=ESP32) or other MCUs that are supported but PlatformIO
- PCA9685 PWM controller for controlling servos: https://www.adafruit.com/product/815
- 12x [DS3218 Digital Servo Motor](https://www.amazon.com/gp/product/B07WYQ9P3F/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1&_encoding=UTF8&tag=aimatt02-20&linkCode=ur2&linkId=aac388a45ead7e4239ba14d217c2a3e6&camp=1789&creative=9325)
- [LiPO Battery](https://www.amazon.com/dp/B086D71TZC?psc=1&ref=ppx_yo2ov_dt_b_product_details&_encoding=UTF8&tag=aimatt02-20&linkCode=ur2&linkId=2e59462443f181c82c3a7a22fd7b1b8c&camp=1789&creative=9325) this one is 5200mAh 7.4v, but you have flexibility here
- [Buck Converter](https://www.amazon.com/dp/B07Y7YB14L?psc=1&ref=ppx_yo2ov_dt_b_product_details&_encoding=UTF8&tag=aimatt02-20&linkCode=ur2&linkId=9f2c21c8d2dcc244b0d85624aa2fc704&camp=1789&creative=9325)
- 2x [Bearings](https://www.amazon.com/dp/B07JHKKGKT?psc=1&ref=ppx_yo2ov_dt_b_product_details&_encoding=UTF8&tag=aimatt02-20&linkCode=ur2&linkId=92a49fcb0e2082979097c1ca9db1d71c&camp=1789&creative=9325)
- [Lots](https://www.amazon.com/dp/B015A30R1I?psc=1&ref=ppx_yo2ov_dt_b_product_details&_encoding=UTF8&tag=aimatt02-20&linkCode=ur2&linkId=4594994ca7d27816592dd19072e0148a&camp=1789&creative=9325) of [m3 screws and nuts (some m2 as well)](https://www.amazon.com/dp/B08JCKH31Q?psc=1&ref=ppx_yo2ov_dt_b_product_details&_encoding=UTF8&tag=aimatt02-20&linkCode=ur2&linkId=85b1b003df00aa2316b49650ea373ecb&camp=1789&creative=9325)
- [PETG Filament](https://www.amazon.com/dp/B08JCKH31Q?psc=1&ref=ppx_yo2ov_dt_b_product_details&_encoding=UTF8&tag=aimatt02-20&linkCode=ur2&linkId=85b1b003df00aa2316b49650ea373ecb&camp=1789&creative=9325)

*Disclaimer: I did add Amazon affiliate links to these*

### Optional

- [Waveshare AC8265](https://www.amazon.com/dp/B07SGDRG34?psc=1&ref=ppx_yo2ov_dt_b_product_details&_encoding=UTF8&tag=aimatt02-20&linkCode=ur2&linkId=5afefd950c9ea730d13b46f8e915f168&camp=1789&creative=9325) WiFi and Bluetooth for Jetson Nano
- [IPEX Wifi Antennas](https://www.amazon.com/Antenna-2-4GHz-Internal-Laptop-Wireless/dp/B08XN6WMXJ/ref=sr_1_3?crid=141NMPSVVCSRX&keywords=IPEX%252Bantenna%252B5ghz&qid=1681301616&s=electronics&sprefix=ipex%252Bantenna%252B5ghz%252Celectronics%252C103&sr=1-3&th=1&_encoding=UTF8&tag=aimatt02-20&linkCode=ur2&linkId=b760f9104679d63096c3b43a57f38a0c&camp=1789&creative=9325)
- [Volt Display](https://www.amazon.com/dp/B0761MG9NS?psc=1&ref=ppx_yo2ov_dt_b_product_details&_encoding=UTF8&tag=aimatt02-20&linkCode=ur2&linkId=fa09f95b7c494d9456b38b2fabb7f38a&camp=1789&creative=9325)
- [Creality Ender 3 S1 Pro 3d printer](https://www.amazon.com/3D-High-Temperature-Removable-Touchscreen-Languages/dp/B09TKCY9HY/ref=sr_1_3?camp=1789&creative=9325&keywords=Creality+Ender+3+S1+Pro&linkCode=ur2&linkId=6745c52615b49225b7af7e0865687db1&qid=1681302453&sr=8-3&ufe=app_do%253Aamzn1.fos.c3015c4a-46bb-44b9-81a4-dc28e6d374b3&_encoding=UTF8&tag=aimatt02-20&linkCode=ur2&linkId=8694ab1cb75ba1d75b8df1cbfc0ca5be&camp=1789&creative=9325)

## 3d Printed

- [SpotMicro prints without supports](https://www.thingiverse.com/thing:4559827)
- [Reinforced shoulders](https://www.thingiverse.com/thing:4937631)
- [Assembly Guide](https://github.com/mattwilliamson/SpotMicroESP32/tree/master/assembly) *Needs some updates*

# Software

- [Champ](https://github.com/chvmp/champ)
- [PlatformIO](https://platformio.org/)
- Ubuntu 20.04 _Ubuntu 18.04 (ROS Melodic) should work as well_
- [ROS 1 Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
- [Jetson Docker Containers](https://github.com/dusty-nv/jetson-containers)

## Building

```sh
# Source your ROS installation
source /opt/ros/noetic/setup.bash

# CD to your workspace source, e.g. ~/ros_ws and clone the repo
cd $YOUR_WS/src
git clone https://github.com/mattwilliamson/fennee.git
vcs import < fennee/fennee_ros/fennee.rosinstall --recursive

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

rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=500000

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

- [ ] Front camera
- [ ] IMU Publisher
- [x] Faster baud rate
- [ ] Option to connecto to TCP rosserial
- [ ] Calibration config file
- [ ] Wiring diagrams
- [ ] 3d print instructions/list
- [ ] Show loading screen while robot is waiting for ROS
- [ ] LED ring
- [ ] LIDAR
