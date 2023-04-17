# https://github.com/dusty-nv/jetson-containers
# https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-tensorflow

# Check your version of L4T (r32.7.3 in this case)
# head -n 1 /etc/nv_tegra_release
# # R32 (release), REVISION: 7.3, GCID: 31982016, BOARD: t210ref, EABI: aarch64, DATE: Tue Nov 22 17:30:08 UTC 2022


# scripts/docker_run.sh -c nvcr.io/nvidia/l4t-pytorch:r32.5.0-pth1.7-py3

# https://github.com/dusty-nv/ros_deep_learning

# on Jetson: `cd /; sudo git clone 

FROM dustynv/ros:noetic-pytorch-l4t-r32.7.1
ADD . /ros_deep_learning/src/
WORKDIR /ros_deep_learning
RUN . /opt/ros/noetic/setup.sh  && catkin_make install