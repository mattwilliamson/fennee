cmake_minimum_required(VERSION 2.8.3)
project(fennee_camera)

find_package(catkin REQUIRED COMPONENTS 
  rospy
  sensor_msgs
)

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

file(GLOB python_scripts scripts/*.py)
catkin_install_python(
  PROGRAMS ${python_scripts}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
