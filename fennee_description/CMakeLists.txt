
cmake_minimum_required(VERSION 2.8.3)
project(fennee_description)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(DIRECTORY meshes launch urdf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
