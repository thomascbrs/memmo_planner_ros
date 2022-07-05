cmake_minimum_required(VERSION 3.0.2)
project(plane_publisher)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  footstep_msgs
  visualization_msgs
)

catkin_python_setup()

## catkin specific configuration
catkin_package()

catkin_install_python(PROGRAMS
  scripts/plane_publisher/main.py
  scripts/plane_publisher/plane_publisher_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
