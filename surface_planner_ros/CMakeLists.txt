cmake_minimum_required(VERSION 3.0.2)
project(surface_planner_ros)

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
  scripts/surface_planner_ros/main.py
  scripts/surface_planner_ros/step_manager_interface.py
  scripts/surface_planner_ros/surface_planner_interface.py
  scripts/surface_planner_ros/surface_planner_node.py
  scripts/surface_planner_ros/elevation_map_interface.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
