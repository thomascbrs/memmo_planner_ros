cmake_minimum_required(VERSION 3.0.2)
project(surface_planner_ros)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  # solo_planner
  footstep_msgs
  # trajectory_msgs
  visualization_msgs
)

catkin_python_setup()

## catkin specific configuration
catkin_package()
