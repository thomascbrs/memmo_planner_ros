cmake_minimum_required(VERSION 3.0.2)
project(footstep_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    message_generation
    std_msgs
)

add_message_files(
  FILES
    FootSurfaces.msg
    FootPosition.msg
    GaitStatusOnNewPhase.msg
    SetSurfaces.msg
    ConvexSurface.msg
)

add_service_files(DIRECTORY srv
  FILES
  Clearmap.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    std_msgs
)
