cmake_minimum_required(VERSION 3.0.2)
project(memmo_teleop)

add_compile_options(-std=c++11)

# Find dependencies
set(dependencies
  geometry_msgs
  roscpp
  sensor_msgs
)
find_package(catkin REQUIRED COMPONENTS ${dependencies})

# Build
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS ${dependencies}
)

add_executable(memmo_teleop_joystick
  src/main_joystick.cpp
  src/memmo_teleop_joystick.cpp
)
add_dependencies(memmo_teleop_joystick ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(memmo_teleop_joystick ${catkin_LIBRARIES})

add_executable(memmo_teleop_keyboard
  src/main_keyboard.cpp
  src/memmo_teleop_keyboard.cpp
)
add_dependencies(memmo_teleop_keyboard ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(memmo_teleop_keyboard ${catkin_LIBRARIES})

# Install
install(TARGETS memmo_teleop_joystick memmo_teleop_keyboard
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Test
# TODO(JaehyunShim): Add test stuff if needed
