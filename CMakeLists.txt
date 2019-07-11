cmake_minimum_required(VERSION 3.5)
project(flock2)

#=============
# Setup
#=============

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Emulate colcon in CLion
if($ENV{CLION_IDE})
  message("Running inside CLion")
  find_package(fastrtps_cmake_module REQUIRED)
  set(FastRTPS_INCLUDE_DIR "/opt/ros/dashing/include")
  set(FastRTPS_LIBRARY_RELEASE "/opt/ros/dashing/lib/libfastrtps.so")
  set(tello_msgs_DIR "${PROJECT_SOURCE_DIR}/../../install/tello_msgs/share/tello_msgs/cmake")
  set(ros2_shared_DIR "${PROJECT_SOURCE_DIR}/../../install/ros2_shared/share/ros2_shared/cmake")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRUN_INSIDE_CLION")
endif()

# Debugging: set _dump_all_variables to true
set(_dump_all_variables false)
if(_dump_all_variables)
  get_cmake_property(_variable_names VARIABLES)
  list(SORT _variable_names)
  foreach(_variable_name ${_variable_names})
    message(STATUS "${_variable_name}=${${_variable_name}}")
  endforeach()
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ros2_shared REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tello_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Local includes
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${ros2_shared_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${tello_msgs_INCLUDE_DIRS}
  ${tf2_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
)

#=============
# Flock base node
#=============

add_executable(
  flock_base
  src/flock_base.cpp
)

ament_target_dependencies(
  flock_base
  rclcpp
  ros2_shared
  sensor_msgs
  std_msgs
)

#=============
# Drone base node
#=============

add_executable(
  drone_base
  src/drone_base.cpp
  src/action_mgr.cpp
  src/flight_controller_basic.cpp
  src/flight_controller_simple.cpp
)

ament_target_dependencies(
  drone_base
  geometry_msgs
  nav_msgs
  rclcpp
  ros2_shared
  sensor_msgs
  std_msgs
  tello_msgs
)

#=============
# Planner node
#=============

add_executable(
  planner_node
  src/planner_node.cpp
  src/simple_planner.cpp
)

ament_target_dependencies(
  planner_node
  nav_msgs
  rclcpp
  ros2_shared
  std_msgs
)

#=============
# Install
#=============

# Install targets
install(
  TARGETS
  flock_base
  drone_base
  planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install all Python programs
install(
  PROGRAMS  # PROGRAMS sets execute bits, FILES clears them
  src/flock_simple_path.py
  src/smooth_path_4poly_2min.py
  src/util.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install various directories
install(
  DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()