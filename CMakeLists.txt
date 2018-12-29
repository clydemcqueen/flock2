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
  set(FastRTPS_INCLUDE_DIR "/opt/ros/crystal/include")
  set(FastRTPS_LIBRARY_RELEASE "/opt/ros/crystal/lib/libfastrtps.so")
  set(tello_msgs_DIR "${PROJECT_SOURCE_DIR}/../../install/tello_msgs/share/tello_msgs/cmake")
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
find_package(cv_bridge REQUIRED)
find_package(Eigen3)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tello_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Local includes
include_directories(
  include
  ${cv_bridge_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${tf2_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
)

#=============
# Detect markers node
#=============

add_executable(detect_markers src/detect_markers.cpp src/eigen_util.cpp)

ament_target_dependencies(
  detect_markers
  cv_bridge
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2_msgs
  visualization_msgs
)

#=============
# USB camera
#=============

add_executable(usb_camera src/usb_camera.cpp)

ament_target_dependencies(
  usb_camera
  cv_bridge
  rclcpp
  sensor_msgs
)

#=============
# Install
#=============

# Install targets
install(
  TARGETS
  detect_markers
  usb_camera
  DESTINATION lib/flock2
)

# Install all Python programs
install(
  PROGRAMS  # PROGRAMS sets execute bits, FILES clears them
  src/action_mgr.py
  src/detect_aruco.py
  src/filter.py
  src/flock_base.py
  src/flock_simple_path.py
  src/kf.py
  src/smooth_path_4poly_2min.py
  src/usb_camera.py
  src/util.py
  src/video_node.py
  DESTINATION lib/flock2
)

# Install cfg, launch and urdf directories
install(
  DIRECTORY
  cfg
  launch
  urdf
  DESTINATION share/flock2
)

ament_package()