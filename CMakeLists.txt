cmake_minimum_required(VERSION 3.5)
project(flock2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Emulate colcon in CLion
if($ENV{CLION_IDE})
  message("Running inside CLion")
  find_package(fastrtps_cmake_module REQUIRED)
  set(FastRTPS_INCLUDE_DIR "/opt/ros/bouncy/include")
  set(FastRTPS_LIBRARY_RELEASE "/opt/ros/bouncy/lib/libfastrtps.so")
  set(tello_msgs_DIR "${PROJECT_SOURCE_DIR}/../../install/tello_msgs/share/tello_msgs/cmake")
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
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tello_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Install all Python programs
install(
  PROGRAMS  # PROGRAMS sets execute bits, FILES clears them
  src/detect_aruco.py
  src/filter.py
  src/flock_base.py
  src/flock_simple_path.py
  src/kf.py
  src/latch_pub.py
  src/latch_sub.py
  src/simple.py
  src/smooth_path_4poly_2min.py
  src/tf2_test.py
  src/usb_camera.py
  src/util.py
  src/video_node.py
  DESTINATION lib/flock2
)

# Install launch and urdf directories
install(
  DIRECTORY
  launch
  urdf
  DESTINATION share/flock2
)

ament_package()