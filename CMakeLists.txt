cmake_minimum_required(VERSION 3.5)
project(flock2)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(flock2
  "msg/FlightData.msg"
  "msg/Flip.msg"
)

install(FILES
  src/detect_aruco.py
  src/flock_base.py
  src/flock_driver.py
  src/latch_pub.py
  src/latch_sub.py
  src/simple.py
  src/tf2_test.py
  src/usb_camera.py
  DESTINATION lib/flock2
)

install(DIRECTORY
  launch
  urdf
  DESTINATION share/flock2
)

ament_package()