# `flock2`

`flock2` can fly a swarm of [DJI Tello](https://store.dji.com/product/tello) drones.
`flock2` is built on top of [ROS2](https://index.ros.org/doc/ros2/),
 [flock_vlam](https://github.com/ptrmu/flock_vlam)
 and [tello_ros](https://github.com/clydemcqueen/tello_ros).

## Installation

### 1. Set up your Linux environment

Set up a Ubuntu 18.04 box or VM. This should include ffmpeg 3.4.4 and OpenCV 3.2.

_TODO OpenCV 3.4_

### 2. Set up your Python environment

Use your favorite Python package manager to set up Python 3.6+ and the following packages:

* numpy 1.15.2
* transformations 2018.9.5

### 3. Set up your ROS environment

[Install ROS2 Crystal Clemmys](https://index.ros.org/doc/ros2/Installation/) with the `ros-crystal-desktop` option.

If you install binaries, be sure to also install the 
[development tools and ROS tools](https://index.ros.org/doc/ros2/Installation/Linux-Development-Setup/)
from the source installation instructions.

Install these additional packages:
~~~
sudo apt install ros-crystal-cv-bridge
~~~

### 4. Install flock2, flock_vlam and tello_ros

Download, compile and install the following packages:
~~~
mkdir -p ~/flock2_ws/src
cd ~/flock2_ws/src
git clone https://github.com/clydemcqueen/flock2.git
git clone https://github.com/ptrmu/flock_vlam.git
git clone https://github.com/clydemcqueen/tello_ros.git
cd ..
source /opt/ros/crystal/setup.bash
colcon build --event-handlers console_direct+
~~~

## Running

### Flying a single drone

`launch_one.py` will allow you to fly a drone using a wired XBox One gamepad.

Turn on the drone, connect to `TELLO-XXXXX` via wifi, and launch ROS2:
~~~
source /opt/ros/crystal/setup.bash
source ~/flock2_ws/install/setup.bash
ros2 launch flock2 launch_one.py
~~~

Gamepad controls:
* menu button to take off
* view button to land
* B to start mission
* A to stop mission
 
### Defining a mission

TODO

### Flying multiple drones

`launch_two.py` provides an example for flying multiple drones.

Key elements of multi-drone missions:

* All drones must be networked together. One way to do this is to connect to each drone's wifi
using a Raspberry Pi 3 or similar device, and forward all UDP packets from the Pi to the host computer.
See [udp_forward](https://github.com/clydemcqueen/udp_forward) for an example using 2 Tello drones.
* Global nodes such as `flock_base` should have exactly 1 instance running.
Per-drone nodes such as `drone_base` should have 1 instance running per drone.
* Each drone has it's own ROS topic namespace. The default namespace for 1 drone is `solo`.
* Each drone must have it's own URDF file with the appropriate coordinate frames.
* The joystick controls one drone at a time. Hit the right bumper to select a different drone.
* All drones participate in the mission.

## Design

### TF Tree

* The world frame is `map`
* Each drone has a base coordinate frame. The default for 1 drone is `base_link`
* Each drone has a camera coordinate frame. The default for 1 drone is `camera_frame`

### Orchestration: `flock_base`

Orchestrates the flight of one or more Tello drones.

#### Subscribed topics

* `~joy` [sensor_msgs/Joy](http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html)

#### Published topics

* `/start_mission` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `/stop_mission` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `~[prefix]/joy` [sensor_msgs/Joy](http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html)

#### Parameters

* `drones` is an array of strings, where each string is a topic prefix

### Drone controller: `drone_base`

Controls a single Tello drone. Akin to `move_base` in the ROS navigation stack.

#### Subscribed topics

* `/start_mission` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `/stop_mission` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `~joy` [sensor_msgs/Joy](http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html)
* `~tello_response` tello_msgs/TelloResponse
* `~flight_data` tello_msgs/FlightData
* `~filtered_odom` [nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)

#### Published topics

* `~cmd_vel` [geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)

#### Published services

* `~tello_command` tello_msgs/TelloCommand

### Odometry estimator: `filter_node`

`flock_vlam` computes a camera pose from ArUco markers placed in the environment,
and `filter_node` uses a Kalman filter to estimate odometry.

Publishes a transform from the map frame to the base frame.

Publishes the estimated path during a mission.

#### Subscribed topics

* `/start_mission` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `/stop_mission` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `~camera_pose` [geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)

#### Published topics

* `~filtered_odom` [nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)
* `~estimated_path` [nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html)
* `/tf` [tf2_msgs/TFMessage](http://docs.ros.org/api/tf2_msgs/html/msg/TFMessage.html)

#### Parameters

* `map_frame` is the world frame. The default is `map`.
* `base_frame` is the coordinate frame of the drone. The default is `base_link`.

### flock_simple_path

_TODO refactor_