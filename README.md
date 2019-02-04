# `flock2`

`flock2` can fly a swarm of [DJI Tello](https://store.dji.com/product/tello) drones.
`flock2` is built on top of [ROS2](https://index.ros.org/doc/ros2/),
 [flock_vlam](https://github.com/ptrmu/flock_vlam)
 and [tello_ros](https://github.com/clydemcqueen/tello_ros).

## TF Tree

* `map` is the world frame
* `base_link` is the Tello
* `camera_frame` is the front-facing camera on the Tello

## Nodes

### flock_base

Controls a flock of Tello drones.

#### Subscribed topics

* `~joy` [sensor_msgs/Joy](http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html)
* `~[prefix/]tello_response` tello_msgs/TelloResponse
* `~[prefix/]flight_data` tello_msgs/FlightData
* `~[prefix/]filtered_odom` [nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)

#### Published topics

* `~[prefix/]cmd_vel` [geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)
* `/start_mission` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `/stop_mission` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)

#### Published services

* `~[prefix/]tello_command` tello_msgs/TelloCommand

#### Parameters

* `drones` is an array of strings, where each string is a topic prefix

### filter_node

Uses a Kalman filter to estimate pose and velocity.
The estimate is published on the `filtered_odom` topic.

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

Generates a continuous flight path, publishes it, and follows it using a P controller.
Send `start_mission` to run the mission, and `stop_mission` to stop it.

#### Subscribed topics

* `/start_mission` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `/stop_mission` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `/tf` [tf2_msgs/TFMessage](http://docs.ros.org/api/tf2_msgs/html/msg/TFMessage.html)

#### Published topics

* `~planned_path` [nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html)
* `~cmd_vel` [geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)

#### Parameters

* `map_frame` is the world frame. The default is `map`.
* `base_frame` is the coordinate frame of the drone. The default is `base_link`.

## Installation

### 1. Set up your Linux environment

Set up a Ubuntu 18.04 box or VM. This should include ffmpeg 3.4.4 and OpenCV 3.2.

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

Download, compile and install the flock packages:
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

`teleop_launch.py` will allow you to fly a drone using a wired XBox One gamepad.

Turn on the drone, connect to `TELLO-XXXXX` via wi-fi, and launch ROS:
~~~
source /opt/ros/crystal/setup.bash
source ~/flock2_ws/install/setup.bash
ros2 launch flock2 teleop_launch.py
~~~

Key controls:
* menu button to take off
* view button to land
* left bumper and B to start mission
* left bumper and A to stop mission
 
### Support for multiple drones

`flock_base` supports an arbitrary number of drones using the `drones` parameter:
* If `drones` is missing or has length 0, a single drone object is created with an empty prefix.
* If `drones` has length 1, a single drone object is created with the specified prefix.
* If `drones` has length N, N drones are created with the specified prefixes.

For example, if the `drone` parameter is set to `['foo', 'bar']` then 2 drone objects will be created,
and the drones will publish velocity commands on topics `foo/cmd_vel` and `bar/cmd_vel`.

The joystick can control one drone at a time.
Hitting the right bumper will select a different drone to control.

All drones participate in a mission.

You'll need a network configuration that allows you to connect to multiple Tello drones.
You may want to use a port forwarding solution such as 
[udp_forward](https://github.com/clydemcqueen/udp_forward).

You'll need to provide a separate URDF file for each drone with the appropriate coordinate frames.

`flock_launch.py` provides an example for 2 drones.