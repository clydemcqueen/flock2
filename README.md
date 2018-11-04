# Flock2

Flock2 is a ROS2 driver for [DJI Tello](https://store.dji.com/product/tello) drones.

## Nodes

### flock_driver

Provides a ROS2 wrapper around TelloPy.

Uses ArUco markers to generate odometry.
Marker positions are published on the `/tf` and `~rviz_markers` topics.

Images can be published on the `~image_marked` topic (CPU intensive), or viewed in a `cv2.imshow` window (fast).

#### Subscribed topics

* `~cmd_vel` [geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)
* `~takeoff` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `~land` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `~flip` flock_msgs/Flip

#### Published topics

* `~flight_data` flock_msgs/FlightData
* `~odom` [nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)
* `~rviz_markers` [visualization_msgs/MarkerArray](http://docs.ros.org/api/visualization_msgs/html/msg/MarkerArray.html)
* `~image_marked` [sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)
* `/tf` [tf2_msgs/TFMessage](http://docs.ros.org/api/tf2_msgs/html/msg/TFMessage.html)

### flock_base

Provides teleop control.

Allows you to start and stop a mission.
Most joystick controls are disabled while running a mission.

#### Subscribed topics

* `~joy` [sensor_msgs/Joy](http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html)

#### Published topics

* `~cmd_vel` [geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)
* `~takeoff` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `~land` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `~flip` flock_msgs/Flip
* `~start_mission` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `~stop_mission` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)

### filter

Uses a Kalman filter to estimate pose and velocity.
The estimate is published on the `filtered_odom` topic.

Publishes a transform from `odom` to `base_link`.

Publishes the estimated path during a mission.

#### Subscribed topics

* `~start_mission` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `~stop_mission` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `~odom` [nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)

#### Published topics

* `~filtered_odom` [nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)
* `~estimated_path` [nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html)
* `/tf` [tf2_msgs/TFMessage](http://docs.ros.org/api/tf2_msgs/html/msg/TFMessage.html)

### flock_simple_path

Generates a continuous flight path, publishes it, and follows it using a P controller.
Send `start_mission` to run the mission, and `stop_mission` to stop it.

#### Subscribed topics

* `~start_mission` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `~stop_mission` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `/tf` [tf2_msgs/TFMessage](http://docs.ros.org/api/tf2_msgs/html/msg/TFMessage.html)

#### Published topics

* `~planned_path` [nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html)
* `~cmd_vel` [geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)

## Installation

### 1. Set up your Linux environment

Set up a Ubuntu 18.04 box or VM. This should include ffmpeg 3.4.4 and OpenCV 3.2.

### 2. Set up your Python environment

Use your favorite Python package manager to set up Python 3.6 and the following packages:

* numpy 1.15.2
* av 0.5.3 or 6.0.0
* opencv-python 3.4.3.18
* opencv-contrib-python 3.4.3.18
* transformations 2018.9.5
* tellopy 0.5.0 or 0.6.0

### 3. Set up your ROS environment

[Install ROS2 Bouncy Bolson](https://github.com/ros2/ros2/wiki/Installation) with the `ros-bouncy-desktop` option.

If you install binaries, be sure to also install the 
[development tools and ROS tools](https://github.com/ros2/ros2/wiki/Linux-Development-Setup#install-development-tools-and-ros-tools)
from the source installation instructions.

Install these additional packages:
~~~
sudo apt install ros-bouncy-joystick-drivers
~~~

### 4. Install Flock2

Download, compile and install flock2:
~~~
mkdir -p ~/flock2_ws/src
cd ~/flock2_ws/src
git clone https://github.com/clydemcqueen/flock2.git
cd ..
source /opt/ros/bouncy/setup.bash
colcon build --symlink-install --event-handlers console_direct+
~~~

## Running

### Test the environment

This script will connect to the drone and display a video feed in an OpenCV window.
It will also look for ArUco 6x6 markers and highlight them in green.
It does not require ROS.

Turn on the drone, connect to `TELLO-XXXXX` via wi-fi, and run this script:
~~~
python ~/flock2_ws/src/flock2/scripts/environment_test.py
~~~

### Teleop

This ROS launch file will allow you to fly the drone using a wired XBox One gamepad.

Turn on the drone, connect to `TELLO-XXXXX` via wi-fi, and launch ROS:
~~~
source /opt/ros/bouncy/setup.bash
source ~/flock2_ws/install/setup.bash
ros2 launch flock2 teleop_launch.py
~~~

* TODO add gamepad controls
* TODO add instructions for running rviz2

## Future Work

The primary goal is to autonomously fly multiple Tello drones in simple patterns.

The flock_driver node may be split off into its own repo.

The authors intend to track the ROS2 releases and migrate the code for each release.
The next ROS2 release is [Crystal](https://discourse.ros.org/t/timeline-for-crystal/6676).