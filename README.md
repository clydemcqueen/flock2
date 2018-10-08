# Flock2

Flock2 is a ROS2 driver for [DJI Tello](https://store.dji.com/product/tello) drones.

## Nodes

### flock_driver

Provides a ROS wrapper around TelloPy.

#### Subscribed topics

* `~cmd_vel` [geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)
* `~takeoff` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `~land` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `~flip` flock_msgs/Flip

### flock_base

Provides teleop and (eventually) autonomous control.

#### Subscribed topics

* `~joy` [sensor_msgs/Joy](http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html)

#### Published topics

* `~cmd_vel` [geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)
* `~takeoff` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `~land` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `~flip` flock_msgs/Flip

## Installation

### 1. Set up your Linux environment

Set up a Ubuntu 18.04 box or VM. This should include ffmpeg 3.4.4 and OpenCV 3.2.

* TODO test on Windows 10
* TODO test on Mac OS X

### 2. Set up your Python environment

Use your favorite Python package manager to set up Python 3.6 and the following packages:

* numpy 1.15.2
* av 0.5.3
* opencv-python 3.4.3.18
* opencv-contrib-python 3.4.3.18
* transformations 2018.9.5
* tellopy 0.5.0

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

* TODO add instructions for running rviz2
