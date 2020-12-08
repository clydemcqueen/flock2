# To build:
# docker build --pull --no-cache --tag flock2:eloquent .

# To run:
# docker run -it flock2:eloquent bash

# I'm using this for smoke tests
# To run flock2 in a docker container you will need to set up ports, x-windows, etc.

FROM osrf/ros:eloquent-desktop

RUN apt-get update
RUN apt-get upgrade -y

RUN apt-get install -y libasio-dev
RUN apt-get install -y python3-pip
RUN yes | pip3 install 'transformations==2018.9.5'

WORKDIR /work/flock2_ws/src

RUN git clone https://github.com/clydemcqueen/flock2.git -b eloquent
RUN git clone https://github.com/clydemcqueen/tello_ros.git -b eloquent
RUN git clone https://github.com/ptrmu/ros2_shared.git -b eloquent
RUN git clone https://github.com/ptrmu/fiducial_vlam.git -b eloquent

WORKDIR /work/flock2_ws

RUN rosdep install -y --from-paths . --ignore-src

RUN /bin/bash -c "source /opt/ros/eloquent/setup.bash && colcon build"
