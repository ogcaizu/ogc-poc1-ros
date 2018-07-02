#!/bin/bash

envsubst < /opt/ros_ws/src/rosbridge/config/mqtt.yaml.template > /opt/ros_ws/src/rosbridge/config/mqtt.yaml
source /opt/ros/kinetic/setup.bash
catkin_make
source /opt/ros_ws/devel/setup.bash
roslaunch rosbridge rosbridge.launch
