#!/usr/bin/python
# -*- coding: utf-8 -*-
import os

import rospy

from rosbridge.ros2mqtt import Ros2MQTT
from rosbridge.external_camera_state_converter import convert_ros_to_mqtt, message_type

from rosbridge.logging import getLogger
logger = getLogger(__name__)

NODE_NAME = os.path.basename(__file__)


def main():
    try:
        rospy.init_node(NODE_NAME)
        Ros2MQTT(NODE_NAME, rospy.get_param('~'), convert_ros_to_mqtt, message_type).connect().start()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
