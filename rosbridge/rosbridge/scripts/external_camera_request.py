#!/usr/bin/python
# -*- coding: utf-8 -*-
import os

import rospy

from rosbridge.mqtt2ros import MQTT2Ros
from rosbridge.external_camera_request_converter import convert_mqtt_to_ros, message_type

from rosbridge.logging import getLogger
logger = getLogger(__name__)

NODE_NAME = os.path.basename(__file__)


def main():
    try:
        rospy.init_node(NODE_NAME)
        MQTT2Ros(NODE_NAME, rospy.get_param('~'), convert_mqtt_to_ros, message_type).connect().start()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
