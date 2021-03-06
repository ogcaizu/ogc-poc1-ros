# -*- coding: utf-8 -*-
import os
import datetime
from threading import Lock

import pytz

import rospy

from rosbridge.base import MQTTBase
from rosbridge.logging import getLogger
logger = getLogger(__name__)


class Ros2MQTT(MQTTBase):
    def __init__(self, node_name, params, converter, message_type):
        self.node_name = node_name
        self._params = params
        self._converter = converter
        self._message_type = message_type
        self._tz = pytz.timezone(self._params["timezone"])
        try:
            self._send_interval_ms = self._params.get('thresholds', {}).get('send_interval_millisec', 0)
        except (TypeError, ValueError):
            self._send_interval_ms = 0
        self._prev_ms = datetime.datetime.now(self._tz)
        self._lock = Lock()
        logger.infof('send_interval_millisec={}', self._send_interval_ms)
        super(Ros2MQTT, self).__init__()

    def start(self):
        logger.infof('start RosRequest on node={}', self.node_name)
        self.__ros_sub = rospy.Subscriber(self._params['topics']['ros'], self._message_type, self._on_receive, queue_size=10)
        rospy.spin()
        logger.infof('stop RosRequest on node={}', self.node_name)

    def _on_receive(self, msg):
        logger.infof('received message from ros : {}', str(msg).replace('\n', ' '))
        now = datetime.datetime.now(self._tz)
        if now >= self._prev_ms + datetime.timedelta(milliseconds=self._send_interval_ms) and self._lock.acquire(False):
            self._prev_ms = now

            result = self._converter(self._tz, msg)
            if result:
                attr_topic = os.path.join(self._params['topics']['mqtt'], 'attrs')
                self._client.publish(attr_topic, result)
                logger.infof('success: {}', result)

            self._lock.release()
