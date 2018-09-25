# -*- coding: utf-8 -*-
import os
import re

import pytz

import rospy

from rosbridge.base import MQTTBase
from rosbridge.logging import getLogger
logger = getLogger(__name__)

CMD_RE = re.compile(r'^(?P<device_id>.+)@(?P<command>[^|]+)\|(?P<value>.+)$')
RESULT_FMT = '{device_id}@{command}|{result}'


class MQTT2Ros(MQTTBase):
    def __init__(self, node_name, params, converter, message_type):
        self.node_name = node_name
        self._params = params
        self._converter = converter
        self._message_type = message_type
        self._tz = pytz.timezone(self._params["timezone"])
        super(MQTT2Ros, self).__init__()

    def start(self):
        logger.infof('start MQTT2Ros on node={}', self.node_name)
        self.__ros_pub = rospy.Publisher(self._params['topics']['ros'], self._message_type, queue_size=10)
        rospy.spin()
        logger.infof('stop MQTT2Ros on node={}', self.node_name)

    def _on_connect(self, client, userdata, flags, response_code):
        logger.infof('connected to mqtt broker, status={}', response_code)
        client.subscribe(os.path.join(self._params['topics']['mqtt'], 'cmd'))

    def _on_message(self, client, userdata, msg):
        payload = str(msg.payload)
        logger.infof('received message from mqtt: {}', payload)

        matcher = CMD_RE.match(payload)

        if matcher:
            device_id = matcher.group('device_id')
            command = matcher.group('command')
            value = matcher.group('value')

            cmd_data = dict(zip(*[iter(value.split('|'))]*2))
            result = self._converter(self._tz, cmd_data, self.__ros_pub)

            result_topic = os.path.join(self._params['topics']['mqtt'], 'cmdexe')
            client.publish(result_topic, RESULT_FMT.format(device_id=device_id, command=command, result=result))
        else:
            logger.errorf('invalid format, payload={}', payload)
