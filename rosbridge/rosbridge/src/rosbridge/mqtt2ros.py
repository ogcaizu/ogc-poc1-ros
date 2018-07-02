# -*- coding: utf-8 -*-
import os
import re
import ssl

import rospy

import paho.mqtt.client as mqtt

from rosbridge.logging import getLogger
logger = getLogger(__name__)

CMD_RE = re.compile(r'^(?P<device_id>.+)@(?P<command>[^|]+)\|(?P<value>.+)$')
RESULT_FMT = '{device_id}@{command}|{result}'


class MQTT2Ros(object):
    def __init__(self, node_name, params, converter, message_type):
        self.node_name = node_name
        self._params = params
        self._converter = converter
        self._message_type = message_type

        self.__client = None

    def connect(self):
        logger.infof('try to Connect mqtt broker, host={}', self._params['mqtt']['host'])
        self.__client = mqtt.Client(protocol=mqtt.MQTTv311)
        self.__client.on_connect = self._on_connect
        self.__client.on_message = self._on_message

        if 'cafile' in self._params['mqtt']:
            cafile = self._params['mqtt']['cafile'].strip()
            if len(cafile) > 0 and os.path.isfile(cafile):
                self.__client.tls_set(cafile, tls_version=ssl.PROTOCOL_TLSv1_2)

        if 'username' in self._params['mqtt'] and 'password' in self._params['mqtt']:
            username = self._params['mqtt']['username'].strip()
            password = self._params['mqtt']['password'].strip()
            if len(username) > 0 and len(password) > 0:
                self.__client.username_pw_set(username, password)

        self.__client.connect(self._params['mqtt']['host'], port=self._params['mqtt']['port'], keepalive=60)
        self.__client.loop_start()
        return self

    def start(self):
        logger.infof('start RosRequest on node={}', self.node_name)
        rospy.on_shutdown(self._on_shutdown)
        self.__ros_pub = rospy.Publisher(self._params['topics']['ros'], self._message_type, queue_size=10)
        rospy.spin()
        logger.infof('stop RosRequest on node={}', self.node_name)

    def _on_connect(self, client, userdata, flags, response_code):
        logger.infof('connected to mqtt broker, status={}', response_code)
        client.subscribe(os.path.join(self._params['topics']['mqtt'], 'cmd'))

    def _on_shutdown(self):
        if self.__client:
            self.__client.loop_stop()
            self.__client.disconnect()

    def _on_message(self, client, userdata, msg):
        payload = str(msg.payload)
        logger.infof('received message from mqtt: {}', payload)

        matcher = CMD_RE.match(payload)

        if matcher:
            device_id = matcher.group('device_id')
            command = matcher.group('command')
            value = matcher.group('value')

            cmd_data = dict(zip(*[iter(value.split('|'))]*2))
            result = self._converter(cmd_data, self.__ros_pub)

            result_topic = os.path.join(self._params['topics']['mqtt'], 'cmdexe')
            client.publish(result_topic, RESULT_FMT.format(device_id=device_id, command=command, result=result))
        else:
            logger.errorf('invalid format, payload={}', payload)
