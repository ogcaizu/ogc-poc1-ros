# -*- coding: utf-8 -*-
import os
import ssl

import rospy

import paho.mqtt.client as mqtt

from rosbridge.logging import getLogger
logger = getLogger(__name__)


class MQTTBase(object):
    def __init__(self):
        self._client = None

    def connect(self):
        logger.infof('try to Connect mqtt broker, host={}', self._params['mqtt']['host'])
        self._client = mqtt.Client(protocol=mqtt.MQTTv311)
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message

        if 'cafile' in self._params['mqtt']:
            cafile = self._params['mqtt']['cafile'].strip()
            if len(cafile) > 0 and os.path.isfile(cafile):
                self._client.tls_set(cafile, tls_version=ssl.PROTOCOL_TLSv1_2)

        if 'username' in self._params['mqtt'] and 'password' in self._params['mqtt']:
            username = self._params['mqtt']['username'].strip()
            password = self._params['mqtt']['password'].strip()
            if len(username) > 0 and len(password) > 0:
                self._client.username_pw_set(username, password)

        self._client.connect(self._params['mqtt']['host'], port=self._params['mqtt']['port'], keepalive=60)
        self._client.loop_start()

        rospy.on_shutdown(self._on_shutdown)
        return self

    def _on_shutdown(self):
        if self._client:
            self._client.loop_stop()
            self._client.disconnect()

    def _on_connect(self, client, userdata, flags, response_code):
        logger.infof('connected to mqtt broker, status={}', response_code)

    def _on_message(self, client, userdata, msg):
        pass
