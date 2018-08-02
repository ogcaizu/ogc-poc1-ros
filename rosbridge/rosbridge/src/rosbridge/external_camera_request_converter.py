# -*- coding: utf-8 -*-
import datetime

import pytz

from external_camera.msg import c_req as message_type

from rosbridge.logging import getLogger
logger = getLogger(__name__)


def convert_mqtt_to_ros(cmd_data, ros_publisher):
    logger.infof('convert mqtt to ros, cmd_data={}', cmd_data)

    try:
        c_cmd = cmd_data['c_cmd']
        if c_cmd not in ('Monitor', 'Standby', 'Clear'):
            return 'failure: unknown c_cmd, c_cmd="{}"'.format(c_cmd)

        msg = message_type()
        msg.time = datetime.datetime.now(pytz.timezone('Asia/Tokyo')).strftime('%Y-%m-%d %H:%M:%S')
        msg.c_cmd = c_cmd

        ros_publisher.publish(msg)
        result = 'result,success/time,{time}/c_cmd,{c_cmd}'.format(
            time=msg.time,
            c_cmd=msg.c_cmd,
        )
        logger.infof(result)
    except (KeyError, ValueError) as e:
        result = 'failure: convert message failed, error={}'.format(e)
        logger.errorf(result)

    return result
