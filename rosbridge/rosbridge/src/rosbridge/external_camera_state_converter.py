# -*- coding: utf-8 -*-
import datetime

from external_camera.msg import c_state as message_type

from rosbridge.logging import getLogger
logger = getLogger(__name__)

PAYLOAD_FMT = '{timestamp}|time|{time}|c_mode|{c_mode}|num_p|{num_p}|position|{position}'


def convert_ros_to_mqtt(tz, msg):
    logger.infof('convert ros to mqtt, msg={}', str(msg).replace('\n', ' '))
    if not isinstance(msg, message_type):
        logger.errorf('failure: message type error')
        return

    if msg.c_mode not in ('Monitor', 'Standby', 'Error'):
        logger.warnf('failure: unknown c_mode, c_mode="{}"'.format(msg.c_mode))
        return

    timestamp = datetime.datetime.now(tz).strftime('%Y-%m-%dT%H:%M:%S.%f%z')
    position_arry = []
    for i, pos in enumerate(msg.position):
        s = 'x[{i}],{x}/y[{i}],{y}'.format(
            i=i,
            x=pos.x,
            y=pos.y,
        )
        position_arry.append(s)

    return PAYLOAD_FMT.format(timestamp=timestamp,
                              time=msg.time,
                              c_mode=msg.c_mode,
                              num_p=msg.num_p,
                              position='/'.join(position_arry) if len(msg.position) > 0 else '-',
                              )
