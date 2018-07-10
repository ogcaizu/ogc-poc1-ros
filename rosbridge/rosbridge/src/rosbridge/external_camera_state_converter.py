# -*- coding: utf-8 -*-
import datetime

import pytz

# from rosbridge.msg import c_state as message_type
from external_camera.msg import c_state as message_type

from rosbridge.logging import getLogger
logger = getLogger(__name__)

PAYLOAD_FMT = '{timestamp}|time|{time}|camera_id|{id}|c_mode|{c_mode}|num_p|{num_p}|p_state|{p_state}'


def convert_ros_to_mqtt(msg):
    logger.infof('convert ros to mqtt, msg={}', str(msg).replace('\n', ' '))
    if not isinstance(msg, message_type):
        logger.errorf('failure: message type error')
        return

    if msg.c_mode not in ('Monitor', 'Standby', 'Error'):
        logger.warnf('failure: unknown c_mode, c_mode="{}"'.format(msg.c_mode))
        return

    timestamp = datetime.datetime.now(pytz.timezone('Asia/Tokyo')).strftime('%Y-%m-%dT%H:%M:%S.%f%z')
    p_state_arry = []
    for p_state in msg.p_state:
        s = 'pos[{i}].x,{x}/pos[{i}].y,{y}/pos[{i}].z,{z}/width[{i}],{w}/height[{i}],{h}/feature_hex[{i}],{f}'.format(
            i=p_state.i,
            x=p_state.pos.x,
            y=p_state.pos.y,
            z=p_state.pos.z,
            w=p_state.size.width,
            h=p_state.size.height,
            f=p_state.feature.encode('hex'),
        )
        p_state_arry.append(s)

    return PAYLOAD_FMT.format(timestamp=timestamp,
                              time=msg.time,
                              id=msg.id,
                              c_mode=msg.c_mode,
                              num_p=msg.num_p,
                              p_state='/'.join(p_state_arry),
                              )
