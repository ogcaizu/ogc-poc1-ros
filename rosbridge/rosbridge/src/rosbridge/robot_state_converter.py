# -*- coding: utf-8 -*-
import datetime

import pytz

from office_guide_robot.msg import r_state as message_type

from rosbridge.logging import getLogger
logger = getLogger(__name__)

PAYLOAD_FMT = '{timestamp}|time|{time}|r_mode|{r_mode}|pos.x|{pos_x}|pos.y|{pos_y}|pos.theta|{pos_theta}'


def convert_ros_to_mqtt(msg):
    logger.infof('convert ros to mqtt, msg={}', str(msg).replace('\n', ' '))
    if not isinstance(msg, message_type):
        logger.errorf('failure: message type error')
        return

    if msg.r_mode not in ('Navi', 'Standby', 'Error'):
        logger.warnf('failure: unknown r_mode, r_mode="{}"'.format(msg.r_mode))
        return

    timestamp = datetime.datetime.now(pytz.timezone('Asia/Tokyo')).strftime('%Y-%m-%dT%H:%M:%S.%f%z')
    return PAYLOAD_FMT.format(timestamp=timestamp,
                              time=msg.time,
                              r_mode=msg.r_mode,
                              pos_x=msg.pos.x,
                              pos_y=msg.pos.y,
                              pos_theta=msg.pos.theta,
                              )
