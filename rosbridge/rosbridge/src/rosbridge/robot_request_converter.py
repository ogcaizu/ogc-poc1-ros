# -*- coding: utf-8 -*-
import datetime

import pytz

from office_guide_robot.msg import r_req as message_type

from rosbridge.logging import getLogger
logger = getLogger(__name__)


def convert_mqtt_to_ros(cmd_data, ros_publisher):
    logger.infof('convert mqtt to ros, cmd_data={}', cmd_data)

    try:
        r_cmd = cmd_data['r_cmd']
        if r_cmd not in ('Navi', 'Standby', 'Clear'):
            return 'failure: unknown r_cmd, r_cmd="{}"'.format(r_cmd)

        msg = message_type()
        msg.time = datetime.datetime.now(pytz.timezone('Asia/Tokyo')).strftime('%Y-%m-%d %H:%M:%S')
        msg.r_cmd = r_cmd
        msg.pos.x = float(cmd_data['x'])
        msg.pos.y = float(cmd_data['y'])

        ros_publisher.publish(msg)
        result = 'result,success/time,{time}/r_cmd,{r_cmd}/pos.x,{pos_x}/pos.y,{pos_y}'.format(
            time=msg.time,
            r_cmd=msg.r_cmd,
            pos_x=msg.pos.x,
            pos_y=msg.pos.y,
        )
        logger.infof(result)
    except (KeyError, ValueError) as e:
        result = 'failure: convert message failed, error={}'.format(e)
        logger.errorf(result)

    return result
