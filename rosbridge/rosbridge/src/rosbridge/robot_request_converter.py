# -*- coding: utf-8 -*-
import datetime

import pytz

from rosbridge.msg import r_req as message_type

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
        msg.id = int(cmd_data['robot_id'])
        msg.r_cmd = r_cmd
        msg.pos.x = float(cmd_data['pos.x'])
        msg.pos.y = float(cmd_data['pos.y'])
        msg.pos.z = float(cmd_data['pos.z'])

        ros_publisher.publish(msg)
        result = 'result,success/time,{time}/id,{id}/r_cmd,{r_cmd}/pos.x,{pos_x}/pos.y,{pos_y}/pos.z,{pos_z}'.format(
            time=msg.time,
            id=msg.id,
            r_cmd=msg.r_cmd,
            pos_x=msg.pos.x,
            pos_y=msg.pos.y,
            pos_z=msg.pos.z,
        )
        logger.infof(result)
    except (KeyError, ValueError) as e:
        result = 'failure: convert message failed, error={}'.format(e)
        logger.errorf(result)

    return result
