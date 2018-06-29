#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
from rosbridge.msg import r_req

def pub():
    pub = rospy.Publisher('Turtlebot2/ROS', r_req, queue_size=10)
    rospy.init_node('order_robot', anonymous=True)
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        msg = r_req()
        msg.id = 1
        msg.time = "yyyy-mm-dd hh:mm:ss"
        msg.r_cmd = "Navi"
        msg.pos.x = 0.1
        msg.pos.y = 0.2
        msg.pos.z = 1.0

        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        pub()
    except rospy.ROSInterruptException:
        pass
