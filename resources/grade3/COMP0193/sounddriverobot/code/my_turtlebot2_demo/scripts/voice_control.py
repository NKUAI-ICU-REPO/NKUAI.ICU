#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def control_robot(msg):
    if msg.data == 'left':
        # 控制机器人左转
        pass
    elif msg.data == 'right':
        # 控制机器人右转
        pass
    else:
        pass

if __name__ == '__main__':
    rospy.init_node('voice_control')
    rospy.Subscriber('/command', String, control_robot)
    rospy.spin()