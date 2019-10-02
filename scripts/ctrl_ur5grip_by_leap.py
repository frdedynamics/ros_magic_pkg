#!/usr/bin/env python
""" module docstring, yo! """

import sys
import tf_conversions
import rospy
from leap_motion.msg import Human
from std_msgs.msg import Bool

cmd_grip_pub = rospy.Publisher('cmd_grip', Bool, queue_size=1)

def callback(data):
    """ function docstring, yo! """
    grip_cmd = False
    if data.left_hand.grab_strength > 0.5:
	grip_cmd = True
    msg = Bool()
    msg.data = grip_cmd
    cmd_grip_pub.publish(msg)




def listener():
    """ function docstring, yo! """
    rospy.init_node('magic_ctrl2', anonymous=True)
    rospy.Subscriber("leap_motion/leap_filtered", Human, callback)
    rospy.spin()


if __name__ == '__main__':
    MYARGV = rospy.myargv(argv=sys.argv)
    if len(MYARGV) > 1:
        rospy.loginfo("args??")
    rospy.loginfo("make the magic happen")
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
