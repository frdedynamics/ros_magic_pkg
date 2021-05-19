#!/usr/bin/env python
""" module docstring, yo! """

import sys
import tf_conversions
import rospy
from leap_motion.msg import Human
from std_msgs.msg import Float64

cmd_grip_pub = rospy.Publisher('/cmd_grip_pos', Float64, queue_size=1)
def callback(data):
    """ function docstring, yo! """

    msg = Float64()
    msg.data = data.left_hand.grab_strength*255.0
    cmd_grip_pub.publish(msg)




def listener():
    """ function docstring, yo! """
    rospy.init_node('leap_to_grab_pub', anonymous=False)
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
