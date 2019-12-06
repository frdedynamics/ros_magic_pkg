#!/usr/bin/env python
""" module docstring, yo! """

import sys
import tf_conversions
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

def callback(data):
    """ function docstring, yo! """
    tmp = tf_conversions.transformations.euler_from_quaternion(
        (
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w
        ), 'sxyz',
        )

    forward = -min(0.2,max(-0.2,tmp[1]))
    sideways = min(0.2,max(-0.2,tmp[0]))
    rotation = min(0.8,max(-0.8,tmp[2]))

    if abs(forward) < 0.2:
	forward = 0.0

    if abs(sideways) < 0.2:
	sideways = 0.0

    if abs(rotation) < 0.1:
	rotation = 0.0

    msg = Twist()
    msg.linear.x = forward
    msg.linear.y = sideways
    msg.angular.z = rotation
    cmd_vel_pub.publish(msg)




def listener():
    """ function docstring, yo! """
    rospy.init_node('magic_ctrl', anonymous=True)
    rospy.Subscriber("xsens_B42F3D", Imu, callback)
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
