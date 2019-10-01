#!/usr/bin/env python
""" module docstring, yo! """

import sys
import tf_conversions
import rospy
import math
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
        )
        )

    msg = Twist()
    msg.angular.z = tmp[2]/math.pi/2

    cmd_vel_pub.publish(msg)




def listener():
    """ function docstring, yo! """
    rospy.init_node('magic_ctrl', anonymous=True)
    rospy.Subscriber("xsens_B42603", Imu, callback)
    rospy.spin()


if __name__ == '__main__':
    MYARGV = rospy.myargv(argv=sys.argv)
    if len(MYARGV) > 1:
        URL = URL.replace("127.0.0.1", MYARGV[1])
    rospy.loginfo("connecting to: %s", URL)
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
