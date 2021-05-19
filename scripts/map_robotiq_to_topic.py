#!/usr/bin/env python
""" module docstring, yo! """
import sys
import rospy
from std_msgs.msg import Float64
from robotiq_urcap_control.msg import Robotiq2FGripper_robot_output as outputMsg

cmd_grip_pub = rospy.Publisher('/Robotiq2FGripperRobotOutput', outputMsg, queue_size=1)

def callback(data):

    msg = outputMsg()
    msg.rPR = data.data
    msg.rSP = 10
    msg.rFR = 10
    cmd_grip_pub.publish(msg)

def listener():
    """ function docstring, yo! """
    rospy.init_node('grab_to_gripper_pub', anonymous=True)
    rospy.Subscriber("/cmd_grip_pos", Float64, callback)
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
