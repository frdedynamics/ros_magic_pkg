#!/usr/bin/env python
""" module docstring, yo! """
import sys
import rospy
from std_msgs.msg import Float64, Bool
from robotiq_urcap_control.msg import Robotiq2FGripper_robot_output as outputMsg

cmd_grip_pub = rospy.Publisher('/Robotiq2FGripperRobotOutput', outputMsg, queue_size=1)

def callback(data):

	msg = outputMsg()
	print data.data

	if data.data:
		msg.rPR = 0
	else:
		msg.rPR = 255
	msg.rSP = 255
	msg.rFR = 255
	cmd_grip_pub.publish(msg)

def listener():
    """ function docstring, yo! """
    rospy.init_node('grab_to_gripper_pub', anonymous=True)
    rospy.Subscriber("/cmd_grip", Bool, callback)
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
