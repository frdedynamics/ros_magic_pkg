#!/usr/bin/env python3

'''
This node subscribes the IMU and EMG readings from MYO armband and publish /cmd_vel
'''


# imports
from sqlalchemy import false
import rospy, rospkg, sys
from ros_myo.msg import EmgArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3, Quaternion
from tf.transformations import euler_from_quaternion as q2e
from ros_robotino_rest_pkg.msg import DigitalReadings

imu_pkg = rospkg.RosPack()
imu_pkg_path = imu_pkg.get_path('imu_human_pkg')+"/src/Classes"
# print(imu_pkg_path)
sys.path.insert(0, imu_pkg_path)
import Kinematics_with_Quaternions as kinematic

vel_cmd = Twist()
imu_ori_deg = Vector3(0.0, 0.0, 0.0)
imu_ori_deg_init= Vector3(0.0, 0.0, 0.0)
imu_ori_quat_init = Quaternion(0.0, 0.0, 0.0, 1.0)
imu_ori_quat = Quaternion(0.0, 0.0, 0.0, 1.0)
emg_sum = 0
emg_th = 3000
calib_index = 0
calib_flag = False
calib_th = 10  # 10ms*20 = 200ms is enough. not realtime so it is even slower
robotino_led_msg = DigitalReadings()


def cb_emg(msg):
	global emg_sum
	emg_sum = sum(msg.data)
	# print(emg_sum)


def cb_imu_ori(msg):
		global imu_ori_deg, calib_flag, calib_index, imu_ori_quat_init, imu_ori_quat
		if not calib_flag:
			imu_ori_quat_init = kinematic.q_invert(msg.orientation)
			# imu_ori_deg_init = q2e(kinematic.q_tf_convert(imu_ori_quat_init), axes='sxyz') # Maybe not need it
			print("calibrating:   ", calib_index)
		imu_ori_quat = kinematic.q_multiply(imu_ori_quat_init, msg.orientation)
		temp_deg = q2e(kinematic.q_tf_convert(imu_ori_quat), axes='sxyz')
		imu_ori_deg.x = temp_deg[0]
		imu_ori_deg.y = temp_deg[1]
		imu_ori_deg.z = temp_deg[2]
		# print(imu_ori_deg)


if __name__ == '__main__':
	rospy.init_node('myo_robotino_teleop', anonymous=False)
	pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	pub_robotino_led = rospy.Publisher('/digitaloutput', DigitalReadings, queue_size=1)
	sub_emg = rospy.Subscriber('/myo_raw/myo_emg', EmgArray, cb_emg)
	sub_imu = rospy.Subscriber('/myo_raw/myo_imu', Imu, cb_imu_ori)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		robotino_led_msg.stamp = rospy.get_rostime()
		if calib_index < calib_th:
			calib_index += 1
			calib_flag = False
		else:
			calib_flag = True

		if not emg_sum > emg_th:
	## This is very mixed angle problem.
			# if imu_ori_deg.z < -1.0:
			# 	vel_cmd.angular = Vector3(0.0,0.0,0.05)
			# 	vel_cmd.linear = Vector3(0.0,0.0,0.0)
				
			# elif imu_ori_deg.x < -1:
			# 	vel_cmd.angular = Vector3(0.0,0.0,0.0)
			# 	vel_cmd.linear = Vector3(-0.05,0.0,0.0)

			# elif imu_ori_deg.x > 1:
			# 	vel_cmd.angular = Vector3(0.0,0.0,0.0)
			# 	vel_cmd.linear = Vector3(0.05,0.0,0.0)

			# elif imu_ori_deg.y < -1:
			# 	vel_cmd.angular = Vector3(0.0,0.0,0.0)
			# 	vel_cmd.linear = Vector3(0.0,-0.05,0.0)

			# elif imu_ori_deg.y > 1:
			# 	vel_cmd.angular = Vector3(0.0,0.0,0.0)
			# 	vel_cmd.linear = Vector3(0.0,0.05,0.0)

			# else:
			# 	vel_cmd = Twist(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0,0.0))

	## Only x for rotation z and y for linear.x
	# add LED control also
			if imu_ori_deg.x < -1:
				vel_cmd.angular = Vector3(0.0,0.0,-0.15)
				vel_cmd.linear = Vector3(0.0,0.0,0.0)
				robotino_led_msg.values = [1,0,0,0,0,0,0,0]
			elif imu_ori_deg.x > 1:
				vel_cmd.angular = Vector3(0.0,0.0,0.15)
				vel_cmd.linear = Vector3(0.0,0.0,0.0)
				robotino_led_msg.values = [1,0,0,0,0,0,0,0]

			# since axis-y and axis-z are very coupled for elbow/arm motion, anyone passes the treshsol will create linear.x
			elif ((imu_ori_deg.y < -1) or (imu_ori_deg.z < -1)):
				vel_cmd.angular = Vector3(0.0,0.0,0.0)
				vel_cmd.linear = Vector3(0.15,0.0,0.0)
				robotino_led_msg.values = [1,0,0,0,0,0,0,0]

			elif ((imu_ori_deg.y > 1) or (imu_ori_deg.z > 1)):
				vel_cmd.angular = Vector3(0.0,0.0,0.0)
				vel_cmd.linear = Vector3(-0.15,0.0,0.0)
				robotino_led_msg.values = [1,0,0,0,0,0,0,0]

			else:
				vel_cmd = Twist(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0,0.0))
				robotino_led_msg.values = [0,1,0,0,0,0,0,0]
		else:
			vel_cmd = Twist(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0,0.0))
			print("Emergency stop")
			robotino_led_msg.values = [0,0,1,0,0,0,0,0]
		print(vel_cmd)
		pub_cmd_vel.publish(vel_cmd)
		pub_robotino_led.publish(robotino_led_msg)
		rate.sleep()
    

"""

## Simple myo demo that listens to std_msgs/UInt8 poses published 
## to the 'myo_gest' topic and drives turtlesim

import rospy, math
from std_msgs.msg import UInt8, String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Vector3
from ros_myo.msg import EmgArray

if __name__ == '__main__':
	global movingState
	global zero
	global y
	global K
	global speed
	speed = 0
	K = 1
	movingState = 0
	zero = 0
	y = 0
	rospy.init_node('robotino_driver', anonymous=True)

	# Publish to the turtlesim movement topic
	tsPub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
	turtlesimPub = rospy.Publisher("directs", String, queue_size=10)

	# Use the calibrated Myo gestures to drive the turtle
	def drive(gest):
		global movingState
		global zero
		global speed
		if gest.data == 1: #FIST
			movingState -= 1
		elif gest.data == 4 or gest.data == 2: #FINGERS_SPREAD
			movingState += 1
		elif gest.data == 3 :
			zero = y

		if movingState > 0 :
			movingState = 1
			turtlesimPub.publish("go forward")
			speed = 1
#			tsPub.publish(Twist(Vector3(1.0, 0, 0), Vector3(0, 0, 0)))
		elif movingState < 0 :
			movingState = -1
			turtlesimPub.publish("go back")
			speed = -1
#			tsPub.publish(Twist(Vector3(-1.0, 0, 0), Vector3(0, 0, 0)))
		else :
			speed = 0
		print (speed)

	def turn(imuRead):
		global zero
		global y
		global K
		global speed
		y = imuRead.orientation.y
		if (imuRead.orientation.y>zero):
			tsPub.publish(Twist(Vector3(speed,0,0),Vector3(0,0,K*(imuRead.orientation.y-zero))))
		if (imuRead.orientation.y<zero):
			tsPub.publish(Twist(Vector3(speed,0,0),Vector3(0,0,-K*(zero-imuRead.orientation.y))))
		#print (y)

	def strength(emgArr1):
		emgArr=emgArr1.data
		# Define proportional control constant:
		K = 0.005
		# Get the average muscle activation of the left, right, and all sides
		aveRight=(emgArr[0]+emgArr[1]+emgArr[2]+emgArr[3])/4
		aveLeft=(emgArr[4]+emgArr[5]+emgArr[6]+emgArr[7])/4
		ave=(aveLeft+aveRight)/2
		# If all muscles activated, drive forward exponentially
		if ave > 500:
			tsPub.publish(Twist(Vector3(0.1*math.exp(K*ave),0,0),Vector3(0,0,0)))
		# If only left muscles activated, rotate proportionally
		elif aveLeft > (aveRight + 200):
			tsPub.publish(Twist(Vector3(0,0,0),Vector3(0,0,K*ave)))
		# If only right muscles activated, rotate proportionally
		elif aveRight > (aveLeft + 200):
			tsPub.publish(Twist(Vector3(0,0,0),Vector3(0,0,-K*ave)))
		# If not very activated, don't move (high-pass filter)
		else:
			tsPub.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))

	rospy.Subscriber("myo_imu", Imu, turn)
	rospy.Subscriber("myo_gest", UInt8, drive)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()


"""
