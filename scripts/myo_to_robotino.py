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
				vel_cmd.angular = Vector3(0.0,0.0,0.5)
				vel_cmd.linear = Vector3(0.0,0.0,0.0)
				robotino_led_msg.values = [1,0,0,0,0,0,0,0]
			elif imu_ori_deg.x > 1:
				vel_cmd.angular = Vector3(0.0,0.0,-0.5)
				vel_cmd.linear = Vector3(0.0,0.0,0.0)
				robotino_led_msg.values = [1,0,0,0,0,0,0,0]

			# since axis-y and axis-z are very coupled for elbow/arm motion, anyone passes the treshsol will create linear.x
			elif ((imu_ori_deg.y < -1) or (imu_ori_deg.z < -1)):
				vel_cmd.angular = Vector3(0.0,0.0,0.0)
				vel_cmd.linear = Vector3(0.5,0.0,0.0)
				robotino_led_msg.values = [1,0,0,0,0,0,0,0]

			elif ((imu_ori_deg.y > 1) or (imu_ori_deg.z > 1)):
				vel_cmd.angular = Vector3(0.0,0.0,0.0)
				vel_cmd.linear = Vector3(-0.5,0.0,0.0)
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
    
