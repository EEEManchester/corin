#!/usr/bin/env python

## Test file for writing to servos
import rospy
import numpy as np
import math

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from robotis_controller_msgs.msg import SyncWriteItem
from robotis_controller_msgs.msg import SyncWriteMulti
# from robotis_controller_msgs import WriteControlTable

class CorinManager:
	def __init__(self):
		rospy.init_node('main_controller') 		#Initialises node
		self.freq	= 1 					# frequency
		self.rate 	= rospy.Rate(self.freq)	# control rate

		self._start()

	def joint_state_callback(self, msg):
		print msg


	def _start(self):
		#######################################
		## initialise publishers/subscribers ##
		#######################################

		##***************** PUBLISHERS ***************##
		self.control_mode_	= rospy.Publisher('/robotis/set_control_mode', String, queue_size=1)
		self.mm_joint_pub_	= rospy.Publisher('/robotis/set_joint_states', JointState, queue_size=1)

		self.sync_itm_pub_  = rospy.Publisher('/robotis/sync_write_item', SyncWriteItem, queue_size=1)
		self.sync_mlt_pub_  = rospy.Publisher('/robotis/sync_write_multi', SyncWriteMulti, queue_size=1)


		##***************** SUBSCRIBERS ***************##
		# self.joint_sub_	= rospy.Subscriber('/robotis/present_joint_states', JointState, self.joint_state_callback, queue_size=5)

		rospy.sleep(0.5)
		# self.control_mode_.publish("DirectControlMode")

	def publish_joint(self, radian, rads):
		# # ROS JointState
		# qp.name.append(str('rr_q3'))
		# qp.position.append(radian)
		# qp.velocity.append(0.)
		# qp.effort.append(0.)
		#
		# qp.header.stamp = rospy.get_rostime()
		#
		# self.mm_joint_pub_.publish(qp)

		## Direct Control
		value_of_0_radian_position_      = 2048
		value_of_min_radian_position_    = 0
		value_of_max_radian_position_    = 4095
		min_radian_                      = -3.14159265
		max_radian_                      =  3.14159265
		velocity_to_value_ratio_ 		 = 1.0/(0.229*2*3.14159265/60.0)
		acceleration_to_value_ratio_	 = 1.0/(214.577*2*3.14159265/(60.0*60.0))

		if (radian > 0):
			value = (radian * (value_of_max_radian_position_ - value_of_0_radian_position_) / max_radian_) + value_of_0_radian_position_;

		elif (radian < 0):
			value = (radian * (value_of_min_radian_position_ - value_of_0_radian_position_) / min_radian_) + value_of_0_radian_position_;
		else:
			value = 2048;

		vel_raw = math.ceil(abs(rads) * velocity_to_value_ratio_)
		if (vel_raw == 0):
			vel_raw = 1

		dqp = SyncWriteMulti()
		dqp.item_name = str("profile_velocity")
		dqp.data_length = 8
		dqp.joint_name.append(str('rr_q3')) 	# joint names to append
		dqp.value.append(int(vel_raw))
		dqp.value.append(int(round(value))) 			# value to append

		print value, vel_raw
		self.sync_mlt_pub_.publish(dqp)

if __name__ == "__main__":

	manager = CorinManager()
	tdelay = 0.1

	## using control table
	# qp = WriteControlTable()

	## Using JointStates
	# raw_input("Start JointState Write")
	# dqp = JointState()
	# dqp.name.append(str('lf_q1'))
	# dqp.position.append(0.5)
	# dqp.velocity.append(0.0)
	# dqp.effort.append(0.0)
	# manager.mm_joint_pub_.publish(dqp)

	## Using sync_write
	raw_input("Start Sync Write")
	dqp = SyncWriteMulti()

	dqp.item_name = str("profile_velocity") 	# Address to start writing to
	dqp.data_length = 8 					# number of bytes to write
	dqp.joint_name.append(str('rr_q3')) 	# joint names to append
	dqp.value.append(int(100)) 				# value to append
	dqp.value.append(int(2048))

	manager.sync_mlt_pub_.publish(dqp) 	# set velocity profile
	#
	# raw_input("Continue")
	#
	# dqp = SyncWriteMulti()
	# dqp.item_name = str("profile_velocity")
	# dqp.data_length = 8
	# dqp.joint_name.append(str('rr_q3')) 	# joint names to append
	# dqp.value.append(int(150)) 				# value to append
	# dqp.value.append(int(2700))
	#
	# manager.sync_pub_.publish(dqp)

	raw_input('start Sinusoidal')
	## Sinusoidal signal
	T  = 2.0 	# period for signal
	nd = 0.05 	# time interval for each sample
	amp = np.pi/2.0 	# signal amplitude

	Fs = int(T/nd) 		# no. samples per period
	print 'Fs: ', Fs
	prevA = 0.0
	n = 0

	manager.rate.sleep()

	while not rospy.is_shutdown():
		A = amp*np.sin(2*np.pi*n/Fs)
		v = (A - prevA)/(1.0/manager.freq)
		ac= v/(1.0/manager.freq)
		if (n==Fs-1):
			n=0
		else:
			n += 1

		print n, np.round(A,4), np.round(v,4), np.round(ac,4)

		manager.publish_joint(A,v)
		print '=================================='
		manager.rate.sleep()
		prevA = A
