#!/usr/bin/env python

## service server for torque enable/disable
import rospy
import numpy as np
import math

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from robotis_controller_msgs.msg import SyncWriteItem
from robotis_controller_msgs.msg import SyncWriteMulti
from robotis_controller_msgs.srv import *

class CorinManager:
	def __init__(self):
		rospy.init_node('torque_server') 		#Initialises node
		self.freq	= 1 						# frequency
		self.rate 	= rospy.Rate(self.freq)		# control rate

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

		##***************** SERVICES ***************##
		self.torque_serv_ 	= rospy.Service('dxl_server/torque_enable', SetTorqueEnable, self.set_torque)

		rospy.sleep(0.5)
		rospy.loginfo("Torque Server Initiated")

	def set_torque(self, req):
		torque_enable = 0;
		
		if (req.enable == True):
			torque_enable = 1
			rospy.loginfo("Enabling Torque")
		elif (req.enable == False):
			torque_enable = 0
			rospy.loginfo("Disabling Torque")
		else:
			ROS_WARN("Invalid Command!")

		dqp = SyncWriteItem()
		dqp.item_name = str("torque_enable") 	# Address to start writing to
		
		# joint names to append [TODO: BASED IT ON AVAILABLE SERVOS]
		dqp.joint_name.append(str('lf_q1_joint'))
		dqp.joint_name.append(str('lf_q2_joint'))
		dqp.joint_name.append(str('lf_q3_joint'))
		dqp.joint_name.append(str('lm_q1_joint'))
		dqp.joint_name.append(str('lm_q2_joint'))
		dqp.joint_name.append(str('lm_q3_joint'))
		dqp.joint_name.append(str('lr_q1_joint'))
		dqp.joint_name.append(str('lr_q2_joint'))
		dqp.joint_name.append(str('lr_q3_joint'))
		dqp.joint_name.append(str('rf_q1_joint'))
		dqp.joint_name.append(str('rf_q2_joint'))
		dqp.joint_name.append(str('rf_q3_joint'))
		dqp.joint_name.append(str('rm_q1_joint'))
		dqp.joint_name.append(str('rm_q2_joint'))
		dqp.joint_name.append(str('rm_q3_joint'))
		dqp.joint_name.append(str('rr_q1_joint'))
		dqp.joint_name.append(str('rr_q2_joint'))
		dqp.joint_name.append(str('rr_q3_joint'))

		for i in range(0,18):
			dqp.value.append(int(torque_enable))	# value to append

		manager.sync_itm_pub_.publish(dqp) 			# publish topic

		return SetTorqueEnableResponse(True)

if __name__ == "__main__":

	manager = CorinManager()
	tdelay = 0.1

	rospy.spin()

	
	