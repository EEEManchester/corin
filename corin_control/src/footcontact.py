#!/usr/bin/env python

import rospy
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'library'))

import numpy as np

from corin_control.constant import *

from gazebo_msgs.msg import ContactsState
from std_msgs.msg import ByteMultiArray
from std_msgs.msg import Float32MultiArray


class ContactForce:
	def __init__(self):
		rospy.init_node('contact_controller') 		#Initialises node
		self.robot_ns = ROBOT_NS
		self.rate 	  = rospy.Rate(200)	# frequency: 200 Hz

		self.contact_state = [None]*6 	# one for each leg
		self.contact_force = [None]*18 	# three (xyz) for each leg

		self._start()

	def _start(self):
		#######################################
		## initialise publishers/subscribers ##
		#######################################

		##***************** SUBSCRIBERS ***************##
		## Foot contact
		self.contact_lf_sub_ = rospy.Subscriber(ROBOT_NS + '/foot_wrench/lf', ContactsState, self.contact_callback)
		self.contact_lm_sub_ = rospy.Subscriber(ROBOT_NS + '/foot_wrench/lm', ContactsState, self.contact_callback)
		self.contact_lr_sub_ = rospy.Subscriber(ROBOT_NS + '/foot_wrench/lr', ContactsState, self.contact_callback)

		self.contact_rf_sub_ = rospy.Subscriber(ROBOT_NS + '/foot_wrench/rf', ContactsState, self.contact_callback)
		self.contact_rm_sub_ = rospy.Subscriber(ROBOT_NS + '/foot_wrench/rm', ContactsState, self.contact_callback)
		self.contact_rr_sub_ = rospy.Subscriber(ROBOT_NS + '/foot_wrench/rr', ContactsState, self.contact_callback)

		##***************** PUBLISHERS ***************##
		self.contact_state_pub_ = rospy.Publisher(ROBOT_NS + '/contact_state', ByteMultiArray, queue_size=1)
		self.contact_force_pub_ = rospy.Publisher(ROBOT_NS + '/contact_force', Float32MultiArray, queue_size=1)

	def set_to_zero(self, i):
		## set forces to zero when there's no contact ##
		for j in range(0,3):
			self.contact_force[i*3+j] = 0.

	def append_to_list(self, i, ros_force):
		## map from ros message to list ##
		self.contact_force[i*3] = ros_force.x
		self.contact_force[i*3+1] = ros_force.y
		self.contact_force[i*3+2] = ros_force.z

	def contact_callback(self, msg):
		## checks if contact exist. Yes: update list with contact force. No: set to zero ##
		if (msg.header.frame_id == 'lf_foot'):
			if (len(msg.states) == 0):
				self.contact_state[0] = 0
				self.set_to_zero(0)
			else:
				self.contact_state[0] = 1
				self.append_to_list(0, msg.states[0].total_wrench.force)

		elif(msg.header.frame_id == 'lm_foot'):
			if (len(msg.states) == 0):
				self.contact_state[1] = 0
				self.set_to_zero(1)
			else:
				self.contact_state[1] = 1
				self.append_to_list(1, msg.states[0].total_wrench.force)

		elif(msg.header.frame_id == 'lr_foot'):
			if (len(msg.states) == 0):
				self.contact_state[2] = 0
				self.set_to_zero(2)
			else:
				self.contact_state[2] = 1
				self.append_to_list(2, msg.states[0].total_wrench.force)

		elif(msg.header.frame_id == 'rf_foot'):
			if (len(msg.states) == 0):
				self.contact_state[3] = 0
				self.set_to_zero(3)
			else:
				self.contact_state[3] = 1
				self.append_to_list(3, msg.states[0].total_wrench.force)

		elif(msg.header.frame_id == 'rm_foot'):
			if (len(msg.states) == 0):
				self.contact_state[4] = 0
				self.set_to_zero(4)
			else:
				self.contact_state[4] = 1
				self.append_to_list(4, msg.states[0].total_wrench.force)

		elif(msg.header.frame_id == 'rr_foot'):
			if (len(msg.states) == 0):
				self.contact_state[5] = 0
				self.set_to_zero(5)
			else:
				self.contact_state[5] = 1
				self.append_to_list(5, msg.states[0].total_wrench.force)

	def publish(self):
		cstate_msg = ByteMultiArray(data = self.contact_state)
		cforce_msg = Float32MultiArray(data = self.contact_force)

		# self.contact_state_pub_.publish(cstate_msg)
		self.contact_force_pub_.publish(cforce_msg)

if __name__ == "__main__":

	contact_manager = ContactForce()
	print "Contact Force node Initiated"

	while not rospy.is_shutdown():
		contact_manager.publish()
		contact_manager.rate.sleep()
