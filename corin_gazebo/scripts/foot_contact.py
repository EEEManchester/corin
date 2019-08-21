#!/usr/bin/env python

""" Subscribes to gazebo contact forces and republishes
	them in 3D force state only, custom for Corin """

import rospy
import os
import sys

import numpy as np
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import ByteMultiArray
from std_msgs.msg import Float32MultiArray


class ContactForce:
	def __init__(self):
		rospy.init_node('contact_force') 		#Initialises node
		self.robot_ns = 'corin'
		self.rate 	  = rospy.Rate(200)	# frequency: 200 Hz

		self.contact_force = [None]*18 	# three (xyz) for each leg

		self._start()

	def _start(self):
		#######################################
		## initialise publishers/subscribers ##
		#######################################

		##***************** SUBSCRIBERS ***************##
		## Foot contact
		self.contact_lf_sub_ = rospy.Subscriber(self.robot_ns + '/foot_wrench/lf', ContactsState, self.contact_callback)
		self.contact_lm_sub_ = rospy.Subscriber(self.robot_ns + '/foot_wrench/lm', ContactsState, self.contact_callback)
		self.contact_lr_sub_ = rospy.Subscriber(self.robot_ns + '/foot_wrench/lr', ContactsState, self.contact_callback)
		self.contact_rf_sub_ = rospy.Subscriber(self.robot_ns + '/foot_wrench/rf', ContactsState, self.contact_callback)
		self.contact_rm_sub_ = rospy.Subscriber(self.robot_ns + '/foot_wrench/rm', ContactsState, self.contact_callback)
		self.contact_rr_sub_ = rospy.Subscriber(self.robot_ns + '/foot_wrench/rr', ContactsState, self.contact_callback)

		##***************** PUBLISHERS ***************##
		self.contact_force_pub_ = rospy.Publisher(self.robot_ns + '/contact_force', Float32MultiArray, queue_size=1)

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
				self.set_to_zero(0)
			else:
				self.append_to_list(0, msg.states[0].total_wrench.force)

		elif(msg.header.frame_id == 'lm_foot'):
			if (len(msg.states) == 0):
				self.set_to_zero(1)
			else:
				self.append_to_list(1, msg.states[0].total_wrench.force)

		elif(msg.header.frame_id == 'lr_foot'):
			if (len(msg.states) == 0):
				self.set_to_zero(2)
			else:
				self.append_to_list(2, msg.states[0].total_wrench.force)

		elif(msg.header.frame_id == 'rf_foot'):
			if (len(msg.states) == 0):
				self.set_to_zero(3)
			else:
				self.append_to_list(3, msg.states[0].total_wrench.force)

		elif(msg.header.frame_id == 'rm_foot'):
			if (len(msg.states) == 0):
				self.set_to_zero(4)
			else:
				self.append_to_list(4, msg.states[0].total_wrench.force)

		elif(msg.header.frame_id == 'rr_foot'):
			if (len(msg.states) == 0):
				self.set_to_zero(5)
			else:
				self.append_to_list(5, msg.states[0].total_wrench.force)

	def publish(self):
		print 'len: ', len(self.contact_force)
		self.contact_force_pub_.publish(Float32MultiArray(data = self.contact_force))

if __name__ == "__main__":

	contact_manager = ContactForce()
	print "Contact Force node Initiated"

	while not rospy.is_shutdown():
		contact_manager.publish()
		contact_manager.rate.sleep()
