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

		self.contact_force = [None]*3 	# three (xyz) for each leg

		self._start()

	def _start(self):
		#######################################
		## initialise publishers/subscribers ##
		#######################################

		##***************** SUBSCRIBERS ***************##
		## Foot contact
		self.contact_lf_sub_ = rospy.Subscriber(ROBOT_NS + '/foot_wrench/leg', ContactsState, self.contact_callback)
		
		##***************** PUBLISHERS ***************##
		self.contact_force_pub_ = rospy.Publisher(ROBOT_NS + '/contact_force', Float32MultiArray, queue_size=1)

	def set_to_zero(self, i):
		## set forces to zero when there's no contact ##
		self.contact_force = [0.]*3

	def append_to_list(self, i, ros_force):
		## map from ros message to list ##
		self.contact_force[i*3] = ros_force.x
		self.contact_force[i*3+1] = ros_force.y
		self.contact_force[i*3+2] = ros_force.z

	def contact_callback(self, msg):
		## checks if contact exist. Yes: update list with contact force. No: set to zero ##
		if (msg.header.frame_id == 'leg_foot'):
			if (len(msg.states) == 0):
				self.set_to_zero(0)
			else:
				self.contact_force[0] = msg.states[0].total_wrench.force.x
				self.contact_force[1] = msg.states[0].total_wrench.force.y
				self.contact_force[2] = msg.states[0].total_wrench.force.z
				

	def publish(self):
		
		self.contact_force_pub_.publish(Float32MultiArray(data = self.contact_force))

if __name__ == "__main__":

	contact_manager = ContactForce()
	rospy.loginfo("Contact Force Node Initiated")

	while not rospy.is_shutdown():
		contact_manager.publish()
		contact_manager.rate.sleep()
