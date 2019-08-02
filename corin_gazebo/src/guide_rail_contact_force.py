#!/usr/bin/env python

import rospy
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'library'))

import numpy as np

from corin_control.constant import *
from corin_control import *

from gazebo_msgs.msg import ContactsState
from std_msgs.msg import ByteMultiArray
from std_msgs.msg import Float32MultiArray

from sensor_msgs.msg import JointState
import tf
from math import pi

def quaternion_matrix(quaternion):
    """Return homogeneous rotation matrix from quaternion."""

    q = np.array(quaternion, dtype=np.float64, copy=True)
    n = np.dot(q, q)
    if n < np.finfo(float).eps * 4.0:
        return np.identity(4)
    q *= math.sqrt(2.0 / n)
    q = np.outer(q, q)
    return np.array([
        [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0], 0.0],
        [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0], 0.0],
        [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2], 0.0],
        [                0.0,                 0.0,                 0.0, 1.0]])

class ContactForce:
	def __init__(self):
		rospy.init_node('contact_controller') 		#Initialises node
		self.robot_ns = ROBOT_NS
		self.rate 	  = rospy.Rate(200)	# frequency: 200 Hz

		self.contact_force = [None]*3 	# three (xyz) for each leg
		self.world_contact_force = [None]*3 	# three (xyz) for each leg

		self.rtf = ArrayHomogeneousTransform(0)
		self.q = None
		self._start()

	def _start(self):
		#######################################
		## initialise publishers/subscribers ##
		#######################################

		##***************** SUBSCRIBERS ***************##
		## Foot contact
		self.contact_lf_sub_ = rospy.Subscriber(ROBOT_NS + '/foot_wrench/leg', ContactsState, self.contact_callback)
		## TF
		self.listener = tf.TransformListener()
		self.joint_sub_ = rospy.Subscriber(ROBOT_NS + '/joint_states', JointState, self.joint_state_callback)

		##***************** PUBLISHERS ***************##
		self.contact_force_pub_ = rospy.Publisher(ROBOT_NS + '/contact_force', Float32MultiArray, queue_size=1)
		self.contact_force_world_pub_ = rospy.Publisher(ROBOT_NS + '/world_contact_force', Float32MultiArray, queue_size=1)

	def set_to_zero(self, i):
		## set forces to zero when there's no contact ##
		self.contact_force = [0.]*3
		self.world_contact_force = [0.]*3

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
				
				try:
					self.rtf.update_coxa_X_foot(self.q)
					self.world_contact_force = mX(self.rtf.coxa_X_foot[:3,:3], self.contact_force)
					# print np.round(self.world_contact_force,4)
				except:
					# print e
					pass

	def joint_state_callback(self, msg):

		self.q = msg.position#[1:4]

	def publish(self):
		
		self.contact_force_pub_.publish(Float32MultiArray(data = self.contact_force))
		self.contact_force_world_pub_.publish(Float32MultiArray(data = self.world_contact_force))

if __name__ == "__main__":

	contact_manager = ContactForce()
	rospy.loginfo("Contact Force Node Initiated")

	while not rospy.is_shutdown():
		contact_manager.publish()
		contact_manager.rate.sleep()
