#!/usr/bin/env python

## Sets Gazebo Corin model to default world position in nominal standing up position

import rospy, sys, os, time
import string
import warnings
import tf
from math import pi
import numpy as np

from gazebo_msgs.srv import *
from gazebo_ros import gazebo_interface
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class LegMove:
	def __init__(self):
		rospy.init_node('LegMover') 		# Initialises node

		self.model_name = 'corin'
		self.joint_pub_ = {}
		self.joint_states = None

		self.initialise_topics()

	def initialise_topics(self):

		self.joint_sub_ = rospy.Subscriber('/corin/joint_states', JointState, self.joint_callback, queue_size=1)

		self.joint_pub_[0]  = rospy.Publisher(self.model_name + '/lf_q1_joint/command', Float64, queue_size=1)
		self.joint_pub_[1]  = rospy.Publisher(self.model_name + '/lf_q2_joint/command', Float64, queue_size=1)
		self.joint_pub_[2]  = rospy.Publisher(self.model_name + '/lf_q3_joint/command', Float64, queue_size=1)
		self.joint_pub_[3]  = rospy.Publisher(self.model_name + '/lm_q1_joint/command', Float64, queue_size=1)
		self.joint_pub_[4]  = rospy.Publisher(self.model_name + '/lm_q2_joint/command', Float64, queue_size=1)
		self.joint_pub_[5]  = rospy.Publisher(self.model_name + '/lm_q3_joint/command', Float64, queue_size=1)
		self.joint_pub_[6]  = rospy.Publisher(self.model_name + '/lr_q1_joint/command', Float64, queue_size=1)
		self.joint_pub_[7]  = rospy.Publisher(self.model_name + '/lr_q2_joint/command', Float64, queue_size=1)
		self.joint_pub_[8]  = rospy.Publisher(self.model_name + '/lr_q3_joint/command', Float64, queue_size=1)
		self.joint_pub_[9]  = rospy.Publisher(self.model_name + '/rf_q1_joint/command', Float64, queue_size=1)
		self.joint_pub_[10] = rospy.Publisher(self.model_name + '/rf_q2_joint/command', Float64, queue_size=1)
		self.joint_pub_[11] = rospy.Publisher(self.model_name + '/rf_q3_joint/command', Float64, queue_size=1)
		self.joint_pub_[12] = rospy.Publisher(self.model_name + '/rm_q1_joint/command', Float64, queue_size=1)
		self.joint_pub_[13] = rospy.Publisher(self.model_name + '/rm_q2_joint/command', Float64, queue_size=1)
		self.joint_pub_[14] = rospy.Publisher(self.model_name + '/rm_q3_joint/command', Float64, queue_size=1)
		self.joint_pub_[15] = rospy.Publisher(self.model_name + '/rr_q1_joint/command', Float64, queue_size=1)
		self.joint_pub_[16] = rospy.Publisher(self.model_name + '/rr_q2_joint/command', Float64, queue_size=1)
		self.joint_pub_[17] = rospy.Publisher(self.model_name + '/rr_q3_joint/command', Float64, queue_size=1)

		rospy.sleep(1.0)

	def joint_callback(self, msg):

		self.joint_states = msg

	def set_leg_pose(self, leg_move, q):
		# Publish joint states
		# print self.joint_states.position
		idx = leg_move*3
		intv = 100
		qc = np.array(self.joint_states.position[idx:idx+3])
		diff = (qd - qc)/50

		for i in range(intv):
			qnow = qc + diff*i
			for j in range(3):
				self.joint_pub_[idx+j].publish(qnow[j])
			rospy.sleep(0.025)

if __name__ == "__main__":

	r_ = LegMove()

	leg_move = 3
	qd = np.array([-0.6981317,  2.5, -2.])

	# if len(sys.argv) > 1:
	# 	if sys.argv[1] == 'fault':
	# 		qd[13] = 0.614#0.2678
	# 		qd[14] = -2.404#-2.212

	r_.set_leg_pose(leg_move, qd)
	