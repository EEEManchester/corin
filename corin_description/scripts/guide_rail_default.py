#!/usr/bin/env python

## Sets Gazebo Corin model to default world position in nominal standing up position

import rospy, sys, os, time
import string
import warnings
import tf
from math import pi

from gazebo_msgs.srv import *
from gazebo_ros import gazebo_interface
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import LinkState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState

class RobotPose:
	def __init__(self):
		rospy.init_node('PoseModifier') 		# Initialises node

		self.model_name = 'corin'
		self.reference_frame = 'world'
		self.joint_pub_ = {}

		self.initialise_topics()

	def initialise_topics(self):

		self.joint_pub_[0]  = rospy.Publisher(self.model_name + '/leg_q1_joint/command', Float64, queue_size=1)
		self.joint_pub_[1]  = rospy.Publisher(self.model_name + '/leg_q2_joint/command', Float64, queue_size=1)
		self.joint_pub_[2]  = rospy.Publisher(self.model_name + '/leg_q3_joint/command', Float64, queue_size=1)
		self.guide_pub_ 	= rospy.Publisher(self.model_name + '/guide_rail_joint/command', Float64, queue_size=1)
		rospy.sleep(0.5)

	def set_full_pose(self, q):
		
		self.guide_pub_.publish(data=20)
		self.set_leg_pose(q)
		self.guide_pub_.publish(data=0)

	def set_leg_pose(self, q):
		# Publish joint states
		for c in range(0,6):
			for j in range(0,3):
				self.joint_pub_[j].publish(q[j])
			rospy.sleep(0.2) 	

if __name__ == "__main__":

	r_ = RobotPose()

	# Default joint
	q1 = 0.0
	q2 = 0.34
	q3 = -1.85
	qd = [q1, q2, q3]

	r_.set_full_pose(qd)
	