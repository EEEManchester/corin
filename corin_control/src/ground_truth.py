#!/usr/bin/env python

## State estimation for the robot
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'library'))
sys.dont_write_bytecode = True

import time
import warnings
import numpy as np
import copy
import random

## Personal libraries
from corin_control import *
import tf 												# SE(3) transformation library

## ROS messages
import rospy
from sensor_msgs.msg import Imu 						# sub msg for IMU
from sensor_msgs.msg import JointState 					# sub msg for joint states
from trajectory_msgs.msg import JointTrajectoryPoint	# pub msg for Robotis joints
from std_msgs.msg import Header
from std_msgs.msg import Float64 						# pub msg for Gazebo joints
from std_msgs.msg import ByteMultiArray 				# foot contact state
from std_msgs.msg import Float32MultiArray				# foot contact force
from std_msgs.msg import Float64MultiArray				# foot contact force
from geometry_msgs.msg import PoseStamped					# IMU pose
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Quaternion
import matplotlib.pyplot as plt

## Robotis ROS msgs for joint control
from robotis_controller_msgs.msg import SyncWriteItem 	# pub msg for Robotis joints
from robotis_controller_msgs.msg import SyncWriteMulti 	# pub msg for Robotis joints


np.set_printoptions(suppress=True) # suppress scientific notation
np.set_printoptions(precision=5) # number display precision
np.set_printoptions(formatter={'float': '{: 0.7f}'.format})
np.set_printoptions(linewidth=1000)

class CorinStateTester:
	def __init__(self):

		rospy.init_node("grou_truth")

		self.model_states  = rospy.Subscriber('gazebo/model_states', ModelStates, self.model_states_callback, queue_size=1)
		self.pub_body_a = rospy.Publisher('body_a', Vector3, queue_size=1)
		self.pub_body_v = rospy.Publisher('body_v', Vector3, queue_size=1)
		self.pub_body_r = rospy.Publisher('body_r', Vector3Stamped, queue_size=1)
		self.pub_body_w = rospy.Publisher('body_w', Vector3, queue_size=1)
		self.pub_body_p = rospy.Publisher('body_p', Vector3, queue_size=1)

		self.q0 = None
		self.body_r0 = None
		self.body_q0 = None

		rospy.sleep(0.5)

	def model_states_callback(self, msg):

		o = msg.pose[1].orientation # quaternion
		q = np.array([o.w, o.x, o.y, o.z])
		r = msg.pose[1].position
		r = np.array([r.x, r.y, r.z])

		if self.body_q0 is None:

			self.q0 = q
			b_q0_conj = q.copy()
			b_q0_conj[1:4] = - b_q0_conj[1:4]
			self.body_q0 = quaternion_multiply(self.q0, b_q0_conj) # Hamilton

			self.body_r0 = r.copy()

		else:

			r = r - self.body_r0
			r = quaternion_matrix(self.body_q0)[0:3,0:3].dot(r) # Hamilton

			q = quaternion_multiply(self.body_q0, q) # Hamilton

			h = Header()
			h.stamp = rospy.Time.now()
			self.pub_body_r.publish(h, Vector3(*r.tolist())) # mm unit

			angles_tuple = euler_from_quaternion_JPL(q, 'rxyz') # relative: rxyz
			angles = np.asarray(angles_tuple).tolist()

			self.pub_body_p.publish(Vector3(*angles))



if __name__ == "__main__":

	state = CorinStateTester()

	#TODO: uncomment for real test
	rospy.spin()
	print "done"
