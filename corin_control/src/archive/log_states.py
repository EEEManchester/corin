#!/usr/bin/env python

## Publishes state for RVIZ

import rospy
import roslib
import os
import sys
# sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'library'))
# sys.dont_write_bytecode = True
from corin_control import *			# library modules to include
from corin_msgs.msg import LoggingState

import numpy as np

import tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu

from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import ByteMultiArray
from std_msgs.msg import Float32MultiArray
# from constant import *
# from robot_transforms import *

class Node_class:
	def __init__(self):
		rospy.init_node("Robot_State_publisher")							# Initiates Ros node
		self.freq = 120
		self.rate = rospy.Rate(self.freq)					# Sets transmit frequency
		# self.robot_broadcaster = tf.TransformBroadcaster()	# Transform for robot pose
		# self.lidar_broadcaster = tf.TransformBroadcaster()	# Transform for robot pose
		# self.jointState  = JointState()

		self.v3cp = np.zeros(3)
		self.v3wp = np.zeros(3)
		self.v3cv = np.zeros(3)
		self.v3wv = np.zeros(3)
		self.joint_states = None
		self.contact_force = None
		self.start()

	def start(self):

		##***************** SUBSCRIBERS ***************##
		# subscribe to gazebo
		if rospy.has_param('/gazebo/auto_disable_bodies'):
			self.robot_state_sub_ = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_callback)

		# subscribe to hardware IMU
		else:
			self.imu_sub_ 		  = rospy.Subscriber("/imu_data", Imu, self.imu_callback)

		# self.imu_sub_	 = rospy.Subscriber(ROBOT_NS + '/imu/base/data', Imu, self.imu_callback, queue_size=1)
		# self.cstate_sub_ = rospy.Subscriber(ROBOT_NS + '/contact_state', ByteMultiArray, self.contact_state_callback, queue_size=1)
		self.cforce_sub_ = rospy.Subscriber(ROBOT_NS + '/contact_force', Float32MultiArray, self.contact_force_callback, queue_size=1)
		self.joint_sub_  = rospy.Subscriber(ROBOT_NS + '/joint_states', JointState, self.joint_state_callback, queue_size=5)

		##***************** PUBLISHERS ***************##
		self.log_actual_pub_ = rospy.Publisher('/corin/log_actual', LoggingState, queue_size=1)	# LOGGING setpoint publisher

	## Transform from gazebo robot_pose to TF for RVIZ visualization
	def model_callback(self, robot_state):
		rs_size = len(robot_state.name)

		for i in range(0,rs_size):
			if (robot_state.name[i]=='corin'):
				self.v3cp[0] = robot_state.pose[i].position.x
				self.v3cp[1] = robot_state.pose[i].position.y
				self.v3cp[2] = robot_state.pose[i].position.z

				rx = robot_state.pose[i].orientation.x
				ry = robot_state.pose[i].orientation.y
				rz = robot_state.pose[i].orientation.z
				rw = robot_state.pose[i].orientation.w

				self.v3wp = np.array(euler_from_quaternion([rw, rx, ry, rz], 'sxyz')).flatten()

				self.v3cv[0] = robot_state.twist[i].linear.x
				self.v3cv[1] = robot_state.twist[i].linear.y
				self.v3cv[2] = robot_state.twist[i].linear.z
				
				self.v3wv[0] = robot_state.twist[i].angular.x
				self.v3wv[1] = robot_state.twist[i].angular.y
				self.v3wv[2] = robot_state.twist[i].angular.z

	def joint_state_callback(self, msg):
		""" robot joint state callback """
		self.joint_states = msg
		
	def contact_force_callback(self, msg):
		""" foot contact force """
		cforce = msg.data

		world_X_base = r3_X_m(self.v3wp)

		self.contact_force = [0]*18
		# Transform to world frame
		for j in range(0,6):
			# try:
			q = self.joint_states.position[j*3:j*3+3]

			tibia_foot_force = cforce[j*3:j*3+3]
			
			coxa_X_foot = mX(rot_Z(q[0]),
							rot_X(np.pi/2),
							rot_Z(q[1]),
							rot_Z(q[2]))
			base_X_foot = mX(rot_Z(ROT_BASE_X_LEG[j]), coxa_X_foot)

			world_foot_force = mX(world_X_base[:3,:3], base_X_foot, tibia_foot_force)

			self.contact_force[j*3:j*3+3] = world_foot_force.tolist()
			if j==5:
				print np.round(self.contact_force[j*3:j*3+3],4)
	def publish_log(self):
		
		# try:
		qlog = LoggingState()
		qlog.positions = self.v3cp.flatten().tolist() + self.v3wp.flatten().tolist() + list(self.joint_states.position)
		qlog.velocities = self.v3cv.flatten().tolist() + self.v3wv.flatten().tolist() + list(self.joint_states.velocity)
		qlog.effort = list(self.joint_states.effort)
		qlog.forces = self.contact_force
		# except:
		# 	pass

		self.log_actual_pub_.publish(qlog)
		self.rate.sleep()

if __name__ == "__main__":
	n = Node_class()
	rospy.sleep(1.0)
	print 'Robot publisher initiated'

	# rospy.spin()
	while not rospy.is_shutdown():
		n.publish_log()