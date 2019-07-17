#!/usr/bin/env python

## Publishes state for RVIZ

import rospy
import roslib
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'library'))
# sys.dont_write_bytecode = True

import numpy as np

import tf
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates


class Node_class:
	def __init__(self):
		rospy.init_node("State_publisher")							# Initiates Ros node
		# self.freq = 80
		# self.rate = rospy.Rate(self.freq)					# Sets transmit frequency
		self.robot_broadcaster = tf.TransformBroadcaster()	# Transform for robot pose
		# self.lidar_broadcaster = tf.TransformBroadcaster()	# Transform for robot pose
		
		self.start()

	def start(self):
		##***************** SUBSCRIBERS ***************##
		# subscribe to gazebo
		if rospy.has_param('/gazebo/auto_disable_bodies'):
			self.robot_state_sub_ = rospy.Subscriber("/gazebo/model_states", ModelStates, self.gz_model_callback)

		# subscribe to state estimator
		else:
			self.robot_state_sub_ = rospy.Subscriber("/corin/model_state", PoseStamped, self.ac_model_callback)

		##***************** PUBLISHERS ***************##


	## Transform from gazebo robot_pose to TF for RVIZ visualization
	def gz_model_callback(self, robot_state):
		rs_size = len(robot_state.name)

		for i in range(0,rs_size):
			if (robot_state.name[i]=='corin'):
				px = robot_state.pose[i].position.x
				py = robot_state.pose[i].position.y
				pz = robot_state.pose[i].position.z

				rx = robot_state.pose[i].orientation.x
				ry = robot_state.pose[i].orientation.y
				rz = robot_state.pose[i].orientation.z
				rw = robot_state.pose[i].orientation.w
				print (px,py,pz)
				self.robot_broadcaster.sendTransform( (px,py,pz), (rx,ry,rz,rw), rospy.Time.now(), "trunk","world")

	def ac_model_callback(self, msg):

		self.robot_broadcaster.sendTransform( (msg.position.x,msg.position.y,msg.position.z), 
												(msg.orientation.x,
													msg.orientation.y,
													msg.orientation.z,
													msg.orientation.w), rospy.Time.now(), "trunk","world") ;


if __name__ == "__main__":
	n = Node_class()

	rospy.loginfo('Robot publisher initiated') 

	rospy.spin()
