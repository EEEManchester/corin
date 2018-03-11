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
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu

from gazebo_msgs.msg import ModelStates

from constant import *

class Node_class:
	def __init__(self):
		rospy.init_node("State_publisher")							# Initiates Ros node
		self.freq = 80
		self.rate = rospy.Rate(self.freq)					# Sets transmit frequency
		self.robot_broadcaster = tf.TransformBroadcaster()	# Transform for robot pose
		self.lidar_broadcaster = tf.TransformBroadcaster()	# Transform for robot pose
		self.jointState  = JointState()

		self.start()

	def start(self):
		##***************** SUBSCRIBERS ***************##
		# subscribe to gazebo
		if rospy.has_param('/gazebo/auto_disable_bodies'):
			self.robot_state_sub_ = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_callback)

		# subscribe to hardware IMU
		else:
			self.imu_sub_ 		  = rospy.Subscriber("/imu_data", Imu, self.imu_callback)

		##***************** PUBLISHERS ***************##


	## Transform from gazebo robot_pose to TF for RVIZ visualization
	def model_callback(self, robot_state):
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

				self.robot_broadcaster.sendTransform( (px,py,pz), (rx,ry,rz,rw), rospy.Time.now(), "trunk", "base_link") ;
				self.lidar_broadcaster.sendTransform( (px,py,pz), (rx,ry,rz,rw), rospy.Time.now(), "trunk_lidar", "trunk") ;


	def imu_callback(self, imu):
	    print imu.orientation.x, imu.orientation.y, imu.orientation.z
	    self.robot_broadcaster.sendTransform( (0.0,0.0,0.1), tf.transformations.quaternion_from_euler(imu.orientation.y, imu.orientation.x, -imu.orientation.z),
	    							rospy.Time.now(), "trunk", "base_link") ;


if __name__ == "__main__":
	n = Node_class()

	print 'Robot publisher initiated'

	rospy.spin()
