#!/usr/bin/env python

## Publishes state for RVIZ
import rospy
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'library'))
sys.dont_write_bytecode = True

import time
from fractions import Fraction
import numpy as np

import tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu

from constant import *

class Node_class:
	def __init__(self):
		rospy.init_node("State_publisher")							# Initiates Ros node
		self.freq = 80
		self.rate = rospy.Rate(self.freq)							# Sets transmit frequency
		self.broadcaster = tf.TransformBroadcaster()
		self.jointState  = JointState()

	def start(self):
		self.imu_sub 	= rospy.Subscriber("/imu/data", Imu, self.imu_callback)
		self.joint_sub 	= rospy.Subscriber(ROBOT_NS + "/joint_states", JointState, self.joint_state_callback)

		## Synchronize topics
		# imu_sub = message_filters.Subscriber('/imu_data', Imu)
		# joint_sub = message_filters.Subscriber('/robot_joint_states', JointState)

		# ts = message_filters.TimeSynchronizer([imu_sub, joint_sub], 10)
		# ts.registerCallback(n.sync_callback)

	def joint_state_callback(self,joint_state):
		self.jointState = joint_state

	def imu_callback(self, imu):
	    print imu.orientation.x, imu.orientation.y, imu.orientation.z 
	    self.broadcaster.sendTransform( (0.0,0.0,0.5), tf.transformations.quaternion_from_euler(imu.orientation.x, imu.orientation.y, imu.orientation.z),
	    							rospy.Time.now(), "trunk", "base_link") ;
	    

	# def sync_callback(self, imu, joint_state):
	# 	# broadcast odometry transform (XYZ, RPY)
	# 	broadcaster.sendTransform( (0.0,0.0,0.5), tf.transformations.quaternion_from_euler(imu.orientation.x, imu.orientation.y, imu.orientation.z),
	#     							rospy.Time.now(), "body", "body_dummy") ;
	# 	# broadcast joint transformation
	# 	self.jointState = joint_state
	# 	self.joint_pub.publish(self.jointState)

if __name__ == "__main__":
	n = Node_class()

	print "Initiation Complete","     Publish Time: ",float(1)/float(n.freq)

	n.start()

	rospy.spin()
	