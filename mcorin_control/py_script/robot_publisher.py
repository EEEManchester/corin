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
		self.imu_sub 	= rospy.Subscriber("/imu_data", Imu, self.imu_callback)
		self.joint_sub 	= rospy.Subscriber(ROBOT_NS + "/joint_states", JointState, self.joint_state_callback) 	# temp not needed

		self.joint_pub  = rospy.Publisher(ROBOT_NS + "/joint_states_new", JointState, queue_size=1)

	def joint_state_callback(self,joint_state):
		self.jointState = joint_state
		qname = [None] * 18
		for i in range(0,18):
			qname[i] = JOINT_TOPICS[int(self.jointState.name[i])-1]

		new_msg = JointState()
		new_msg.name = qname
		new_msg.position = joint_state.position
		new_msg.velocity = joint_state.velocity
		new_msg.effort = joint_state.effort

		self.joint_pub.publish(new_msg)

	def imu_callback(self, imu):
	    print imu.orientation.x, imu.orientation.y, imu.orientation.z
	    self.broadcaster.sendTransform( (0.0,0.0,0.5), tf.transformations.quaternion_from_euler(imu.orientation.x, imu.orientation.y, imu.orientation.z),
	    							rospy.Time.now(), "trunk", "base_link") ;


if __name__ == "__main__":
	n = Node_class()

	print "Initiation Complete","     Publish Time: ",float(1)/float(n.freq)

	n.start()

	rospy.spin()
