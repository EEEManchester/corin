#!/usr/bin/env python

## Sets Gazebo Corin model to default world position in nominal standing up position

import rospy, sys, os, time
import string
import warnings
import tf
from math import pi
# import copy
import numpy as np

from gazebo_msgs.srv import *
from gazebo_ros import gazebo_interface
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

def deg2rad(q):
	return (q*pi/180)

JOINT_NAME 	  = [None]*18
JOINT_NAME[0] = 'lf_q1_joint'
JOINT_NAME[1] = 'lf_q2_joint'
JOINT_NAME[2] = 'lf_q3_joint'
JOINT_NAME[3] = 'lm_q1_joint'
JOINT_NAME[4] = 'lm_q2_joint'
JOINT_NAME[5] = 'lm_q3_joint'
JOINT_NAME[6] = 'lr_q1_joint'
JOINT_NAME[7] = 'lr_q2_joint'
JOINT_NAME[8] = 'lr_q3_joint'
JOINT_NAME[9] = 'rf_q1_joint'
JOINT_NAME[10] = 'rf_q2_joint'
JOINT_NAME[11] = 'rf_q3_joint'
JOINT_NAME[12] = 'rm_q1_joint'
JOINT_NAME[13] = 'rm_q2_joint'
JOINT_NAME[14] = 'rm_q3_joint'
JOINT_NAME[15] = 'rr_q1_joint'
JOINT_NAME[16] = 'rr_q2_joint'
JOINT_NAME[17] = 'rr_q3_joint'

if __name__ == "__main__":

	rospy.init_node('initialise_node') 		#Initialises node

	model_name = 'corin'

	joint_pub_ = rospy.Publisher(model_name + '/joint_states', JointState, queue_size=1)

	dqp = JointState()
	dqp.header.stamp = rospy.Time.now()

	# joint name
	for n in range(18):
		dqp.name.append(str(JOINT_NAME[n])) 	

	# joint angles
	q = [0.0, 0.34, -1.85]
	q = [0.0, 0.93, -1.85]
	for j in range(0,6): 	# loop for each leg
		for k in range(0,3):	
			dqp.position.append(q[k])			# joint angle
	# dqp.position[0] = deg2rad(40.)
	# dqp.position[15] = deg2rad(40.)
	# dqp.position[0] = deg2rad(40.)
	# dqp.position[6] = deg2rad(-40.)
	# dqp.position[9] = deg2rad(-40.)

	qb = np.array([0., 0., 0., 0., 0., 0.])
	quat = tf.transformations.quaternion_from_euler(qb[3].copy(), qb[4].copy(), qb[5].copy())
	robot_broadcaster = tf.TransformBroadcaster()

	for i in range(3):
		robot_broadcaster.sendTransform( (qb[0],qb[1],qb[2]), quat, rospy.Time.now(), "trunk", "world");
		joint_pub_.publish(dqp)
		rospy.sleep(1.0)