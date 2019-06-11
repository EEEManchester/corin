#!/usr/bin/env python

## Sets Gazebo Corin model to default world position in nominal standing up position

import rospy, sys, os, time
import string
import warnings
import tf
from math import pi
# import copy
import numpy as np
from fault_controller import *
from robot_class import *

from gazebo_msgs.srv import *
from gazebo_ros import gazebo_interface
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

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

class RVIZ:
	def __init__(self):
		rospy.init_node('rviz_node')
		
		# self.Fault = FaultController()
		# self.Robot = RobotState()
		self.joint_pub_ = rospy.Publisher(ROBOT_NS + '/joint_states', JointState, queue_size=1)

	def publish_robot(self, qb, q):

		print 'Setting to new state'
		quat = tf.transformations.quaternion_from_euler(qb[3].copy(), qb[4].copy(), qb[5].copy())
		robot_broadcaster = tf.TransformBroadcaster()

		dqp = JointState()
		dqp.header.stamp = rospy.Time.now()
		dqp.name = JOINT_NAME
		dqp.position = q

		for i in range(5):
			robot_broadcaster.sendTransform( (qb[0],qb[1],qb[2]), quat, rospy.Time.now(), "trunk", "world");
			self.joint_pub_.publish(dqp)
			rospy.sleep(1.0)

if __name__ == "__main__":

	RVIZ = RVIZ()
	Robot = RobotState()
	Robot.Fault.status = True
	Robot.Fault.fault_index[0] = True
	Robot.Fault.fault_index[2] = True
	# Robot.Fault.fault_index[5] = True

	Robot.Fault.base_X_foot[0] = np.array([ 0.25,  0.251, -0.15])
	Robot.Fault.base_X_foot[1] = np.array([ 0.,    0.300, -BODY_HEIGHT])
	Robot.Fault.base_X_foot[2] = np.array([-0.25,  0.251, -0.12])
	Robot.Fault.base_X_foot[3] = np.array([ 0.25, -0.251, -BODY_HEIGHT])
	Robot.Fault.base_X_foot[4] = np.array([ 0.,   -0.300, -BODY_HEIGHT])
	Robot.Fault.base_X_foot[5] = np.array([-0.25, -0.251, -0.07])

	xb = Robot.Fault.get_fault_pose().flatten()
	
	# Offset for convenience
	xb[0] = 0.3
	xb[1] = 0.39

	# Update robot's base
	Robot.XHd.update_world_X_base(xb)
	
	# Update leg configurations 
	for j in range(0,6):
		# Fault leg - use fault configuration
		if Robot.Fault.fault_index[j] == True:
			Robot.Leg[j].XHd.coxa_X_foot = mX(Robot.Leg[j].XHd.coxa_X_base, v3_X_m(Robot.Fault.base_X_foot[j]))

		# Working leg - update foot position
		else:
			# Propogate base offset to legs
			Robot.Leg[j].XHd.world_X_foot[0,3] += xb[0]
			Robot.Leg[j].XHd.world_X_foot[1,3] += xb[1]
			
			Robot.Leg[j].XHd.coxa_X_foot = mX(Robot.Leg[j].XHd.coxa_X_base, 
												Robot.XHd.base_X_world, 
												Robot.Leg[j].XHd.world_X_foot)
			# Robot.Leg[j].XHd.coxa_X_foot = mX(Robot.Leg[j].XHd.coxa_X_base, Robot.Leg[j].XHd.base_X_foot)

	qd, tXj_error = Robot.task_X_joint()
	
	if (Robot.invalid == True):
		print 'Error Occured, robot invalid! ', tXj_error
	else:
		RVIZ.publish_robot(xb, qd.xp)
