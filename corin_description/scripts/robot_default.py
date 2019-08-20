#!/usr/bin/env python

## Sets Gazebo Corin model to default world position in nominal standing up position

import rospy, sys, os, time
import string
import warnings
import tf
from math import pi

from gazebo_msgs.srv import *
from gazebo_ros import gazebo_interface
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class RobotPose:
	def __init__(self):
		rospy.init_node('PoseModifier') 		# Initialises node

		self.model_name = 'corin'
		self.reference_frame = 'world'
		self.joint_pub_ = {}

		self.initialise_topics()

	def initialise_topics(self):

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

		rospy.sleep(0.5)

	def set_full_pose(self, qb, q):
		
		self.set_leg_pose(q)
		self.set_body_pose(qb)
		self.set_leg_pose(q)

		print 'Robot Pose Set!'

	def set_leg_pose(self, q):
		# Publish joint states
		for c in range(0,3):
			for j in range(0,18):
				self.joint_pub_[j].publish(q[j])
			rospy.sleep(0.4) 	

	def set_body_pose(self, qb):
		# Define variables
		model_state = ModelState()
		pose  		= Pose()
		twist 		= Twist()

		model_name 		= 'corin'
		reference_frame	= 'world'
		
		pose.position.x		= qb[0]#pose_x
		pose.position.y		= qb[1]#pose_y
		pose.position.z		= qb[2]#pose_z
		roll 				= qb[3]#pose_roll
		pitch 				= qb[4]#pose_pitch
		yaw 				= qb[5]#pose_yaw
		quaternion 			= tf.transformations.quaternion_from_euler(roll, pitch, yaw)
		pose.orientation.x	= quaternion[0]
		pose.orientation.y	= quaternion[1]
		pose.orientation.z	= quaternion[2]
		pose.orientation.w	= quaternion[3]

		model_state.model_name 		= model_name
		model_state.pose 			= pose
		model_state.reference_frame = reference_frame

		gazebo_namespace = '/gazebo'

		try:
			rospy.wait_for_service('%s/set_model_state'%(gazebo_namespace), 2)
			set_model_state = rospy.ServiceProxy('%s/set_model_state'%(gazebo_namespace), SetModelState)
			# rospy.loginfo("Calling service %s/set_model_state"%gazebo_namespace)
			resp = set_model_state(model_state)
			# rospy.loginfo("Set model state status: %s"%resp.status_message)
			print resp.status_message

		except rospy.ServiceException as e:
			print("Service call failed: %s" % e)
		except rospy.ROSException as e:
			# Gazebo inactive
			print("Service call failed: %s" % e)

if __name__ == "__main__":

	r_ = RobotPose()

	# Default pose
	pose_x		= 0.0
	pose_y		= 0.
	pose_z		= 0.15
	pose_roll 	= 0.
	pose_pitch 	= 0.
	pose_yaw 	= 0.
	qb = [pose_x, pose_y, pose_z, pose_roll, pose_pitch, pose_yaw]
	q1 = 0.0
	q2 = 0.34
	q3 = -1.85
	qd = [0]*18

	for j in range(0,6):
		qd[j*3+0] = q1
		qd[j*3+1] = q2
		qd[j*3+2] = q3

	if len(sys.argv) > 1:
		if sys.argv[1] == 'fault':
			qd[13] = 0.614#0.2678
			qd[14] = -2.404#-2.212
			# qd[10] = 1.0
			# qd[16] = 1.0

		elif sys.argv[1] == 'chimney':
			# path width 0.72m
			qd = [0.6981317007977319, 0.7953988301841435, -1.590797660368287, 0.0, 0.7953988301841435, -1.590797660368287, 
				 -0.6981317007977319, 0.7953988301841435, -1.590797660368287, -0.6981317007977319, 0.7953988301841435, -1.590797660368287, 
				  0.0, 0.7953988301841435, -1.590797660368287, 0.6981317007977319, 0.7953988301841435, -1.590797660368287]
			# path width 0.63m
			qd = [ 0.6981317, 1.00826008, -2.01652016,  0.,          1.00826008, -2.01652016,
				 -0.6981317, 1.00826008, -2.01652016, -0.6981317,   1.00826008, -2.01652016,
				  0.       , 1.00826008, -2.01652016,  0.6981317,   1.00826008, -2.01652016]
			qd = [ 0.6981317,   0.96834169, -1.93668339,  0.,          0.96834169, -1.93668339,
				  -0.6981317,   0.96834169, -1.93668339, -0.6981317,   0.96834169, -1.93668339,
				   0.       ,   0.96834169, -1.93668339,  0.6981317,   0.96834169, -1.93668339]

			qb[0] = 1.0
			r_.set_full_pose(qb, qd)
			rospy.sleep(1.0)
			qb[0] = 0.0
			
		elif sys.argv[1] == 'narrow':
			qd = [0.698125654154321, 0.3504038976152293, -1.8613961429267607, 
					-5.055311231849657e-06, 0.35041520913878266, -1.8614356008819217, 
					-0.6981367983856765, 0.3504334946312726, -1.861375759204364, 
					-0.6981360860479988, 0.35040176334250006, -1.8614423083962612, 
					-4.6744877684190556e-06, 0.35041726156598063, -1.86139044270324, 
					0.6981272563163854, 0.350433456487087, -1.8613747680378978]

		elif sys.argv[1] == 'wall':
			pose_x		= 0.0
			pose_y		= 0.16
			pose_z		= 0.38
			pose_roll 	= 1.22
			pose_pitch 	= 0.
			pose_yaw 	= 0.
			qb = [pose_x, pose_y, pose_z, pose_roll, pose_pitch, pose_yaw]
			qd = [	0.6981317,  0.6939246 , -2.03135032, 
					0.       ,  0.6939246 , -2.03135032,
				  	-0.6981317, 0.6939246 , -2.03135032, 
					# -0.6981317,  2.5, -2.,
					-0.6981317, 0.81969609, -1.22860141,
				  	0.       ,  0.81969609, -1.22860141, 
					0.6981317,  0.81969609, -1.22860141]

	r_.set_full_pose(qb, qd)
	