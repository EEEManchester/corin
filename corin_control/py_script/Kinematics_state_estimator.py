#!/usr/bin/env python

## State estimation for the robot
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'library'))
sys.dont_write_bytecode = True

import time
import warnings
import numpy as np

## Personal libraries
from constant import *
import control_interface 								# action selection from ROS parameter server
import robot_class 										# class for robot states and function
#import transformations as tf 							# SE(3) transformation library

## ROS messages
import rospy
from sensor_msgs.msg import Imu 						# sub msg for IMU
from sensor_msgs.msg import JointState 					# sub msg for joint states
from trajectory_msgs.msg import JointTrajectoryPoint	# pub msg for Robotis joints
from std_msgs.msg import Float64 						# pub msg for Gazebo joints
from std_msgs.msg import ByteMultiArray 				# foot contact state
from std_msgs.msg import Float32MultiArray				# foot contact force
from geometry_msgs.msg import Vector3
from gazebo_msgs.msg import ModelStates

## Robotis ROS msgs for joint control
from robotis_controller_msgs.msg import SyncWriteItem 	# pub msg for Robotis joints
from robotis_controller_msgs.msg import SyncWriteMulti 	# pub msg for Robotis joints


## State estimation definitions
START = 0
TRIPOD_1 = 1
TRIPOD_2 = 2

class CorinState:
	def __init__(self):
		rospy.init_node('state_estimator') 		#Initialises node
		self.tripod_1 = None
		self.stepping = None

		self.corin_state = None

		self.robot_ns = ROBOT_NS
		self.rate 	  = rospy.Rate(CTR_RATE)	# frequency

		self.Robot 	  = robot_class.RobotState() 		# robot class

		self.interface = 'gazebo'

		if (self.interface == 'gazebo'):
			self.joint_sub_  = rospy.Subscriber(self.robot_ns + '/joint_states', JointState, self.joint_state_callback, queue_size=5)
		elif (self.interface == 'robotis'):
			self.joint_sub_  = rospy.Subscriber('robotis/present_joint_states', JointState, self.joint_state_callback, queue_size=5)

#`		self.joint_sub_  = rospy.Subscriber('gazebo/model_states', ModelStates, self.joint_state_callback, queue_size=1)

		self.pub_ = rospy.Publisher('planarity', Float64, queue_size=1)
		self.pub_2 = rospy.Publisher('stance', Float64, queue_size=1)
		self.pub_3 = rospy.Publisher('T1_z', Float64, queue_size=1)
		self.pub_4 = rospy.Publisher('T2_z', Float64, queue_size=1)
		self.pub_robot_pose = rospy.Publisher('robot_pose', Vector3, queue_size=1)

		# update and reset states
		self.Robot.reset_state = True
		self.Robot.suspend = False

		rospy.sleep(1)

		self.Robot.update_state()

		np.set_printoptions(suppress=True) # suppress scientific notation
		np.set_printoptions(precision=5) # number display precision

	def corin_state_callback(self, msg):
		self.corin_state = msg

	def joint_state_callback(self, msg): 		# robot joint state callback
		self.Robot.qc = msg
		self.Robot.update_state()

		# list of foot positions
		s = []
		for j in range(0,self.Robot.active_legs):
			#print(j, self.Robot.Leg[j].base_X_ee.cs.xp)
			s.append( self.Robot.Leg[j].XHc.base_X_foot[:3,3] )
			#print(s[j])

		# rotation matrix from tripod frame 1 to body frame
		R1 = gram_schmidt(s[4]-s[0], s[2]-s[0])
		T_temp = np.hstack(( R1, s[0].reshape((3,1)) ))
		# homogenous transformation from tripod frame 1 to body frame
		T1 = np.vstack(( T_temp, np.array([[0, 0, 0, 1]]) ))
		self.pub_3.publish(s[0][2])

		R2 = gram_schmidt(s[1]-s[3], s[5]-s[3])
		T_temp = np.hstack(( R2, s[3].reshape((3,1)) ))
		T2 = np.vstack(( T_temp, np.array([[0, 0, 0, 1]]) ))
		self.pub_4.publish(s[3][2]-s[0][2])
		#print ("point:", np.dot(T1, np.array([0, 0.2, 0.05, 1]).reshape(4, 1)))

		current_T = np.zeros((4,4))

		height = s[3][2]-s[0][2]
		print(height)

		# We know nothing yet of Corin's status
		if self.stepping is None:
			#self.corin_pose_0 = self.corin_state.pose[1].position # Store robot's initial position
			self.stepping = abs(height) > 0.01 # Deterimine if stationary or stepping
			# If Corin is stepping, we know which tripod to use
			if self.stepping:
				self.tripod_1 = height > 0
				# Initialise body state to use current tripod frame as world frame
				self.body_state = T1 if self.tripod_1 else T2
			else:
				# Store current tripod frames for when Corin starts stepping
				# because we don't know which tripod will be used first
				self.T1 = T1
				self.T2 = T2
				self.body_state = np.eye(4)
		elif self.stepping: # We know the tripod at this stage
			# if height reverses sign (+ -> - or vice versa), the legs must
			# be on the same plane, i.e. not stepping
			if (self.tripod_1 and height < 0) or (not self.tripod_1 and height > 0):
				self.stepping = False
				# Store frames as described above
				self.T1 = T1
				self.T2 = T2
		else: # self.stepping == False
			# move to new state if legs start to move
			if abs(height) > 0.01:
				self.stepping = True
				# Calculate transformation from body frame to previous tripod frame
				if self.tripod_1 is None:
					oldInvT = np.eye(4)
				else:
					oldInvT = np.linalg.inv(T1) if self.tripod_1 else np.linalg.inv(T2) #TODO why not self.T1/self.T2
				# Evaluate new tripod frame
				self.tripod_1 = height > 0
				newT = self.T1 if self.tripod_1 else self.T2
				# Calculate transformation from new tripod to previous tripod frame
				tripod_T = np.dot(oldInvT, newT)
				# Add transformation to world frame
				self.body_state = np.dot(self.body_state, tripod_T)

		# Display transformation from body frame to world frame
		stance = "Stepping" if self.stepping else "Idle"
		if self.tripod_1 is None:
			tripod = "unknown"
			invT = np.eye(4)
		else:
			tripod = "1" if self.tripod_1 else "2"
			invT = np.linalg.inv(T1) if self.tripod_1 else np.linalg.inv(T2)
		print(stance + ": TRIPOD_" + tripod)
		current_T = np.dot(self.body_state, invT)
		pose_estimate = current_T[:,3]
		print(pose_estimate)

		temp = Vector3()
		temp.x = pose_estimate[0]
		temp.y = pose_estimate[1]
		temp.z = pose_estimate[2]

		self.pub_robot_pose.publish(temp)



# Return a frame with   x-axis along e1,
#                       y-axis in the plane containing e1 and e2
#                       z-axis is obvious
def gram_schmidt(e1, e2):
	# simply
	q1 = e1
	# projection of e2 along e1
	proj_vec = (np.dot(e1, e2) / np.dot(e1, e1)) * e1
	# subtract the projection from e2 to get a vector normal to e1
	q2 = e2 - proj_vec
	# obviously
	q3 = np.cross(q1, q2)

	# convert to column vector and normalise
	q1 = q1.reshape((3,1)) / np.linalg.norm(q1)
	q2 = q2.reshape((3,1)) / np.linalg.norm(q2)
	q3 = q3.reshape((3,1)) / np.linalg.norm(q3)

	#return q1, q2, q3
	return np.hstack((q1, q2, q3))

if __name__ == "__main__":

	state = CorinState()

	rospy.spin()
