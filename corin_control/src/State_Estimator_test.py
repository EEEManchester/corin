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
		#---------------------------------------------------------------------#
		# Tuning parameters                                                   #
		#---------------------------------------------------------------------#
		self.Qf = 0.1#10**-7.02#1E-6         # (0.00025**2)* # Noise power density (m/s2 / sqrt(Hz))
		self.Qbf = 1E-7#10**-7.15#1E-6       # 9

		self.Qw = 0.01#10**-5.8#-5.6#1E-7
		self.Qbw = 2E-11#10**-13.3#-2.2#1E-8          # 10

		self.Qp = 3E-6#10**-6.76#-4.3#1E-5
		self.Qp_xy = 1E-5
		self.Qp_z = 1E-5

		self.Rs = 2E-3 #10**-3.356#1E-3
		self.Ra = 3E-4#3E-4 # 0.0003

		self.force_threshold = 2.7#2.9#3.5 #3

		rospy.init_node('IMU_estimator') 		#Initialises node

		# print quaternion_from_matrix(np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]]))
		# exit()

		self.data_source = "imu"	# ideal, imu, imu2 (model states), imu3 (link states)
		self.control_mode = "normal"
		# self.rate   = rospy.Rate(100)	# frequency
		self.reset = True
		self.T_sim = 0.0001#0.0001
		self.T_imu = 0.00515#0.002
		self.model_states = ModelStates()
		self.link_states = ModelStates()
		self.imu1 = Imu()
		self.robot 	= robot_class.RobotState()
		self.estimator = state_estimator.StateEstimator(self.robot, self.T_imu)
		# self.estimator = state_estimator_old.StateEstimator(self.robot, self.T_imu)
		self.estimator.Qf = self.Qf*np.eye(3)
		self.estimator.Qbf = self.Qbf*np.eye(3)
		self.estimator.Qw = self.Qw*np.eye(3)
		self.estimator.Qbw = self.Qbw*np.eye(3)
		self.estimator.Qp = self.Qp*np.eye(3)
		self.estimator.Rs = self.Rs*np.eye(3)
		self.estimator.Ra = self.Ra*np.eye(3)
		self.estimator.force_threshold = self.force_threshold

		self.estimator.IMU_r = 0.001*np.array([0, 0, 0])
		self.estimator.IMU_R = np.eye(3)

		# self.estimator.P[0:3, 0:3] = 0*np.eye(3)
		# self.estimator.P[3:6, 3:6] = 0*np.eye(3)
		# self.estimator.P[6:9, 6:9] = 1E-6*np.eye(3)
		# self.estimator.P[8, 8] = 0 # we known heading = 0
		# self.estimator.P[9:27, 9:27] = 1E-6*np.eye(18) # all feet
		# self.estimator.P[27:30, 27:30] = 1E-6*np.eye(3)
		# self.estimator.P[30:33, 30:33] = 1E-6*np.eye(3)

		self.counter = 0
		# self.update_i = 0
		# self.update_N = 3
		self.imu_a_z = np.array([])
		self.body_a_z = np.array([])
		self.imu_t = np.array([])
		self.body_t = np.array([])

		# self.cal_N = 10
		# self.cal_sum = np.zeros(4)
		# self.cal_i = 0.0
		# self.level = 10

		self.pub_imu_a = rospy.Publisher('imu_a', Vector3, queue_size=1)
		self.pub_imu_v = rospy.Publisher('imu_v', Vector3, queue_size=1)
		self.pub_imu_r = rospy.Publisher('imu_r', Vector3, queue_size=1)
		self.pub_imu_w = rospy.Publisher('imu_w', Vector3, queue_size=1)
		self.pub_imu_p = rospy.Publisher('imu_p', Vector3, queue_size=1)
		self.pub_imu_bf = rospy.Publisher('imu_bf', Vector3, queue_size=1)
		self.pub_imu_bw = rospy.Publisher('imu_bw', Vector3, queue_size=1)

		self.pub_body_a = rospy.Publisher('body_a', Vector3, queue_size=1)
		self.pub_body_v = rospy.Publisher('body_v', Vector3, queue_size=1)
		self.pub_body_r = rospy.Publisher('body_r', Vector3Stamped, queue_size=1)
		self.pub_body_w = rospy.Publisher('body_w', Vector3, queue_size=1)
		self.pub_body_p = rospy.Publisher('body_p', Vector3, queue_size=1)

		#self.pub_imu2_a = rospy.Publisher('imu2_a', Vector3, queue_size=1)
		#self.pub_imu2_v = rospy.Publisher('imu2_v', Vector3, queue_size=1)
		self.pub_imu2_w = rospy.Publisher('imu2_w', Vector3, queue_size=1)


		self.pub_body2_v = rospy.Publisher('body2_v', Vector3, queue_size=1)
		self.pub_body2_r = rospy.Publisher('body2_r', Vector3, queue_size=1)

		self.pub_x = rospy.Publisher('imu_x', Float64, queue_size=1)
		self.pub_y = rospy.Publisher('imu_y', Float64, queue_size=1)
		self.pub_z = rospy.Publisher('imu_z', Float64, queue_size=1)
		self.pub_pose = rospy.Publisher('IMU_pose', PoseStamped, queue_size=1)

		self.pub_update = rospy.Publisher('update', Float64, queue_size=5)

		self.pub_force_ = rospy.Publisher('force', Float32MultiArray, queue_size=1)
		self.pub_error_ = rospy.Publisher('error', Float32MultiArray, queue_size=1)

		#self.IMU_sub_  = rospy.Subscriber('/imu/data', Imu, self.Imu_callback, queue_size=5)
		if self.data_source == 'ideal':
			self.T_imu = self.T_sim
			self.estimator.T = self.T_imu
		else:
			self.IMU_sub_  = rospy.Subscriber('corin/imu/base/data', Imu, self.imu_callback, queue_size=1)

		self.IMU_sub_2  = rospy.Subscriber('corin/imu/base/data2', Imu, self.imu_callback2, queue_size=1)
		self.joint_sub_  = rospy.Subscriber('corin/joint_states', JointState, self.joint_state_callback, queue_size=5)
		self.cstate_sub_  = rospy.Subscriber('corin/contact_state', ByteMultiArray, self.contact_state_callback, queue_size=1)
		self.cforce_sub_ = rospy.Subscriber('corin/contact_force', Float32MultiArray, self.contact_force_callback, queue_size=1)
		self.model_states  = rospy.Subscriber('gazebo/model_states', ModelStates, self.model_states_callback, queue_size=1)
		self.link_states  = rospy.Subscriber('gazebo/link_states', ModelStates, self.link_states_callback, queue_size=5)


		# modified IMU values
		self.IMU_mod_pub_  = rospy.Publisher('imu_mod', Imu, queue_size=1)

		#self.IMU2_sub_  = rospy.Subscriber('corin/imu/base/data2', Imu, self.Imu2_callback, queue_size=1)

		self.p = None
		self.v = np.array([0.0, 0.0, 0.0])
		self.t = time.time()
		self.x = np.array([0.0, 0.0, 0.0])
		self.g_vec = [0.0] * 1000
		self.speed = np.zeros(3)
		self.speed2 = np.zeros(3)
		self.r = np.zeros(3)
		self.p0 = None
		self.v0 = None
		self.dp = None

		self.q0 = None
		self.body_r0 = None
		self.body_q0 = None

		self.w  = np.array([0, 0, 0])

		rospy.sleep(0.5)

	def joint_state_callback(self, msg):
		""" robot joint state callback """
		self.robot.qc = msg

	def contact_state_callback(self, msg):
		""" foot contact binary state """
		self.robot.cstate = msg.data

	def contact_force_callback(self, msg):
		""" foot contact force """
		self.robot.cforce = msg.data
		norms = []
		norms.append( np.linalg.norm(self.robot.cforce[0:3]) )
		norms.append( np.linalg.norm(self.robot.cforce[3:6]) )
		norms.append( np.linalg.norm(self.robot.cforce[6:9]) )
		norms.append( np.linalg.norm(self.robot.cforce[9:12]) )
		norms.append( np.linalg.norm(self.robot.cforce[12:15]) )
		norms.append( np.linalg.norm(self.robot.cforce[15:18]) )
		norms_msg = Float32MultiArray(data = norms)
		self.pub_force_.publish(norms_msg)

	def imu_callback2(self, msg): 		# robot joint state callback
		w = msg.angular_velocity
		w = np.array([w.x, w.y, w.z])
		a = msg.linear_acceleration
		a = np.array([a.x, a.y, a.z])
		R = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
		r = np.array([-0.1, 0.15, 0])

		dw = (w - self.w)*1000
		dw_skew = skew(dw)

		w_skew = skew(w)
		a1 = a + w_skew.dot(w_skew).dot(r) + dw_skew.dot(r)
		a1 = R.dot(a1)

		o = self.imu1.orientation
		q = np.array([o.w, o.x, o.y, o.z])
		g = np.array([0, 0, -9.80]) 		# gravity vector in the world frame
		# Rotation matrix from world to body frame
		C = quaternion_matrix_JPL(q)[0:3, 0:3]
		#a1 = C.T.dot(a) + g

		self.pub_imu2_w.publish(Vector3(*a1.tolist()))

		self.w = w

	def imu_callback(self, msg): 		# robot joint state callback

		#msg.angular_velocity.x = 0
		#msg.angular_velocity.y = 0
		#msg.angular_velocity.z = 0
		self.imu1 = copy.deepcopy(msg)
		self.robot.imu = msg


		if self.data_source == "imu":
			o = msg.orientation
			av = msg.angular_velocity
			o = np.array([o.w, o.x, o.y, o.z])
			angular_velocity = np.array([av.x, av.y, av.z])
			self.p = 0
		else:
			if self.data_source == "imu2":
				# print "model states"
				ms = self.model_states
				pose = ms.pose[1]
				twist = ms.twist[1]
			elif self.data_source == "imu3":
				# print "link states"
				ms = self.link_states
				pose = ms.pose[20]
				twist = ms.twist[20]

			p = pose.position
			o = pose.orientation # quaternion
			v = twist.linear # linear velocity

			p = np.array([p.x, p.y, p.z])
			o = np.array([o.w, o.x, o.y, o.z])
			v = np.array([v.x, v.y, v.z])
			C = quaternion_matrix_JPL(o)[0:3, 0:3]
			# print o

			msg.orientation = copy.deepcopy(pose.orientation)

			if self.p is None:
				#dp = (p - self.p0)/(self.T_sim*self.counter)
				dp = 0
				# msg.linear_acceleration = Vector3(0, 0, 0)
				# msg.angular_velocity = Vector3(0, 0, 0)
			#if self.dp is None:
			else:
				dp = (p - self.p)/(self.T_imu)
				g = np.array([0, 0, -9.80])
				#a = C.dot( (v - self.dp)/(self.T_sim*self.counter) - g )
				a = C.dot( (dp - self.dp)/(self.T_imu) - g )
				self.counter = 0
				#a = self.estimator.IMU_R.T.dot(a)
				msg.linear_acceleration = Vector3(*a.tolist())

				self.o[1:4] = -self.o[1:4] # inverse
				dq = quaternion_multiply_JPL(o, self.o)
				# print dq
				#dq = dq.round(6)	# ensure dq[0] !> 1
				phi_norm = 2 * np.arccos(dq[0])
				if phi_norm > 0:
					phi = dq[1:4] * phi_norm / np.sin(phi_norm/2)
					#print "if"
				else:
					phi = np.zeros(3)
					#print "else"
				# print phi

				w = phi/self.T_imu
				#w = self.estimator.IMU_R.T.dot(w)
				msg.angular_velocity = Vector3(*w.tolist())
				#print "model:", w

			self.o = o
			#self.dp = v
			self.dp = dp
			self.p = p

			# ADD noise
			acc = msg.linear_acceleration
							# acc.x +=  1/np.sqrt(self.T_imu) * np.random.normal(0, .1)# (1/np.sqrt(self.T_imu)) * np.random.normal(0, .01) #u, std. dev.
			# add noise
			#acc.x +=  1/np.sqrt(self.T_imu) * random.gauss(0, 1000)
			self.IMU_mod_pub_.publish(msg)


		self.estimator.estimate()

		if(self.estimator.calibrated):

			if self.q0 is None:
				self.q0 = self.estimator.q

			self.pub_imu_a.publish(Vector3(*self.estimator.a.tolist()))
			self.pub_imu_v.publish(Vector3(*(1.0*self.estimator.v).tolist()))
			self.pub_imu_r.publish(Vector3(*(1.0*self.estimator.r).tolist()))
			self.pub_imu_w.publish(Vector3(*self.estimator.w.tolist()))
			self.pub_imu_p.publish(Vector3(*self.estimator.get_fixed_angles().tolist()))
			self.pub_imu_bf.publish(Vector3(*self.estimator.bf.tolist()))
			self.pub_imu_bw.publish(Vector3(*self.estimator.bw.tolist()))

			stddev = np.sqrt(self.estimator.P[0,0])
			self.pub_y.publish(1.0*(self.estimator.r[0])+stddev)
			self.pub_x.publish(1.0*(self.estimator.r[0])-stddev)


	def model_states_callback(self, msg):
		self.model_states = msg
		if self.data_source == 'ideal':
			self.imu_callback(Imu())
		self.counter += 1
		# contains arrays of states
		# msg.name = [ground_plane, corin]
		# The following describe state of 'corin' (indx 1) w.r.t. world frame
		# msg.pose[1].position.x/y/z
		# msg.pose[1].orientation.w/y/z/w
		# msg.twist[1].linear.x/y/z ()
		# msg.twist[1].angular.x/y/z ()
		t = time.time()
		o = msg.pose[1].orientation # quaternion
		q = np.array([o.w, o.x, o.y, o.z])
		w = msg.twist[1].angular #angular velocity
		w = np.array([w.x, w.y, w.z])
		r = msg.pose[1].position
		r = np.array([r.x, r.y, r.z])
		v = msg.twist[1].linear # linear velocity
		v = np.array([v.x, v.y, v.z])
		a = (v - self.v)/self.T_sim
		self.v = v
		self.t = t
		#print "m_sta:", w


		''' # same as body v (obviously as it's the integral of the derivate)
		# negligible error in r because of non-zero v
		self.speed += self.T_sim*a*1000
		#self.pub_body2_v.publish(*self.speed.tolist())
		self.r += self.T_sim*self.speed + (self.T_sim**2/2)*a*1000
		self.pub_body2_r.publish(*self.r.tolist())
		'''

		C = quaternion_matrix_JPL(self.estimator.q)[0:3, 0:3]
		C2 = quaternion_matrix_JPL(q)[0:3, 0:3]
		#print "bod:", o
		#print "model:", C.dot(w) # approx 1kHz but variable
		#self.pub_y.publish(C.dot(w)[0])
		self.pub_z.publish(C2.dot(v)[0])
		a2 = C2.dot(a)
		a_mag = np.sqrt(np.dot(a2, a2))
		#self.pub_z.publish(a_mag)
		if self.body_q0 is None:
			if self.q0 is not None: # initialise starting position when imu loop starts

				b_q0_conj = q.copy()
				b_q0_conj[1:4] = - b_q0_conj[1:4]
				self.body_q0 = quaternion_multiply(self.q0, b_q0_conj) # Hamilton

				self.body_r0 = r.copy()

		else:

			r = r - self.body_r0

			r = quaternion_matrix(self.body_q0)[0:3,0:3].dot(r) # Hamilton

			q = quaternion_multiply(self.body_q0, q) # Hamilton


			self.pub_body_a.publish(Vector3(*a.tolist()))		# m unit
			self.pub_body_v.publish(Vector3(*(1.0*v).tolist())) # mm unit
			#print "real", 1.0*v[2]
			h = Header()
			h.stamp = rospy.Time.now()
			self.pub_body_r.publish(h, Vector3(*r.tolist())) # mm unit
			self.pub_body_w.publish(Vector3(*w.tolist()))

			angles_tuple = euler_from_quaternion_JPL(q, 'rxyz') # relative: rxyz
			angles = np.asarray(angles_tuple).tolist()

			self.pub_body_p.publish(Vector3(*angles))

			self.body_a_z = np.append(self.body_a_z, r[0])
			self.body_t = np.append(self.body_t, time.time())

	def link_states_callback(self, msg):
		self.link_states = msg

if __name__ == "__main__":

	state = CorinStateTester()

	#TODO: uncomment for real test
	rospy.spin()
	print "done"
