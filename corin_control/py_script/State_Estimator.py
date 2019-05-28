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
from library import *			# library modules to include
from constant import *
import control_interface 								# action selection from ROS parameter server
import robot_class 										# class for robot states and function
from termcolor import colored

class StateEstimator:

	def __init__(self, robot, T):
		self.robot = robot	# reference to owner: instance of Robot
		self.T = T # sampling time

		self.N = 34 	# number of states
		self.Ne = 33 	# number of error states

		# Define state vector components
		self.a = np.zeros(3)
		self.o = np.array([1, 0, 0, 0])
		self.v2 = np.zeros(3)
		self.r = np.zeros(3)	# body position in world frame (inertial frame)
		self.v = np.zeros(3)	# body velocity in world frame
		# body quaternion mapping vectors from world to body frame (body frame rotated -90)
		self.q = quaternion_from_matrix_JPL(np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])) #
		self.q2 = quaternion_from_matrix(np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]))
		# print self.q
		# print self.q2

		self.y = np.zeros(18) # feet position errors

		# self.R1 = np.array([[1, 0, 0, 4],
		# 				 	[0, 1, 0, 8],
		# 					[0, 0, 1, 27],
		# 					[0, 0, 0, 1]])
		#
		# self.R2 = np.array([[-1, 0, 0, -123],
		# 					[0, 1, 0, -5.25],
		# 					[0, 0, -1, 24.5],
		# 					[0, 0, 0, 1]])

		self.IMU_r = np.array([-123, 5.25, 24.5]) # Body frame origin relative to IMU frame, described in the IMU frame
		self.IMU_R = np.array([[-1, 0, 0],
								[0, 1, 0],
								[0, 0, -1]])	# Rotation matrix mapping vectors from IMU frame to body frame
		self.IMU_r = np.zeros(3)
		self.IMU_R = np.eye(3)

		# self.IMU_r = np.array([-0.1, 0.2, 0])
		# self.IMU_R = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])

		self.IMU_q = quaternion_from_matrix_JPL(self.IMU_R) # Quaternion corresponding to self.IMU_R
		self.w1 = np.zeros(3) # Initial angular velocity of IMU

		self.p0 = np.zeros(3)	# foot contact points 0 to 5 in world frame
		self.p1 = np.zeros(3)
		self.p2 = np.zeros(3)
		self.p3 = np.zeros(3)
		self.p4 = np.zeros(3)
		self.p5 = np.zeros(3)
		self.bf = np.zeros(3)	# accelerometer bias in body frame
		self.bw = np.zeros(3)	# gyroscope bias in body frame

		# define state vector
		#self.x = [r, v, q, p1, p2, p3, p4, p5, p6, bf, bw]

		# Define state error vector components
		self.dr = np.zeros(3)
		self.dv = np.zeros(3)
		self.dphi = np.zeros(3)	# body quaternion from world to body frame
		self.dp0 = np.zeros(3)
		self.dp1 = np.zeros(3)
		self.dp2 = np.zeros(3)
		self.dp3 = np.zeros(3)
		self.dp4 = np.zeros(3)
		self.dp5 = np.zeros(3)
		self.dbf = np.zeros(3)	# accelerometer bias in body frame
		self.dbw = np.zeros(3)	# gyroscope bias in body frame

		# State error cova<origin xyz="0 0 0.0" rpy="0 0 0"/> riance matrix
		self.P = np.zeros((self.Ne, self.Ne))


		# Process noise covariances: Qp, Qf, Qbf, Qw, Qbw

		# Sensor noise covariance matrices
		self.Qf = (0.026831**2)*np.eye(3)		# accelerometer noise covariance matrix
		self.Qbf = (0.01**2)*np.eye(3) 	# accelerometer bias derivative noise covariance matrix
		self.Qw = (0.002428**2)*np.eye(3) 	# gyroscope noise covariance matrix
		self.Qbw = (0.000000**2)*np.eye(3) 	# gyroscope bias derivative noise covariance matrix

		self.Qf = (0.001)*np.eye(3)#*np.sqrt(self.T)		# accelerometer noise covariance matrix
		# self.Qf[0,0] = 1000000 #.01
		self.Qbf = 0*(0.001)*np.eye(3) 	# accelerometer bias derivative noise covariance matrix
		self.Qw = 0*(0.001**1)*np.eye(3)#*np.sqrt(self.T) 	# gyroscope noise covariance matrix
		self.Qbw = 0*(0.001)*np.eye(3) 	# gyroscope bias derivative noise covariance matrix

		# Foot position noise covariance matrix
		# 1) This accounts for foot slippage in x,y,z directions in body frame
		# 2) Set to infinity when foot not in contact, because the position is no longer known
		self.Qp = 1*np.diagflat([0.001, 0.001, 0.001])	# [<dx> m, <dy> m, <dz> m]
		# self.Qp1 = np.diagflat([1E-5, 1E-5, 1E-6]) # 559
		# self.Qp2 = np.diagflat([1E-5, 1E-6, 1E-5]) # 595

		# Measurement noise covariances: Ra, Rs

		# Kinematic model error covariance matrix in the body frame (x, y, z)
		self.Rs = 0.0000001*np.eye(3) # metres
		# Joint angle covariance matrix
		self.Ra = 0.0004*np.eye(3) # rad^2  (1 deg = 0.02 rad) #0 before

		self.threshold = [0] * 6

		self.force_threshold = 5

	# Combine all states to produce state vector
	def get_state_vector(self):
		x = np.zeros(self.N)
		x[0:3] = self.r
		x[3:6] = self.v
		x[6:10] = self.q
		x[10:13] = self.p0
		x[13:16] = self.p1
		x[16:19] = self.p2
		x[19:22] = self.p3
		x[22:25] = self.p4
		x[25:28] = self.p5
		x[28:31] = self.bf
		x[31:34] = self.bw
		return x

	# Map state vector onto individual states
	def set_states(self, state_vector):
		pass

	# Construct and return error state vector
	def get_dx(self):
		dx = np.zeros(self.Ne)
		dx[0:3] = self.dr
		dx[3:6] = self.dv
		dx[6:9] = self.dphi
		dx[9:12] = self.dp0
		dx[12:15] = self.dp1
		dx[15:18] = self.dp2
		dx[18:21] = self.dp3
		dx[21:24] = self.dp4
		dx[24:27] = self.dp5
		dx[27:30] = self.dbf
		dx[30:33] = self.dbw
		return dx

	# Get ith foot position
	def get_p(self, index):
		return [self.p0, self.p1, self.p2, self.p3, self.p4, self.p5][index]

	def set_p(self, index, value):
		p = [self.p0, self.p1, self.p2, self.p3, self.p4, self.p5][index]
		p[0:3] = value	# replace the values in the array to which p (p1/../p6) points

	# Map error state vector to individual error vectors
	def set_dx(self, dx):
		self.dr  = dx[0:3]
		self.dv  = dx[3:6]
		self.dphi= dx[6:9]
		self.dp0 = dx[9:12]
		self.dp1 = dx[12:15]
		self.dp2 = dx[15:18]
		self.dp3 = dx[18:21]
		self.dp4 = dx[21:24]
		self.dp5 = dx[24:27]
		self.dbf = dx[27:30]
		self.dbw = dx[30:33]

	def get_fixed_angles(self):
		# euler angles of rotation that maps vectors from world frame to body frame
		angles_tuple = euler_from_quaternion_JPL(self.q, 'rxyz') # relative: rxyz
		return np.asarray(angles_tuple)#*180.0/np.pi

	def get_fixed_angles2(self):
		# euler angles of rotation that maps vectors from body frame to world frame
		angles_tuple = euler_from_quaternion(self.q2, 'rxyz') # relative: rxyz
		return np.asarray(angles_tuple)#*180.0/np.pi

	def get_o_fixed_angles(self):
		angles_tuple = euler_from_quaternion_JPL(self.o, 'rxyz') # relative: rxyz
		return np.asarray(angles_tuple)#*180.0/np.pi

#	def set_q_using_IMU(self, IMU_q):


	def reset_foot_positions(self):
		if self.robot.qc is not None:
			C = quaternion_matrix_JPL(self.q)[0:3, 0:3]
			self.robot.update_state() # sets the new foot positions
			for j in range(0,self.robot.active_legs):
				s = self.robot.Leg[j].XHc.base_X_foot[:3,3]
				self.set_p(j, C.T.dot(s)) # s - C.dot(p-r) should be zero
			return True
		else:
			return False

	# implements Kalman Filter state/error-state prediction upon receiving IMU data
	def predict_state(self):

		T = self.T					# sampling time (s)

		imu = self.robot.imu			# update imu values
		a1 = imu.linear_acceleration
		w1 = imu.angular_velocity
		o1 = imu.orientation			# output of Kalman filter on board the IMU

		a1 = np.array([a1.x, a1.y, a1.z])
		w1 = np.array([w1.x, w1.y, w1.z])
		w1_skew = skew(w1)
		o1 = np.array([o1.w, o1.x, o1.y, o1.z])

		dw1 = (w1 - self.w1)/T		# Angular acceleration
		self.w1 = w1
		dw1_skew = skew(dw1)

		# Map angular velocity (with respect to world frame) from IMU frame to body frame
		# Angular velocity of the two frames is the same (if described in the same frame)
		# since they are on a rigid body
		w = self.IMU_R.dot(w1)

		q0 = quaternion_from_matrix_JPL(self.IMU_R) # Rotates points from IMU to base (body)
		o = quaternion_multiply_JPL(q0, o1)

		# Map acceleration (with respect to world frame) from IMU frame to body frame
		a = a1 + w1_skew.dot(w1_skew).dot(self.IMU_r) + dw1_skew.dot(self.IMU_r)
		a = self.IMU_R.dot(a)

		# acceleration without bias, in the body frame
		f = a - self.bf
		f_skew = skew(f) 				# in matrix_transforms.py

		# angular velocity without bias, in the body frame
		w = w - self.bw

		g = np.array([0, 0, -9.80]) 		# gravity vector in the world frame

		# IMPORTANT
		# quaternion in paper (JPL convention): (i, j, k, real)
		# quaternion default in transformations.py (Hamilton): (real, i, j, k)
		# used here modified JPL: (real, i, j, k)

		# Rotation matrix from world to body frame
		C = quaternion_matrix_JPL(self.q)[0:3, 0:3]
		#print "f:", f
		#print "dv:", C.T.dot(f)
		# Nominal state update (independent of Kalman Filter)
		self.a = C.T.dot(f) + g
		self.v = self.v + T*self.a
		self.r = self.r + T*self.v + (T**2/2) * self.a
		self.w = C.T.dot(w) # w in world frame
		self.o = o
		self.q = quaternion_multiply_JPL(self.rotation_to_quaternion(T*w), self.q) # JPL
		# self.q = o1
		self.q2 = quaternion_multiply(self.q2, self.rotation_to_quaternion(T*w)) # Hamilton

		# state error transition matrix (discrete linearised error dynamics)
		F = np.zeros((self.Ne, self.Ne))

		I3 = np.eye(3)
		# first row
		F[0:3, 0:3] = I3
		F[0:3, 3:6] = I3 * T
		F[0:3, 6:9] = -(T**2/2) * C.T.dot(f_skew)
		F[0:3, 27:30] = -(T**2/2) * C.T
		F[0:3, 30:33] = (T**3/6) * C.T.dot(f_skew)
		# second row
		F[3:6, 3:6] = I3
		F[3:6, 6:9] = -T * C.T.dot(f_skew)
		F[3:6, 27:30] = -T * C.T
		F[3:6, 30:33] = (T**2/2) * C.T.dot(f_skew)
		# third row
		F[6:9, 6:9] = self.mySeries(w, T, 0).T
		F[6:9, 30:33] = - self.mySeries(w, T, 1).T
		# all the rest
		F[9:33, 9:33] = np.eye(24)


		# discrete process noise covariance matrix
		Q = np.zeros(F.shape)

		Q[0:3, 0:3] = (	(T**3/3) * self.Qf + (T**5/20) * self.Qbf )# k11
						# + (T**5/20) * C.T.dot(f_skew).dot(self.Qw).dot(f_skew.T).dot(C)
						# + (T**7/252) * C.T.dot(f_skew).dot(self.Qbw).dot(f_skew.T).dot(C)	)
		Q[0:3, 3:6] = (	(T**2/2) * self.Qf + (T**4/8) * self.Qbf )# k12 = k21
						# + (T**4/8) * C.T.dot(f_skew).dot(self.Qw).dot(f_skew.T).dot(C)
						# + (T**6/72) * C.T.dot(f_skew).dot(self.Qbw).dot(f_skew.T).dot(C)	)
		Q[0:3, 6:9] = -(T**3/6)*C.T.dot(f_skew).dot(self.Qw) -(T**5/30)*C.T.dot(f_skew).dot(self.Qbw) # k13
		Q[0:3, 27:30] = -(T**3/6)*C.T.dot(self.Qbf) # k14
		Q[0:3, 30:33] = -(T**4/24)*C.T.dot(f_skew).dot(self.Qbw) #k15

		Q[3:6, 0:3] = Q[0:3, 3:6].T # k21
		Q[3:6, 3:6] = (	T * self.Qf + (T**3/3) * self.Qbf )# k22
						# + (T**3/3) * C.T.dot(f_skew).dot(self.Qw).dot(f_skew.T).dot(C)
						# + (T**5/20) * C.T.dot(f_skew).dot(self.Qbw).dot(f_skew.T).dot(C)	)
		Q[3:6, 6:9] = -(T**2/2)*C.T.dot(f_skew).dot(self.Qw) - (T**4/8)*C.T.dot(f_skew).dot(self.Qbw) # k23
		Q[3:6, 27:30] = -(T**2/2)*C.T.dot(self.Qbf) #k24
		Q[3:6, 30:33] = -(T**3/6)*C.T.dot(f_skew).dot(self.Qbw) #k25

		Q[6:9, 0:3] = Q[0:3, 6:9].T #k31
		Q[6:9, 3:6] = Q[3:6, 6:9].T #k32
		temp = self.mySeries(w, T, 3) + self.mySeries(w, T, 3).T
		Q[6:9, 6:9] = T * self.Qw + temp.dot(self.Qbw) # k33
		Q[6:9, 30:33] = - self.mySeries(w, T, 2).T.dot(self.Qbw) #k35

		# feet
		'''Q[9:12, 9:12] = T * C.T.dot(self.Qp).dot(C)
		Q[12:15, 12:15] = Q[9:12, 9:12]
		Q[15:18, 15:18] = Q[9:12, 9:12]
		Q[18:21, 18:21] = Q[9:12, 9:12]
		Q[21:24, 21:24] = Q[9:12, 9:12]
		Q[24:27, 24:27] = Q[9:12, 9:12]'''

		self.robot.update_state()
		for j in range(0,self.robot.active_legs):
			# if self.robot.Leg[j].cstate == True:
			force = np.linalg.norm(self.robot.Leg[j].F6c.tibia_X_foot[:3])
			# force_z = self.robot.Leg[j].F6c.base_X_foot[2]
			# force_xy = np.max(np.abs(self.robot.Leg[j].F6c.base_X_foot[0:2]))
			# qp_xy = 0.002 + (0.002-0.002)/20 * force_xy
			# Qp = np.diagflat([qp_xy**2, qp_xy**2, 0.001**2])

			# if j!=4 and (force > self.force_threshold and force < 150): #Gazebo: force > 3 and force < 25
			# 	cov = T * C.T.dot(self.Qp).dot(C)
			# else:
			# 	cov = 1000*np.eye(3) # foot positions unknown during swing phase (high std. dev.)

			# Hysteresis based contact detection
			th = self.threshold[j]
			lim = 8
			if th > lim:
				if force < self.force_threshold:
					th = 0
			else:
				if force > self.force_threshold: #3
					th += 1
				else:
					th = 0

			self.threshold[j] = th
			#
			if th > lim: #Gazebo: force > 3 and force < 25
				cov = T * C.T.dot(self.Qp).dot(C)
				# cov = T * C.T.dot(self.Qp1 if j<3 else self.Qp2).dot(C)
			else:
				cov = 1000*np.eye(3) # f

			i = 9+j*3
			k = 9+(j+1)*3
			Q[i:k, i:k] = cov

		Q[27:30, 0:3] = -(T**3/6) * self.Qbf.dot(C) # k41
		Q[27:30, 3:6] = -(T**2/2) * self.Qbf.dot(C) # k42
		Q[27:30, 27:30] = T * self.Qbf #k44

		Q[30:33, 0:3] = Q[0:3, 30:33].T #k51
		Q[30:33, 3:6] = Q[3:6, 30:33].T #k52
		Q[30:33, 6:9] = - self.Qbw.dot(self.mySeries(w, T, 2)) # k53
		Q[30:33, 30:33] = T * self.Qbw # k55

		# update equation
		# self.set_dx(F.dot(self.get_dx()))		# update error state vector: always zero!
		self.P = F.dot(self.P).dot(F.T) + Q		# update covariance matrix
		#self.P[self.P > 100000] = 100000
		#print colored('****************************', 'yellow')
		#print colored('P1', 'yellow'), self.P[10]
		#print colored('****************************', 'yellow')
	#print self.P[0:3, 0:3]

	def update_state(self):

		C = quaternion_matrix_JPL(self.q)[0:3, 0:3]
		r = self.r

		# Calculate measurement residual (difference between prediction and measurement)
		y = np.zeros(18)
		# Evalute Jacobian of residual
		H = np.zeros((18, self.Ne))
		# Evaluate measurement noise matrix
		R = np.zeros((18,18))

		# Evaluate robot foot positions based on latest angle measurements
		self.robot.update_state()
		for j in range(0,self.robot.active_legs):
			s = self.robot.Leg[j].XHc.base_X_foot[:3,3]
			J = self.robot.Leg[j].XHc.j_base_X_foot # jacobian
			p = self.get_p(j)
			#print "p", p
			# print "j:", j, self.robot.Leg[j].cstate
			# print self.robot.Leg[j].F6c.tibia_X_foot[:3]
			y[j*3:(j+1)*3] = s - C.dot(p-r)

			H[j*3:(j+1)*3, 0:3] = -C
			H[j*3:(j+1)*3, 6:9] = skew(C.dot(p-r))
			H[j*3:(j+1)*3, j*3+9:(j+1)*3+9] = C

			R[j*3:(j+1)*3, j*3:(j+1)*3] = self.Rs + J.dot(self.Ra).dot(J.T)
			#print colored(j, 'yellow'), "s-p:", s - C.dot(p-r)
			#print colored(j, 'yellow'), "s:", s
			#print "p", p


		# Calculate Kalman gain
		S = H.dot(self.P).dot(H.T) + R
		#print "P:", self. P[0]
		K = self.P.dot(H.T).dot(np.linalg.pinv(S))
		#print "K:", K[0:2]

		# Correct/update state vector
		dxp = K.dot(y)	# a posteriori state error estimate (correction vector)
		self.set_dx(dxp)
		self.r = self.r + self.dr
		self.v = self.v + self.dv
		self.q = quaternion_multiply_JPL( self.rotation_to_quaternion(self.dphi), self.q)
		self.p0 = self.p0 + self.dp0
		self.p1 = self.p1 + self.dp1
		self.p2 = self.p2 + self.dp2
		self.p3 = self.p3 + self.dp3
		self.p4 = self.p4 + self.dp4
		self.p5 = self.p5 + self.dp5
		self.bf = self.bf + self.dbf
		self.bw = self.bw + self.dbw

		# Reset state errors as their corresponding corrections have already been applied
		self.set_dx(np.zeros(self.Ne))

		'''print colored('P[0]:', 'red'), self.P[0]
		print colored('H[0]:', 'blue'), H[0]
		print "R[0]:", R[0]
		print colored('K[0]:', 'green'), K[0]
		print colored('y:', 'cyan'), y
		print colored('dx:', 'yellow'), dxp
		print "dr:", self.dr
		print "------------------------------------------------------------------"
		'''

		# Calculate a posteriori estimate of error state covariance matrix
		self.P = (np.eye(self.Ne)-K.dot(H)).dot(self.P)
		self.y = y # for debugging
		return np.concatenate((self.r, self.get_fixed_angles()))

	@staticmethod
	def rotation_to_quaternion(phi):
		phi_norm = np.sqrt(np.dot(phi, phi))
		q = np.zeros(4)
		q[0] = np.cos(phi_norm/2)
		if phi_norm > 0:
			q[1:4] = np.sin(phi_norm/2) * phi / phi_norm
		return q

	# Implements auxiliary quantity defined in State Estimation paper by Bloesch
	@staticmethod
	def mySeries(w, T, n, N=3):
		w_skew = skew(w)
		# S = np.zeros((3, 3))
		# for i in range(N):
		# 	temp = ( T**(i+n) / math.factorial(i+n) ) * np.linalg.matrix_power(w_skew, i)
		# 	S += temp

		S = ( (T**n / math.factorial(n) ) * np.eye(3)
			 +(T**(n+1) / math.factorial(1+n) ) * w_skew )
		return S
