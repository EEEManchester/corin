#!/usr/bin/env python

## Forward and inverse kinematics of the 3 DOF leg joints
## Transformation with respect to SCS (origin is joint 1 frame)
import sys; sys.dont_write_bytecode = True
import numpy as np
from scipy import linalg

from matrix_transforms import *
from constant import *

class KDL():

	def __init__(self):
		self.link	= [L1, L2, L3]

	def leg_jacobian(self, q=None):

		try:
			q1 = q.item(0); q2 = q.item(1); q3 = q.item(2)
			# print 'q: ', q1, q2, q3
			Jv = np.matrix([	[-np.sin(q1)*(L3*np.cos(q2+q3)+np.cos(q2)*L2+L1),     -np.cos(q1)*(L3*np.sin(q2+q3)+np.sin(q2)*L2),     -np.sin(q2+q3)*np.cos(q1)*L3 ],
								[ np.cos(q1)*(L3*np.cos(q2+q3)+np.cos(q2)*L2+L1),     -np.sin(q1)*(L3*np.sin(q2+q3)+np.sin(q2)*L2),     -np.sin(q2+q3)*np.sin(q1)*L3 ],
								[ 0,                                          		   L3*np.cos(q2+q3)+np.cos(q2)*L2,                	 L3*np.cos(q2+q3)]			])

			# print Jv
			return Jv
		except:
			return None


	def transpose_leg_jacobian(self, q=None):
		return self.leg_jacobian(q).transpose()

	def leg_FK(self, q=None):

		try:
			q1 = q[0]; q2 = q[1]; q3 = q[2]

			x = np.cos(q1) * (L3*np.cos(q2+q3) + np.cos(q2) * L2 + L1)
			y = np.sin(q1) * (L3*np.cos(q2+q3) + np.cos(q2) * L2 + L1)
			z = L3*np.sin(q2+q3) + np.sin(q2) * L2

			return np.array([[x],[y],[z]])
		except Exception, e:
			print 'FK: ', e
			pass

	def leg_IK(self, p=None):

		try:
			x = p[0];	y = p[1];	z = p[2];
			
			q1 = np.arctan2(y,x)

			# Adding the square of element (1,4) & (2,4) in inv(H01)*Psym = T12*T23
			c3  = ( (x*np.cos(q1) + y*np.sin(q1) - L1)**2 + z**2 -L3**2-L2**2 )/(2*L2*L3);
			a3  = 1.0-c3**2
			if (a3 < 0):
				raise Exception, "Negative value encountered for ", p
			s3  = np.sqrt(1.0-c3**2);
			q3t = [np.arctan2(s3,c3), np.arctan2(-s3,c3)];

			if (q3t[0] < 0):
				q3 = q3t[0];
			else:
				q3 = q3t[1];

			# Dividing the element (1,4) & (2,4) in inv(H01)*Psym = T12*T23
			xp = x*np.cos(q1) + np.sin(q1)*y - L1;
			yp = z;
			q2t = [np.arctan2(yp,xp) - np.arctan2(L3*np.sin(q3), L2+L3*np.cos(q3)), np.arctan2(yp,xp) - np.arctan2(L3*np.sin(q3), L2+L3*np.cos(q3))];

			if (q2t[0] < 0):
				q2 = q2t[0];
			else:
				q2 = q2t[1];
			#return q2
			return np.array([q1, q2, q3])
		except Exception, e:
			print 'KDL-IK(): ', e
			return None

	def check_singularity(self, q=None):
		""" checks if robot configuration is singular 			"""
		""" Input: 	1) q -> joint angles (2D array) in radians
			Output: 1) Flag -> True: singular, False: OK 		"""

		try:
			rank = np.linalg.matrix_rank(self.leg_jacobian(q))
			if (rank < 3):
				return True
			else:
				return False
		except Exception, e:
			return None

	def joint_speed(self, q=None, v=None, a=None):

		try:
			q1 = q.item(0); q2 = q.item(1); q3 = q.item(2)

			vel = np.matrix([ [v.item(0)], [v.item(1)], [v.item(2)] ])
			acc = np.matrix([ [a.item(0)], [a.item(1)], [a.item(2)] ])

			Jv = self.leg_jacobian(q)

			qd = linalg.solve(Jv,vel)

			qd1 = qd.item(0); qd2 = qd.item(1); qd3 = qd.item(2);

			Ja11 = -np.cos(q2)*np.cos(q1)*L2*qd1+np.sin(q2)*np.sin(q1)*L2*qd2-np.cos(q2+q3)*np.cos(q1)*L3*qd1+np.sin(q2+q3)*np.sin(q1)*L3*qd2+qd3*np.sin(q2+q3)*np.sin(q1)*L3-np.cos(q1)*L1*qd1;
			Ja12 = -np.cos(q2)*np.cos(q1)*L2*qd2+np.sin(q2)*np.sin(q1)*L2*qd1-qd2*np.cos(q1)*L3*np.cos(q2+q3)-qd3*np.cos(q1)*L3*np.cos(q2+q3)+qd1*np.sin(q2+q3)*np.sin(q1)*L3;
			Ja13 = -L3*(np.cos(q2+q3)*np.cos(q1)*qd2+np.cos(q2+q3)*np.cos(q1)*qd3-np.sin(q2+q3)*np.sin(q1)*qd1);
			Ja21 = -np.cos(q2)*np.sin(q1)*L2*qd1-np.sin(q2)*np.cos(q1)*L2*qd2-np.cos(q2+q3)*np.sin(q1)*L3*qd1-np.sin(q2+q3)*np.cos(q1)*L3*qd2-qd3*np.sin(q2+q3)*np.cos(q1)*L3-np.sin(q1)*L1*qd1;
			Ja22 = -np.cos(q2)*np.sin(q1)*L2*qd2-np.sin(q2)*np.cos(q1)*L2*qd1-qd2*np.sin(q1)*L3*np.cos(q2+q3)-qd3*np.sin(q1)*L3*np.cos(q2+q3)-qd1*np.sin(q2+q3)*np.cos(q1)*L3;
			Ja23 = -L3*(np.cos(q2+q3)*np.sin(q1)*qd2+np.cos(q2+q3)*np.sin(q1)*qd3+np.sin(q2+q3)*np.cos(q1)*qd1);
			Ja31 = 0;
			Ja32 = -np.sin(q2)*L2*qd2-qd2*L3*np.sin(q2+q3)-qd3*L3*np.sin(q2+q3);
			Ja33 = -np.sin(q2+q3)*L3*(qd2+qd3);

			Ja = np.matrix([ [Ja11, Ja12, Ja13], [Ja21, Ja22, Ja23], [Ja31, Ja32, Ja33] ])

			qdd = linalg.solve(Jv,(acc - Ja*qd))

			# return np.insert(np.array(qd), 0, 0), np.insert(np.array(qdd), 0, 0)
			return qd, qdd
		except Exception, e:
			print 'Error in Jv: ', e
			return None, None

	## Torque to force mapping and vice versa uses the relationship: tau = J^(T)*f
	def force_to_torque(self, q=None, f=None):
		return self.transpose_leg_jacobian(q)*f

	def torque_to_force(self, q=None, tau=None):
		# f = J^(-T)*tau
		return inv(self.transpose_leg_jacobian(q))*tau

	def update_nominal_stance(self, bodypose, base_X_surface, qsurface):
		""" updates the robot nominal stance (REP/NRP) """

		# print 'bodypose: ', bodypose
		# print 'b X surf: ', base_X_surface
		# print 'q surf  : ', qsurface

		qrx = qsurface.item(1)

		world_hip_X_wall_Y = (np.dot(rotation_zyx(bodypose[3:7]), (base_X_surface - COXA_Y)*np.array([ [0.],[1.],[0,] ])) ).item(1)
		world_hip_X_wall_Z = bodypose.item(2) + (np.dot(rotation_zyx(bodypose[3:7]), np.array([ [0.], [COXA_Y], [0.] ]) ) ).item(2)

		# print 'f tf: ', np.round(world_hip_X_wall_Y, 3), np.round(world_hip_X_wall_Z, 3)

		# foothold algorithm
		# Step 1:
		h2 = L3*np.sin(np.pi/2 - qrx);
		d2 = np.sqrt(L3**2 - h2**2)
		# print 's1: ', h2, d2
		# Step 2:
		d1  = world_hip_X_wall_Y - d2
		phi = np.arcsin(d1/L2)
		# print 's2: ', d1, L2, phi
		# Step 3:
		hs = np.sqrt(L2**2 - d1**2)
		h1 = hs - h2

		# Step 4:
		world_X_wall_Z = world_hip_X_wall_Z + h1

		# transform to world frame
		world_X_wall_Y = (np.dot(rotation_zyx(bodypose[3:7]), base_X_surface*np.array([ [0.],[1.],[0,] ]))).item(1)
		base_X_wall_Z  = world_X_wall_Z - bodypose.item(2)

		# transform to base frame
		# base_X_nom = np.dot(rotation_zyx(-bodypose[3:7]), np.array([ [0.],[world_X_wall_Y], [base_X_wall_Z] ]))

		# output: in base frame, from hip to nominal position
		base_hip_X_nom = np.dot(rotation_zyx(-bodypose[3:7]), np.array([ [0.],[world_hip_X_wall_Y], [base_X_wall_Z] ]))

		# print world_X_wall_Y, base_X_wall_Z
		# print base_X_nom.transpose()
		# print base_hip_X_nom.transpose()

		# return base_X_nom
		return base_hip_X_nom



## ================================================================================================ ##
## 												TESTING 											##
## ================================================================================================ ##
CK = KDL()

qsurface = np.array([0.,-np.pi/2,0.])
base_X_surface = 0.29
bodypose = np.array([0.,0.,BODY_HEIGHT, 0.,0.,0.])

CK.update_nominal_stance(bodypose, base_X_surface, qsurface)

# print bodypose[3:7]
# qs = [0., 1.238, -1.724] #[0., 1.195, -1.773] 	# left side
qs = [0., 0.45, -2.033]	# right side

cd = CK.leg_FK(qs)
# print cd.flatten()
cd = [ 0.21, 0., -0.1]
qp = CK.leg_IK(cd)
# print 'q: ', qp
# print qp
# if (not CK.check_singularity(qp)):
# 	qd,qdd = CK.joint_speed(qp, v, a)
