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
		self.link	 = [L1, L2, L3]
		self.knee_up = True

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
			
			if (a3 < 0.):
				raise ValueError("Negative value ", np.round(p,4)) 
			s3  = np.sqrt(1.0-c3**2);
			q3t = [np.arctan2(s3,c3), np.arctan2(-s3,c3)];
			
			# first choice is (-ve) - correspond to knee up config.
			if (self.knee_up is True):
				q3 = q3t[0] if (q3t[0] < 0) else q3t[1]
			else:
				q3 = q3t[0] if (q3t[0] > 0) else q3t[1]
			
			# Dividing the element (1,4) & (2,4) in inv(H01)*Psym = T12*T23
			xp = x*np.cos(q1) + np.sin(q1)*y - L1;
			yp = z;
			q2t = [np.arctan2(yp,xp) - np.arctan2(L3*np.sin(q3), L2+L3*np.cos(q3)), np.arctan2(yp,xp) - np.arctan2(L3*np.sin(q3), L2+L3*np.cos(q3))];

			if (self.knee_up is True):
				q2 = q2t[0] if (q2t[0] < 0) else q2t[1]
			else:
				q2 = q2t[0] if (q2t[0] > 0) else q2t[1]
			# if (q2t[0] < 0):
			# 	q2 = q2t[0];
			# else:
			# 	q2 = q2t[1];
			
			return np.array([q1, q2, q3])
		except ValueError, e:
			print 'KDL-IK(): ', e
			return None
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

	def singularity_approach(self, q=None):
		""" checks how close the robot is to singular, sqrt[ det(J*J^T)]	"""
		""" Input: 	1) q -> joint angles (2D array) in radians
			Output: 1) scalar value how close to singular 		"""

		try:
			return np.nan_to_num(np.sqrt(np.linalg.det( mX(self.leg_jacobian(q),self.transpose_leg_jacobian(q)) )))
		except Exception, e:
			return 0

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

	
## ================================================================================================ ##
## 												TESTING 											##
## ================================================================================================ ##
CK = KDL()

qsurface = np.array([0.,-np.pi/2,0.])
base_X_surface = 0.29
bodypose = np.array([0.,0.,BODY_HEIGHT, 0.,0.,0.])

# CK.update_nominal_stance(bodypose, base_X_surface, qsurface)

# print bodypose[3:7]
# qs = [0., 1.238, -1.724] #[0., 1.195, -1.773] 	# left side
qs = np.array([0., 0.15, -0.10])	# right side
# print CK.singularity_approach(qs)
# cd = CK.leg_FK(qs)
# print cd.flatten()
# cd = [ 0.21, 0., -0.1]
cd = [0.295, -0.0056, 0.1865]
# qp = CK.leg_IK(cd)
# print 'q: ', qp
# print qp
# if (not CK.check_singularity(qp)):
# 	qd,qdd = CK.joint_speed(qp, v, a)
