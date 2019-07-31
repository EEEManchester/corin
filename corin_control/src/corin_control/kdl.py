#!/usr/bin/env python

## Forward and inverse kinematics of the 3 DOF leg joints
## Transformation with respect to SCS (origin is joint 1 frame)
import sys; sys.dont_write_bytecode = True
import numpy as np
import timeit
from scipy import linalg

from matrix_transforms import *
from constant import *

class KDL():

	def __init__(self):
		self.link	 = [L1, L2, L3]
		self.knee_up = True

	def leg_jacobian(self, q=None):
		""" Jacobian in leg frame """

		Jv = np.zeros((3,3))
		try:
			# q1 = q.item(0); q2 = q.item(1); q3 = q.item(2)
			q1 = q[0]; q2 = q[1]; q3 = q[2]
			# print 'q: ', q1, q2, q3
			# Jv = np.array([	[-np.sin(q1)*(L3*np.cos(q2+q3)+np.cos(q2)*L2+L1),     -np.cos(q1)*(L3*np.sin(q2+q3)+np.sin(q2)*L2),     -np.sin(q2+q3)*np.cos(q1)*L3 ],
			# 					[ np.cos(q1)*(L3*np.cos(q2+q3)+np.cos(q2)*L2+L1),     -np.sin(q1)*(L3*np.sin(q2+q3)+np.sin(q2)*L2),     -np.sin(q2+q3)*np.sin(q1)*L3 ],
			# 					[ 0,                                          		   L3*np.cos(q2+q3)+np.cos(q2)*L2,                	 L3*np.cos(q2+q3)]			])

			Jv[0,0] = -np.sin(q1)*(L3*np.cos(q2+q3)+np.cos(q2)*L2+L1)
			Jv[0,1] = -np.cos(q1)*(L3*np.sin(q2+q3)+np.sin(q2)*L2)
			Jv[0,2] = -np.sin(q2+q3)*np.cos(q1)*L3
			Jv[1,0] = np.cos(q1)*(L3*np.cos(q2+q3)+np.cos(q2)*L2+L1)
			Jv[1,1] = -np.sin(q1)*(L3*np.sin(q2+q3)+np.sin(q2)*L2)
			Jv[1,2] = -np.sin(q2+q3)*np.sin(q1)*L3
			Jv[2,1] = L3*np.cos(q2+q3)+np.cos(q2)*L2
			Jv[2,2] = L3*np.cos(q2+q3)
			# print Jv
			return Jv
		except:
			return None


	def transpose_leg_jacobian(self, q=None):
		
		try:
			return self.leg_jacobian(q).transpose()
		except: 
			return None

	def world_leg_jacobian(self, leg_no, qb, q):
		""" Jacobian in world frame """

		if leg_no >= 0 and leg_no <=5:
			hip_yaw = np.deg2rad(ROT_BASE_X_LEG[leg_no])
		else:
			return None

		roll  = qb[0]
		pitch = qb[1]
		yaw   = qb[2]
		q1 = q[0]
		q2 = q[1]
		q3 = q[2]
		Jv = np.zeros((3,3))

		Jv[0,0] = np.sin(q1)*(np.sin(hip_yaw)*(np.cos(roll)*np.sin(yaw) - np.cos(yaw)*np.sin(pitch)*np.sin(roll)) - np.cos(pitch)*np.cos(hip_yaw)*np.cos(yaw))*(L1 + L3*np.cos(q2 + q3) + L2*np.cos(q2)) - np.cos(q1)*(np.cos(hip_yaw)*(np.cos(roll)*np.sin(yaw) - np.cos(yaw)*np.sin(pitch)*np.sin(roll)) + np.cos(pitch)*np.cos(yaw)*np.sin(hip_yaw))*(L1 + L3*np.cos(q2 + q3) + L2*np.cos(q2));
		Jv[0,1] = (L3*np.cos(q2 + q3) + L2*np.cos(q2))*(np.sin(roll)*np.sin(yaw) + np.cos(roll)*np.cos(yaw)*np.sin(pitch)) + np.cos(q1)*(np.sin(hip_yaw)*(np.cos(roll)*np.sin(yaw) - np.cos(yaw)*np.sin(pitch)*np.sin(roll)) - np.cos(pitch)*np.cos(hip_yaw)*np.cos(yaw))*(L3*np.sin(q2 + q3) + L2*np.sin(q2)) + np.sin(q1)*(np.cos(hip_yaw)*(np.cos(roll)*np.sin(yaw) - np.cos(yaw)*np.sin(pitch)*np.sin(roll)) + np.cos(pitch)*np.cos(yaw)*np.sin(hip_yaw))*(L3*np.sin(q2 + q3) + L2*np.sin(q2));
		Jv[0,2] = L3*np.cos(q2 + q3)*(np.sin(roll)*np.sin(yaw) + np.cos(roll)*np.cos(yaw)*np.sin(pitch)) + L3*np.sin(q2 + q3)*np.cos(q1)*(np.sin(hip_yaw)*(np.cos(roll)*np.sin(yaw) - np.cos(yaw)*np.sin(pitch)*np.sin(roll)) - np.cos(pitch)*np.cos(hip_yaw)*np.cos(yaw)) + L3*np.sin(q2 + q3)*np.sin(q1)*(np.cos(hip_yaw)*(np.cos(roll)*np.sin(yaw) - np.cos(yaw)*np.sin(pitch)*np.sin(roll)) + np.cos(pitch)*np.cos(yaw)*np.sin(hip_yaw));
		Jv[1,0] = np.cos(q1)*(np.cos(hip_yaw)*(np.cos(roll)*np.cos(yaw) + np.sin(pitch)*np.sin(roll)*np.sin(yaw)) - np.cos(pitch)*np.sin(hip_yaw)*np.sin(yaw))*(L1 + L3*np.cos(q2 + q3) + L2*np.cos(q2)) - np.sin(q1)*(np.sin(hip_yaw)*(np.cos(roll)*np.cos(yaw) + np.sin(pitch)*np.sin(roll)*np.sin(yaw)) + np.cos(pitch)*np.cos(hip_yaw)*np.sin(yaw))*(L1 + L3*np.cos(q2 + q3) + L2*np.cos(q2));
		Jv[1,1] = - (L3*np.cos(q2 + q3) + L2*np.cos(q2))*(np.cos(yaw)*np.sin(roll) - np.cos(roll)*np.sin(pitch)*np.sin(yaw)) - np.cos(q1)*(np.sin(hip_yaw)*(np.cos(roll)*np.cos(yaw) + np.sin(pitch)*np.sin(roll)*np.sin(yaw)) + np.cos(pitch)*np.cos(hip_yaw)*np.sin(yaw))*(L3*np.sin(q2 + q3) + L2*np.sin(q2)) - np.sin(q1)*(np.cos(hip_yaw)*(np.cos(roll)*np.cos(yaw) + np.sin(pitch)*np.sin(roll)*np.sin(yaw)) - np.cos(pitch)*np.sin(hip_yaw)*np.sin(yaw))*(L3*np.sin(q2 + q3) + L2*np.sin(q2));
		Jv[1,2] = - L3*np.cos(q2 + q3)*(np.cos(yaw)*np.sin(roll) - np.cos(roll)*np.sin(pitch)*np.sin(yaw)) - L3*np.sin(q2 + q3)*np.cos(q1)*(np.sin(hip_yaw)*(np.cos(roll)*np.cos(yaw) + np.sin(pitch)*np.sin(roll)*np.sin(yaw)) + np.cos(pitch)*np.cos(hip_yaw)*np.sin(yaw)) - L3*np.sin(q2 + q3)*np.sin(q1)*(np.cos(hip_yaw)*(np.cos(roll)*np.cos(yaw) + np.sin(pitch)*np.sin(roll)*np.sin(yaw)) - np.cos(pitch)*np.sin(hip_yaw)*np.sin(yaw));
		Jv[2,0] = np.cos(q1)*(np.sin(pitch)*np.sin(hip_yaw) + np.cos(pitch)*np.cos(hip_yaw)*np.sin(roll))*(L1 + L3*np.cos(q2 + q3) + L2*np.cos(q2)) + np.sin(q1)*(np.cos(hip_yaw)*np.sin(pitch) - np.cos(pitch)*np.sin(hip_yaw)*np.sin(roll))*(L1 + L3*np.cos(q2 + q3) + L2*np.cos(q2));
		Jv[2,1] = np.cos(q1)*(np.cos(hip_yaw)*np.sin(pitch) - np.cos(pitch)*np.sin(hip_yaw)*np.sin(roll))*(L3*np.sin(q2 + q3) + L2*np.sin(q2)) - np.sin(q1)*(np.sin(pitch)*np.sin(hip_yaw) + np.cos(pitch)*np.cos(hip_yaw)*np.sin(roll))*(L3*np.sin(q2 + q3) + L2*np.sin(q2)) + np.cos(pitch)*np.cos(roll)*(L3*np.cos(q2 + q3) + L2*np.cos(q2));
		Jv[2,2] = L3*np.cos(q2 + q3)*np.cos(pitch)*np.cos(roll) + L3*np.sin(q2 + q3)*np.cos(q1)*(np.cos(hip_yaw)*np.sin(pitch) - np.cos(pitch)*np.sin(hip_yaw)*np.sin(roll)) - L3*np.sin(q2 + q3)*np.sin(q1)*(np.sin(pitch)*np.sin(hip_yaw) + np.cos(pitch)*np.cos(hip_yaw)*np.sin(roll));
		
		# temp = mX(rot_Z(yaw), rot_Y(pitch), rot_X(roll),rot_Z(hip_yaw),self.leg_jacobian(q))

		return Jv

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

	def leg_IK(self, p=None, leg_no=None):

		try:
			x = p[0];	y = p[1];	z = p[2];

			q1 = np.arctan2(y,x)
			
			# Leg along direction of body - wall walking cornering
			# if ( (abs(np.rad2deg(q1)) >= 50.)):
			# 	if (leg_no==0 or leg_no==3):
			# 		q1 = -np.deg2rad(50);
			# 	elif (leg_no==2):
			# 		q1 = np.deg2rad(50);		
			# 	else:
			# 		# print abs(np.rad2deg(q1))
			# 		print 'Invalid condition!'
				
			# if leg_no == 0:
			# 	print 'q1 ', np.round(q1,3), np.round(p,4)

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
		return np.dot(self.transpose_leg_jacobian(q),f)

	def torque_to_force(self, q=None, tau=None):
		# f = J^(-T)*tau
		return np.dot(inv(self.transpose_leg_jacobian(q)),tau)


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

# print CK.singularity_approach(qs)
# cd = CK.leg_FK(qs)
# print cd.flatten()
cd = [0.15 , -0.0,  -0.06]
qp = CK.leg_IK(cd)

# print np.rad2deg(qp[1])
# print qp
# CK.check_singularity(qp)
# if (not CK.check_singularity(qp)):
# 	qd,qdd = CK.joint_speed(qp, v, a)
# qpd = np.array([0., 0.34, -1.85])
q = [0.0 , 0.4640929590272851, -1.9360360660532043]
print CK.leg_FK(q).flatten()

# code snippet to be executed only once 
mysetup = "from math import sqrt"
  
# code snippet whose execution time is to be measured 
mycode = ''' 
try:
	q = [-0.,          0.92729522, -1.85459044]
	Jv = np.zeros((3,3))
	try:
		q1 = q[0]; q2 = q[1]; q3 = q[2]
		
		Jv[0,0] = -np.sin(q1)*(L3*np.cos(q2+q3)+np.cos(q2)*L2+L1)
		Jv[0,1] = -np.cos(q1)*(L3*np.sin(q2+q3)+np.sin(q2)*L2)
		Jv[0,2] = -np.sin(q2+q3)*np.cos(q1)*L3
		Jv[1,0] = np.cos(q1)*(L3*np.cos(q2+q3)+np.cos(q2)*L2+L1)
		Jv[1,1] = -np.sin(q1)*(L3*np.sin(q2+q3)+np.sin(q2)*L2)
		Jv[1,2] = -np.sin(q2+q3)*np.sin(q1)*L3
		Jv[2,1] = L3*np.cos(q2+q3)+np.cos(q2)*L2
		Jv[2,2] = L3*np.cos(q2+q3)
	except:
		pass

	rank = np.linalg.matrix_rank(Jv)
	if (rank < 3):
		pass
	else:
		pass
except Exception, e:
	pass
'''
  
# timeit statement 
# t_iter = 100000
# t_total = timeit.timeit(setup = mysetup, 
#                     stmt = mycode, 
#                     number = 100000) 
# print t_total/t_iter