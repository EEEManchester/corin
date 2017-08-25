#!/usr/bin/env python

## Forward and inverse kinematics of the 3 DOF leg joints
## Transformation with respect to SCS (origin is joint 1 frame)
import sys; sys.dont_write_bytecode = True
import numpy as np
from constant import *
from scipy import linalg

l1 = LL_LENG_1
l2 = LL_LENG_2
l3 = LL_LENG_3
ld = LL_LENG_d

class corin_kinematics():
	
	def __init__(self, limb):
		self.link	= [l1, l2, l3]

	def jacobian(self, q=None):

		try:
			q1 = q.item(0); q2 = q.item(1); q3 = q.item(2)
			
			Jv = np.matrix([ [-np.sin(q1)*(l3*np.cos(q2 + q3) - ld*np.sin(q2 + q3) + l2*np.cos(q2)), -np.cos(q1)*(ld*np.cos(q2 + q3) + l3*np.sin(q2 + q3) + l2*np.sin(q2)), -np.cos(q1)*(ld*np.cos(q2 + q3) + l3*np.sin(q2 + q3))],
							 [ np.cos(q1)*(l3*np.cos(q2 + q3) - ld*np.sin(q2 + q3) + l2*np.cos(q2)), -np.sin(q1)*(ld*np.cos(q2 + q3) + l3*np.sin(q2 + q3) + l2*np.sin(q2)), -np.sin(q1)*(ld*np.cos(q2 + q3) + l3*np.sin(q2 + q3))],
							 [																	 0, 	          l3*np.cos(q2 + q3) - ld*np.sin(q2 + q3) + l2*np.cos(q2),            	l3*np.cos(q2 + q3) - ld*np.sin(q2 + q3)] ])

			# print 'Jv completed'
			# print Jv
			return Jv 
		except:
			return None
		
	def jacobian_transpose(self, q=None):
		return self.jacobian(q).transpose()

	def FK(self, q=None):

		try:
			q1 = q[0]; q2 = q[1]; q3 = q[2]

			x = np.cos(q1)*(l3*np.cos(q2 + q3) - ld*np.sin(q2 + q3) + l2*np.cos(q2))
			y = np.sin(q1)*(l3*np.cos(q2 + q3) - ld*np.sin(q2 + q3) + l2*np.cos(q2))	
			z = l1 + ld*np.cos(q2 + q3) + l3*np.sin(q2 + q3) + l2*np.sin(q2)

			return np.array([[x],[y],[z]])
		except Exception, e:
			print 'FK: ', e
			pass

	def IK(self, p=None):
		p = p.flatten()

		try:
			x = p[0];	y = p[1];	z = p[2];
			
			q1 = np.arctan2(y,x)

			l3p = np.sqrt(l3**2 + ld**2)    	#projected hypoteneus from link 3-temp. removes offset
			c = np.arctan2(l3,ld)
			a = np.sqrt(x**2 + y**2)   		#actual distance of joint 2 to EE
			h = np.sqrt(a**2 + (l1-z)**2)     #hypoteneus from origin to EE
			
			alpha = np.real(np.arccos( np.complex((l2**2 + l3p**2 - h**2)/(2*l2*l3p) )))
			q3 = c + alpha - 3*np.pi/2

			omega = np.arctan2(z-l1,a)
			beta = np.real(np.arccos(np.complex((h**2 + l2**2 - l3p**2)/(2*h*l2))))
			q2 = beta + omega

			return np.array([q1, q2, q3])
		except Exception, e:
			print 'IK: ', e
			pass

	def singularity_check(self, q=None):
		rank = np.linalg.matrix_rank(self.jacobian(q))
		
		if (rank < 3):
			return 1
		else:
			return 0

	def joint_speed(self, q=None, v=None, a=None):

		try:
			q1 = q.item(0); q2 = q.item(1); q3 = q.item(2)

			vel = np.matrix([ [v.item(0)], [v.item(1)], [v.item(2)] ])
			acc = np.matrix([ [a.item(0)], [a.item(1)], [a.item(2)] ])
			
			Jv = self.jacobian(q)

			qd = linalg.solve(Jv,vel)
			
			qd1 = qd.item(0); qd2 = qd.item(1); qd3 = qd.item(2);

			Ja11 = -np.cos(q2)*np.cos(q1)*l2*qd1+np.sin(q2)*np.sin(q1)*l2*qd2-np.cos(q2+q3)*np.cos(q1)*l3*qd1+np.sin(q2+q3)*np.sin(q1)*l3*qd2+qd3*np.sin(q2+q3)*np.sin(q1)*l3-np.cos(q1)*l1*qd1;
			Ja12 = -np.cos(q2)*np.cos(q1)*l2*qd2+np.sin(q2)*np.sin(q1)*l2*qd1-qd2*np.cos(q1)*l3*np.cos(q2+q3)-qd3*np.cos(q1)*l3*np.cos(q2+q3)+qd1*np.sin(q2+q3)*np.sin(q1)*l3;
			Ja13 = -l3*(np.cos(q2+q3)*np.cos(q1)*qd2+np.cos(q2+q3)*np.cos(q1)*qd3-np.sin(q2+q3)*np.sin(q1)*qd1);
			Ja21 = -np.cos(q2)*np.sin(q1)*l2*qd1-np.sin(q2)*np.cos(q1)*l2*qd2-np.cos(q2+q3)*np.sin(q1)*l3*qd1-np.sin(q2+q3)*np.cos(q1)*l3*qd2-qd3*np.sin(q2+q3)*np.cos(q1)*l3-np.sin(q1)*l1*qd1;
			Ja22 = -np.cos(q2)*np.sin(q1)*l2*qd2-np.sin(q2)*np.cos(q1)*l2*qd1-qd2*np.sin(q1)*l3*np.cos(q2+q3)-qd3*np.sin(q1)*l3*np.cos(q2+q3)-qd1*np.sin(q2+q3)*np.cos(q1)*l3;
			Ja23 = -l3*(np.cos(q2+q3)*np.sin(q1)*qd2+np.cos(q2+q3)*np.sin(q1)*qd3+np.sin(q2+q3)*np.cos(q1)*qd1);
			Ja31 = 0;
			Ja32 = -np.sin(q2)*l2*qd2-qd2*l3*np.sin(q2+q3)-qd3*l3*np.sin(q2+q3);
			Ja33 = -np.sin(q2+q3)*l3*(qd2+qd3);    

			Ja = np.matrix([ [Ja11, Ja12, Ja13], [Ja21, Ja22, Ja23], [Ja31, Ja32, Ja33] ])  
			
			qdd = linalg.solve(Jv,(acc - Ja*qd))

			return qd.flatten(), qdd.flatten()

		except Exception, e:
			print 'Error in Jv: ', e
			return None, None

	## Torque to force mapping and vice versa uses the relationship: tau = J^(T)*f
	def force_to_torque(self, q=None, f=None):
		return self.jacobian_transpose(q)*f

	def torque_to_force(self, q=None, tau=None):
		# f = J^(-T)*tau
		return inv(self.jacobian_transpose(q))*tau

	def nominal_stance(self, bodypose, base_X_surface, qsurface):
		# print 'bodypose: ', bodypose
		# print 'b X surf: ', base_X_surface
		# print 'q surf  : ', qsurface

		qrx = qsurface.item(1)

		world_hip_X_wall_Y = (np.dot(tf.rotation_zyx(bodypose[3:7]), (base_X_surface - COXA_Y)*np.array([ [0.],[1.],[0,] ])) ).item(1) 
		world_hip_X_wall_Z = bodypose.item(2) + (np.dot(tf.rotation_zyx(bodypose[3:7]), np.array([ [0.], [COXA_Y], [0.] ]) ) ).item(2)

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
		world_X_wall_Y = (np.dot(tf.rotation_zyx(bodypose[3:7]), base_X_surface*np.array([ [0.],[1.],[0,] ]))).item(1) 
		base_X_wall_Z  = world_X_wall_Z - bodypose.item(2)

		# transform to base frame
		# base_X_nom = np.dot(tf.rotation_zyx(-bodypose[3:7]), np.array([ [0.],[world_X_wall_Y], [base_X_wall_Z] ]))

		# output: in base frame, from hip to nominal position
		base_hip_X_nom = np.dot(tf.rotation_zyx(-bodypose[3:7]), np.array([ [0.],[world_hip_X_wall_Y], [base_X_wall_Z] ]))
		
		# print world_X_wall_Y, base_X_wall_Z
		# print base_X_nom.transpose()
		# print base_hip_X_nom.transpose()

		# return base_X_nom
		return base_hip_X_nom

# cp  = np.array([ 0.214,-0.05,-0.0851 ])
# cp  = np.array([ 0.214,0.0,-0.0771 ])
# CK = corin_kinematics(1)
# cd = CK.IK(cp)
# qp = cd.transpose()
# print qp
# print CK.FK(qp).transpose()
# print CK.singularity_check(qp)



# cpost = np.matrix([0.214,-0.05,-0.0851])
# # STANCE_WIDTH, 0.+STEP_STROKE/2., 
# jta = joint_angle(cpost)
# print jta #*180/np.pi
# print forwardkinematics(jta)