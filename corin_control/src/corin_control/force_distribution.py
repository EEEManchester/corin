#!/usr/bin/env python
# -*- coding: utf-8 -*-

## QP for solving foot force distribution ##
import sys; sys.dont_write_bytecode = True
import numpy as np
from scipy import linalg
from scipy import integrate
from scipy.integrate import odeint
import matplotlib.pyplot as plt

import matrix_transforms as mtf
from constant import *

from cvxopt import matrix
from cvxopt import solvers

count_i = 0
spline_count = 0.
## cvxopt input form: # P, q, G, h, None, A, b
class QPForceDistribution():
	def __init__(self):
		self.sum_forces = np.zeros((3,1))
		self.sum_moment = np.zeros((3,1))
		self.d_forces = np.zeros((3,1))
		self.d_moment = np.zeros((3,1))

	def solve_example(self):
		# setting via numpy
		Pn = cvxmatrix(np.diag([1,0]), tc='d')
		qn = matrix(np.array([3,4]), tc='d')
		Gn = matrix(np.array([[-1,0],[0,-1],[-1,-3],[2,5],[3,4]]), tc='d')
		hn = matrix(np.array([0,0,-15,100,80]), tc='d')

		# direct setting
		Pn = matrix([[1.0,0.0],[0.0,0.0]])
		qn = matrix([3.0,4.0])
		Gn = matrix([[-1.0,0.0,-1.0,2.0,3.0],[0.0,-1.0,-3.0,5.0,4.0]])
		hn = matrix([0.0,0.0,-15.0,100.0,80.0])

		sol = solvers.qp(Pn,qn,Gn,hn)

		print sol['x']
		# return sol

	def compute_tangent(self, snorm):
		# Find first tangential vector
		if (snorm[0] == 0):
			t1 = np.array([1,0,0])
		elif (snorm[1] == 0):
			t1 = np.array([0,1,0])
		elif (snorm[2] == 0):
			t1 = np.array([0,0,1])
		else:
			# set x,y to 1
			t1 = np.array([1, 1, -1.0 * (snorm[0] + snorm[1])/snorm[2]])
			t1 = t1/np.linalg.norm(t1)

		# Find second tangential vector
		t2 = np.cross(snorm,t1)

		return t1, t2

	# def resolve_force(self, v3ca, w3ca, p_foot, Ig, contact):
	def resolve_force(self, v3gv, v3ca, w3ca, p_foot, x_com, Ig_com, gphase, snorm=0):
		""" QP problem to resolve foot force distribution 
			Input: 	v3gv: Re(1x3) gravity vector
					v3ca: Re(1x3) linear acceleration for body
					w3ca: Re(1x3) angular acceleration for body
					p_foot: Re(3c) array of foot position
					x_com: Re(3x1) CoM position from base
					Ig_com: Re(3x3) Composite Rigid Body Intertia
					gphase: Re(6) gait phase of legs
					snorm: Re(3c) surface normal vector for each leg
			Output: force_vector, Re(18) 							"""

		## Declare variables
		n_contact = len(p_foot)
		gv = matrix(v3gv, (3,1), tc='d') 	# gravitational matrix
		xa = matrix(v3ca, (3,1), tc='d') 	# CoM linear acceleration
		wa = matrix(w3ca, (3,1), tc='d') 	# angular base acceleration
		ig = matrix(Ig_com, tc='d') 		# centroidal moment of inertia
		mu = 0.5; 							# Coefficient of friction

		## Linear equality (system equation)
		A = matrix(np.zeros((6,3*n_contact)), (6,3*n_contact), tc='d')
		b = matrix([ROBOT_MASS*(xa+gv), ig*wa])

		for i in range(0, n_contact):	
			A[:3,(i*3):(i*3+3)]  = np.eye(3)
			A[3:6,(i*3):(i*3+3)] = mtf.skew(p_foot[i])
			
		## =================== Bounded Force ============================ ##
		f_min = 0.
		f_max = 40.

		D  = np.zeros(6);
		D[4] = f_min
		D[5] = f_max
		inq_D = D
		
		## Interpolate upper and lower boundary of inequality constraint
		for i in range(0,n_contact-1):
			# ## Test for interpolation
			# if (i==4 and n_contact == 6):
			# 	print 'yes'
			# 	D[5] = (f_max/10.)*(10 - count_i)	
			# 	D[5] = 20
			# else:
			# 	D[5] = 20

			## do not touch
			inq_D = np.hstack((inq_D, D))
		inq_D = matrix(inq_D, tc='d')

		## Friction cone coefficient - TODO: SET BASED ON SURFACE NORMAL
		inq_C = matrix(np.zeros((6*n_contact, 3*n_contact)), tc='d')
		
		for i in range(0,n_contact):
			
			# Compute according to surface normal
			if (snorm is not 0):
				sinst = snorm[i*3:i*3+3]
				C = np.zeros((6,3))

				## Surface normal direction
				t1, t2 = self.compute_tangent(sinst.flatten())

				# print 't1 ', t1, t2
				C[0,:] = -mu*sinst.flatten() + t1
				C[1,:] = -mu*sinst.flatten() + t2
				C[2,:] = -(mu*sinst.flatten() + t1)
				C[3,:] = -(mu*sinst.flatten() + t2)
				C[4,:] = -sinst.flatten()
				C[5,:] =  sinst.flatten()
				inq_C[(i*6):(i+1)*6,(i*3):(i+1)*3] = matrix(C, tc='d')
				# if (i==0):
				# 	print t1, t2
				# 	print C
				# 	print D
			else:
				C = matrix([ [1,-1,0,0,0,0],[0,0,1,-1,0,0],[-mu,-mu,-mu,-mu,-1,1] ])
				inq_C[(i*6):(i+1)*6,(i*3):(i+1)*3] = C
		
			# if (i == 2):
			# 	print sinst.flatten(), mX(rot_Y(PI/2), sinst.flatten())
			# 	print t1, t2
		# print np.round(C_old,3)
		# print np.round(C,3)

		## Method A: Using system equation with weightage
		s_weight = np.array([1, 1, 1, 1, 1, 1]);
		S = matrix(np.diag(s_weight), tc='d');

		H = 2*A.T*S*A
		q = (-2*b.T*S*A).T
		
		## Set Solver parameters
		solvers.options['abstol']  = 1e-10
		solvers.options['reltol']  = 1e-10
		solvers.options['feastol'] = 1e-10
		solvers.options['show_progress'] = False

		## Solve QP problem
		sol = solvers.qp(H,q, inq_C, inq_D) 

		# print np.round(np.array(sol['x']).transpose(),4)
		# print A*sol['x']

		## Method B: Method 3 with regularization on joint torque
		# alpha = 0.01
		# w_weight = np.array([5, 50, 2])*10e-3;
		# S = matrix(np.diag(w_weight), tc='d');
		# H = 2*(A.T*S*A + alpha*W)
		
		## Rearrange to publish output
		qc = 0
		force_vector = np.zeros((18,1))

		for i in range(0,6):
			if (gphase[i] == 0):
				for j in range(0,3):
					force_vector[i*3+j,0] = -sol['x'][qc]
					qc += 1
			else:
				for j in range(0,3):
					force_vector[i*3+j,0] = 0.0

		## Check solution
		fcounter = 0
		self.sum_forces = self.sum_moment = 0
		for i in range(0,6):
			if (gphase[i]==0):
				self.sum_forces += force_vector[i*3:i*3+3]
				self.sum_moment += np.cross( p_foot[fcounter].reshape(1,3), force_vector[i*3:i*3+3].reshape(1,3)).reshape(3,1)
				fcounter += 1
				# print np.round(force_vector[i*3:i*3+3].flatten(),3)
		self.d_forces = np.array(ROBOT_MASS*(xa+gv))
		self.d_moment = np.array(ig*wa)
		print 'Comp: ', np.round(A*sol['x'],3).flatten()
		print 'Orig: ', np.round(b,3).flatten()
		# error_forces = ROBOT_MASS*(xa+gv) - sum_forces
		# error_moment = ig*wa - sum_moment
		# print np.round(force_vector.flatten(),3)
		# print np.transpose(np.round(error_forces,4))
		# print np.transpose(np.round(error_moment,4))
		# print '========================================='
		
		return force_vector

qprog = QPForceDistribution()
## Body linear and angular parameters

Ig_com = np.eye(3)
xb_com = np.array([0.,0.,0.]) 
xa_com = np.array([0.,0.,0.])
wa_com = np.array([0.,0.,0.])

## Foot position
# p1 = np.array([ [0.125] ,[ 0.285],[ 0.1] ])
# p2 = np.array([ [0.00]  ,[ 0.285],[ 0.1] ])
# p3 = np.array([ [-0.125],[ 0.285],[ 0.1] ])
# p4 = np.array([ [0.125] ,[-0.285],[-0.1] ])
# p5 = np.array([ [0.00]  ,[-0.285],[-0.1] ])
# p6 = np.array([ [-0.125],[-0.285],[-0.1] ])
p1 = np.array([ [0.075] ,[ 0.31],[ 0.05] ])
p2 = np.array([ [0.00]  ,[ 0.31],[ 0.05] ])
p3 = np.array([ [-0.075],[ 0.31],[ 0.05] ])
p4 = np.array([ [0.115] ,[-0.31],[ 0.05] ])
p5 = np.array([ [0.00]  ,[-0.31],[ 0.05] ])
p6 = np.array([ [-0.115],[-0.31],[ 0.05] ])

p_foot = [p1, p2, p3, p4, p5, p6] 			# leg position wrt CoM expressed in world frame
contacts = [0,0,0,0,0,0]
gvset = np.array([0,0,9.81])
# print gvset
snorm = np.zeros((18,1))
for i in range(0,3):
	# snorm[i*3:i*3+3] = mX(rot_X(PI/2.),np.array([0,0,1])).reshape((3,1))
	snorm[i*3:i*3+3] = np.array([0.,-1.,0.]).reshape((3,1))
for i in range(3,6):
	snorm[i*3:i*3+3] = np.array([0,1,0]).reshape((3,1))

force_vector = qprog.resolve_force(gvset, xa_com, wa_com, p_foot, xb_com, Ig_com, contacts, snorm)
# print np.round(-force_vector.flatten(),5)

# f = open('qp_discont.csv', 'w')

# for count_i in range(0,22):
# 	if (count_i < 10):
# 		p_foot = [p1, p2, p3, p4, p5, p6] 			
# 		contacts = [0,0,0,0,0,0]
# 	else:
# 		p_foot = [p1, p2, p3, p4, p5] 		
# 		contacts = [0,0,0,0,0,1]

# 	force_vector, twist = qprog.resolve_force(x_com, w_com, p_foot, Ig_com, contacts)
# 	data = np.array(force_vector).flatten()
	
# 	long_stq = ""
# 	for q in range(0,18):
# 		long_stq = long_stq + "," + str(data.item(q))

# 	# f.write(long_stq + '\n')

##########################################################################################
## 										Sample 											##
##########################################################################################
## e.g. min (x) for (Ax-b)^2
## Taken from https://scaron.info/blog/quadratic-programming-in-python.html

# Matrix variables
# Mn = np.array([[1., 2., 0.], [-8., 3., 2.], [0., 1., 1.]]) 	# objective function A
# Pn = np.dot(M.T, M)
# qn = np.dot(np.array([3., 2., 3.]), M).reshape((3,))			# objective function B
# Gn = np.array([[1., 2., 1.], [2., 0., 1.], [-1., 2., -1.]]) 	# inequality constraint, A
# hn = np.array([3., 2., -2.]).reshape((3,)) 					# inequality constraint, b

# print cvxopt_solve_qp(P, q, G, h)
# print quadprog_solve_qp(P, q, G, h)
def cvxopt_solve_qp(P, q, G=None, h=None, A=None, b=None):
	# make sure P is symmetric
	P = .5 * (P + P.T)  
	args = [matrix(P), matrix(q)]
	if G is not None:
		args.extend([matrix(G), matrix(h)])
		if A is not None:
			args.extend([matrix(A), matrix(b)])

	sol = solvers.qp(*args)

	if 'optimal' not in sol['status']:
		return None

	return np.array(sol['x']).reshape((P.shape[1],))

def quadprog_solve_qp(P, q, G=None, h=None, A=None, b=None):
    qp_G = .5 * (P + P.T)   # make sure P is symmetric
    qp_a = -q
    if A is not None:
        qp_C = -np.vstack([A, G]).T
        qp_b = -np.hstack([b, h])
        meq = A.shape[0]
    else:  # no equality constraint
        qp_C = -G.T
        qp_b = -h
        meq = 0
    return quadprog.solve_qp(qp_G, qp_a, qp_C, qp_b, meq)[0]