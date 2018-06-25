#!/usr/bin/env python
# -*- coding: utf-8 -*-

## QP for solving foot force distribution ##
import sys; sys.dont_write_bytecode = True
import numpy as np
from scipy import linalg
from scipy import integrate
from scipy.integrate import odeint
import matplotlib.pyplot as plt

import transformations as tf
from constant import *

from cvxopt import matrix
from cvxopt import solvers

import quadprog

count_i = 0
spline_count = 0.
## cvxopt input form: # P, q, G, h, None, A, b
class QuadProgClass():
	def __init__(self):
		self.ode_initial  = [0, 0]
		

	def solve(self):
		# setting via numpy
		P = cvxmatrix(np.diag([1,0]), tc='d')
		q = matrix(np.array([3,4]), tc='d')
		G = matrix(np.array([[-1,0],[0,-1],[-1,-3],[2,5],[3,4]]), tc='d')
		h = matrix(np.array([0,0,-15,100,80]), tc='d')

		# direct setting
		P = matrix([[1.0,0.0],[0.0,0.0]])
		q = matrix([3.0,4.0])
		G = matrix([[-1.0,0.0,-1.0,2.0,3.0],[0.0,-1.0,-3.0,5.0,4.0]])
		h = matrix([0.0,0.0,-15.0,100.0,80.0])

		sol = solvers.qp(P,q,G,h)

		print sol['x']
		# return sol

	# def cvxopt_qp_foot(self, xa_com, wa_com, p_com, Ig, contact):
	def cvxopt_qp_foot(self, xa_com, wa_com, p_com, Ig_com, cstate):
		## xa_com = Re(1x3) linear acceleration for body
		## wa_com = Re(1x3) angular acceleration for body
		## p_com  = Re(3c)  array of foot position
		
		n_contact = len(p_com)
		
		## Declare variables
		gv = matrix([0.0, 0.0, 9.81]) 			# gravitational matrix
		xa = matrix(xa_com, (3,1), tc='d') 		# CoM linear acceleration
		wa = matrix(wa_com, (3,1), tc='d') 		# angular base acceleration
		ig = matrix(Ig_com, tc='d') 			# centroidal moment of inertia

		AU = np.eye(3)
		AL = tf.skew(p_com[0])
		Hs = np.array([0.5, 0.5, 1]) 	# hessian weightage for single foot
		HC = Hs

		for i in range(1,n_contact):
			AU = np.hstack(( AU, np.eye(3) ))
			AL = np.hstack(( AL, tf.skew(p_com[i]))) 
			HC = np.hstack(( HC, Hs))

		## Linear equality (system equation)
		## Arrange into Ax = b form
		A = matrix([matrix(AU, (3,3*n_contact), tc='d'), matrix(AL, (3,3*n_contact), tc='d')])
		b = matrix([MASS*(xa+gv), ig*wa])
		# print b
		H = matrix(np.diag(HC), tc='d')
		q = matrix(np.ones(n_contact*3), tc='d')

		## Linear inequality (friction constraint)
		# surface tangent and normal directions
		tx = np.array([1, 0, 0]);
		ty = np.array([0, 1, 0]);
		tn = np.array([0, 0, 1]);

		# Coefficient of friction
		mu = 0.5;

		## =================== UnBounded Force ============================ ##
		# D  = np.zeros(5);
		# inq_D = D

		# for i in range(0,n_contact-1):
		# 	inq_D = np.hstack((inq_D, D))
		# inq_D = matrix(inq_D, tc='d')

		# C = matrix([ [1,-1,0,0,0],[0,0,1,-1,0],[-mu,-mu,-mu,-mu,-1] ])
		# inq_C = matrix(np.zeros((5*n_contact, 3*n_contact)), tc='d')
		
		# for i in range(0,n_contact):
		# 	inq_C[(i*5):(i+1)*5,(i*3):(i+1)*3] = C

		## =================== Bounded Force ============================ ##
		f_min = 1.
		f_max = 20.

		D  = np.zeros(6);
		D[4] = f_min
		D[5] = f_max
		inq_D = D
		# print n_contact
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

		# if (count_i < 11):
		# 	print inq_D

		C = matrix([ [1,-1,0,0,0,0],[0,0,1,-1,0,0],[-mu,-mu,-mu,-mu,-1,1] ])
		inq_C = matrix(np.zeros((6*n_contact, 3*n_contact)), tc='d')
		
		for i in range(0,n_contact):
			inq_C[(i*6):(i+1)*6,(i*3):(i+1)*3] = C

		# Method 1: Using sys. eq. as constraints
		# sol = solvers.qp(H,q, inq_C, inq_D, A,b) 	
		# print np.round(sol['x'],3)

		## Method 2: Using system equation as Hessian matrices
		H = 2*A.T*A
		q = (-2*b.T*A).T

		# sol = solvers.qp(H,q, -inq_C, inq_D) 	
		# print np.round(sol['x'],8)
		
		## Method 3: Using system equation with weightage
		s_weight = np.array([1, 1, 10, 10, 10, 10]);
		S = matrix(np.diag(s_weight), tc='d');

		H = 2*A.T*S*A
		q = (-2*b.T*S*A).T

		solvers.options['show_progress'] = False
		sol = solvers.qp(H,q, inq_C, inq_D) 	
		
		# print np.round(np.array(sol['x']).transpose(),4)
		# print A*sol['x']

		## Method 4: Method 3 with regularization on joint torque
		# alpha = 0.01
		# w_weight = np.array([5, 50, 2])*10e-3;
		# S = matrix(np.diag(w_weight), tc='d');
		# H = 2*(A.T*S*A + alpha*W)
		
		## Rearrange to publish output
		qc = 0
		force_vector = np.zeros((18,1))

		# if (n_contact == 5):
		for i in range(0,6):
			if (cstate[i] == 0):
				for j in range(0,3):
					force_vector[i*3+j,0] = sol['x'][qc]
					qc += 1
			else:
				for j in range(0,3):
					force_vector[i*3+j,0] = 0.0

		# print force_vector
		# print count_i
		return force_vector, b

qprog = QuadProgClass()
## Body linear and angular parameters

Ig_com = np.zeros((3,3))
x_com = np.array([0,0,0]) # + MASS*np.array([ [0.,0.,g] ])
w_com = np.array([0.,0.,0.])
## Foot position
p1 = np.array([  [2],  [1], [-1] ])
p2 = np.array([ [-2],  [1], [-1] ])
p3 = np.array([ [ 0], [-1], [-1] ])

p4 = np.array([ [-2], [-1],[0] ])
p5 = np.array([  [2], [-1],[0] ])
p6 = np.array([  [0], [-1],[0] ])

p1 = np.array([ [0.125] ,[0.285],[-0.1] ])
p2 = np.array([ [0.00]  ,[0.285],[-0.1] ])
p3 = np.array([ [-0.125],[0.285],[-0.1] ])
p4 = np.array([ [0.125] ,[-0.285],[-0.1] ])
p5 = np.array([ [0.00]  ,[-0.285],[-0.1] ])
p6 = np.array([ [-0.125],[-0.285],[-0.1] ])

p_com = [p1, p2, p3, p4] 			# leg position wrt CoM expressed in world frame
contacts = [True,False,True,False,False,False]
# force_vector, twist = qprog.cvxopt_qp_foot(x_com, w_com, p_com, Ig_com, contacts)
# print wrench[0:3]

# f = open('qp_discont.csv', 'w')

# for count_i in range(0,22):
# 	if (count_i < 10):
# 		p_com = [p1, p2, p3, p4, p5, p6] 			
# 		contacts = [0,0,0,0,0,0]
# 	else:
# 		p_com = [p1, p2, p3, p4, p5] 		
# 		contacts = [0,0,0,0,0,1]

# 	force_vector, twist = qprog.cvxopt_qp_foot(x_com, w_com, p_com, Ig_com, contacts)
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
M = np.array([[1., 2., 0.], [-8., 3., 2.], [0., 1., 1.]]) 	# objective function A
P = np.dot(M.T, M)
q = np.dot(np.array([3., 2., 3.]), M).reshape((3,))			# objective function B
G = np.array([[1., 2., 1.], [2., 0., 1.], [-1., 2., -1.]]) 	# inequality constraint, A
h = np.array([3., 2., -2.]).reshape((3,)) 					# inequality constraint, b

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