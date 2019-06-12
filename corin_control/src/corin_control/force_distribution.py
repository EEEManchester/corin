#!/usr/bin/env python
# -*- coding: utf-8 -*-

## QP for solving foot force distribution ##
import sys; sys.dont_write_bytecode = True
import numpy as np
from scipy import linalg
from scipy import integrate
from scipy.integrate import odeint
import matplotlib.pyplot as plt
import copy

import kdl
import matrix_transforms as mtf
from constant import *

from cvxopt import matrix
from cvxopt import solvers
from qpsolvers import solve_qp
import quadprog

def isPD(B):
    """Returns true when input is positive-definite, via Cholesky"""
    try:
        _ = linalg.cholesky(B)
        return True
    except linalg.LinAlgError:
        return False

def nearestPD(A):
    """Find the nearest positive-definite matrix to input
    [1] https://www.mathworks.com/matlabcentral/fileexchange/42885-nearestspd
    [2] N.J. Higham, "Computing a nearest symmetric positive semidefinite
    matrix" (1988): https://doi.org/10.1016/0024-3795(88)90223-6
    """

    B = (A + A.T) / 2
    _, s, V = linalg.svd(B)
    # B = UH: polar decomposition
    H = np.dot(V.T, np.dot(np.diag(s), V))
    # A2: unique positie approximation of A in the Frobenius norm
    A2 = (B + H) / 2

    A3 = (A2 + A2.T) / 2

    if isPD(A3):
        return A3
    
    spacing = np.spacing(linalg.norm(A))
    # The above is different from [1]. It appears that MATLAB's `chol` Cholesky
    # decomposition will accept matrixes with exactly 0-eigenvalue, whereas
    # Numpy's will not. So where [1] uses `eps(mineig)` (where `eps` is Matlab
    # for `np.spacing`), we use the above definition. CAVEAT: our `spacing`
    # will be much larger than [1]'s `eps(mineig)`, since `mineig` is usually on
    # the order of 1e-16, and `eps(1e-16)` is on the order of 1e-34, whereas
    # `spacing` will, for Gaussian random matrixes of small dimension, be on
    # othe order of 1e-16. In practice, both ways converge, as the unit test
    # below suggests.
    I = np.eye(A.shape[0])
    k = 1
    while not isPD(A3):
        mineig = np.min(np.real(linalg.eigvals(A3)))
        A3 += I * (-mineig * k**2 + spacing)
        k += 1

    return A3

def cvxopt_solve_qp(P, q, inq_C, inq_D):
	solvers.options['abstol']  = 1e-10
	solvers.options['reltol']  = 1e-10
	solvers.options['feastol'] = 1e-10
	solvers.options['show_progress'] = False

	## Solve QP problem
	tmp_sol = solvers.qp(matrix(P, tc='d'), 
							matrix(q, tc='d'), 
							matrix(inq_C, tc='d'), 
							matrix(inq_D, tc='d')) 
	cvx_sol = np.array(tmp_sol['x']).flatten()

	return cvx_sol

def quadprog_solve_qp(P, q, G=None, h=None, A=None, b=None):
	# make sure P is symmetric and positive definite
	qp_G = 0.5 * (P + P.transpose()) + np.eye(len(P))*(0.0000001)
	# qp_G = nearestPD(P)
	qp_a = -q

	if A is not None:
		qp_C = -np.vstack([A, G]).T
		qp_b = -np.hstack([b, h])
		meq = A.shape[0]
	else:  # no equality constraint
		qp_C = -G.T
		qp_b = -h
		meq = 0
	
	try:
		sol_full = quadprog.solve_qp(qp_G, qp_a, qp_C, qp_b, meq)
		return sol_full[0]
	except ValueError as e:
		print e
		return None

# def leg_jacobian(q):


class QPForceDistribution():
	def __init__(self):
		self.sum_forces = np.zeros(3)
		self.sum_moments = np.zeros(3)
		self.desired_forces = np.zeros(3)
		self.desired_moments = np.zeros(3)
		self.KDL = kdl.KDL()

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
			t1 = t1/linalg.norm(t1)

		# Find second tangential vector
		t2 = np.cross(snorm,t1)

		return t1, t2

	def resolve_force(self, v3ca, w3ca, p_foot, x_com, Ig_com, gphase, fmax=None, snorm=0, qb=None, q=None):
		""" QP problem to resolve foot force distribution 
			Input: 	v3ca: Re(1x3) linear acceleration for body
					w3ca: Re(1x3) angular acceleration for body
					p_foot: Re(3c) array of foot position
					x_com: Re(3x1) CoM position from base
					Ig_com: Re(3x3) Composite Rigid Body Intertia
					gphase: Re(6) gait phase of legs
					fmax: Re(6) maximum force constraint for each leg 
					snorm: Re(3c) surface normal vector for each leg
			Output: force_vector, Re(18) 							"""

		## Declare variables
		n_contact = len(p_foot)
		index_c = [i for i, j in enumerate(contacts) if j == 0.]
		gv = np.array([0., 0., G]) 	# gravitational vector
		mu = SURFACE_FRICTION		# Coefficient of friction
		
		## Linear equality (system equation)
		A = np.zeros((6,3*n_contact))
		b = np.array([ROBOT_MASS*(v3ca.reshape((3,))+gv), 
						mX(Ig_com, w3ca).reshape((3,))]).flatten()
		
		for i in range(0, n_contact):	
			A[:3,(i*3):(i*3+3)]  = np.eye(3)
			A[3:6,(i*3):(i*3+3)] = mtf.skew(p_foot[i])
		
		W = np.zeros((3*n_contact, 3*n_contact))	# secondary hessian matrix
		J = np.zeros((3*n_contact, 3*n_contact))	# contact jacobian matrix

		s_weight = np.array([1, 1, 1, 1, 1, 1]);
		w_weight = np.array([5, 50, 2])*10e-3;
		# print A
		# print b
		## =================== Bounded Force ============================ ##
		
		## Interpolate upper and lower boundary of inequality constraint
		inq_D = np.zeros((6*n_contact,))
		for i in range(0, n_contact):
			inq_D[6*i+4,] = 0.0
			inq_D[6*i+5,] = fmax[i]
		
		## Friction cone coefficient - TODO: SET BASED ON SURFACE NORMAL
		inq_C = np.zeros((6*n_contact, 3*n_contact))
		
		for i in range(0,n_contact):
			
			# Compute according to surface normal
			if (snorm is not 0):
				# sinst = snorm[i*3:i*3+3]
				sinst = snorm[i]
			else:
				sinst = np.array([0., 0., 1.])

			# Calculate tangent to surface normal
			t1, t2 = self.compute_tangent(sinst.flatten())

			C = np.zeros((6,3))
			C[0,:] = -mu*sinst.flatten() + t1
			C[1,:] = -mu*sinst.flatten() + t2
			C[2,:] = -(mu*sinst.flatten() + t1)
			C[3,:] = -(mu*sinst.flatten() + t2)
			C[4,:] = -sinst.flatten()
			C[5,:] =  sinst.flatten()
			inq_C[(i*6):(i+1)*6,(i*3):(i+1)*3] = C
			
			# Indirect torque minimization
			if q is not None:
				leg_no = index_c[i]
				J[(i*3):(i*3)+3, (i*3):(i*3)+3] = self.KDL.world_leg_jacobian(leg_no, qb, q[(i*3):(i*3)+3])
				W[(i*3):(i*3)+3, (i*3):(i*3)+3] = np.diag(w_weight)
			
		# print np.round(J,3)
		# print np.round(W,4)
		# print np.round(inq_C,3)
		# print np.round(inq_D.flatten(),3)

		## Method A: Using system equation with weightage
		S = np.diag(s_weight)

		H = 2*mX(A.T, S, A)
		q = (-2*mX(b.T, S, A)).T

		## Method B: System equation with weightage AND regularization on joint torque
		alpha = 0.01
		H = 2*mX(A.T, S, A) + alpha*mX(J.T, W, J)
		q = (-2*mX(b.T, S, A)).T

		## Solve QP
		# qpg_sol = cvxopt_solve_qp(H, q, inq_C, inq_D)
		qpg_sol = quadprog_solve_qp(H, q, inq_C, inq_D)

		if qpg_sol is None:
			print 'Error!'
			# qpg_sol = quadprog_solve_qp(H, q, inq_C, inq_D+0.001)
			print v3ca.flatten(), w3ca.flatten(), x_com
			print p_foot
			print gphase
			print fmax
			
		qp_sol = qpg_sol

		## Rearrange to publish output
		qc = 0
		force_vector = np.zeros((18,))

		for i in range(0,6):
			if (gphase[i] == 0):
				for j in range(0,3):
					force_vector[i*3+j,] = qp_sol[qc]
					qc += 1
			else:
				for j in range(0,3):
					force_vector[i*3+j,] = 0.0
		# print np.round(inq_D[15:18].flatten(),3)
		# print np.round(force_vector[15:18],3)
		## Check solution
		fcounter = 0
		## Reset variables
		self.sum_forces = np.zeros(3)
		self.sum_moments = np.zeros(3)
		for i in range(0,6):
			if (gphase[i]==0):
				self.sum_forces  += force_vector[i*3:i*3+3]
				self.sum_moments += np.cross( p_foot[fcounter].reshape(1,3), force_vector[i*3:i*3+3].reshape(1,3)).flatten()
				fcounter += 1

		self.desired_forces  = b[0:3] # ROBOT_MASS*(v3ca+gv)
		self.desired_moments = b[3:6] # Ig_com*w3ca
		error_forces = self.desired_forces - self.sum_forces
		error_moment = self.desired_moments - self.sum_moments
		# print np.round(force_vector.flatten(),3)
		# print np.transpose(np.round(self.sum_forces,3))
		# print np.transpose(np.round(self.desired_forces,4))
		# print np.transpose(np.round(self.sum_forces,4))
		print 'Fe :', np.transpose(np.round(error_forces,4))
		# print 'Md :', np.transpose(np.round(self.desired_moments,4))
		# print 'Mc :', np.transpose(np.round(self.sum_moments,4))
		print 'Me :', np.transpose(np.round(error_moment,4))
		# print '========================================='
		
		return force_vector.reshape((18,1))

## ================================================================================================ ##
## 												TESTING 											##
## ================================================================================================ ##

qprog = QPForceDistribution()
## Body linear and angular parameters

Ig_com = np.eye(3)
xb_com = np.array([0.,0.,0.]).reshape((3,1)) 
xa_com = np.array([0.,0.,0.]).reshape((3,1))
wa_com = np.array([0.,0.,0.]).reshape((3,1))

## Test set - all legs in contact
## Foot position - CoM to foot wrt world frame
p1 = np.array([ [0.125] ,[ 0.285],[-0.1] ])
p2 = np.array([ [0.00]  ,[ 0.285],[-0.1] ])
p3 = np.array([ [-0.125],[ 0.285],[-0.1] ])
p4 = np.array([ [0.125] ,[-0.285],[-0.1] ])
p5 = np.array([ [0.00]  ,[-0.285],[-0.1] ])
p6 = np.array([ [-0.125],[-0.285],[-0.1] ])
# p1 = np.array([ [0.075] ,[ 0.31],[ -0.05] ])
# p2 = np.array([ [0.00]  ,[ 0.31],[ -0.05] ])
# p3 = np.array([ [-0.075],[ 0.31],[ -0.05] ])
# p4 = np.array([ [0.115] ,[-0.31],[ -0.05] ])
# p5 = np.array([ [0.00]  ,[-0.31],[ -0.05] ])
# p6 = np.array([ [-0.115],[-0.31],[ -0.05] ])

p_foot = [p1, p2, p3, p4, p5, p6] 			# leg position wrt CoM expressed in world frame
contacts = [0,0,0,0,0,0]

p_foot = [p1, p3, p5] 			# leg position wrt CoM expressed in world frame
contacts = [0,1,0,1,0,1]

snorm =  []
snorm.append(np.array([0., -1., 0.]))
snorm.append(np.array([0., -1., 0.]))
snorm.append(np.array([0., 1., 0.]))
# for i in range(0,3):
# 	snorm.append(np.array([0., -1., 0.]))
# for i in range(3,6):
# 	snorm.append(np.array([0., 1., 0.]))
farr = [F_MAX,F_MAX,F_MAX,F_MAX,F_MAX,F_MAX]

## Test set - 2 legs in contact
# c2 = np.array([ 0,  0.3, 0.0])
# c5 = np.array([ 0, -0.3, 0.0])
# pb = np.array([ 0,  0.0, 0.1])
# p2 = (-pb + c2).reshape((3,1))
# p5 = (-pb + c5).reshape((3,1))
# p_foot = [p2,p5]
# contacts = [1,0,1,1,0,1]
# q = np.array([0, 0.3381, -1.8523, 0, 0.3381, -1.8523])
qb = np.zeros(3)

# snorm = np.array([0., -1., 0., 0., 1., 0.])
# farr = [80., 80.]

## Test set - Inconsistent solution 
# xb_com = np.array( [[ 0.06581812], [ 0.        ], [ 0.]]) 
# wb_com = np.array( [[ 0.],[ 0.1],[ 0.]] )
# xb_com = np.array( [ -1.93852815e-03,   7.85639857e-05,  -7.98505770e-05])
# p_foot = [np.array([ 0.19890486,  0.25071915, -0.0983407 ]), np.array([ 0.05235609,  0.2998471 , -0.09992533]), 
# 			np.array([-0.19749435,  0.2510643 , -0.09987582]), 
# 			np.array([ 0.30152783, -0.25136944, -0.09986481]), 
# 			np.array([ 0.0516551 , -0.30015232, -0.09986504]), 
# 			np.array([-0.1987471 , -0.25067399, -0.09986653])]
# contacts = [0, 0, 0, 0, 0, 0]
# farr = [0.0, 80.0, 80.0, 80.0, 80.0, 80.0]

force_vector = qprog.resolve_force(xa_com, wa_com, p_foot, xb_com, Ig_com, contacts, farr, snorm)
# force_vector = qprog.resolve_force(xa_com, wa_com, p_foot, xb_com, Ig_com, contacts, farr, snorm, qb, q)
print np.round(force_vector.flatten(),3)

# from robot_transforms import *
# F6c = ArrayVector6D()
# snorm = np.array([1.,0.,1.])
# F6c.world_X_foot[:3] = np.array([2.,5.,10.]).reshape((3,1))

# print np.linalg.norm(mX(np.diag(snorm), F6c.world_X_foot[0:3]))


