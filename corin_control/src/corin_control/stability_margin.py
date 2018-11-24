#!/usr/bin/env python

""" Evaluates the stability margin of the robot """
import sys; sys.dont_write_bytecode = True
import numpy as np
from scipy import linalg
from scipy import spatial

from constant import *
import matplotlib.pyplot as plt

def shortest_distance(p1, p2):

	v_lr  = p2 - p1 												# support edge vector
	p_p   = ( np.dot(-p1,v_lr)/np.linalg.norm(v_lr)**2 )*v_lr 		# foot vector projected onto support edge
	p_f   = p1 + p_p  												# rejection vector from base
	return np.sqrt(p_f[0]**2 + p_f[1]**2) 							# magnitude of rejection vector

class StabilityMargin():
	def __init__(self):
		self.sm = np.zeros(2) 	# current stability margin
		self.min = 0.
		self.valid = True

	def support_leg_identification(self, leg_phase):
		""" Identifies front and rear legs in support phase """ 
		p_state = []

		## Front SM
		if (leg_phase[0]==0):
			p_state.append(0)
		elif(leg_phase[1]==0):
			p_state.append(1)
		else:
			print "error"

		if (leg_phase[3]==0):
			p_state.append(3)
		elif(leg_phase[4]==0):
			p_state.append(4)
		else:
			print "error"

		## Rear SM
		if (leg_phase[2]==0):
			p_state.append(2)
		elif(leg_phase[1]==0):
			p_state.append(1)
		else:
			print "error"

		if (leg_phase[5]==0):
			p_state.append(5)
		elif(leg_phase[4]==0):
			p_state.append(4)
		else:
			print "error"
		return p_state

	def LSM(self, p, leg_phase):
		""" computes longitudinal stability margin 						"""
		""" Input  : 1) p -> array of foot positions: world_base_X_foot
				 	 2) leg_phase -> a list on the phase of legs
			Output : array of the front and rear magnitude of vector 
						perpendicular to support edge					"""

		## Define Variables ##
		sm = np.zeros(2)
		pf = self.support_leg_identification(leg_phase)
		# print 'support ', pf, leg_phase
		for i in range(0,2):
			p1 = p[pf[0+i*2]]
			p2 = p[pf[1+i*2]]
			
			v_lr  = p2 - p1 												# support edge vector
			p_p   = ( np.dot(-p1,v_lr)/np.linalg.norm(v_lr)**2 )*v_lr 		# foot vector projected onto support edge
			p_f   = p1 + p_p  												# rejection vector from base
			sm[i] = np.sqrt(p_f[0]**2 + p_f[1]**2) 							# magnitude of rejection vector

			if (i==0 and p_f[0] < 0.):
				sm[i] = -1.
			elif (i==1 and p_f[0] > 0.):
				sm[i] = -1.

			# if (i==1):
			# 	print np.round(p1,4)
			# 	print np.round(p2,4)
			# 	print p_f
		self.sm = sm
		return sm

	def force_moment(self):
		pass

	def point_in_convex(self, p, full_stance, leg_phase, disp=False):

		if disp:
			print p
			print full_stance
			print leg_phase

		# Get list of legs in stance phase
		stance = []
		for j in range(0,6):
			if leg_phase[j] == 0:
				stance.append(full_stance[j][0:2])

		# Generate convex hull for support polygon
		vertices = np.asarray(stance)
		hull = spatial.ConvexHull(vertices)

		ns = len(vertices[hull.vertices,0])
		xh = vertices[hull.vertices,0].reshape((ns,1))
		yh = vertices[hull.vertices,1].reshape((ns,1))
		hull_arr = np.concatenate((xh,yh),axis=1)
		
		# Check if CoB lies inside convex hull
		if not isinstance(hull, spatial.Delaunay):
			de_hull = spatial.Delaunay(hull_arr)
		self.valid = de_hull.find_simplex(p[0:2])>=0

		# Get shortest distance
		darr = []
		for i in range(0,ns):
			p1 = hull_arr[i,:]
			if i==ns-1:
				p2 = hull_arr[0,:]
			else:
				p2 = hull_arr[i+1,:]
			darr.append(shortest_distance(p1, p2))
		
		self.min = min(darr) if self.valid else -min(darr)
		
		# plt.plot(vertices[:,0], vertices[:,1], 'o')
		# plt.plot(p[0], p[1], '*')
		# for simplex in hull.simplices:
		# 	plt.plot(vertices[simplex, 0], vertices[simplex, 1], 'k-')
		# plt.show()

		return self.valid, self.min #de_hull.find_simplex(p[0:2])>=0

	def check_angle(self, full_stance, leg_phase, disp=False):

		if disp:
			print p
			print full_stance
			print leg_phase

		# Get list of legs in stance phase
		stance = []
		for j in range(0,6):
			if leg_phase[j] == 0:
				stance.append(full_stance[j][0:2])

		## Check theta method
		q1 = np.round(np.rad2deg(np.arctan(full_stance[0][0]/full_stance[0][1])), 3)
		q2 = np.round(np.rad2deg(np.arctan(-1.*full_stance[4][0]/full_stance[4][1])), 3)
		print 'theta ', q1, q2
## ================================================================================================ ##
## 												TESTING 											##
## ================================================================================================ ##

BASE_X_LF_FOOT = np.array([0., 2., 0.1])
BASE_X_LM_FOOT = np.array([0., 2., 0.1])
BASE_X_LR_FOOT = np.array([-1., 2., 0.1])
BASE_X_RF_FOOT = np.array([1.,-2., 0.1])
BASE_X_RM_FOOT = np.array([0.,-2., 0.1])
BASE_X_RR_FOOT = np.array([-0.5,-2., 0.1])

Legs = [BASE_X_LF_FOOT, BASE_X_LM_FOOT, BASE_X_LR_FOOT,
		BASE_X_RF_FOOT, BASE_X_RM_FOOT, BASE_X_RR_FOOT]

Legs = [np.array([0.06685698, 0.30925057, 0.        ]), np.array([-0.00814302,  0.30925057,  0.        ]), np.array([-0.08314302,  0.30925057,  0.        ]), np.array([ 1.53567418e-01, -3.10811033e-01,  1.38777878e-17]), np.array([ 0.03889217, -0.31039762,  0.        ]), np.array([-7.55365249e-02, -3.10348441e-01, -2.77555756e-17])]
gphase = [0,0,1,0,0,1]
p = np.array([0.06,0.,0.])

Legs = [np.array([0.09919876, 0.12064476, 0.29351066]), np.array([-0.01680924,  0.12064476,  0.27825104]), np.array([-0.13281724,  0.12064476,  0.26299141]), np.array([ 0.12504486, -0.07413727, -0.42      ]), np.array([ 0.03175676, -0.08795496, -0.41690475]), np.array([-0.10295665, -0.10439134, -0.42      ])]
gphase = [0, 0, 0, 0, 1, 0]
p = np.zeros(3) #
# p = np.array([ 0.25146748, -0.52064476,  0.42      ])

Legs = [np.array([-0.1630743 ,  0.32203309,  0.        ]), np.array([-0.18062982,  0.22510528,  0.        ]), np.array([-2.78601965e-01,  2.25076089e-01,  1.38777878e-17]), np.array([ 4.35710618e-01, -5.12294616e-02,  4.16333634e-17]), np.array([ 0.15111983, -0.37431678,  0.        ]), np.array([ 6.95810360e-02, -3.74002016e-01, -2.77555756e-17])]
Legs = [np.array([-1.64207938e-01,  3.22204156e-01, -1.38777878e-17]), np.array([-0.18176346,  0.22527634,  0.        ]), np.array([-2.79735602e-01,  2.25247153e-01,  4.16333634e-17]), np.array([ 0.40436391, -0.00871848,  0.        ]), np.array([ 1.90498403e-01, -3.73826321e-01, -2.08166817e-17]), np.array([ 6.84473985e-02, -3.73830951e-01,  1.38777878e-17])]
gphase = [0, 0, 0, 1, 0, 0]
Legs = [np.array([-2.21998450e-01,  2.80855690e-01,  5.55111512e-17]), np.array([-0.22182818,  0.18271897,  0.        ]), np.array([-3.18639671e-01,  1.65395701e-01,  1.38777878e-17]), np.array([ 3.77314358e-01, -2.32872695e-02,  5.55111512e-17]), np.array([ 3.77292937e-01, -1.54514558e-01,  1.38777878e-17]), np.array([ 0.01116949, -0.43351627,  0.        ])]
gphase =  [0, 0, 0, 0, 0, 1]
SM = StabilityMargin()
# print SM.LSM(Legs, gphase)
valid, sm = SM.point_in_convex(p, Legs, gphase)
print valid, sm
