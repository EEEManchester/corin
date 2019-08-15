#!/usr/bin/env python

""" Evaluates the stability margin of the robot """
import sys; sys.dont_write_bytecode = True
import numpy as np
from scipy import linalg
from scipy import spatial
from matrix_transforms import *
from termcolor import colored

from constant import *
import matplotlib.pyplot as plt
import stabilipy

def shortest_distance(p, p1, p2):

	v_lr  = p2 - p1 												# support edge vector
	p_p   = ( np.dot(-p1+p,v_lr)/np.linalg.norm(v_lr)**2 )*v_lr 		# foot vector projected onto support edge
	p_f   = p1 + p_p  												# rejection vector from base
	return np.sqrt(p_f[0]**2 + p_f[1]**2) 							# magnitude of rejection vector

class StabilityMargin():
	def __init__(self):
		self.sm = np.zeros(2) 	# current stability margin
		self.min = 0.
		self.valid = True
		self.convex_hull = None
		self.Poly = stabilipy.StabilityPolygon(ROBOT_MASS, 2, -9.81)
		self.Poly.display_final = True
		self.prev_topology = [1]*6

	def compute_support_region(self, stack_world_X_foot, stack_normals):
		""" Computes the stability region for a set of fixed contacts - Bretl2008 """

		print stack_world_X_foot
		print stack_normals
		self.Poly.contacts = [stabilipy.Contact(SURFACE_FRICTION, np.array(p).T,
								stabilipy.utils.normalize(np.array(n).T))
								for p, n in zip(stack_world_X_foot, stack_normals)]
		try:
			self.Poly.compute(stabilipy.Mode.best, epsilon=1e-3, 
													maxIter=10, 
													solver='cdd',
													plot_error=False,
													plot_init=False,
													plot_step=False,
													record_anim=False,
													plot_direction=False,
													plot_final=False)
			# Construct convex hull from inner_polygon
			self.convex_hull = self.Poly.inner_polyhedron()
			
			# Convex hull triangulation
			# de_hull = spatial.Delaunay(self.convex_hull[:,0:2])
			# print de_hull.simplices
		except RuntimeError,e:
			print stack_world_X_foot
			print stack_normals
			print colored('ERROR: Support region - %s'%e,'red')

	def point_in_convex(self, p, disp=False):
		""" Checks if a point is in convex hull and distance to the edge """
		if disp:
			print 'CoM: ', p.flatten()
			print 'Hull: \n', self.convex_hull

		self.valid, self.min = self.Poly.point_in_hull(p)

		return self.valid, self.min #de_hull.find_simplex(p[0:2])>=0

	def force_moment(self, p, forces, world_X_foot, normals):

		equilibrium = True
		## Check force and moment constraints
		gv = np.array([0., 0., G])
		mg = ROBOT_MASS*gv
		sum_forces = np.zeros((3,1)) - mg.reshape((3,1))
		sum_moment = np.zeros((3,1)) - np.cross(p, mg).reshape((3,1))
		nc = len(world_X_foot)

		for j in range(nc):
			sum_forces += forces[j]
			sum_moment += np.cross(world_X_foot[j], forces[j].flatten()).reshape((3,1))
			# print 'force : ', np.round(forces[j].flatten(),3)
			# print 'posit : ', np.round(world_X_foot[j],3)
			# print np.round(np.cross(world_X_foot[j], forces[j].flatten()),3)
			# print normals[j]
		norm_f = np.linalg.norm(sum_forces)
		norm_m = np.linalg.norm(sum_moment)
		norm_e = np.linalg.norm(norm_f + norm_m)
		if norm_e > 1.0:
			print colored("ERROR: Equilibrium - Force/moment exceeds threshold",'red')
			equilibrium = False
		# print 'sum forces: ', sum_forces.flatten()
		# print 'sum moment: ', sum_moment.flatten()
		# print 'norm error: ', norm_e
		
		## Check friction constraints
		for j in range(nc):
			fz = np.dot(normals[j].T, forces[j])
			ft = np.dot(np.eye(3) - tangential(normals[j]), forces[j].flatten())
			if np.linalg.norm(ft) > SURFACE_FRICTION*fz:
				print colored("ERROR: Equilibrium - Friction constraint violation on leg %i"%j,'red')
				equilibrium = False
		# print 'Force/moment: ', np.round(norm_e,4)

	def compute_support_polygon(self, stack_world_X_foot):
		""" Convex hull formed by supporting legs. """

		# Extract stance to 2D
		stance = [stack_world_X_foot[j][0:2] for j in range(len(stack_world_X_foot))]
		
		# Generate convex hull for support polygon
		vertices = np.asarray(stance)
		hull = spatial.ConvexHull(vertices)
		
		# Rearrange vertex sequence
		ns = len(vertices[hull.vertices,0])
		xh = vertices[hull.vertices,0].reshape((ns,1))
		yh = vertices[hull.vertices,1].reshape((ns,1))
		self.convex_hull = np.concatenate((xh,yh),axis=1)

	def point_in_polygon(self, p):
		""" Check if CoB lies inside convex hull """

		if not isinstance(self.convex_hull, spatial.Delaunay):
			de_hull = spatial.Delaunay(self.convex_hull)
		self.valid = de_hull.find_simplex(p[0:2].flatten())>=0
		# print self.convex_hull
		# Get shortest distance
		darr = []
		ns = len(self.convex_hull)
		for i in range(0,ns):
			p1 = self.convex_hull[i,:]
			if i==ns-1:
				p2 = self.convex_hull[0,:]
			else:
				p2 = self.convex_hull[i+1,:]
			# print p1, p2
			darr.append(shortest_distance(p[0:2], p1, p2))
		
		self.min = min(darr) if self.valid else -min(darr)
		# print p, self.min
		# plt.plot(vertices[:,0], vertices[:,1], 'o')
		# plt.plot(p[0], p[1], '*')
		# for simplex in hull.simplices:
		# 	plt.plot(vertices[simplex, 0], vertices[simplex, 1], 'k-')
		# plt.show()

		return self.valid, self.min #de_hull.find_simplex(p[0:2])>=0

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

# p = np.zeros(3) #
p = np.array([ 0.25, 0.,  0.      ])

# Legs = [np.array([-0.1630743 ,  0.32203309,  0.        ]), np.array([-0.18062982,  0.22510528,  0.        ]), np.array([-2.78601965e-01,  2.25076089e-01,  1.38777878e-17]), np.array([ 4.35710618e-01, -5.12294616e-02,  4.16333634e-17]), np.array([ 0.15111983, -0.37431678,  0.        ]), np.array([ 6.95810360e-02, -3.74002016e-01, -2.77555756e-17])]
# Legs = [np.array([-1.64207938e-01,  3.22204156e-01, -1.38777878e-17]), np.array([-0.18176346,  0.22527634,  0.        ]), np.array([-2.79735602e-01,  2.25247153e-01,  4.16333634e-17]), np.array([ 0.40436391, -0.00871848,  0.        ]), np.array([ 1.90498403e-01, -3.73826321e-01, -2.08166817e-17]), np.array([ 6.84473985e-02, -3.73830951e-01,  1.38777878e-17])]
# gphase = [0, 0, 0, 1, 0, 0]
# Legs = [np.array([-2.21998450e-01,  2.80855690e-01,  5.55111512e-17]), np.array([-0.22182818,  0.18271897,  0.        ]), np.array([-3.18639671e-01,  1.65395701e-01,  1.38777878e-17]), np.array([ 3.77314358e-01, -2.32872695e-02,  5.55111512e-17]), np.array([ 3.77292937e-01, -1.54514558e-01,  1.38777878e-17]), np.array([ 0.01116949, -0.43351627,  0.        ])]
# gphase =  [0, 0, 0, 0, 0, 1]
Legs = [np.array([ 0.25, 0.00, 0.]),
		np.array([-0.25, 0.00, 0.]),
		np.array([ 0.00, 0.3, 0.]),
		np.array([ 0.00,-0.3, 0.])]
# RR transfer
Legs = [[0.374, 0.556, 0.5], [0.299, 0.556, 0.5], [0.224, 0.556, 0.5], [0.414, 0.011, 0.5], [0.299, 0.011, 0.5]]
p = np.array([0.29682765, 0.28308646, 0.50082906])
# RM transfer
Legs = [[0.374, 0.556, 0.5], [0.299, 0.556, 0.5], [0.224, 0.556, 0.5], [0.414, 0.011, 0.5], [0.2222, 0.0111, 0.5]]
# p = np.array([0.29776476, 0.28311895, 0.50082744])

## Test set
# Legs = [[ 0.2, 0.2, 0.],
# 		[-0.2, 0.2, 0.],
# 		[ 0.0,-0.2, 0.],
# 		[-0.2,-0.2, 0.]]
# p = np.array([0., 0., 0.])

SP = StabilityMargin()
SP.compute_support_polygon(Legs)
print SP.point_in_polygon(p)

# normals = [[[0.0, -1.0, 0.0]], 
# 		   [[0.0, -1.0, 0.0]], 
# 		   [[-1.0, 0.0, 0.0]], 
# 		   [[0.0, 1.0, 0.0]], 
# 		   [[0.0, 1.0, 0.0]]]
normals = [[[0.0, 0.0, 1.0]], 
		   [[0.0, 0.0, 1.0]], 
		   [[0.0, 0.0, 1.0]], 
		   [[0.0, 0.0, 1.0]], 
		   [[0.0, 0.0, 1.0]]]
SR = StabilityMargin()
# SR.compute_support_region(Legs, normals)
# print SR.point_in_polygon(p)

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

points = SP.convex_hull
hull = spatial.ConvexHull(points)

plt.plot(points[:,0], points[:,1], 'o')
plt.plot(p[0],p[1], '*')
cent = np.mean(points, 0)
pts = []
for pt in points[hull.simplices]:
    pts.append(pt[0].tolist())
    pts.append(pt[1].tolist())

pts.sort(key=lambda p: np.arctan2(p[1] - cent[1],
                                p[0] - cent[0]))
pts = pts[0::2]  # Deleting duplicates
pts.insert(len(pts), pts[0])
k = 1.
color = 'green'
poly = Polygon(k*(np.array(pts)- cent) + cent,
               facecolor=color, alpha=0.2)
poly.set_capstyle('round')
plt.gca().add_patch(poly)
# plt.show()