#!/usr/bin/env python

import numpy as np
import stabilipy
import time
from scipy.spatial import ConvexHull, Delaunay
math = stabilipy.Math()
# from math_tools import Math
# mass, dimension, gravity, radius, force limit, robust sphere (not enforced)
poly = stabilipy.StabilityPolygon(5.0, 2, -9.81, 2., 1., -1, 0.)
poly.display_final = True
## Nominal Stance - Ground Walking
# pos = [ [ 0.25,  0.25, 0.0], 
# 		[ 0.00,  0.30, 0.0], 
# 		[-0.25,  0.25, 0.0], 
# 		[ 0.25, -0.25, 0.0], 
# 		[ 0.00, -0.30, 0.0], 
# 		[-0.25, -0.25, 0.0]]
# pos = [[0.36, 0.646979, 6e-06], [0.245, 0.650653, 6e-06], [0.475002, 0.060001, 6e-06], [0.36, 0.06, 6e-06], [0.244998, 0.060001, 6e-06]]
pos = [[0.25, 0.2509, 0.0], [0.0, 0.3, 0.0], [-0.25, 0.2509, 0.0], [0.25, -0.2509, 0.0], [0.0, -0.3, 0.0], [-0.25, -0.2509, 0.0]]
normals = [[[0.0, 0.0, 1.0]], [[0.0, 0.0, 1.0]], [[0.0, 0.0, 1.0]], [[0.0, 0.0, 1.0]], [[0.0, 0.0, 1.0]], [[0.0, 0.0, 1.0]]]
# normals = [[[0.0, 0.0, 1.0]], 
# 			[[0.0, 0.0, 1.0]], 
# 			[[0.0, 0.0, 1.0]], 
# 			[[0.0, 0.0, 1.0]], 
# 			# [[0.0, 0.0, 1.0]], 
# 			[[0.0, 0.0, 1.0]]]

## Nominal Stance - Chimney Walking
# pos =[	[0.579985, 0.640869, 0.0], 
# 		[0.33, 0.69, 0.0], 
# 		[0.080015, 0.640869, 0.0], 
# 		[0.579985, 0.139131, 0.0], 
# 		[0.33, 0.09, 0.0], 
# 		[0.080015, 0.139131, 0.0]]
# normals = [[ [0.0, 1.0, 0.0]], 
# 			[[0.0, 1.0, 0.0]], 
# 			[[0.0, 1.0, 0.0]], 
# 			[[0.0,-1.0, 0.0]], 
# 			[[0.0,-1.0, 0.0]], 
# 			[[0.0,-1.0, 0.0]]]

## Nominal Stance - Wall Walking
# pos = [ [ 0.25,  0.1, 0.8], 
# 		[ 0.00,  0.1, 0.8], 
# 		[-0.25,  0.1, 0.8], 
# 		[ 0.25, -0.1, 0.0], 
# 		[ 0.00, -0.1, 0.0]]#, 
# 		# [-0.25, -0.1, 0.0]]
# normals = [[ [0.0,-1.0, 0.0]], 
# 			[[0.0,-1.0, 0.0]], 
# 			[[0.0,-1.0, 0.0]], 
# 			[[0.0, 0.0, 1.0]], 
# 			[[0.0, 0.0, 1.0]]]#, 
# 			# [[0.0, 0.0, 1.0]]]
pos = [[0.333, 0.234, -0.0], [0.219, 0.219, 0.0], [0.105, 0.203, -0.0], [0.362, 0.012, 0.707], [0.246, 0.012, 0.722], [0.13, 0.012, 0.737]]
normals = [[[0.0, 0.0, 1.0]], [[0.0, 0.0, 1.0]], [[0.0, 0.0, 1.0]], [[0.0, 1.0, 0.0]], [[0.0, 1.0, 0.0]], [[0.0, 1.0, 0.0]]]

## No solution - primal infeasible
# pos = [[0.3738, 0.5781, 0.5], [0.2813, 0.5562, 0.5], [0.9174, 0.2989, 0.5], [0.9173, 0.153, 0.5], [0.9123, 0.011, 0.5]]
# normals = [[[0.0, -1.0, 0.0]], [[0.0, -1.0, 0.0]], [[-1.0, 0.0, 0.0]], [[-1.0, 0.0, 0.0]], [[0.0, 1.0, 0.0]]]

## Small region
# pos = [[0.3703, 0.5559, 0.5], [0.266, 0.556, 0.5], [0.9176, 0.1531, 0.5], [0.8954, 0.0108, 0.5], [0.7356, 0.011, 0.5]]
# normals = [[[0.0, -1.0, 0.0]], [[0.0, -1.0, 0.0]], [[-1.0, 0.0, 0.0]], [[0.0, 1.0, 0.0]], [[0.0, 1.0, 0.0]]]
# pos = [[0.3401, 0.7068, 0.5], [0.2496, 0.7069, 0.5], [1.0337, 0.4133, 0.5], [0.7848, 0.0467, 0.5], [0.6461, 0.0469, 0.5]]
# normals = [[[0.0, -1.0, 0.0]], [[0.0, -1.0, 0.0]], [[-1.0, 0.0, 0.0]], [[0.0, 1.0, 0.0]], [[0.0, 1.0, 0.0]]]
# pos = [[0.3481, 0.7069, 0.5], [0.2542, 0.707, 0.5], [1.0338, 0.4378, 0.5], [0.8403, 0.0468, 0.5], [0.7012, 0.0469, 0.5]]
# normals = [[[0.0, -1.0, 0.0]], [[0.0, -1.0, 0.0]], [[-1.0, 0.0, 0.0]], [[0.0, 1.0, 0.0]], [[0.0, 1.0, 0.0]]]
# pos = [	[-0.22561972,  0.30945383, -0.00034246], 
# 			[-0.31952697,  0.3094684 , -0.00034287], 
# 			[ 4.60182659e-01,  4.04583972e-02, -3.42660774e-04], 
# 			[ 2.66755852e-01, -3.50555585e-01, -3.42615948e-04], 
# 			[ 1.27600822e-01, -3.50571928e-01, -3.42023479e-04]]

# Chimney heuristic - initial point close together
# pos = [[0.3742, 0.8429, 0.5], [0.3741, 0.7601, 0.5], [0.3202, 0.7067, 0.5], [1.034, 0.69, 0.5], [1.0339, 0.5697, 0.5], [1.0338, 0.4495, 0.5]]
# normals = [[[1.0, 0.0, 0.0]], [[1.0, 0.0, 0.0]], [[0.0, -1.0, 0.0]], [[-1.0, 0.0, 0.0]], [[-1.0, 0.0, 0.0]], [[-1.0, 0.0, 0.0]]]


# axisZ = np.array([[0.0], [1.0], [0.0]])
# n4 = np.transpose(np.transpose(math.rpyToRot(1.0,0.0,0.0)).dot(axisZ))
# normals[0] = n4.tolist()
# normals[3] = n4.tolist()
# friction coefficient
mu = 0.8
contacts = [stabilipy.Contact(mu, np.array(p).T,
                              stabilipy.utils.normalize(np.array(n).T))
            for p, n in zip(pos, normals)]
poly.contacts = contacts

# modes: best, iteration, precision
now = time.time()
poly.compute(stabilipy.Mode.best, epsilon=1e-3, 
									maxIter=20, 
									solver='cdd',
									plot_error=False,
									plot_init=False,
									plot_step=False,
									record_anim=True,
									plot_direction=False,
									plot_final=False)
# end = time.time()
# print 'td: ', end-now
# # # convert to 2D array
# vertices = np.array([np.array([i[1], i[2], 0.]) for i in poly.inner if i[0]==1 ])
# print np.round(vertices,3)

# p = np.array([0.32773164, 0.39001203, 0.09993361])
# poly.com = np.array([0.1, 0.0, 0.09993361])
# valid, d = poly.point_in_hull(poly.com)
# print valid, d
# poly.plot_2D()
# poly.show()
# print poly.inner

def seq_convex_points(points):
	hull = ConvexHull(points)
	# Rearrange vertex sequence
	ns = len(points[hull.vertices,0])
	xh = points[hull.vertices,0].reshape((ns,1))
	yh = points[hull.vertices,1].reshape((ns,1))
	return np.concatenate((xh,yh),axis=1)
	 
import cdd
mat = cdd.Matrix([  [1, 2.0, 0.],
					[1, 0., 2.0] ], number_type='float')
mat.rep_type = cdd.RepType.GENERATOR# INEQUALITY
# ineq = np.array(cdd.Polyhedron(mat).get_inequalities())
# poly = cdd.Polyhedron(mat)
# print mat
# print cdd.Polyhedron(mat).get_inequalities()


mat = cdd.Matrix([ [6.742269945E-02,  -7.385241116E-01,  6.742270661E-01],
					[4.641596856E-02, -8.875760933E-01,  4.606611321E-01],
					[-1,  1.922693825E+01, -10] ], number_type='float')
mat = cdd.Matrix([ [9.999999649E-02,  0,  1],
 [-1,  0, -10],
 [ 6.742269945E-02, -7.385241116E-01,  6.742270661E-01],
 [ 2.674874263E-01,  8.798466141E-01,  4.752577572E-01]], number_type='float')
# mat = cdd.Matrix([[ 8.299699798E-01, -1,  0],
# 				 [ 1.699699828E-01,  1,  0],
# 				 [ 9.408689929E-01,  0, -1],
# 				 [ 1.608689949E-01,  0,  1],
# 				 [ 1.063501949E+00, -7.367158837E-01, -6.762024154E-01],
# 				 [ 5.365489497E-01, -7.379147048E-01,  6.748939831E-01],
# 				 [ 4.819916385E-02,  7.430692952E-01,  6.692144817E-01],
# 				 [-4.759153493E+06,  1, -2.958403281E+07]], number_type='float')
# print mat
## Perturb matrix
mat.rep_type = cdd.RepType.INEQUALITY
# ineq = np.array(cdd.Polyhedron(mat).get_inequalities())
# ineq[1][1] += 0.001
# mat = cdd.Matrix( ineq, number_type='float')

# poly = cdd.Polyhedron(mat)
# hrep = poly.get_inequalities()
# hrep.canonicalize()
# points = np.array(cdd.Polyhedron(mat).get_generators())[:, 1:]
# print points

# outer = cdd.Matrix( [[8.299699798E-01, -1,  0],
#   					 [1.699699828E-01,  1,  0],
#   					 [9.408689929E-01,  0, -1],
#   					 [1.608689949E-01,  0,  1],
#   					 [1.063501949E+00, -7.367158837E-01, -6.762024154E-01],
#   					 [5.365489497E-01, -7.379147048E-01,  6.748939831E-01],
#   					 [4.819916385E-02,  7.430692952E-01,  6.692144817E-01]], number_type='float')
# outer.rep_type = cdd.RepType.INEQUALITY
# outer_points = np.array(cdd.Polyhedron(outer).get_generators())[:, 1:]
# inner = cdd.Matrix( [ [1,  8.299699798E-01,  3.900000038E-01],
# 						[1, -1.699699828E-01,  3.900000032E-01],
# 						[1,  3.243490895E-01,  9.408689929E-01],
# 						[1,  5.799849752E-01,  9.408688745E-01],
# 						[1,  5.799849852E-01, -1.608689709E-01],
# 						[1,  8.001500377E-02, -1.608689878E-01]], number_type='float')
# inner.rep_type = cdd.RepType.GENERATOR

# t = []
# for k in range(1000):
# 	now = time.time()
# 	outer_points = seq_convex_points(outer_points)
# 	inner_points = seq_convex_points(np.array(inner)[:, 1:])

# 	# First find which points from triangle coincident with inner polygon
# 	log_inner = []
# 	for p in range(len(inner_points)):
# 		for j in range(len(points)):
# 			dif = inner_points[p] - points[j]
# 			mag = np.linalg.norm(dif)
# 			if mag < 0.0001:
# 				log_inner.append(p)

# 	# print 'Inner points: ', log_inner
# 	# print 'Inner: ', inner_points[log_inner[0]]
# 	# print 'Inner: ', inner_points[log_inner[1]]
# 	# print 'Canonical points\n', points
# 	if len(log_inner) == 2:
# 		log = []
# 		for p in range(len(outer_points)):
# 			for j in range(2):
# 				dif = outer_points[p] - inner_points[log_inner[j]]
# 				mag = np.linalg.norm(dif)
# 				if mag < 0.0001:
# 					log.append(p)
# 					# print p, outer_points[p]
# 					# print j, inner_points[log_inner[j]]
# 					# print mag, dif
# 					# print '-------------------'
# 		# print log
# 		if log:
# 			if abs(log[1] - log[0]) == 1:
# 				print 'volume is zero'

# 	t.append(time.time()-now)
# print sum(t)/1000.