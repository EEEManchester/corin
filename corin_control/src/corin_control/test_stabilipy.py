#!/usr/bin/env python

import numpy as np
import stabilipy
import time
from scipy.spatial import ConvexHull, Delaunay
math = stabilipy.Math()
# from math_tools import Math
# mass, dimension, gravity, radius, force limit, robust sphere (not enforced)
poly = stabilipy.StabilityPolygon(5.0, 2, -9.81, 2., 1., -1, 0.)
# contact position
# pos = [[[0., 0., 0.0]], 
# 		[[1., 0., 0.]],
# 		[[0., 1., 0.]],
# 		[[1., 1., 0.]]]

# pos = [ [[0.5,   0.2, -0.]],
# 		[[1.0,  -0.2, -0.]],
# 		[[-1.0,  0.2, -0.]]]#,
# 		# [[-1.0, -0.2, -0.]]]
# # surface normals
# normals = [[[0., 0., 1.]], 
# 		   [[0., 0., 1.]],
# 		   [[0., 0., 1.]]]#,
# 			# [[0., 0.0, 1.]]] 
# pos = [ [ 0.25,  0.25, 0.0], 
# 		[ 0.00,  0.30, 0.0], 
# 		[-0.25,  0.25, 0.0], 
# 		[ 0.25, -0.25, 0.0], 
# 		[ 0.00, -0.30, 0.0], 
# 		[-0.25, -0.25, 0.0]]
# normals = [[[0.0, 0.0, 1.0]], 
# 			[[0.0, 0.0, 1.0]], 
# 			[[0.0, 0.0, 1.0]], 
# 			[[0.0, 0.0, 1.0]], 
# 			[[0.0, 0.0, 1.0]], 
# 			[[0.0, 0.0, 1.0]]]

pos =[[0.579985, 0.640869, 0.0], 
		[0.33, 0.69, 0.0], 
		[0.080015, 0.640869, 0.0], 
		[0.579985, 0.139131, 0.0], 
		[0.33, 0.09, 0.0], 
		[0.080015, 0.139131, 0.0]]
normals = [[ [0.0, 1.0, 0.0]], 
			[[0.0, 1.0, 0.0]], 
			[[0.0, 1.0, 0.0]], 
			[[0.0,-1.0, 0.0]], 
			[[0.0,-1.0, 0.0]], 
			[[0.0,-1.0, 0.0]]]

axisZ = np.array([[0.0], [1.0], [0.0]])
n4 = np.transpose(np.transpose(math.rpyToRot(1.0,0.0,0.0)).dot(axisZ))
normals[0] = n4.tolist()
# normals[3] = n4.tolist()
# friction coefficient
mu = 0.5
contacts = [stabilipy.Contact(mu, np.array(p).T,
                              stabilipy.utils.normalize(np.array(n).T))
            for p, n in zip(pos, normals)]
poly.contacts = contacts

# modes: best, iteration, precision
now = time.time()
poly.compute(stabilipy.Mode.best, epsilon=1e-5, 
									maxIter=10, 
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
	ns = len(outer_points[hull.vertices,0])
	xh = points[hull.vertices,0].reshape((ns,1))
	yh = points[hull.vertices,1].reshape((ns,1))
	return np.concatenate((xh,yh),axis=1)
	 
# import cdd
# mat = cdd.Matrix([[ 8.299699798E-01, -1,  0],
# 				 [ 1.699699828E-01,  1,  0],
# 				 [ 9.408689929E-01,  0, -1],
# 				 [ 1.608689949E-01,  0,  1],
# 				 [ 1.063501949E+00, -7.367158837E-01, -6.762024154E-01],
# 				 [ 5.365489497E-01, -7.379147048E-01,  6.748939831E-01],
# 				 [ 4.819916385E-02,  7.430692952E-01,  6.692144817E-01],
# 				 [-4.759153493E+06,  1, -2.958403281E+07]], number_type='float')
# mat.rep_type = cdd.RepType.INEQUALITY
# poly = cdd.Polyhedron(mat)
# hrep = poly.get_inequalities()
# hrep.canonicalize()
# points = np.array(cdd.Polyhedron(hrep).get_generators())[:, 1:]
# # print points

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