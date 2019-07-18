#!/usr/bin/env python
import cdd
import numpy as np
from scipy.spatial import ConvexHull

def seq_convex_points(points):
	hull = ConvexHull(points)
	# Rearrange vertex sequence
	ns = len(points[hull.vertices,0])
	xh = points[hull.vertices,0].reshape((ns,1))
	yh = points[hull.vertices,1].reshape((ns,1))
	return np.concatenate((xh,yh),axis=1)

# H-representation
# linearity 2  1 2
# begin
# 4 3 real
# 9.999999649E-02  0  1
# -1  0 -10
# 6.742269945E-02 -7.385241116E-01  6.742270661E-01
# 2.674874263E-01  8.798466141E-01  4.752577572E-01
# end 
mat = cdd.Matrix([ [ 0.1,  0,  1],
				   [-1.0,  0, -10],
				   [ 6.742269945E-02, -7.385241116E-01,  6.742270661E-01],
				   [ 2.674874263E-01,  8.798466141E-01,  4.752577572E-01]], 
					 number_type='float')
mat.rep_type = cdd.RepType.INEQUALITY
# print mat.canonicalize()
# print mat
# print np.round(np.array(cdd.Polyhedron(mat).get_generators())[:, 1:],3)

# [[-0.25 -0.1 ]
#  [-0.   -0.1 ]]

mat = cdd.Matrix([  [1, 0.1, 0.1],
					[1,-0.1, 0.1],
					[1, 0.1, -0.1],
					[1,-0.1, -0.1],
					[1,-0.11, -0.11],
					[1,-0.11, -0.11] ], number_type='float')
mat = cdd.Matrix([  [1,  3.391342617E-01,  2.639093957E-01],
					[1, -1.878330346E-01,  2.619061906E-01],
					[1,  1.330173384E-01,  3.666666537E-01],
					[1,  0, -9.999998811E-02],
					[1,  2.499999989E-01, -9.999999588E-02],
					[1, -2.811883725E-02,  3.506651552E-01],
					[1, -1.875054327E-01,  2.605155125E-01]], number_type='float')
mat.rep_type = cdd.RepType.GENERATOR
vertices = np.array(mat)[:, 1:]
print mat
newp = seq_convex_points(vertices)
newp = np.round(np.hstack((np.ones((len(vertices),1)),newp)),4)
mat = cdd.Matrix(newp, number_type='float')
mat.rep_type = cdd.RepType.GENERATOR
mat.canonicalize()
ineq = cdd.Polyhedron(mat).get_inequalities()
# print cdd.Polyhedron(ineq).get_generators()
print 9.999999588E-02
mat = cdd.Matrix([[2, 1, 2, 3], [0, 1, 2, 3], [3, 0, 1, 2], [0, -2, -4, -6]], number_type='fraction')
# mat = cdd.Matrix([[2, 1, 2, 3], [0, 1, 2, 3], [3, 0, 1, 2]], number_type='fraction')
mat.rep_type = cdd.RepType.INEQUALITY
# print mat.canonicalize()
# print mat

# begin
#  2 4 rational
#  0 1 2 3
#  3 0 1 2
# end