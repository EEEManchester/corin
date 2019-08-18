#!/usr/bin/env python

import numpy as np

from robot_transforms import *
from matrix_transforms import *
from kdl import *

kdl = KDL()
cd = [0.2096,  0.0003, -0.0934]
# cd = [ 0.0063, 0., -0.2   ]
# cd =  [-0.0007,  0.0008 ,-0.1998]
# cd = [ 0.15589821,  0.13081413, -0.10648934]
cd = [  7.49098763e-05,  -5.34469213e-05, -1.99807989e-01]
cd = [  1.77496449e-04,  -1.75704838e-04,  -1.99810853e-01]
cd = [  1.43292740e-04,  -1.34942445e-04,  -1.99809899e-01]
cd = [  4.07306569e-05,  -1.27137139e-05, -1.99807032e-01] 	# near singular
cd = [ 0.00022889, -0.00023695, -0.19984341]
cd = [  1.96681324e-04,  -1.98568951e-04,  -1.99842435e-01]
# cd = [  1.64515153e-04,  -1.60234801e-04 , -1.99841461e-01]
# new:  [-0.3026 -1.0605 -1.6037]
qp = kdl.leg_IK(cd, 0)
print np.round(qp,4)
dsing = kdl.singularity_approach(qp)
epsilon = 1e-6
qp_prev = [-0.35, -1.065, -1.6037]
print dsing, epsilon
UL = 0.02
if dsing < epsilon:
	print 'near singular: '#, qp
	error = qp - qp_prev
	# print 'error: ', error
	sgn = [np.sign(i) for i in qp]
	# print sgn
	for i in range(3):
		if abs(error[i]) > UL:
			qp[i] = qp_prev[i] - sgn[i]*UL
		else:
			qp[i] = qp_prev[i] + error[i]
	# qp = kdl.leg_IK(cd, 0)
	# print kdl.singularity_approach(qp)
print np.round(qp,4)


snorm_1 = np.array([ 1.,0.,0.])
# snorm_1 = np.array([ 0.,-1.,0.])

cXf = update_coxa_X_foot(qp)

vt = np.dot(cXf[:3,:3], np.array([1.,0.,0.]))
d1 = np.dot(snorm_1, vt)
print 'd: ', d1
if d1 > 0.:
	qp = kdl.leg_IK(cd, 0, True)
	print np.round(qp,4)

# d:  0.000267348916522
# ori:  [ 0.6981  0.2943 -1.8654]
# new:  [-2.4435 -2.4317 -0.6517]
# cp:  [ 0.15589821  0.13081413 -0.10648934]
# norm:  [ 0. -1.  0.]
