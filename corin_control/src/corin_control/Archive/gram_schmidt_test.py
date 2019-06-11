#!/usr/bin/env python

import time
import warnings
import numpy as np
from matrix_transforms import *
from constant import *
from kdl import *

# Hybrid stance
base_X_foot = [ np.array([ 0.25,  0.251, -0.1]),
				np.array([ 0.,    0.300, -0.1]),
				np.array([-0.25,  0.251, -0.1]),
				np.array([ 0.25, -0.251, -0.1]),
				np.array([ 0.,   -0.300, -0.15]),
				np.array([-0.25, -0.251, -0.1])]

# Rectangular stance
# base_X_foot = [ np.array([ 0.25,  0.3, -0.1]),
# 				np.array([ 0.,    0.3, -0.1]),
# 				np.array([-0.25,  0.3, -0.1]),
# 				np.array([ 0.25, -0.25, -0.1]),
# 				np.array([ 0.,   -0.3, -0.05]),
# 				np.array([-0.25, -0.3, -0.1])]

e1 = base_X_foot[0] - base_X_foot[4]
e2 = base_X_foot[2] - base_X_foot[4]
R1 = gram_schmidt(e1,e2)

T_temp = np.hstack(( R1, base_X_foot[5].reshape((3,1)) ))
# homogenous transformation from tripod frame 1 to body frame
T1 = np.vstack(( T_temp, np.array([[0, 0, 0, 1]]) ))
# print R1
# print np.dot(R1.T, base_X_foot[5])

rpy = np.array(euler_from_matrix(R1.T,'sxyz'))
xyz = np.dot(R1.T, base_X_foot[4])

print 'roll:  \t', np.round((rpy[0]),4) #np.rad2deg
print 'pitch: \t', np.round((rpy[1]),4) #np.rad2deg
print 'yaw:   \t', np.round((rpy[2]),4) #np.rad2deg
print 'xyz: \t', np.round(xyz, 4), type(xyz), type(rpy)
xb = np.hstack((xyz,rpy))
print (xb)
KDL = KDL()
coxa_X_foot = np.array([ 0.21, 0., -0.15])
print 'leg q: ', KDL.leg_IK(coxa_X_foot)