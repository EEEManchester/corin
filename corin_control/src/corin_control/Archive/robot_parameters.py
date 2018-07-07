#!/usr/bin/env python

import numpy as np
import transformations as tf
from numpy.linalg import inv
from fractions import Fraction

def inertia_tensor(Ixx, Iyy, Izz):
	return np.array([ [Ixx, 0, 0],[0, Iyy, 0], [0, 0, Izz] ])

# Transformation parameters
ORIGIN = (0,0,0)
X_AXIS = (1,0,0)
Y_AXIS = (0,1,0)
Z_AXIS = (0,0,1)

g = 9.81

# Leg Link Length, Mass
l1 = 0.077; 	m1 = 0.175
l2 = 0.150;		m2 = 0.156
l3 = 0.170;		m3 = 0.054

lc1 = 0.0580; 	
lc2 = 0.1213; 
lc3 = 0.0958;

[I1x, I1y, I1z] = [3.20e-5, 5.60e-5, 4.00e-5]
[I2x, I2y, I2z] = [1.90e-5, 6.80e-5, 7.90e-5]
[I3x, I3y, I3z] = [1.60e-5, 11.9e-5, 13.0e-5]

# Joint damping
zq1 = 0.2
zq2 = zq1
zq3 = zq1