#!/usr/bin/env python

## Forward and inverse kinematics of the 3 DOF leg joints
## Transformation with respect to SCS (origin is joint 1 frame)
import sys; sys.dont_write_bytecode = True
import numpy as np

from scipy import linalg
from sympy import *

l1, l2, l3, q1, q2, q3, q4 = symbols('l1 l2 l3 q1 q2 q3 q4')
cx, cy, qbx, qby, qbz, roll, pitch, yaw = symbols('cx cy qbx qby qbz roll pitch yaw')
qc1, qc2, qc3, qc4, qc5, qc6 = symbols('qc1 qc2 qc3 qc4 qc5 qc6')

LF_x = qbx + qby + qbz + (sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))*(l3*sin(q2 + q3) + l2*sin(q2)) - cos(q1)*(sin(qc1)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) - cos(pitch)*cos(qc1)*cos(yaw))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) - sin(q1)*(cos(qc1)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) + cos(pitch)*cos(yaw)*sin(qc1))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) + cos(pitch)*cos(yaw)*(cx + cy);
LF_y = cos(q1)*(sin(qc1)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) + cos(pitch)*cos(qc1)*sin(yaw))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) - (cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw))*(l3*sin(q2 + q3) + l2*sin(q2)) + sin(q1)*(cos(qc1)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) - cos(pitch)*sin(qc1)*sin(yaw))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) + cos(pitch)*sin(yaw)*(cx + cy);
LF_z = sin(q1)*(sin(pitch)*sin(qc1) + cos(pitch)*cos(qc1)*sin(roll))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) - cos(q1)*(cos(qc1)*sin(pitch) - cos(pitch)*sin(qc1)*sin(roll))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) - sin(pitch)*(cx + cy) + cos(pitch)*cos(roll)*(l3*sin(q2 + q3) + l2*sin(q2));

LM_x = qbx + qby + qbz + (sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))*(l3*sin(q2 + q3) + l2*sin(q2)) - cos(q1)*(sin(qc2)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) - cos(pitch)*cos(qc2)*cos(yaw))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) - sin(q1)*(cos(qc2)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) + cos(pitch)*cos(yaw)*sin(qc2))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) + cy*cos(pitch)*cos(yaw);
LM_y = cos(q1)*(sin(qc2)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) + cos(pitch)*cos(qc2)*sin(yaw))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) - (cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw))*(l3*sin(q2 + q3) + l2*sin(q2)) + sin(q1)*(cos(qc2)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) - cos(pitch)*sin(qc2)*sin(yaw))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) + cy*cos(pitch)*sin(yaw);
LM_z = sin(q1)*(sin(pitch)*sin(qc2) + cos(pitch)*cos(qc2)*sin(roll))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) - cos(q1)*(cos(qc2)*sin(pitch) - cos(pitch)*sin(qc2)*sin(roll))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) - cy*sin(pitch) + cos(pitch)*cos(roll)*(l3*sin(q2 + q3) + l2*sin(q2));

LR_x = qbx + qby + qbz + (sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))*(l3*sin(q2 + q3) + l2*sin(q2)) - cos(q1)*(sin(qc3)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) - cos(pitch)*cos(qc3)*cos(yaw))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) - sin(q1)*(cos(qc3)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) + cos(pitch)*cos(yaw)*sin(qc3))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) - cos(pitch)*cos(yaw)*(cx - cy);
LR_y = cos(q1)*(sin(qc3)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) + cos(pitch)*cos(qc3)*sin(yaw))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) - (cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw))*(l3*sin(q2 + q3) + l2*sin(q2)) + sin(q1)*(cos(qc3)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) - cos(pitch)*sin(qc3)*sin(yaw))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) - cos(pitch)*sin(yaw)*(cx - cy);
LR_z = sin(pitch)*(cx - cy) - cos(q1)*(cos(qc3)*sin(pitch) - cos(pitch)*sin(qc3)*sin(roll))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) + sin(q1)*(sin(pitch)*sin(qc3) + cos(pitch)*cos(qc3)*sin(roll))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) + cos(pitch)*cos(roll)*(l3*sin(q2 + q3) + l2*sin(q2));

RF_x = qbx + qby + qbz + (sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))*(l3*sin(q2 + q3) + l2*sin(q2)) - cos(q1)*(sin(qc4)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) - cos(pitch)*cos(qc4)*cos(yaw))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) - sin(q1)*(cos(qc4)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) + cos(pitch)*cos(yaw)*sin(qc4))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) + cos(pitch)*cos(yaw)*(cx - cy);
RF_y = cos(q1)*(sin(qc4)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) + cos(pitch)*cos(qc4)*sin(yaw))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) - (cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw))*(l3*sin(q2 + q3) + l2*sin(q2)) + sin(q1)*(cos(qc4)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) - cos(pitch)*sin(qc4)*sin(yaw))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) + cos(pitch)*sin(yaw)*(cx - cy);
RF_z = sin(q1)*(sin(pitch)*sin(qc4) + cos(pitch)*cos(qc4)*sin(roll))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) - cos(q1)*(cos(qc4)*sin(pitch) - cos(pitch)*sin(qc4)*sin(roll))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) - sin(pitch)*(cx - cy) + cos(pitch)*cos(roll)*(l3*sin(q2 + q3) + l2*sin(q2));

RM_x = qbx + qby + qbz + (sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))*(l3*sin(q2 + q3) + l2*sin(q2)) - cos(q1)*(sin(qc5)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) - cos(pitch)*cos(qc5)*cos(yaw))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) - sin(q1)*(cos(qc5)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) + cos(pitch)*cos(yaw)*sin(qc5))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) - cy*cos(pitch)*cos(yaw);
RM_y = cos(q1)*(sin(qc5)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) + cos(pitch)*cos(qc5)*sin(yaw))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) - (cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw))*(l3*sin(q2 + q3) + l2*sin(q2)) + sin(q1)*(cos(qc5)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) - cos(pitch)*sin(qc5)*sin(yaw))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) - cy*cos(pitch)*sin(yaw);
RM_z = cy*sin(pitch) - cos(q1)*(cos(qc5)*sin(pitch) - cos(pitch)*sin(qc5)*sin(roll))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) + sin(q1)*(sin(pitch)*sin(qc5) + cos(pitch)*cos(qc5)*sin(roll))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) + cos(pitch)*cos(roll)*(l3*sin(q2 + q3) + l2*sin(q2));

RR_x = qbx + qby + qbz + (sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))*(l3*sin(q2 + q3) + l2*sin(q2)) - cos(q1)*(sin(qc6)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) - cos(pitch)*cos(qc6)*cos(yaw))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) - sin(q1)*(cos(qc6)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) + cos(pitch)*cos(yaw)*sin(qc6))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) - cos(pitch)*cos(yaw)*(cx + cy);
RR_y = cos(q1)*(sin(qc6)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) + cos(pitch)*cos(qc6)*sin(yaw))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) - (cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw))*(l3*sin(q2 + q3) + l2*sin(q2)) + sin(q1)*(cos(qc6)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) - cos(pitch)*sin(qc6)*sin(yaw))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) - cos(pitch)*sin(yaw)*(cx + cy);
RR_z = sin(pitch)*(cx + cy) - cos(q1)*(cos(qc6)*sin(pitch) - cos(pitch)*sin(qc6)*sin(roll))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) + sin(q1)*(sin(pitch)*sin(qc6) + cos(pitch)*cos(qc6)*sin(roll))*(l1 + l3*cos(q2 + q3) + l2*cos(q2)) + cos(pitch)*cos(roll)*(l3*sin(q2 + q3) + l2*sin(q2));

print diff(LR_x, q1)
print diff(LR_y, q1)
print diff(LR_z, q1)
print diff(LR_x, q2)
print diff(LR_y, q2)
print diff(LR_z, q2)
print diff(LR_x, q3)
print diff(LR_y, q3)
print diff(LR_z, q3)

## LF
# J21 = -((np.sin(pitch)*np.sin(roll)*np.sin(yaw) + np.cos(roll)*np.cos(yaw))*np.sin(qc1) + np.sin(yaw)*np.cos(pitch)*np.cos(qc1))*(l1 + l2*np.cos(q2) + l3*np.cos(q2 + q3))*np.sin(q1) + ((np.sin(pitch)*np.sin(roll)*np.sin(yaw) + np.cos(roll)*np.cos(yaw))*np.cos(qc1) - np.sin(qc1)*np.sin(yaw)*np.cos(pitch))*(l1 + l2*np.cos(q2) + l3*np.cos(q2 + q3))*np.cos(q1)
# J31 = (np.sin(pitch)*np.sin(qc1) + np.sin(roll)*np.cos(pitch)*np.cos(qc1))*(l1 + l2*np.cos(q2) + l3*np.cos(q2 + q3))*np.cos(q1) - (-np.sin(pitch)*np.cos(qc1) + np.sin(qc1)*np.sin(roll)*np.cos(pitch))*(l1 + l2*np.cos(q2) + l3*np.cos(q2 + q3))*np.sin(q1)
# J12 = (-l2*np.sin(q2) - l3*np.sin(q2 + q3))*(-(-np.sin(pitch)*np.sin(roll)*np.cos(yaw) + np.sin(yaw)*np.cos(roll))*np.sin(qc1) + np.cos(pitch)*np.cos(qc1)*np.cos(yaw))*np.cos(q1) + (-l2*np.sin(q2) - l3*np.sin(q2 + q3))*(-(-np.sin(pitch)*np.sin(roll)*np.cos(yaw) + np.sin(yaw)*np.cos(roll))*np.cos(qc1) - np.sin(qc1)*np.cos(pitch)*np.cos(yaw))*np.sin(q1) + (l2*np.cos(q2) + l3*np.cos(q2 + q3))*(np.sin(pitch)*np.cos(roll)*np.cos(yaw) + np.sin(roll)*np.sin(yaw))
# J22 = (-l2*np.sin(q2) - l3*np.sin(q2 + q3))*((np.sin(pitch)*np.sin(roll)*np.sin(yaw) + np.cos(roll)*np.cos(yaw))*np.sin(qc1) + np.sin(yaw)*np.cos(pitch)*np.cos(qc1))*np.cos(q1) + (-l2*np.sin(q2) - l3*np.sin(q2 + q3))*((np.sin(pitch)*np.sin(roll)*np.sin(yaw) + np.cos(roll)*np.cos(yaw))*np.cos(qc1) - np.sin(qc1)*np.sin(yaw)*np.cos(pitch))*np.sin(q1) + (-l2*np.cos(q2) - l3*np.cos(q2 + q3))*(-np.sin(pitch)*np.sin(yaw)*np.cos(roll) + np.sin(roll)*np.cos(yaw))
# J32 = (-l2*np.sin(q2) - l3*np.sin(q2 + q3))*(np.sin(pitch)*np.sin(qc1) + np.sin(roll)*np.cos(pitch)*np.cos(qc1))*np.sin(q1) + (-l2*np.sin(q2) - l3*np.sin(q2 + q3))*(-np.sin(pitch)*np.cos(qc1) + np.sin(qc1)*np.sin(roll)*np.cos(pitch))*np.cos(q1) + (l2*np.cos(q2) + l3*np.cos(q2 + q3))*np.cos(pitch)*np.cos(roll)
# J31 = -l3*(-(-np.sin(pitch)*np.sin(roll)*np.cos(yaw) + np.sin(yaw)*np.cos(roll))*np.sin(qc1) + np.cos(pitch)*np.cos(qc1)*np.cos(yaw))*np.sin(q2 + q3)*np.cos(q1) - l3*(-(-np.sin(pitch)*np.sin(roll)*np.cos(yaw) + np.sin(yaw)*np.cos(roll))*np.cos(qc1) - np.sin(qc1)*np.cos(pitch)*np.cos(yaw))*np.sin(q1)*np.sin(q2 + q3) + l3*(np.sin(pitch)*np.cos(roll)*np.cos(yaw) + np.sin(roll)*np.sin(yaw))*np.cos(q2 + q3)
# J32 = -l3*((np.sin(pitch)*np.sin(roll)*np.sin(yaw) + np.cos(roll)*np.cos(yaw))*np.sin(qc1) + np.sin(yaw)*np.cos(pitch)*np.cos(qc1))*np.sin(q2 + q3)*np.cos(q1) - l3*((np.sin(pitch)*np.sin(roll)*np.sin(yaw) + np.cos(roll)*np.cos(yaw))*np.cos(qc1) - np.sin(qc1)*np.sin(yaw)*np.cos(pitch))*np.sin(q1)*np.sin(q2 + q3) - l3*(-np.sin(pitch)*np.sin(yaw)*np.cos(roll) + np.sin(roll)*np.cos(yaw))*np.cos(q2 + q3)
# J33 = -l3*(np.sin(pitch)*np.sin(qc1) + np.sin(roll)*np.cos(pitch)*np.cos(qc1))*np.sin(q1)*np.sin(q2 + q3) - l3*(-np.sin(pitch)*np.cos(qc1) + np.sin(qc1)*np.sin(roll)*np.cos(pitch))*np.sin(q2 + q3)*np.cos(q1) + l3*np.cos(pitch)*np.cos(roll)*np.cos(q2 + q3)

## LM
J11 = -(-(-sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(qc2) + cos(pitch)*cos(qc2)*cos(yaw))*(l1 + l2*cos(q2) + l3*cos(q2 + q3))*sin(q1) + (-(-sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(qc2) - sin(qc2)*cos(pitch)*cos(yaw))*(l1 + l2*cos(q2) + l3*cos(q2 + q3))*cos(q1)
J21 = -((sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(qc2) + sin(yaw)*cos(pitch)*cos(qc2))*(l1 + l2*cos(q2) + l3*cos(q2 + q3))*sin(q1) + ((sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(qc2) - sin(qc2)*sin(yaw)*cos(pitch))*(l1 + l2*cos(q2) + l3*cos(q2 + q3))*cos(q1)
J31 = (sin(pitch)*sin(qc2) + sin(roll)*cos(pitch)*cos(qc2))*(l1 + l2*cos(q2) + l3*cos(q2 + q3))*cos(q1) - (-sin(pitch)*cos(qc2) + sin(qc2)*sin(roll)*cos(pitch))*(l1 + l2*cos(q2) + l3*cos(q2 + q3))*sin(q1)
J12 = (-l2*sin(q2) - l3*sin(q2 + q3))*(-(-sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(qc2) + cos(pitch)*cos(qc2)*cos(yaw))*cos(q1) + (-l2*sin(q2) - l3*sin(q2 + q3))*(-(-sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(qc2) - sin(qc2)*cos(pitch)*cos(yaw))*sin(q1) + (l2*cos(q2) + l3*cos(q2 + q3))*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))
J22 = (-l2*sin(q2) - l3*sin(q2 + q3))*((sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(qc2) + sin(yaw)*cos(pitch)*cos(qc2))*cos(q1) + (-l2*sin(q2) - l3*sin(q2 + q3))*((sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(qc2) - sin(qc2)*sin(yaw)*cos(pitch))*sin(q1) + (-l2*cos(q2) - l3*cos(q2 + q3))*(-sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))
J32 = (-l2*sin(q2) - l3*sin(q2 + q3))*(sin(pitch)*sin(qc2) + sin(roll)*cos(pitch)*cos(qc2))*sin(q1) + (-l2*sin(q2) - l3*sin(q2 + q3))*(-sin(pitch)*cos(qc2) + sin(qc2)*sin(roll)*cos(pitch))*cos(q1) + (l2*cos(q2) + l3*cos(q2 + q3))*cos(pitch)*cos(roll)
J31 = -l3*(-(-sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(qc2) + cos(pitch)*cos(qc2)*cos(yaw))*sin(q2 + q3)*cos(q1) - l3*(-(-sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(qc2) - sin(qc2)*cos(pitch)*cos(yaw))*sin(q1)*sin(q2 + q3) + l3*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(q2 + q3)
J32 = -l3*((sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(qc2) + sin(yaw)*cos(pitch)*cos(qc2))*sin(q2 + q3)*cos(q1) - l3*((sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(qc2) - sin(qc2)*sin(yaw)*cos(pitch))*sin(q1)*sin(q2 + q3) - l3*(-sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(q2 + q3)
J33 = -l3*(sin(pitch)*sin(qc2) + sin(roll)*cos(pitch)*cos(qc2))*sin(q1)*sin(q2 + q3) - l3*(-sin(pitch)*cos(qc2) + sin(qc2)*sin(roll)*cos(pitch))*sin(q2 + q3)*cos(q1) + l3*cos(pitch)*cos(roll)*cos(q2 + q3)

## LR
-(-(-sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(qc3) + cos(pitch)*cos(qc3)*cos(yaw))*(l1 + l2*cos(q2) + l3*cos(q2 + q3))*sin(q1) + (-(-sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(qc3) - sin(qc3)*cos(pitch)*cos(yaw))*(l1 + l2*cos(q2) + l3*cos(q2 + q3))*cos(q1)
-((sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(qc3) + sin(yaw)*cos(pitch)*cos(qc3))*(l1 + l2*cos(q2) + l3*cos(q2 + q3))*sin(q1) + ((sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(qc3) - sin(qc3)*sin(yaw)*cos(pitch))*(l1 + l2*cos(q2) + l3*cos(q2 + q3))*cos(q1)
(sin(pitch)*sin(qc3) + sin(roll)*cos(pitch)*cos(qc3))*(l1 + l2*cos(q2) + l3*cos(q2 + q3))*cos(q1) - (-sin(pitch)*cos(qc3) + sin(qc3)*sin(roll)*cos(pitch))*(l1 + l2*cos(q2) + l3*cos(q2 + q3))*sin(q1)
(-l2*sin(q2) - l3*sin(q2 + q3))*(-(-sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(qc3) + cos(pitch)*cos(qc3)*cos(yaw))*cos(q1) + (-l2*sin(q2) - l3*sin(q2 + q3))*(-(-sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(qc3) - sin(qc3)*cos(pitch)*cos(yaw))*sin(q1) + (l2*cos(q2) + l3*cos(q2 + q3))*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))
(-l2*sin(q2) - l3*sin(q2 + q3))*((sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(qc3) + sin(yaw)*cos(pitch)*cos(qc3))*cos(q1) + (-l2*sin(q2) - l3*sin(q2 + q3))*((sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(qc3) - sin(qc3)*sin(yaw)*cos(pitch))*sin(q1) + (-l2*cos(q2) - l3*cos(q2 + q3))*(-sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))
(-l2*sin(q2) - l3*sin(q2 + q3))*(sin(pitch)*sin(qc3) + sin(roll)*cos(pitch)*cos(qc3))*sin(q1) + (-l2*sin(q2) - l3*sin(q2 + q3))*(-sin(pitch)*cos(qc3) + sin(qc3)*sin(roll)*cos(pitch))*cos(q1) + (l2*cos(q2) + l3*cos(q2 + q3))*cos(pitch)*cos(roll)
-l3*(-(-sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*sin(qc3) + cos(pitch)*cos(qc3)*cos(yaw))*sin(q2 + q3)*cos(q1) - l3*(-(-sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll))*cos(qc3) - sin(qc3)*cos(pitch)*cos(yaw))*sin(q1)*sin(q2 + q3) + l3*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*cos(q2 + q3)
-l3*((sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(qc3) + sin(yaw)*cos(pitch)*cos(qc3))*sin(q2 + q3)*cos(q1) - l3*((sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*cos(qc3) - sin(qc3)*sin(yaw)*cos(pitch))*sin(q1)*sin(q2 + q3) - l3*(-sin(pitch)*sin(yaw)*cos(roll) + sin(roll)*cos(yaw))*cos(q2 + q3)
-l3*(sin(pitch)*sin(qc3) + sin(roll)*cos(pitch)*cos(qc3))*sin(q1)*sin(q2 + q3) - l3*(-sin(pitch)*cos(qc3) + sin(qc3)*sin(roll)*cos(pitch))*sin(q2 + q3)*cos(q1) + l3*cos(pitch)*cos(roll)*cos(q2 + q3)


## ================================================================================================ ##
## 												TESTING 											##
## ================================================================================================ ##
# qc1 = np.deg2rad(50.)
# qc2 = np.deg2rad(90.)
# qc3 = np.deg2rad(130.)
# qc4 = np.deg2rad(-50.)
# qc5 = np.deg2rad(-90.)
# qc6 = np.deg2rad(-130.)
# cx = 0.115
# cy = 0.09
# l1 = 0.060
# l2 = 0.15
# l3 = 0.15
# roll  = 0.
# pitch = 0.
# yaw   = 0.
# q1 = 0.
# q2 = 0.4
# q3 = -0.5

# py = -(-(-np.sin(pitch)*np.sin(roll)*np.cos(yaw) + np.sin(yaw)*np.cos(roll))*np.sin(qc1) + np.cos(pitch)*np.cos(qc1)*np.cos(yaw))*(l1 + l2*np.cos(q2) + l3*np.cos(q2 + q3))*np.sin(q1) + (-(-np.sin(pitch)*np.sin(roll)*np.cos(yaw) + np.sin(yaw)*np.cos(roll))*np.cos(qc1) - np.sin(qc1)*np.cos(pitch)*np.cos(yaw))*(l1 + l2*np.cos(q2) + l3*np.cos(q2 + q3))*np.cos(q1)

# ml = np.sin(q1)*(np.sin(qc1)*(np.cos(roll)*np.sin(yaw) - np.cos(yaw)*np.sin(pitch)*np.sin(roll)) - np.cos(pitch)*np.cos(qc1)*np.cos(yaw))*(l1 + l3*np.cos(q2 + q3) + l2*np.cos(q2)) - np.cos(q1)*(np.cos(qc1)*(np.cos(roll)*np.sin(yaw) - np.cos(yaw)*np.sin(pitch)*np.sin(roll)) + np.cos(pitch)*np.cos(yaw)*np.sin(qc1))*(l1 + l3*np.cos(q2 + q3) + l2*np.cos(q2))

# print py
# print ml