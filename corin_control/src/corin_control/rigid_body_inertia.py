#!/usr/bin/env python

## Class for Corin rigid body inertia
## Indexing for leg starts with 0, 1 .... 5
## 2D numpy arrays used

import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'class'))
sys.dont_write_bytecode = True

import numpy as np
from constant import * 		# constants used
import kdl 					# corin kinematic library
from matrix_transforms import *

class RigidBodyInertia:
	def __init__(self):
		self.com = np.zeros((3,1))
		self.crbi = np.identity(3)

	def update_CRBI(self, q):
		self.crbi = np.array([ [0.0686,	0,	0],
						       [0, 0.0448, 0],
						       [0, 0, 0.1090] ])

	def update_CoM(self, q):
		""" Updates robot CoM position wrt CoB
			Input: 	q: joint angles, Re^18
			Output: x_com, Re^3 """
		
		# update CoM of each leg wrt to CoB
		LF = self.LF_com(q[0:3])
		LM = self.LM_com(q[3:6])
		LR = self.LR_com(q[6:9])
		RF = self.RF_com(q[9:12])
		RM = self.RM_com(q[12:15])
		RR = self.RR_com(q[15:18])

		base_com = np.array([BODY_COM[0], BODY_COM[1], BODY_COM[2]])

		self.com = (LEG_MASS*(LF + LM + LR + RF + RM + RR) + BODY_MASS*base_com)/(6*LEG_MASS + BODY_MASS)

		return self.com

	def LF_com(self, q):
		q1 = np.deg2rad(q[0]);
		q2 = np.deg2rad(q[1]);
		q3 = np.deg2rad(q[2]);

		cos_q1 = np.cos(q1)
		sin_q1 = np.sin(q1)
		cos_q2 = np.cos(q2)
		sin_q2 = np.sin(q2)
		cos_q3 = np.cos(q3)
		sin_q3 = np.sin(q3)

		LF = np.zeros(3)
		LF[0] = COXA_X - (6899914937159737*(COXA_M*(L1_COM[1]*cos_q1 + L1_COM[0]*sin_q1) + TIBIA_M*(L3_COM[1]*cos_q1 - L3_COM[2]*(cos_q2*sin_q1*sin_q3 + cos_q3*sin_q1*sin_q2) - L3_COM[0]*(sin_q1*sin_q2*sin_q3 - cos_q2*cos_q3*sin_q1) + L1*sin_q1 + L2*cos_q2*sin_q1) + FEMUR_M*(L2_COM[1]*cos_q1 + L1*sin_q1 + L2_COM[0]*cos_q2*sin_q1 - L2_COM[2]*sin_q1*sin_q2)))/(9007199254740992*(COXA_M + FEMUR_M + TIBIA_M)) + (5789716078925341*(TIBIA_M*(L3_COM[0]*(cos_q1*cos_q2*cos_q3 - cos_q1*sin_q2*sin_q3) - L3_COM[2]*(cos_q1*cos_q2*sin_q3 + cos_q1*cos_q3*sin_q2) + L1*cos_q1 - L3_COM[1]*sin_q1 + L2*cos_q1*cos_q2) + FEMUR_M*(L1*cos_q1 - L2_COM[1]*sin_q1 + L2_COM[0]*cos_q1*cos_q2 - L2_COM[2]*cos_q1*sin_q2) + COXA_M*(L1_COM[0]*cos_q1 - L1_COM[1]*sin_q1)))/(9007199254740992*(COXA_M + FEMUR_M + TIBIA_M));
		LF[1] = COXA_Y + (5789716078925341*(COXA_M*(L1_COM[1]*cos_q1 + L1_COM[0]*sin_q1) + TIBIA_M*(L3_COM[1]*cos_q1 - L3_COM[2]*(cos_q2*sin_q1*sin_q3 + cos_q3*sin_q1*sin_q2) - L3_COM[0]*(sin_q1*sin_q2*sin_q3 - cos_q2*cos_q3*sin_q1) + L1*sin_q1 + L2*cos_q2*sin_q1) + FEMUR_M*(L2_COM[1]*cos_q1 + L1*sin_q1 + L2_COM[0]*cos_q2*sin_q1 - L2_COM[2]*sin_q1*sin_q2)))/(9007199254740992*(COXA_M + FEMUR_M + TIBIA_M)) + (6899914937159737*(TIBIA_M*(L3_COM[0]*(cos_q1*cos_q2*cos_q3 - cos_q1*sin_q2*sin_q3) - L3_COM[2]*(cos_q1*cos_q2*sin_q3 + cos_q1*cos_q3*sin_q2) + L1*cos_q1 - L3_COM[1]*sin_q1 + L2*cos_q1*cos_q2) + FEMUR_M*(L1*cos_q1 - L2_COM[1]*sin_q1 + L2_COM[0]*cos_q1*cos_q2 - L2_COM[2]*cos_q1*sin_q2) + COXA_M*(L1_COM[0]*cos_q1 - L1_COM[1]*sin_q1)))/(9007199254740992*(COXA_M + FEMUR_M + TIBIA_M));
		LF[2] = (COXA_M*L1_COM[2] + FEMUR_M*(L2_COM[2]*cos_q2 + L2_COM[0]*sin_q2) + TIBIA_M*(L3_COM[0]*(cos_q2*sin_q3 + cos_q3*sin_q2) + L3_COM[2]*(cos_q2*cos_q3 - sin_q2*sin_q3) + L2*sin_q2))/(COXA_M + FEMUR_M + TIBIA_M);

		return LF

	def LM_com(self, q):
		q1 = np.deg2rad(q[0]);
		q2 = np.deg2rad(q[1]);
		q3 = np.deg2rad(q[2]);

		cos_q1 = np.cos(q1)
		sin_q1 = np.sin(q1)
		cos_q2 = np.cos(q2)
		sin_q2 = np.sin(q2)
		cos_q3 = np.cos(q3)
		sin_q3 = np.sin(q3)

		LM = np.zeros(3)
		LM[0] =(4967757600021511*(TIBIA_M*(L3_COM[0]*(cos_q1*cos_q2*cos_q3 - cos_q1*sin_q2*sin_q3) - L3_COM[2]*(cos_q1*cos_q2*sin_q3 + cos_q1*cos_q3*sin_q2) + L1*cos_q1 - L3_COM[1]*sin_q1 + L2*cos_q1*cos_q2) + FEMUR_M*(L1*cos_q1 - L2_COM[1]*sin_q1 + L2_COM[0]*cos_q1*cos_q2 - L2_COM[2]*cos_q1*sin_q2) + COXA_M*(L1_COM[0]*cos_q1 - L1_COM[1]*sin_q1)))/(81129638414606681695789005144064*(COXA_M + FEMUR_M + TIBIA_M)) - (COXA_M*(L1_COM[1]*cos_q1 + L1_COM[0]*sin_q1) + TIBIA_M*(L3_COM[1]*cos_q1 - L3_COM[2]*(cos_q2*sin_q1*sin_q3 + cos_q3*sin_q1*sin_q2) - L3_COM[0]*(sin_q1*sin_q2*sin_q3 - cos_q2*cos_q3*sin_q1) + L1*sin_q1 + L2*cos_q2*sin_q1) + FEMUR_M*(L2_COM[1]*cos_q1 + L1*sin_q1 + L2_COM[0]*cos_q2*sin_q1 - L2_COM[2]*sin_q1*sin_q2))/(COXA_M + FEMUR_M + TIBIA_M)
		LM[1] = COXA_Y + (4967757600021511*(COXA_M*(L1_COM[1]*cos_q1 + L1_COM[0]*sin_q1) + TIBIA_M*(L3_COM[1]*cos_q1 - L3_COM[2]*(cos_q2*sin_q1*sin_q3 + cos_q3*sin_q1*sin_q2) - L3_COM[0]*(sin_q1*sin_q2*sin_q3 - cos_q2*cos_q3*sin_q1) + L1*sin_q1 + L2*cos_q2*sin_q1) + FEMUR_M*(L2_COM[1]*cos_q1 + L1*sin_q1 + L2_COM[0]*cos_q2*sin_q1 - L2_COM[2]*sin_q1*sin_q2)))/(81129638414606681695789005144064*(COXA_M + FEMUR_M + TIBIA_M)) + (TIBIA_M*(L3_COM[0]*(cos_q1*cos_q2*cos_q3 - cos_q1*sin_q2*sin_q3) - L3_COM[2]*(cos_q1*cos_q2*sin_q3 + cos_q1*cos_q3*sin_q2) + L1*cos_q1 - L3_COM[1]*sin_q1 + L2*cos_q1*cos_q2) + FEMUR_M*(L1*cos_q1 - L2_COM[1]*sin_q1 + L2_COM[0]*cos_q1*cos_q2 - L2_COM[2]*cos_q1*sin_q2) + COXA_M*(L1_COM[0]*cos_q1 - L1_COM[1]*sin_q1))/(COXA_M + FEMUR_M + TIBIA_M)
		LM[2] = (COXA_M*L1_COM[2] + FEMUR_M*(L2_COM[2]*cos_q2 + L2_COM[0]*sin_q2) + TIBIA_M*(L3_COM[0]*(cos_q2*sin_q3 + cos_q3*sin_q2) + L3_COM[2]*(cos_q2*cos_q3 - sin_q2*sin_q3) + L2*sin_q2))/(COXA_M + FEMUR_M + TIBIA_M)

		return LM

	def LR_com(self, q):
		q1 = np.deg2rad(q[0]);
		q2 = np.deg2rad(q[1]);
		q3 = np.deg2rad(q[2]);

		cos_q1 = np.cos(q1)
		sin_q1 = np.sin(q1)
		cos_q2 = np.cos(q2)
		sin_q2 = np.sin(q2)
		cos_q3 = np.cos(q3)
		sin_q3 = np.sin(q3)

		LR = np.zeros(3)
		LR[0] = - COXA_X - (6899914937159737*(COXA_M*(L1_COM[1]*cos_q1 + L1_COM[0]*sin_q1) + TIBIA_M*(L3_COM[1]*cos_q1 - L3_COM[2]*(cos_q2*sin_q1*sin_q3 + cos_q3*sin_q1*sin_q2) - L3_COM[0]*(sin_q1*sin_q2*sin_q3 - cos_q2*cos_q3*sin_q1) + L1*sin_q1 + L2*cos_q2*sin_q1) + FEMUR_M*(L2_COM[1]*cos_q1 + L1*sin_q1 + L2_COM[0]*cos_q2*sin_q1 - L2_COM[2]*sin_q1*sin_q2)))/(9007199254740992*(COXA_M + FEMUR_M + TIBIA_M)) - (5789716078925341*(TIBIA_M*(L3_COM[0]*(cos_q1*cos_q2*cos_q3 - cos_q1*sin_q2*sin_q3) - L3_COM[2]*(cos_q1*cos_q2*sin_q3 + cos_q1*cos_q3*sin_q2) + L1*cos_q1 - L3_COM[1]*sin_q1 + L2*cos_q1*cos_q2) + FEMUR_M*(L1*cos_q1 - L2_COM[1]*sin_q1 + L2_COM[0]*cos_q1*cos_q2 - L2_COM[2]*cos_q1*sin_q2) + COXA_M*(L1_COM[0]*cos_q1 - L1_COM[1]*sin_q1)))/(9007199254740992*(COXA_M + FEMUR_M + TIBIA_M))
		LR[1] = COXA_Y - (5789716078925341*(COXA_M*(L1_COM[1]*cos_q1 + L1_COM[0]*sin_q1) + TIBIA_M*(L3_COM[1]*cos_q1 - L3_COM[2]*(cos_q2*sin_q1*sin_q3 + cos_q3*sin_q1*sin_q2) - L3_COM[0]*(sin_q1*sin_q2*sin_q3 - cos_q2*cos_q3*sin_q1) + L1*sin_q1 + L2*cos_q2*sin_q1) + FEMUR_M*(L2_COM[1]*cos_q1 + L1*sin_q1 + L2_COM[0]*cos_q2*sin_q1 - L2_COM[2]*sin_q1*sin_q2)))/(9007199254740992*(COXA_M + FEMUR_M + TIBIA_M)) + (6899914937159737*(TIBIA_M*(L3_COM[0]*(cos_q1*cos_q2*cos_q3 - cos_q1*sin_q2*sin_q3) - L3_COM[2]*(cos_q1*cos_q2*sin_q3 + cos_q1*cos_q3*sin_q2) + L1*cos_q1 - L3_COM[1]*sin_q1 + L2*cos_q1*cos_q2) + FEMUR_M*(L1*cos_q1 - L2_COM[1]*sin_q1 + L2_COM[0]*cos_q1*cos_q2 - L2_COM[2]*cos_q1*sin_q2) + COXA_M*(L1_COM[0]*cos_q1 - L1_COM[1]*sin_q1)))/(9007199254740992*(COXA_M + FEMUR_M + TIBIA_M))
		LR[2] = (COXA_M*L1_COM[2] + FEMUR_M*(L2_COM[2]*cos_q2 + L2_COM[0]*sin_q2) + TIBIA_M*(L3_COM[0]*(cos_q2*sin_q3 + cos_q3*sin_q2) + L3_COM[2]*(cos_q2*cos_q3 - sin_q2*sin_q3) + L2*sin_q2))/(COXA_M + FEMUR_M + TIBIA_M)
		return LR

	def RF_com(self, q):
		q1 = np.deg2rad(q[0]);
		q2 = np.deg2rad(q[1]);
		q3 = np.deg2rad(q[2]);

		cos_q1 = np.cos(q1)
		sin_q1 = np.sin(q1)
		cos_q2 = np.cos(q2)
		sin_q2 = np.sin(q2)
		cos_q3 = np.cos(q3)
		sin_q3 = np.sin(q3)

		RF = np.zeros(3)
		RF[0] = COXA_X + (6899914937159737*(COXA_M*(L1_COM[1]*cos_q1 + L1_COM[0]*sin_q1) + TIBIA_M*(L3_COM[1]*cos_q1 - L3_COM[2]*(cos_q2*sin_q1*sin_q3 + cos_q3*sin_q1*sin_q2) - L3_COM[0]*(sin_q1*sin_q2*sin_q3 - cos_q2*cos_q3*sin_q1) + L1*sin_q1 + L2*cos_q2*sin_q1) + FEMUR_M*(L2_COM[1]*cos_q1 + L1*sin_q1 + L2_COM[0]*cos_q2*sin_q1 - L2_COM[2]*sin_q1*sin_q2)))/(9007199254740992*(COXA_M + FEMUR_M + TIBIA_M)) + (5789716078925341*(TIBIA_M*(L3_COM[0]*(cos_q1*cos_q2*cos_q3 - cos_q1*sin_q2*sin_q3) - L3_COM[2]*(cos_q1*cos_q2*sin_q3 + cos_q1*cos_q3*sin_q2) + L1*cos_q1 - L3_COM[1]*sin_q1 + L2*cos_q1*cos_q2) + FEMUR_M*(L1*cos_q1 - L2_COM[1]*sin_q1 + L2_COM[0]*cos_q1*cos_q2 - L2_COM[2]*cos_q1*sin_q2) + COXA_M*(L1_COM[0]*cos_q1 - L1_COM[1]*sin_q1)))/(9007199254740992*(COXA_M + FEMUR_M + TIBIA_M))
		RF[1] = (5789716078925341*(COXA_M*(L1_COM[1]*cos_q1 + L1_COM[0]*sin_q1) + TIBIA_M*(L3_COM[1]*cos_q1 - L3_COM[2]*(cos_q2*sin_q1*sin_q3 + cos_q3*sin_q1*sin_q2) - L3_COM[0]*(sin_q1*sin_q2*sin_q3 - cos_q2*cos_q3*sin_q1) + L1*sin_q1 + L2*cos_q2*sin_q1) + FEMUR_M*(L2_COM[1]*cos_q1 + L1*sin_q1 + L2_COM[0]*cos_q2*sin_q1 - L2_COM[2]*sin_q1*sin_q2)))/(9007199254740992*(COXA_M + FEMUR_M + TIBIA_M)) - COXA_Y - (6899914937159737*(TIBIA_M*(L3_COM[0]*(cos_q1*cos_q2*cos_q3 - cos_q1*sin_q2*sin_q3) - L3_COM[2]*(cos_q1*cos_q2*sin_q3 + cos_q1*cos_q3*sin_q2) + L1*cos_q1 - L3_COM[1]*sin_q1 + L2*cos_q1*cos_q2) + FEMUR_M*(L1*cos_q1 - L2_COM[1]*sin_q1 + L2_COM[0]*cos_q1*cos_q2 - L2_COM[2]*cos_q1*sin_q2) + COXA_M*(L1_COM[0]*cos_q1 - L1_COM[1]*sin_q1)))/(9007199254740992*(COXA_M + FEMUR_M + TIBIA_M))
		RF[2] = (COXA_M*L1_COM[2] + FEMUR_M*(L2_COM[2]*cos_q2 + L2_COM[0]*sin_q2) + TIBIA_M*(L3_COM[0]*(cos_q2*sin_q3 + cos_q3*sin_q2) + L3_COM[2]*(cos_q2*cos_q3 - sin_q2*sin_q3) + L2*sin_q2))/(COXA_M + FEMUR_M + TIBIA_M)
		return RF

	def RM_com(self, q):
		q1 = np.deg2rad(q[0]);
		q2 = np.deg2rad(q[1]);
		q3 = np.deg2rad(q[2]);

		cos_q1 = np.cos(q1)
		sin_q1 = np.sin(q1)
		cos_q2 = np.cos(q2)
		sin_q2 = np.sin(q2)
		cos_q3 = np.cos(q3)
		sin_q3 = np.sin(q3)

		RM = np.zeros(3)
		RM[0] = (COXA_M*(L1_COM[1]*cos_q1 + L1_COM[0]*sin_q1) + TIBIA_M*(L3_COM[1]*cos_q1 - L3_COM[2]*(cos_q2*sin_q1*sin_q3 + cos_q3*sin_q1*sin_q2) - L3_COM[0]*(sin_q1*sin_q2*sin_q3 - cos_q2*cos_q3*sin_q1) + L1*sin_q1 + L2*cos_q2*sin_q1) + FEMUR_M*(L2_COM[1]*cos_q1 + L1*sin_q1 + L2_COM[0]*cos_q2*sin_q1 - L2_COM[2]*sin_q1*sin_q2))/(COXA_M + FEMUR_M + TIBIA_M) + (4967757600021511*(TIBIA_M*(L3_COM[0]*(cos_q1*cos_q2*cos_q3 - cos_q1*sin_q2*sin_q3) - L3_COM[2]*(cos_q1*cos_q2*sin_q3 + cos_q1*cos_q3*sin_q2) + L1*cos_q1 - L3_COM[1]*sin_q1 + L2*cos_q1*cos_q2) + FEMUR_M*(L1*cos_q1 - L2_COM[1]*sin_q1 + L2_COM[0]*cos_q1*cos_q2 - L2_COM[2]*cos_q1*sin_q2) + COXA_M*(L1_COM[0]*cos_q1 - L1_COM[1]*sin_q1)))/(81129638414606681695789005144064*(COXA_M + FEMUR_M + TIBIA_M))
		RM[1] = (4967757600021511*(COXA_M*(L1_COM[1]*cos_q1 + L1_COM[0]*sin_q1) + TIBIA_M*(L3_COM[1]*cos_q1 - L3_COM[2]*(cos_q2*sin_q1*sin_q3 + cos_q3*sin_q1*sin_q2) - L3_COM[0]*(sin_q1*sin_q2*sin_q3 - cos_q2*cos_q3*sin_q1) + L1*sin_q1 + L2*cos_q2*sin_q1) + FEMUR_M*(L2_COM[1]*cos_q1 + L1*sin_q1 + L2_COM[0]*cos_q2*sin_q1 - L2_COM[2]*sin_q1*sin_q2)))/(81129638414606681695789005144064*(COXA_M + FEMUR_M + TIBIA_M)) - COXA_Y - (TIBIA_M*(L3_COM[0]*(cos_q1*cos_q2*cos_q3 - cos_q1*sin_q2*sin_q3) - L3_COM[2]*(cos_q1*cos_q2*sin_q3 + cos_q1*cos_q3*sin_q2) + L1*cos_q1 - L3_COM[1]*sin_q1 + L2*cos_q1*cos_q2) + FEMUR_M*(L1*cos_q1 - L2_COM[1]*sin_q1 + L2_COM[0]*cos_q1*cos_q2 - L2_COM[2]*cos_q1*sin_q2) + COXA_M*(L1_COM[0]*cos_q1 - L1_COM[1]*sin_q1))/(COXA_M + FEMUR_M + TIBIA_M)
		RM[2] = (COXA_M*L1_COM[2] + FEMUR_M*(L2_COM[2]*cos_q2 + L2_COM[0]*sin_q2) + TIBIA_M*(L3_COM[0]*(cos_q2*sin_q3 + cos_q3*sin_q2) + L3_COM[2]*(cos_q2*cos_q3 - sin_q2*sin_q3) + L2*sin_q2))/(COXA_M + FEMUR_M + TIBIA_M)
		return RM

	def RR_com(self, q):
		q1 = np.deg2rad(q[0]);
		q2 = np.deg2rad(q[1]);
		q3 = np.deg2rad(q[2]);

		cos_q1 = np.cos(q1)
		sin_q1 = np.sin(q1)
		cos_q2 = np.cos(q2)
		sin_q2 = np.sin(q2)
		cos_q3 = np.cos(q3)
		sin_q3 = np.sin(q3)

		RR = np.zeros(3)
		RR[0] = (6899914937159737*(COXA_M*(L1_COM[1]*cos_q1 + L1_COM[0]*sin_q1) + TIBIA_M*(L3_COM[1]*cos_q1 - L3_COM[2]*(cos_q2*sin_q1*sin_q3 + cos_q3*sin_q1*sin_q2) - L3_COM[0]*(sin_q1*sin_q2*sin_q3 - cos_q2*cos_q3*sin_q1) + L1*sin_q1 + L2*cos_q2*sin_q1) + FEMUR_M*(L2_COM[1]*cos_q1 + L1*sin_q1 + L2_COM[0]*cos_q2*sin_q1 - L2_COM[2]*sin_q1*sin_q2)))/(9007199254740992*(COXA_M + FEMUR_M + TIBIA_M)) - COXA_X - (5789716078925341*(TIBIA_M*(L3_COM[0]*(cos_q1*cos_q2*cos_q3 - cos_q1*sin_q2*sin_q3) - L3_COM[2]*(cos_q1*cos_q2*sin_q3 + cos_q1*cos_q3*sin_q2) + L1*cos_q1 - L3_COM[1]*sin_q1 + L2*cos_q1*cos_q2) + FEMUR_M*(L1*cos_q1 - L2_COM[1]*sin_q1 + L2_COM[0]*cos_q1*cos_q2 - L2_COM[2]*cos_q1*sin_q2) + COXA_M*(L1_COM[0]*cos_q1 - L1_COM[1]*sin_q1)))/(9007199254740992*(COXA_M + FEMUR_M + TIBIA_M))
		RR[1] = - COXA_Y - (5789716078925341*(COXA_M*(L1_COM[1]*cos_q1 + L1_COM[0]*sin_q1) + TIBIA_M*(L3_COM[1]*cos_q1 - L3_COM[2]*(cos_q2*sin_q1*sin_q3 + cos_q3*sin_q1*sin_q2) - L3_COM[0]*(sin_q1*sin_q2*sin_q3 - cos_q2*cos_q3*sin_q1) + L1*sin_q1 + L2*cos_q2*sin_q1) + FEMUR_M*(L2_COM[1]*cos_q1 + L1*sin_q1 + L2_COM[0]*cos_q2*sin_q1 - L2_COM[2]*sin_q1*sin_q2)))/(9007199254740992*(COXA_M + FEMUR_M + TIBIA_M)) - (6899914937159737*(TIBIA_M*(L3_COM[0]*(cos_q1*cos_q2*cos_q3 - cos_q1*sin_q2*sin_q3) - L3_COM[2]*(cos_q1*cos_q2*sin_q3 + cos_q1*cos_q3*sin_q2) + L1*cos_q1 - L3_COM[1]*sin_q1 + L2*cos_q1*cos_q2) + FEMUR_M*(L1*cos_q1 - L2_COM[1]*sin_q1 + L2_COM[0]*cos_q1*cos_q2 - L2_COM[2]*cos_q1*sin_q2) + COXA_M*(L1_COM[0]*cos_q1 - L1_COM[1]*sin_q1)))/(9007199254740992*(COXA_M + FEMUR_M + TIBIA_M))
		RR[2] = (COXA_M*L1_COM[2] + FEMUR_M*(L2_COM[2]*cos_q2 + L2_COM[0]*sin_q2) + TIBIA_M*(L3_COM[0]*(cos_q2*sin_q3 + cos_q3*sin_q2) + L3_COM[2]*(cos_q2*cos_q3 - sin_q2*sin_q3) + L2*sin_q2))/(COXA_M + FEMUR_M + TIBIA_M)
		return RR 

q = [0, 30, -80, 0, 30, -80, 0, 30, -80, 0, 30, -80, 0, 30, -80, 0, 30, -80]

# a = RigidBodyInertia()
# print np.round(a.update_CoM(q),4)

	# def leg_com(self, q):
	# 	L1 = 0.06;
	# 	L2 = 0.15;
	# 	L3 = 0.15;

	# 	COXA_M = 0.040586;
	# 	FEMUR_M = 0.347529;
	# 	TIBIA_M = 0.10313;
	# 	# LEG_MASS = COXA_M + FEMUR_M + TIBIA_M

	# 	L1_COM[0] = 0.024713;
	# 	L1_COM[1] = 0;
	# 	L1_COM[2] = 0;

	# 	L2_COM[0] = 0.075000;
	# 	L2_COM[1] = -0.000231;
	# 	L2_COM[2] = 0;

	# 	L3_COM[0] = 0.10585;
	# 	L3_COM[1] = 0;
	# 	L3_COM[2] = 0;

	# 	q1 = np.deg2rad(q[0]);
	# 	q2 = np.deg2rad(q[1]);
	# 	q3 = np.deg2rad(q[2]);

	# 	cos_q1 = np.cos(q1)
	# 	sin_q1 = np.sin(q1)
	# 	cos_q2 = np.cos(q2)
	# 	sin_q2 = np.sin(q2)
	# 	cos_q3 = np.cos(q3)
	# 	sin_q3 = np.sin(q3)

	# 	com_x = (TIBIA_M*(L3_COM[0]*(cos_q1*cos_q2*cos_q3 - cos_q1*sin_q2*sin_q3) - L3_COM[2]*(cos_q1*cos_q2*sin_q3 + cos_q1*cos_q3*sin_q2) + L1*cos_q1 - L3_COM[1]*sin_q1 + L2*cos_q1*cos_q2) + FEMUR_M*(L1*cos_q1 - L2_COM[1]*sin_q1 + L2_COM[0]*cos_q1*cos_q2 - L2_COM[2]*cos_q1*sin_q2) + COXA_M*(L1_COM[0]*cos_q1 - L1_COM[1]*sin_q1))/(COXA_M + FEMUR_M + TIBIA_M);
	# 	com_y = (COXA_M*(L1_COM[1]*cos_q1 + L1_COM[0]*sin_q1) + TIBIA_M*(L3_COM[1]*cos_q1 - L3_COM[2]*(cos_q2*sin_q1*sin_q3 + cos_q3*sin_q1*sin_q2) - L3_COM[0]*(sin_q1*sin_q2*sin_q3 - cos_q2*cos_q3*sin_q1) + L1*sin_q1 + L2*cos_q2*sin_q1) + FEMUR_M*(L2_COM[1]*cos_q1 + L1*sin_q1 + L2_COM[0]*cos_q2*sin_q1 - L2_COM[2]*sin_q1*sin_q2))/(COXA_M + FEMUR_M + TIBIA_M);
	# 	com_z = (COXA_M*L1_COM[2] + FEMUR_M*(L2_COM[2]*cos_q2 + L2_COM[0]*sin_q2) + TIBIA_M*(L3_COM[0]*(cos_q2*sin_q3 + cos_q3*sin_q2) + L3_COM[2]*(cos_q2*cos_q3 - sin_q2*sin_q3) + L2*sin_q2))/(COXA_M + FEMUR_M + TIBIA_M);
		 
	# 	return np.array([com_x, com_y, com_z])

