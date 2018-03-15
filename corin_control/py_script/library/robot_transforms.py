#!/usr/bin/env python

## Class for Corin robot homogeneous transformations
## Indexing for leg starts with 0, 1 .... 5
## 2D numpy arrays used

import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'class'))
sys.dont_write_bytecode = True

import numpy as np
from constant import * 		# constants used
import kdl 					# corin kinematic library

# import transformations as TF

Column6D = (6,1)

def deg2rad(q):
	return (q*np.pi/180.)

class Vector6D:
	""" 6D vector for robot """ 
	def __init__(self):
		self.world_X_base 	 = np.zeros(Column6D)
		self.world_X_LF_foot = np.zeros(Column6D)
		self.world_X_LM_foot = np.zeros(Column6D)
		self.world_X_LR_foot = np.zeros(Column6D)
		self.world_X_RF_foot = np.zeros(Column6D)
		self.world_X_RM_foot = np.zeros(Column6D)
		self.world_X_RR_foot = np.zeros(Column6D)

		## TODO: following needs expanding if to be used
		self.base_X_coxa  = np.zeros(Column6D)
		self.base_X_foot  = np.zeros(Column6D)
		self.base_X_AEP   = np.zeros(Column6D)
		self.base_X_PEP   = np.zeros(Column6D)
		self.base_X_NRP   = np.zeros(Column6D)
		self.coxa_X_foot  = np.zeros(Column6D)
		self.coxa_X_AEP   = np.zeros(Column6D)
		self.coxa_X_PEP   = np.zeros(Column6D)
		self.coxa_X_NRP   = np.zeros(Column6D)

class ArrayVector6D:
	""" 6D vector for robot's legs """ 
	def __init__(self):
		self.world_X_foot = np.zeros(Column6D)
		self.base_X_coxa  = np.zeros(Column6D)
		self.base_X_foot  = np.zeros(Column6D)
		self.coxa_X_base  = np.zeros(Column6D)
		self.coxa_X_foot  = np.zeros(Column6D)
		

class ArrayHomogeneousTransform:
	""" SE(3) for robot's leg transformations """
	def __init__(self, leg_no):
		self.n = leg_no

		# Transformation wrt world frame: world to parts
		# self.world_X_base = np.identity(4)
		self.world_X_foot = np.identity(4)
		
		# Transformations wrt world frame: base to parts
		self.world_base_X_foot = np.identity(4)
		self.world_base_X_AEP  = np.identity(4)
		self.world_base_X_PEP  = np.identity(4)
		self.world_base_X_NRP  = np.identity(4)

		# Transformations wrt base frame: base to parts
		self.base_X_coxa = np.identity(4)
		self.base_X_foot = np.identity(4)
		self.base_X_AEP  = np.identity(4)
		self.base_X_PEP  = np.identity(4)
		self.base_X_NRP  = np.identity(4)

		# Transformations wrt leg frame: leg to parts
		self.coxa_X_base = np.identity(4)
		self.coxa_X_foot = np.identity(4)
		self.coxa_X_AEP  = np.identity(4)
		self.coxa_X_PEP  = np.identity(4)
		self.coxa_X_NRP  = np.identity(4)

		## CoM ##
		self.base_X_COM = np.identity(4)		# done

		self._initialize_()

	def _initialize_(self):
		""" Set default values for robot's nominal stance """

		## Define Variables ##
		# qb  = np.zeros((6,1))
		q   = np.zeros((3,1))
		KDL = kdl.KDL()

		# set base to be standing up height
		# qb[2] = BODY_HEIGHT
		# set legs to default standup position
		q = KDL.leg_IK([STANCE_WIDTH,0.,-BODY_HEIGHT])

		self.update_base_X_coxa(ROT_BASE_X_LEG[self.n],TRN_BASE_X_LEG[self.n])
		self.update_base_X_foot(q)
		self.update_coxa_X_foot(q)

	def update_base_X_coxa(self, q, tx):
		self.base_X_coxa[0,0] =  np.cos(deg2rad(q))
		self.base_X_coxa[0,1] = -np.sin(deg2rad(q))
		self.base_X_coxa[1,0] =  np.sin(deg2rad(q))
		self.base_X_coxa[1,1] =  np.cos(deg2rad(q))
		self.base_X_coxa[0,3] =  tx[0]
		self.base_X_coxa[1,3] =  tx[1]

		self.coxa_X_base = np.linalg.inv(self.base_X_coxa)

	def update_base_X_foot(self,q):	
		q1_sin = np.sin(q[0])
		q2_sin = np.sin(q[1])
		q3_sin = np.sin(q[2])
		q1_cos = np.cos(q[0])
		q2_cos = np.cos(q[1])
		q3_cos = np.cos(q[2])
		rZ_sin = np.sin(deg2rad(ROT_BASE_X_LEG[self.n]))
		rZ_cos = np.cos(deg2rad(ROT_BASE_X_LEG[self.n]))
		tX = TRN_BASE_X_LEG[self.n][0]
		tY = TRN_BASE_X_LEG[self.n][1]

		self.base_X_foot[0,0] = -(q2_cos*q3_cos - q2_sin*q3_sin)*(rZ_sin*q1_sin - rZ_cos*q1_cos)
		self.base_X_foot[0,1] =  (q2_sin*q3_cos + q2_cos*q3_sin)*(rZ_sin*q1_sin - rZ_cos*q1_cos)
		self.base_X_foot[0,2] =  rZ_cos*q1_sin + rZ_sin*q1_cos
		self.base_X_foot[0,3] = -rZ_sin*(q2_cos*q3_cos - q2_sin*q3_sin)*q1_sin*L3 - rZ_sin*q2_cos*q1_sin*L2 + (q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos*rZ_cos*L3 + q2_cos*q1_cos*rZ_cos*L2 - rZ_sin*q1_sin*L1 + q1_cos*rZ_cos*L1 + tX
		self.base_X_foot[1,0] =  (q2_cos*q3_cos - q2_sin*q3_sin)*(rZ_cos*q1_sin + rZ_sin*q1_cos)
		self.base_X_foot[1,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*(rZ_cos*q1_sin + rZ_sin*q1_cos)
		self.base_X_foot[1,2] =  rZ_sin*q1_sin - rZ_cos*q1_cos
		self.base_X_foot[1,3] =  rZ_sin*(q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos*L3 + rZ_sin*q2_cos*q1_cos*L2 + (q2_cos*q3_cos - q2_sin*q3_sin)*rZ_cos*q1_sin*L3 + q2_cos*rZ_cos*q1_sin*L2 + rZ_sin*q1_cos*L1 + rZ_cos*q1_sin*L1 + tY
		self.base_X_foot[2,0] =  (q2_sin*q3_cos + q2_cos*q3_sin)
		self.base_X_foot[2,1] =  (q2_cos*q3_cos - q2_sin*q3_sin)
		self.base_X_foot[2,1] =  0.
		self.base_X_foot[2,3] =  (q2_sin*q3_cos + q2_cos*q3_sin)*L3 + q2_sin*L2

	def update_world_base_X_foot(self, mx_world_X_base, q):
		q1_sin = np.sin(q[0])
		q2_sin = np.sin(q[1])
		q3_sin = np.sin(q[2])
		q1_cos = np.cos(q[0])
		q2_cos = np.cos(q[1])
		q3_cos = np.cos(q[2])
		rZ_sin = np.sin(deg2rad(ROT_BASE_X_LEG[self.n]))
		rZ_cos = np.cos(deg2rad(ROT_BASE_X_LEG[self.n]))
		tX = TRN_BASE_X_LEG[self.n][0]
		tY = TRN_BASE_X_LEG[self.n][1]

		mx_world_X_base[:3,3:4] = np.zeros((3,1)) 	# overwrite linear components to ignore them
		
		self.world_X_foot = np.dot(mx_world_X_base, self.base_X_foot)
		self.world_base_X_foot = np.dot(self.world_X_foot, mx_world_X_base)

	def update_base_X_NRP(self, q):
		self.update_coxa_X_NRP(q)
		self.base_X_NRP = np.dot(self.base_X_coxa, self.coxa_X_NRP)
		self.world_base_X_NRP = self.base_X_NRP.copy()

	def update_base_X_AEP(self, q):
		self.update_coxa_X_AEP(q)
		self.base_X_AEP = np.dot(self.base_X_coxa, self.coxa_X_AEP)
		self.world_base_X_AEP = self.base_X_AEP.copy()

	def update_base_X_PEP(self, q):
		self.update_coxa_X_PEP(q)
		self.base_X_PEP = np.dot(self.base_X_coxa, self.coxa_X_PEP)
		self.world_base_X_PEP = self.base_X_PEP.copy()

	def update_coxa_X_foot(self, q):
		q1_sin = np.sin(q[0])
		q2_sin = np.sin(q[1])
		q3_sin = np.sin(q[2])
		q1_cos = np.cos(q[0])
		q2_cos = np.cos(q[1])
		q3_cos = np.cos(q[2])

		self.coxa_X_foot[0,0] = (q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos
		self.coxa_X_foot[0,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*q1_cos
		self.coxa_X_foot[0,2] = q1_sin
		self.coxa_X_foot[0,3] = q1_cos*(L1 + L3*(q2_cos*q3_cos - q2_sin*q3_sin) + L2*q2_cos)
		self.coxa_X_foot[1,0] = (q2_cos*q3_cos - q2_sin*q3_sin)*q1_sin
		self.coxa_X_foot[1,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*q1_sin
		self.coxa_X_foot[1,2] = -q1_cos
		self.coxa_X_foot[1,3] = q1_sin*(L1 + L3*(q2_cos*q3_cos - q2_sin*q3_sin) + L2*q2_cos)
		self.coxa_X_foot[2,0] = (q2_sin*q3_cos + q2_cos*q3_sin)
		self.coxa_X_foot[2,1] = (q2_cos*q3_cos - q2_sin*q3_sin)
		self.coxa_X_foot[2,3] = L3*(q2_sin*q3_cos + q2_cos*q3_sin) + L2*q2_sin

	def update_coxa_X_NRP(self, q):
		q1_sin = np.sin(q[0])
		q2_sin = np.sin(q[1])
		q3_sin = np.sin(q[2])
		q1_cos = np.cos(q[0])
		q2_cos = np.cos(q[1])
		q3_cos = np.cos(q[2])

		self.coxa_X_NRP[0,0] = (q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos
		self.coxa_X_NRP[0,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*q1_cos
		self.coxa_X_NRP[0,2] = q1_sin
		self.coxa_X_NRP[0,3] = q1_cos*(L1 + L3*(q2_cos*q3_cos - q2_sin*q3_sin) + L2*q2_cos)
		self.coxa_X_NRP[1,0] = (q2_cos*q3_cos - q2_sin*q3_sin)*q1_sin
		self.coxa_X_NRP[1,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*q1_sin
		self.coxa_X_NRP[1,2] = -q1_cos
		self.coxa_X_NRP[1,3] = q1_sin*(L1 + L3*(q2_cos*q3_cos - q2_sin*q3_sin) + L2*q2_cos)
		self.coxa_X_NRP[2,0] = (q2_sin*q3_cos + q2_cos*q3_sin)
		self.coxa_X_NRP[2,1] = (q2_cos*q3_cos - q2_sin*q3_sin)
		self.coxa_X_NRP[2,3] = L3*(q2_sin*q3_cos + q2_cos*q3_sin) + L2*q2_sin

	def update_coxa_X_AEP(self, q):
		q1_sin = np.sin(q[0])
		q2_sin = np.sin(q[1])
		q3_sin = np.sin(q[2])
		q1_cos = np.cos(q[0])
		q2_cos = np.cos(q[1])
		q3_cos = np.cos(q[2])

		self.coxa_X_AEP[0,0] = (q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos
		self.coxa_X_AEP[0,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*q1_cos
		self.coxa_X_AEP[0,2] = q1_sin
		self.coxa_X_AEP[0,3] = q1_cos*(L1 + L3*(q2_cos*q3_cos - q2_sin*q3_sin) + L2*q2_cos)
		self.coxa_X_AEP[1,0] = (q2_cos*q3_cos - q2_sin*q3_sin)*q1_sin
		self.coxa_X_AEP[1,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*q1_sin
		self.coxa_X_AEP[1,2] = -q1_cos
		self.coxa_X_AEP[1,3] = q1_sin*(L1 + L3*(q2_cos*q3_cos - q2_sin*q3_sin) + L2*q2_cos)
		self.coxa_X_AEP[2,0] = (q2_sin*q3_cos + q2_cos*q3_sin)
		self.coxa_X_AEP[2,1] = (q2_cos*q3_cos - q2_sin*q3_sin)
		self.coxa_X_AEP[2,3] = L3*(q2_sin*q3_cos + q2_cos*q3_sin) + L2*q2_sin

	def update_coxa_X_PEP(self, q):
		q1_sin = np.sin(q[0])
		q2_sin = np.sin(q[1])
		q3_sin = np.sin(q[2])
		q1_cos = np.cos(q[0])
		q2_cos = np.cos(q[1])
		q3_cos = np.cos(q[2])

		self.coxa_X_PEP[0,0] = (q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos
		self.coxa_X_PEP[0,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*q1_cos
		self.coxa_X_PEP[0,2] = q1_sin
		self.coxa_X_PEP[0,3] = q1_cos*(L1 + L3*(q2_cos*q3_cos - q2_sin*q3_sin) + L2*q2_cos)
		self.coxa_X_PEP[1,0] = (q2_cos*q3_cos - q2_sin*q3_sin)*q1_sin
		self.coxa_X_PEP[1,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*q1_sin
		self.coxa_X_PEP[1,2] = -q1_cos
		self.coxa_X_PEP[1,3] = q1_sin*(L1 + L3*(q2_cos*q3_cos - q2_sin*q3_sin) + L2*q2_cos)
		self.coxa_X_PEP[2,0] = (q2_sin*q3_cos + q2_cos*q3_sin)
		self.coxa_X_PEP[2,1] = (q2_cos*q3_cos - q2_sin*q3_sin)
		self.coxa_X_PEP[2,3] = L3*(q2_sin*q3_cos + q2_cos*q3_sin) + L2*q2_sin

	def update_base_X_COM(self):
		# TODO: what if base is rotated?
		self.base_X_COM[0:3] = BODY_COM[0] + LEG_MASS*(self.LF_COM[0,3])
		self.base_X_COM[1:3] = BODY_COM[1] + LEG_MASS*(self.LF_COM[1,3])
		self.base_X_COM[2:3] = BODY_COM[2] + LEG_MASS*(self.LF_COM[2,3])

XH = ArrayHomogeneousTransform(1)
# print XH.base_X_foot
# print XH.world_base_X_foot

## ============================================================================================================================= ##
## ============================================================================================================================= ##
## ============================================================================================================================= ##

class HomogeneousTransform:
	def __init__(self):
		self.world_X_base = np.identity(4)
		self.base_X_world = np.identity(4)

		## ============================================= ##
		# self.Leg = [None]*6
		# for j in range(0,6):
		# 	self.Leg[j] = ArrayHomogeneousTransform()

		## ============================================= ##
		self.world_X_LF_foot = np.identity(4)
		self.world_X_LM_foot = np.identity(4)
		self.world_X_LR_foot = np.identity(4)
		self.world_X_RF_foot = np.identity(4)
		self.world_X_RM_foot = np.identity(4)
		self.world_X_RR_foot = np.identity(4)
		
		## base to hip frame of each leg; one-off
		self.base_X_LF_coxa  = np.identity(4)
		self.base_X_LM_coxa  = np.identity(4)
		self.base_X_LR_coxa  = np.identity(4)
		self.base_X_RF_coxa  = np.identity(4)
		self.base_X_RM_coxa  = np.identity(4)
		self.base_X_RR_coxa  = np.identity(4)

		self.base_X_LF_foot = np.identity(4)
		self.base_X_LM_foot = np.identity(4)
		self.base_X_LR_foot = np.identity(4)
		self.base_X_RF_foot = np.identity(4)
		self.base_X_RM_foot = np.identity(4)
		self.base_X_RR_foot = np.identity(4)

		self.LF_coxa_X_LF_foot = np.identity(4)
		self.LM_coxa_X_LM_foot = np.identity(4)
		self.LR_coxa_X_LR_foot = np.identity(4)
		self.RF_coxa_X_RF_foot = np.identity(4)
		self.RM_coxa_X_RM_foot = np.identity(4)
		self.RR_coxa_X_RR_foot = np.identity(4)

		## Anterior Extreme Position
		self.base_X_LF_AEP = np.identity(4)
		self.base_X_LM_AEP = np.identity(4)
		self.base_X_LR_AEP = np.identity(4)
		self.base_X_RF_AEP = np.identity(4)
		self.base_X_RM_AEP = np.identity(4)
		self.base_X_RR_AEP = np.identity(4)

		## Posterior Extreme Position
		self.base_X_LF_PEP = np.identity(4)
		self.base_X_LM_PEP = np.identity(4)
		self.base_X_LR_PEP = np.identity(4)
		self.base_X_RF_PEP = np.identity(4)
		self.base_X_RM_PEP = np.identity(4)
		self.base_X_RR_PEP = np.identity(4)

		## Nominal Resting Position
		self.base_X_LF_NRP = np.identity(4)
		self.base_X_LM_NRP = np.identity(4)
		self.base_X_LR_NRP = np.identity(4)
		self.base_X_RF_NRP = np.identity(4)
		self.base_X_RM_NRP = np.identity(4)
		self.base_X_RR_NRP = np.identity(4)

		## CoM frames
		self.base_X_COM = np.identity(4)
		self.LF_COM = np.identity(4)
		self.LM_COM = np.identity(4)
		self.LR_COM = np.identity(4)
		self.RF_COM = np.identity(4)
		self.RM_COM = np.identity(4)
		self.RR_COM = np.identity(4)

		self.LF_coxa_X_coxa_COM	= np.identity(4)
		self.LF_coxa_X_femur_COM	= np.identity(4)
		self.LF_coxa_X_tibia_COM	= np.identity(4)

		## Edges of foothold area - inner bottom to outer bottom; one-off
		self.base_X_LF_edge_1 = np.identity(4)
		self.base_X_LF_edge_2 = np.identity(4)
		self.base_X_LF_edge_3 = np.identity(4)
		self.base_X_LF_edge_4 = np.identity(4)
		self.base_X_LM_edge_1 = np.identity(4)
		self.base_X_LM_edge_2 = np.identity(4)
		self.base_X_LM_edge_3 = np.identity(4)
		self.base_X_LM_edge_4 = np.identity(4)
		self.base_X_LR_edge_1 = np.identity(4)
		self.base_X_LR_edge_2 = np.identity(4)
		self.base_X_LR_edge_3 = np.identity(4)
		self.base_X_LR_edge_4 = np.identity(4)
		self.base_X_RF_edge_1 = np.identity(4)
		self.base_X_RF_edge_2 = np.identity(4)
		self.base_X_RF_edge_3 = np.identity(4)
		self.base_X_RF_edge_4 = np.identity(4)
		self.base_X_RM_edge_1 = np.identity(4)
		self.base_X_RM_edge_2 = np.identity(4)
		self.base_X_RM_edge_3 = np.identity(4)
		self.base_X_RM_edge_4 = np.identity(4)
		self.base_X_RR_edge_1 = np.identity(4)
		self.base_X_RR_edge_2 = np.identity(4)
		self.base_X_RR_edge_3 = np.identity(4)
		self.base_X_RR_edge_4 = np.identity(4)
		
		self._initialize()

	def _initialize(self):
		## Set default values for robot's nominal stance
		q  = np.zeros(18)
		qb = np.zeros(6)

		KDL = kdl.KDL()
		xp  = KDL.leg_IK([STANCE_WIDTH,0.,-BODY_HEIGHT])
		# set legs to default standup position
		for i in range(0,6): 	
			q[0+i*3] = xp[0]
			q[1+i*3] = xp[1]
			q[2+i*3] = xp[2]
		qb[5] = BODY_HEIGHT

		self.__init_update() 		# single update
		self.update_robot(qb,q) # runtime update

	def __init_update(self):
		self.update_base_X_LF_coxa(0)
		self.update_base_X_LM_coxa(0)
		self.update_base_X_LR_coxa(0)
		self.update_base_X_RF_coxa(0)
		self.update_base_X_RM_coxa(0)
		self.update_base_X_RR_coxa(0)

		## Edges of leg bounding box - used in motion planning
		# self.update_base_X_LF_edge()
		# self.update_base_X_LM_edge()
		# self.update_base_X_LR_edge()
		# self.update_base_X_RF_edge()
		# self.update_base_X_RM_edge()
		# self.update_base_X_RR_edge()

		# self.coxa_X_coxaCOM()
		# self.femur_X_femurCOM()
		# self.tibia_X_tibiaCOM()

	def update_robot(self,qb,q):
		self.update_base(qb)
		self.update_legs(q)

	def update_base(self,qb):
		self.update_world_X_base(qb)

	def update_legs(self,q):
		## Forward Kinematics using leg joint angles
		self.update_base_X_LF_foot(q)
		self.update_base_X_LM_foot(q)
		self.update_base_X_LR_foot(q)
		self.update_base_X_RF_foot(q)
		self.update_base_X_RM_foot(q)
		self.update_base_X_RR_foot(q)

		## Forward Kinematics using leg joint angles
		self.update_LF_coxa_X_LF_foot(q)
		self.update_LM_coxa_X_LM_foot(q)
		self.update_LR_coxa_X_LR_foot(q)
		self.update_RF_coxa_X_RF_foot(q)
		self.update_RM_coxa_X_RM_foot(q)
		self.update_RR_coxa_X_RR_foot(q)

		self.update_LF_coxa_X_LF_COM(q)
		self.update_base_X_COM()

	def update_base_X_COM(self):
		# TODO: what if base is rotated?
		self.base_X_COM[0:3] = BODY_COM[0] + LEG_MASS*(self.LF_COM[0,3])
		self.base_X_COM[1:3] = BODY_COM[1] + LEG_MASS*(self.LF_COM[1,3])
		self.base_X_COM[2:3] = BODY_COM[2] + LEG_MASS*(self.LF_COM[2,3])

	def update_LF_coxa_X_LF_COM(self,q):
		# TODO: check against solidworks
		q1_sin = np.sin(q[LF_Q1_JOINT])
		q2_sin = np.sin(q[LF_Q2_JOINT])
		q3_sin = np.sin(q[LF_Q3_JOINT])
		q1_cos = np.cos(q[LF_Q1_JOINT])
		q2_cos = np.cos(q[LF_Q2_JOINT])
		q3_cos = np.cos(q[LF_Q3_JOINT])

		self.LF_coxa_X_coxa_COM[0,3] = L1_COM[0]*q1_cos - L1_COM[1]*q1_sin
		self.LF_coxa_X_coxa_COM[1,3] = L1_COM[1]*q1_sin + L1_COM[0]*q1_cos
		self.LF_coxa_X_coxa_COM[2,3] = L1_COM[2]

		self.LF_coxa_X_femur_COM[0,3] = L1*q1_cos + L2_COM[0]*q1_cos*q2_cos - L2_COM[1]*q1_cos*q2_sin + L2_COM[2]*q1_sin
		self.LF_coxa_X_femur_COM[1,3] = L1*q1_sin + L2_COM[0]*q1_sin*q2_cos - L2_COM[1]*q1_sin*q2_sin - L2_COM[2]*q1_cos
		self.LF_coxa_X_femur_COM[2,3] = L2_COM[0]*q2_sin + L2_COM[1]*q2_cos

		self.LF_coxa_X_tibia_COM[0,3] = (L1*q1_cos + L2*q1_cos*q2_cos + L3_COM[0]*q1_cos*q2_cos*q3_cos - L3_COM[0]*q1_cos*q2_sin*q3_sin +
												L3_COM[2]*q1_sin - L3_COM[1]*q1_cos*q2_cos*q3_sin - L3_COM[1]*q1_cos*q2_sin*q3_cos)
		self.LF_coxa_X_tibia_COM[1,3] = (L1*q1_sin + L2*q1_sin*q2_cos + L3_COM[0]*q1_sin*q2_cos*q3_cos - L3_COM[0]*q1_sin*q2_sin*q3_sin
		 										- L3_COM[1]*q1_sin*q2_cos*q3_sin - L3_COM[1]*q1_sin*q2_sin*q3_cos - L3_COM[1]*q1_cos)
		self.LF_coxa_X_tibia_COM[2,3] = (L2*q2_sin + L3_COM[0]*q2_sin*q3_cos + L3_COM[0]*q2_cos*q3_sin + L3_COM[1]*q2_cos*q3_cos - 
												L3_COM[1]*q2_sin*q3_sin)

		self.LF_COM[0,3] = (L1_MASS*self.LF_coxa_X_coxa_COM[0,3] + L2_MASS*self.LF_coxa_X_femur_COM[0,3] + L3_MASS*self.LF_coxa_X_tibia_COM[0,3])/LEG_MASS
		self.LF_COM[1,3] = (L1_MASS*self.LF_coxa_X_coxa_COM[1,3] + L2_MASS*self.LF_coxa_X_femur_COM[1,3] + L3_MASS*self.LF_coxa_X_tibia_COM[1,3])/LEG_MASS
		self.LF_COM[2,3] = (L1_MASS*self.LF_coxa_X_coxa_COM[2,3] + L2_MASS*self.LF_coxa_X_femur_COM[2,3] + L3_MASS*self.LF_coxa_X_tibia_COM[2,3])/LEG_MASS

	def update_world_X_base(self,q):
		# rotates in sequence x, y, z in world frame 
		tx = q[0]
		ty = q[1]
		tz = q[2]
		qx_sin = np.sin(q[3])
		qy_sin = np.sin(q[4])
		qz_sin = np.sin(q[5])
		qx_cos = np.cos(q[3])
		qy_cos = np.cos(q[4])
		qz_cos = np.cos(q[5])

		self.world_X_base[0,0] =  qz_cos*qy_cos
		self.world_X_base[0,1] = -qz_sin*qx_cos + qz_cos*qy_sin*qx_sin
		self.world_X_base[0,2] =  qz_sin*qx_sin + qz_cos*qy_sin*qx_cos
		self.world_X_base[0,3] =  (qz_sin*qx_sin + qz_cos*qy_sin*qx_cos)*tz + (-qz_sin*qx_cos + qz_cos*qy_sin*qx_sin)*ty + qz_cos*qy_cos*tx
		self.world_X_base[1,0] =  qz_sin*qy_cos
		self.world_X_base[1,1] =  qz_cos*qx_cos + qz_sin*qy_sin*qx_sin
		self.world_X_base[1,2] = -qz_cos*qx_sin + qz_sin*qy_sin*qx_cos
		self.world_X_base[1,3] =  (-qz_cos*qx_sin + qz_sin*qy_sin*qx_cos)*tz + (qz_cos*qx_cos + qz_sin*qy_sin*qx_sin)*ty + qz_sin*qy_cos*tx
		self.world_X_base[2,0] = -qy_sin
		self.world_X_base[2,1] =  qy_cos*qx_sin
		self.world_X_base[2,2] =	 qy_cos*qx_cos
		self.world_X_base[2,3] =  qy_cos*qx_cos*tz + qy_cos*qx_sin*ty - qy_sin*tx

		self.update_base_X_world()
		
	def update_base_X_world(self):
		self.base_X_world = np.linalg.inv(self.world_X_base)

	def update_base_X_coxa(self, q, tx):
		""" generic method for updating each leg """

		self.base_X_coxa[0,0] =  np.cos(deg2rad(q))
		self.base_X_coxa[0,1] = -np.sin(deg2rad(q))
		self.base_X_coxa[1,0] =  np.sin(deg2rad(q))
		self.base_X_coxa[1,1] =  np.cos(deg2rad(q))
		self.base_X_coxa[0,3] =  tx[0]
		self.base_X_coxa[1,3] =  tx[1]

	def update_base_X_LF_coxa(self,q=None):
		self.base_X_LF_coxa[0,0] =  np.cos(deg2rad(ROT_BASE_X_LF))
		self.base_X_LF_coxa[0,1] = -np.sin(deg2rad(ROT_BASE_X_LF))
		self.base_X_LF_coxa[1,0] =  np.sin(deg2rad(ROT_BASE_X_LF))
		self.base_X_LF_coxa[1,1] =  np.cos(deg2rad(ROT_BASE_X_LF))
		self.base_X_LF_coxa[0,3] =  COXA_X
		self.base_X_LF_coxa[1,3] =  COXA_Y

	def update_base_X_LM_coxa(self,q=None):
		self.base_X_LM_coxa[0,0] =  np.cos(deg2rad(ROT_BASE_X_LM))
		self.base_X_LM_coxa[0,1] = -np.sin(deg2rad(ROT_BASE_X_LM))
		self.base_X_LM_coxa[1,0] =  np.sin(deg2rad(ROT_BASE_X_LM))
		self.base_X_LM_coxa[1,1] =  np.cos(deg2rad(ROT_BASE_X_LM))
		self.base_X_LM_coxa[1,3] =  COXA_Y

	def update_base_X_LR_coxa(self,q=None):
		self.base_X_LR_coxa[0,0] =  np.cos(deg2rad(ROT_BASE_X_LR))
		self.base_X_LR_coxa[0,1] = -np.sin(deg2rad(ROT_BASE_X_LR))
		self.base_X_LR_coxa[1,0] =  np.sin(deg2rad(ROT_BASE_X_LR))
		self.base_X_LR_coxa[1,1] =  np.cos(deg2rad(ROT_BASE_X_LR))
		self.base_X_LR_coxa[0,3] = -COXA_X
		self.base_X_LR_coxa[1,3] =  COXA_Y

	def update_base_X_RF_coxa(self,q=None):
		self.base_X_RF_coxa[0,0] =  np.cos(deg2rad(ROT_BASE_X_RF))
		self.base_X_RF_coxa[0,1] = -np.sin(deg2rad(ROT_BASE_X_RF))
		self.base_X_RF_coxa[1,0] =  np.sin(deg2rad(ROT_BASE_X_RF))
		self.base_X_RF_coxa[1,1] =  np.cos(deg2rad(ROT_BASE_X_RF))
		self.base_X_RF_coxa[0,3] =  COXA_X
		self.base_X_RF_coxa[1,3] = -COXA_Y

	def update_base_X_RM_coxa(self,q=None):
		self.base_X_RM_coxa[0,0] =  np.cos(deg2rad(ROT_BASE_X_RM))
		self.base_X_RM_coxa[0,1] = -np.sin(deg2rad(ROT_BASE_X_RM))
		self.base_X_RM_coxa[1,0] =  np.sin(deg2rad(ROT_BASE_X_RM))
		self.base_X_RM_coxa[1,1] =  np.cos(deg2rad(ROT_BASE_X_RM))
		self.base_X_RM_coxa[1,3] = -COXA_Y

	def update_base_X_RR_coxa(self,q=None):
		self.base_X_RR_coxa[0,0] =  np.cos(deg2rad(ROT_BASE_X_RR))
		self.base_X_RR_coxa[0,1] = -np.sin(deg2rad(ROT_BASE_X_RR))
		self.base_X_RR_coxa[1,0] =  np.sin(deg2rad(ROT_BASE_X_RR))
		self.base_X_RR_coxa[1,1] =  np.cos(deg2rad(ROT_BASE_X_RR))
		self.base_X_RR_coxa[0,3] = -COXA_X
		self.base_X_RR_coxa[1,3] = -COXA_Y

	def update_base_X_LF_NRP(self,q):
		self.update_LF_coxa_X_LF_foot(q)
		self.base_X_LF_NRP = np.dot(self.base_X_LF_coxa, self.LF_coxa_X_LF_foot)

	def update_base_X_LM_NRP(self,q):
		self.update_LM_coxa_X_LM_foot(q)
		self.base_X_LM_NRP = np.dot(self.base_X_LM_coxa, self.LM_coxa_X_LM_foot)

	def update_base_X_LR_NRP(self,q):
		self.update_LR_coxa_X_LR_foot(q)
		self.base_X_LR_NRP = np.dot(self.base_X_LR_coxa, self.LR_coxa_X_LR_foot)

	def update_base_X_RF_NRP(self,q):
		self.update_RF_coxa_X_RF_foot(q)
		self.base_X_RF_NRP = np.dot(self.base_X_RF_coxa, self.RF_coxa_X_RF_foot)

	def update_base_X_RM_NRP(self,q):
		self.update_RM_coxa_X_RM_foot(q)
		self.base_X_RM_NRP = np.dot(self.base_X_RM_coxa, self.RM_coxa_X_RM_foot)

	def update_base_X_RR_NRP(self,q):
		self.update_RR_coxa_X_RR_foot(q)
		self.base_X_RR_NRP = np.dot(self.base_X_RR_coxa, self.RR_coxa_X_RR_foot)

	def update_LF_coxa_X_LF_foot(self,q):
		if (len(q)>3):
			q1_sin = np.sin(q[LF_Q1_JOINT])
			q2_sin = np.sin(q[LF_Q2_JOINT])
			q3_sin = np.sin(q[LF_Q3_JOINT])
			q1_cos = np.cos(q[LF_Q1_JOINT])
			q2_cos = np.cos(q[LF_Q2_JOINT])
			q3_cos = np.cos(q[LF_Q3_JOINT])
		else:
			q1_sin = np.sin(q[0])
			q2_sin = np.sin(q[1])
			q3_sin = np.sin(q[2])
			q1_cos = np.cos(q[0])
			q2_cos = np.cos(q[1])
			q3_cos = np.cos(q[2])

		self.LF_coxa_X_LF_foot[0,0] = (q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos
		self.LF_coxa_X_LF_foot[0,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*q1_cos
		self.LF_coxa_X_LF_foot[0,2] = q1_sin
		self.LF_coxa_X_LF_foot[0,3] = q1_cos*(L1 + L3*(q2_cos*q3_cos - q2_sin*q3_sin) + L2*q2_cos)
		self.LF_coxa_X_LF_foot[1,0] = (q2_cos*q3_cos - q2_sin*q3_sin)*q1_sin
		self.LF_coxa_X_LF_foot[1,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*q1_sin
		self.LF_coxa_X_LF_foot[1,2] = -q1_cos
		self.LF_coxa_X_LF_foot[1,3] = q1_sin*(L1 + L3*(q2_cos*q3_cos - q2_sin*q3_sin) + L2*q2_cos)
		self.LF_coxa_X_LF_foot[2,0] = (q2_sin*q3_cos + q2_cos*q3_sin)
		self.LF_coxa_X_LF_foot[2,1] = (q2_cos*q3_cos - q2_sin*q3_sin)
		self.LF_coxa_X_LF_foot[2,3] = L3*(q2_sin*q3_cos + q2_cos*q3_sin) + L2*q2_sin

	def update_LM_coxa_X_LM_foot(self,q):
		if (len(q)>3):
			q1_sin = np.sin(q[LM_Q1_JOINT])
			q2_sin = np.sin(q[LM_Q2_JOINT])
			q3_sin = np.sin(q[LM_Q3_JOINT])
			q1_cos = np.cos(q[LM_Q1_JOINT])
			q2_cos = np.cos(q[LM_Q2_JOINT])
			q3_cos = np.cos(q[LM_Q3_JOINT])
		else:
			q1_sin = np.sin(q[0])
			q2_sin = np.sin(q[1])
			q3_sin = np.sin(q[2])
			q1_cos = np.cos(q[0])
			q2_cos = np.cos(q[1])
			q3_cos = np.cos(q[2])

		self.LM_coxa_X_LM_foot[0,0] = (q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos
		self.LM_coxa_X_LM_foot[0,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*q1_cos
		self.LM_coxa_X_LM_foot[0,2] = q1_sin
		self.LM_coxa_X_LM_foot[0,3] = q1_cos*(L1 + L3*(q2_cos*q3_cos - q2_sin*q3_sin) + L2*q2_cos)
		self.LM_coxa_X_LM_foot[1,0] = (q2_cos*q3_cos - q2_sin*q3_sin)*q1_sin
		self.LM_coxa_X_LM_foot[1,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*q1_sin
		self.LM_coxa_X_LM_foot[1,2] = -q1_cos
		self.LM_coxa_X_LM_foot[1,3] = q1_sin*(L1 + L3*(q2_cos*q3_cos - q2_sin*q3_sin) + L2*q2_cos)
		self.LM_coxa_X_LM_foot[2,0] = (q2_sin*q3_cos + q2_cos*q3_sin)
		self.LM_coxa_X_LM_foot[2,1] = (q2_cos*q3_cos - q2_sin*q3_sin)
		self.LM_coxa_X_LM_foot[2,3] = L3*(q2_sin*q3_cos + q2_cos*q3_sin) + L2*q2_sin

	def update_LR_coxa_X_LR_foot(self,q):
		if (len(q)>3):
			q1_sin = np.sin(q[LR_Q1_JOINT])
			q2_sin = np.sin(q[LR_Q2_JOINT])
			q3_sin = np.sin(q[LR_Q3_JOINT])
			q1_cos = np.cos(q[LR_Q1_JOINT])
			q2_cos = np.cos(q[LR_Q2_JOINT])
			q3_cos = np.cos(q[LR_Q3_JOINT])
		else:
			q1_sin = np.sin(q[0])
			q2_sin = np.sin(q[1])
			q3_sin = np.sin(q[2])
			q1_cos = np.cos(q[0])
			q2_cos = np.cos(q[1])
			q3_cos = np.cos(q[2])

		self.LR_coxa_X_LR_foot[0,0] = (q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos
		self.LR_coxa_X_LR_foot[0,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*q1_cos
		self.LR_coxa_X_LR_foot[0,2] = q1_sin
		self.LR_coxa_X_LR_foot[0,3] = q1_cos*(L1 + L3*(q2_cos*q3_cos - q2_sin*q3_sin) + L2*q2_cos)
		self.LR_coxa_X_LR_foot[1,0] = (q2_cos*q3_cos - q2_sin*q3_sin)*q1_sin
		self.LR_coxa_X_LR_foot[1,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*q1_sin
		self.LR_coxa_X_LR_foot[1,2] = -q1_cos
		self.LR_coxa_X_LR_foot[1,3] = q1_sin*(L1 + L3*(q2_cos*q3_cos - q2_sin*q3_sin) + L2*q2_cos)
		self.LR_coxa_X_LR_foot[2,0] = (q2_sin*q3_cos + q2_cos*q3_sin)
		self.LR_coxa_X_LR_foot[2,1] = (q2_cos*q3_cos - q2_sin*q3_sin)
		self.LR_coxa_X_LR_foot[2,3] = L3*(q2_sin*q3_cos + q2_cos*q3_sin) + L2*q2_sin

	def update_RF_coxa_X_RF_foot(self,q):
		if (len(q)>3):
			q1_sin = np.sin(q[RF_Q1_JOINT])
			q2_sin = np.sin(q[RF_Q2_JOINT])
			q3_sin = np.sin(q[RF_Q3_JOINT])
			q1_cos = np.cos(q[RF_Q1_JOINT])
			q2_cos = np.cos(q[RF_Q2_JOINT])
			q3_cos = np.cos(q[RF_Q3_JOINT])
		else:
			q1_sin = np.sin(q[0])
			q2_sin = np.sin(q[1])
			q3_sin = np.sin(q[2])
			q1_cos = np.cos(q[0])
			q2_cos = np.cos(q[1])
			q3_cos = np.cos(q[2])

		self.RF_coxa_X_RF_foot[0,0] = (q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos
		self.RF_coxa_X_RF_foot[0,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*q1_cos
		self.RF_coxa_X_RF_foot[0,2] = q1_sin
		self.RF_coxa_X_RF_foot[0,3] = q1_cos*(L1 + L3*(q2_cos*q3_cos - q2_sin*q3_sin) + L2*q2_cos)
		self.RF_coxa_X_RF_foot[1,0] = (q2_cos*q3_cos - q2_sin*q3_sin)*q1_sin
		self.RF_coxa_X_RF_foot[1,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*q1_sin
		self.RF_coxa_X_RF_foot[1,2] = -q1_cos
		self.RF_coxa_X_RF_foot[1,3] = q1_sin*(L1 + L3*(q2_cos*q3_cos - q2_sin*q3_sin) + L2*q2_cos)
		self.RF_coxa_X_RF_foot[2,0] = (q2_sin*q3_cos + q2_cos*q3_sin)
		self.RF_coxa_X_RF_foot[2,1] = (q2_cos*q3_cos - q2_sin*q3_sin)
		self.RF_coxa_X_RF_foot[2,3] = L3*(q2_sin*q3_cos + q2_cos*q3_sin) + L2*q2_sin

	def update_RM_coxa_X_RM_foot(self,q):
		if (len(q)>3):
			q1_sin = np.sin(q[RM_Q1_JOINT])
			q2_sin = np.sin(q[RM_Q2_JOINT])
			q3_sin = np.sin(q[RM_Q3_JOINT])
			q1_cos = np.cos(q[RM_Q1_JOINT])
			q2_cos = np.cos(q[RM_Q2_JOINT])
			q3_cos = np.cos(q[RM_Q3_JOINT])
		else:
			q1_sin = np.sin(q[0])
			q2_sin = np.sin(q[1])
			q3_sin = np.sin(q[2])
			q1_cos = np.cos(q[0])
			q2_cos = np.cos(q[1])
			q3_cos = np.cos(q[2])

		self.RM_coxa_X_RM_foot[0,0] = (q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos
		self.RM_coxa_X_RM_foot[0,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*q1_cos
		self.RM_coxa_X_RM_foot[0,2] = q1_sin
		self.RM_coxa_X_RM_foot[0,3] = q1_cos*(L1 + L3*(q2_cos*q3_cos - q2_sin*q3_sin) + L2*q2_cos)
		self.RM_coxa_X_RM_foot[1,0] = (q2_cos*q3_cos - q2_sin*q3_sin)*q1_sin
		self.RM_coxa_X_RM_foot[1,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*q1_sin
		self.RM_coxa_X_RM_foot[1,2] = -q1_cos
		self.RM_coxa_X_RM_foot[1,3] = q1_sin*(L1 + L3*(q2_cos*q3_cos - q2_sin*q3_sin) + L2*q2_cos)
		self.RM_coxa_X_RM_foot[2,0] = (q2_sin*q3_cos + q2_cos*q3_sin)
		self.RM_coxa_X_RM_foot[2,1] = (q2_cos*q3_cos - q2_sin*q3_sin)
		self.RM_coxa_X_RM_foot[2,3] = L3*(q2_sin*q3_cos + q2_cos*q3_sin) + L2*q2_sin

	def update_RR_coxa_X_RR_foot(self,q):
		if (len(q)>3):
			q1_sin = np.sin(q[RR_Q1_JOINT])
			q2_sin = np.sin(q[RR_Q2_JOINT])
			q3_sin = np.sin(q[RR_Q3_JOINT])
			q1_cos = np.cos(q[RR_Q1_JOINT])
			q2_cos = np.cos(q[RR_Q2_JOINT])
			q3_cos = np.cos(q[RR_Q3_JOINT])
		else:
			q1_sin = np.sin(q[0])
			q2_sin = np.sin(q[1])
			q3_sin = np.sin(q[2])
			q1_cos = np.cos(q[0])
			q2_cos = np.cos(q[1])
			q3_cos = np.cos(q[2])

		self.RR_coxa_X_RR_foot[0,0] = (q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos
		self.RR_coxa_X_RR_foot[0,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*q1_cos
		self.RR_coxa_X_RR_foot[0,2] = q1_sin
		self.RR_coxa_X_RR_foot[0,3] = q1_cos*(L1 + L3*(q2_cos*q3_cos - q2_sin*q3_sin) + L2*q2_cos)
		self.RR_coxa_X_RR_foot[1,0] = (q2_cos*q3_cos - q2_sin*q3_sin)*q1_sin
		self.RR_coxa_X_RR_foot[1,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*q1_sin
		self.RR_coxa_X_RR_foot[1,2] = -q1_cos
		self.RR_coxa_X_RR_foot[1,3] = q1_sin*(L1 + L3*(q2_cos*q3_cos - q2_sin*q3_sin) + L2*q2_cos)
		self.RR_coxa_X_RR_foot[2,0] = (q2_sin*q3_cos + q2_cos*q3_sin)
		self.RR_coxa_X_RR_foot[2,1] = (q2_cos*q3_cos - q2_sin*q3_sin)
		self.RR_coxa_X_RR_foot[2,3] = L3*(q2_sin*q3_cos + q2_cos*q3_sin) + L2*q2_sin


	def update_base_X_LF_foot(self,q):
		if (len(q)>3):
			q1_sin = np.sin(q[LF_Q1_JOINT])
			q2_sin = np.sin(q[LF_Q2_JOINT])
			q3_sin = np.sin(q[LF_Q3_JOINT])
			q1_cos = np.cos(q[LF_Q1_JOINT])
			q2_cos = np.cos(q[LF_Q2_JOINT])
			q3_cos = np.cos(q[LF_Q3_JOINT])
		else:
			q1_sin = np.sin(q[0])
			q2_sin = np.sin(q[1])
			q3_sin = np.sin(q[2])
			q1_cos = np.cos(q[0])
			q2_cos = np.cos(q[1])
			q3_cos = np.cos(q[2])
		rZ_sin = np.sin(deg2rad(ROT_BASE_X_LF))
		rZ_cos = np.cos(deg2rad(ROT_BASE_X_LF))

		self.base_X_LF_foot[0,0] = -(q2_cos*q3_cos - q2_sin*q3_sin)*(rZ_sin*q1_sin - rZ_cos*q1_cos)
		self.base_X_LF_foot[0,1] =  (q2_sin*q3_cos + q2_cos*q3_sin)*(rZ_sin*q1_sin - rZ_cos*q1_cos)
		self.base_X_LF_foot[0,2] =  rZ_cos*q1_sin + rZ_sin*q1_cos
		self.base_X_LF_foot[0,3] = -rZ_sin*(q2_cos*q3_cos - q2_sin*q3_sin)*q1_sin*L3 - rZ_sin*q2_cos*q1_sin*L2 + (q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos*rZ_cos*L3 + q2_cos*q1_cos*rZ_cos*L2 - rZ_sin*q1_sin*L1 + q1_cos*rZ_cos*L1 + COXA_X
		self.base_X_LF_foot[1,0] =  (q2_cos*q3_cos - q2_sin*q3_sin)*(rZ_cos*q1_sin + rZ_sin*q1_cos)
		self.base_X_LF_foot[1,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*(rZ_cos*q1_sin + rZ_sin*q1_cos)
		self.base_X_LF_foot[1,2] =  rZ_sin*q1_sin - rZ_cos*q1_cos
		self.base_X_LF_foot[1,3] =  rZ_sin*(q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos*L3 + rZ_sin*q2_cos*q1_cos*L2 + (q2_cos*q3_cos - q2_sin*q3_sin)*rZ_cos*q1_sin*L3 + q2_cos*rZ_cos*q1_sin*L2 + rZ_sin*q1_cos*L1 + rZ_cos*q1_sin*L1 + COXA_Y
		self.base_X_LF_foot[2,0] =  (q2_sin*q3_cos + q2_cos*q3_sin)
		self.base_X_LF_foot[2,1] =  (q2_cos*q3_cos - q2_sin*q3_sin)
		self.base_X_LF_foot[2,1] =  0.
		self.base_X_LF_foot[2,3] =  (q2_sin*q3_cos + q2_cos*q3_sin)*L3 + q2_sin*L2

	def update_base_X_LM_foot(self,q):
		if (len(q)>3):
			q1_sin = np.sin(q[LM_Q1_JOINT])
			q2_sin = np.sin(q[LM_Q2_JOINT])
			q3_sin = np.sin(q[LM_Q3_JOINT])
			q1_cos = np.cos(q[LM_Q1_JOINT])
			q2_cos = np.cos(q[LM_Q2_JOINT])
			q3_cos = np.cos(q[LM_Q3_JOINT])
		else:
			q1_sin = np.sin(q[0])
			q2_sin = np.sin(q[1])
			q3_sin = np.sin(q[2])
			q1_cos = np.cos(q[0])
			q2_cos = np.cos(q[1])
			q3_cos = np.cos(q[2])
		rZ_sin = np.sin(deg2rad(ROT_BASE_X_LM))
		rZ_cos = np.cos(deg2rad(ROT_BASE_X_LM))

		self.base_X_LM_foot[0,0] = -(q2_cos*q3_cos - q2_sin*q3_sin)*(rZ_sin*q1_sin - rZ_cos*q1_cos)
		self.base_X_LM_foot[0,1] =  (q2_sin*q3_cos + q2_cos*q3_sin)*(rZ_sin*q1_sin - rZ_cos*q1_cos)
		self.base_X_LM_foot[0,2] =  rZ_cos*q1_sin + rZ_sin*q1_cos
		self.base_X_LM_foot[0,3] = -rZ_sin*(q2_cos*q3_cos - q2_sin*q3_sin)*q1_sin*L3 - rZ_sin*q2_cos*q1_sin*L2 + (q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos*rZ_cos*L3 + q2_cos*q1_cos*rZ_cos*L2 - rZ_sin*q1_sin*L1 + q1_cos*rZ_cos*L1
		self.base_X_LM_foot[1,0] =  (q2_cos*q3_cos - q2_sin*q3_sin)*(rZ_cos*q1_sin + rZ_sin*q1_cos)
		self.base_X_LM_foot[1,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*(rZ_cos*q1_sin + rZ_sin*q1_cos)
		self.base_X_LM_foot[1,2] =  rZ_sin*q1_sin - rZ_cos*q1_cos
		self.base_X_LM_foot[1,3] =  rZ_sin*(q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos*L3 + rZ_sin*q2_cos*q1_cos*L2 + (q2_cos*q3_cos - q2_sin*q3_sin)*rZ_cos*q1_sin*L3 + q2_cos*rZ_cos*q1_sin*L2 + rZ_sin*q1_cos*L1 + rZ_cos*q1_sin*L1 + COXA_Y
		self.base_X_LM_foot[2,0] =  (q2_sin*q3_cos + q2_cos*q3_sin)
		self.base_X_LM_foot[2,1] =  (q2_cos*q3_cos - q2_sin*q3_sin)
		self.base_X_LM_foot[2,1] =  0.
		self.base_X_LM_foot[2,3] =  (q2_sin*q3_cos + q2_cos*q3_sin)*L3 + q2_sin*L2

	def update_base_X_LR_foot(self,q):
		if (len(q)>3):
			q1_sin = np.sin(q[LR_Q1_JOINT])
			q2_sin = np.sin(q[LR_Q2_JOINT])
			q3_sin = np.sin(q[LR_Q3_JOINT])
			q1_cos = np.cos(q[LR_Q1_JOINT])
			q2_cos = np.cos(q[LR_Q2_JOINT])
			q3_cos = np.cos(q[LR_Q3_JOINT])
		else:
			q1_sin = np.sin(q[0])
			q2_sin = np.sin(q[1])
			q3_sin = np.sin(q[2])
			q1_cos = np.cos(q[0])
			q2_cos = np.cos(q[1])
			q3_cos = np.cos(q[2])
		rZ_sin = np.sin(deg2rad(ROT_BASE_X_LR))
		rZ_cos = np.cos(deg2rad(ROT_BASE_X_LR))

		self.base_X_LR_foot[0,0] = -(q2_cos*q3_cos - q2_sin*q3_sin)*(rZ_sin*q1_sin - rZ_cos*q1_cos)
		self.base_X_LR_foot[0,1] =  (q2_sin*q3_cos + q2_cos*q3_sin)*(rZ_sin*q1_sin - rZ_cos*q1_cos)
		self.base_X_LR_foot[0,2] =  rZ_cos*q1_sin + rZ_sin*q1_cos
		self.base_X_LR_foot[0,3] = -rZ_sin*(q2_cos*q3_cos - q2_sin*q3_sin)*q1_sin*L3 - rZ_sin*q2_cos*q1_sin*L2 + (q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos*rZ_cos*L3 + q2_cos*q1_cos*rZ_cos*L2 - rZ_sin*q1_sin*L1 + q1_cos*rZ_cos*L1 - COXA_X
		self.base_X_LR_foot[1,0] =  (q2_cos*q3_cos - q2_sin*q3_sin)*(rZ_cos*q1_sin + rZ_sin*q1_cos)
		self.base_X_LR_foot[1,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*(rZ_cos*q1_sin + rZ_sin*q1_cos)
		self.base_X_LR_foot[1,2] =  rZ_sin*q1_sin - rZ_cos*q1_cos
		self.base_X_LR_foot[1,3] =  rZ_sin*(q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos*L3 + rZ_sin*q2_cos*q1_cos*L2 + (q2_cos*q3_cos - q2_sin*q3_sin)*rZ_cos*q1_sin*L3 + q2_cos*rZ_cos*q1_sin*L2 + rZ_sin*q1_cos*L1 + rZ_cos*q1_sin*L1 + COXA_Y
		self.base_X_LR_foot[2,0] =  (q2_sin*q3_cos + q2_cos*q3_sin)
		self.base_X_LR_foot[2,1] =  (q2_cos*q3_cos - q2_sin*q3_sin)
		self.base_X_LR_foot[2,1] =  0.
		self.base_X_LR_foot[2,3] =  (q2_sin*q3_cos + q2_cos*q3_sin)*L3 + q2_sin*L2

	def update_base_X_RF_foot(self,q):
		if (len(q)>3):
			q1_sin = np.sin(q[RF_Q1_JOINT])
			q2_sin = np.sin(q[RF_Q2_JOINT])
			q3_sin = np.sin(q[RF_Q3_JOINT])
			q1_cos = np.cos(q[RF_Q1_JOINT])
			q2_cos = np.cos(q[RF_Q2_JOINT])
			q3_cos = np.cos(q[RF_Q3_JOINT])
		else:
			q1_sin = np.sin(q[0])
			q2_sin = np.sin(q[1])
			q3_sin = np.sin(q[2])
			q1_cos = np.cos(q[0])
			q2_cos = np.cos(q[1])
			q3_cos = np.cos(q[2])
		rZ_sin = np.sin(deg2rad(ROT_BASE_X_RF))
		rZ_cos = np.cos(deg2rad(ROT_BASE_X_RF))

		self.base_X_RF_foot[0,0] = -(q2_cos*q3_cos - q2_sin*q3_sin)*(rZ_sin*q1_sin - rZ_cos*q1_cos)
		self.base_X_RF_foot[0,1] =  (q2_sin*q3_cos + q2_cos*q3_sin)*(rZ_sin*q1_sin - rZ_cos*q1_cos)
		self.base_X_RF_foot[0,2] =  rZ_cos*q1_sin + rZ_sin*q1_cos
		self.base_X_RF_foot[0,3] = -rZ_sin*(q2_cos*q3_cos - q2_sin*q3_sin)*q1_sin*L3 - rZ_sin*q2_cos*q1_sin*L2 + (q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos*rZ_cos*L3 + q2_cos*q1_cos*rZ_cos*L2 - rZ_sin*q1_sin*L1 + q1_cos*rZ_cos*L1 + COXA_X
		self.base_X_RF_foot[1,0] =  (q2_cos*q3_cos - q2_sin*q3_sin)*(rZ_cos*q1_sin + rZ_sin*q1_cos)
		self.base_X_RF_foot[1,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*(rZ_cos*q1_sin + rZ_sin*q1_cos)
		self.base_X_RF_foot[1,2] =  rZ_sin*q1_sin - rZ_cos*q1_cos
		self.base_X_RF_foot[1,3] =  rZ_sin*(q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos*L3 + rZ_sin*q2_cos*q1_cos*L2 + (q2_cos*q3_cos - q2_sin*q3_sin)*rZ_cos*q1_sin*L3 + q2_cos*rZ_cos*q1_sin*L2 + rZ_sin*q1_cos*L1 + rZ_cos*q1_sin*L1 - COXA_Y
		self.base_X_RF_foot[2,0] =  (q2_sin*q3_cos + q2_cos*q3_sin)
		self.base_X_RF_foot[2,1] =  (q2_cos*q3_cos - q2_sin*q3_sin)
		self.base_X_RF_foot[2,1] =  0.
		self.base_X_RF_foot[2,3] =  (q2_sin*q3_cos + q2_cos*q3_sin)*L3 + q2_sin*L2

	def update_base_X_RM_foot(self,q):
		if (len(q)>3):
			q1_sin = np.sin(q[RM_Q1_JOINT])
			q2_sin = np.sin(q[RM_Q2_JOINT])
			q3_sin = np.sin(q[RM_Q3_JOINT])
			q1_cos = np.cos(q[RM_Q1_JOINT])
			q2_cos = np.cos(q[RM_Q2_JOINT])
			q3_cos = np.cos(q[RM_Q3_JOINT])
		else:
			q1_sin = np.sin(q[0])
			q2_sin = np.sin(q[1])
			q3_sin = np.sin(q[2])
			q1_cos = np.cos(q[0])
			q2_cos = np.cos(q[1])
			q3_cos = np.cos(q[2])
		rZ_sin = np.sin(deg2rad(ROT_BASE_X_RM))
		rZ_cos = np.cos(deg2rad(ROT_BASE_X_RM))

		self.base_X_RM_foot[0,0] = -(q2_cos*q3_cos - q2_sin*q3_sin)*(rZ_sin*q1_sin - rZ_cos*q1_cos)
		self.base_X_RM_foot[0,1] =  (q2_sin*q3_cos + q2_cos*q3_sin)*(rZ_sin*q1_sin - rZ_cos*q1_cos)
		self.base_X_RM_foot[0,2] =  rZ_cos*q1_sin + rZ_sin*q1_cos
		self.base_X_RM_foot[0,3] = -rZ_sin*(q2_cos*q3_cos - q2_sin*q3_sin)*q1_sin*L3 - rZ_sin*q2_cos*q1_sin*L2 + (q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos*rZ_cos*L3 + q2_cos*q1_cos*rZ_cos*L2 - rZ_sin*q1_sin*L1 + q1_cos*rZ_cos*L1
		self.base_X_RM_foot[1,0] =  (q2_cos*q3_cos - q2_sin*q3_sin)*(rZ_cos*q1_sin + rZ_sin*q1_cos)
		self.base_X_RM_foot[1,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*(rZ_cos*q1_sin + rZ_sin*q1_cos)
		self.base_X_RM_foot[1,2] =  rZ_sin*q1_sin - rZ_cos*q1_cos
		self.base_X_RM_foot[1,3] =  rZ_sin*(q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos*L3 + rZ_sin*q2_cos*q1_cos*L2 + (q2_cos*q3_cos - q2_sin*q3_sin)*rZ_cos*q1_sin*L3 + q2_cos*rZ_cos*q1_sin*L2 + rZ_sin*q1_cos*L1 + rZ_cos*q1_sin*L1 - COXA_Y
		self.base_X_RM_foot[2,0] =  (q2_sin*q3_cos + q2_cos*q3_sin)
		self.base_X_RM_foot[2,1] =  (q2_cos*q3_cos - q2_sin*q3_sin)
		self.base_X_RM_foot[2,1] =  0.
		self.base_X_RM_foot[2,3] =  (q2_sin*q3_cos + q2_cos*q3_sin)*L3 + q2_sin*L2

	def update_base_X_RR_foot(self,q):
		if (len(q)>3):
			q1_sin = np.sin(q[RR_Q1_JOINT])
			q2_sin = np.sin(q[RR_Q2_JOINT])
			q3_sin = np.sin(q[RR_Q3_JOINT])
			q1_cos = np.cos(q[RR_Q1_JOINT])
			q2_cos = np.cos(q[RR_Q2_JOINT])
			q3_cos = np.cos(q[RR_Q3_JOINT])
		else:
			q1_sin = np.sin(q[0])
			q2_sin = np.sin(q[1])
			q3_sin = np.sin(q[2])
			q1_cos = np.cos(q[0])
			q2_cos = np.cos(q[1])
			q3_cos = np.cos(q[2])
		rZ_sin = np.sin(deg2rad(ROT_BASE_X_RR))
		rZ_cos = np.cos(deg2rad(ROT_BASE_X_RR))

		self.base_X_RR_foot[0,0] = -(q2_cos*q3_cos - q2_sin*q3_sin)*(rZ_sin*q1_sin - rZ_cos*q1_cos)
		self.base_X_RR_foot[0,1] =  (q2_sin*q3_cos + q2_cos*q3_sin)*(rZ_sin*q1_sin - rZ_cos*q1_cos)
		self.base_X_RR_foot[0,2] =  rZ_cos*q1_sin + rZ_sin*q1_cos
		self.base_X_RR_foot[0,3] = -rZ_sin*(q2_cos*q3_cos - q2_sin*q3_sin)*q1_sin*L3 - rZ_sin*q2_cos*q1_sin*L2 + (q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos*rZ_cos*L3 + q2_cos*q1_cos*rZ_cos*L2 - rZ_sin*q1_sin*L1 + q1_cos*rZ_cos*L1 - COXA_X
		self.base_X_RR_foot[1,0] =  (q2_cos*q3_cos - q2_sin*q3_sin)*(rZ_cos*q1_sin + rZ_sin*q1_cos)
		self.base_X_RR_foot[1,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*(rZ_cos*q1_sin + rZ_sin*q1_cos)
		self.base_X_RR_foot[1,2] =  rZ_sin*q1_sin - rZ_cos*q1_cos
		self.base_X_RR_foot[1,3] =  rZ_sin*(q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos*L3 + rZ_sin*q2_cos*q1_cos*L2 + (q2_cos*q3_cos - q2_sin*q3_sin)*rZ_cos*q1_sin*L3 + q2_cos*rZ_cos*q1_sin*L2 + rZ_sin*q1_cos*L1 + rZ_cos*q1_sin*L1 - COXA_Y
		self.base_X_RR_foot[2,0] =  (q2_sin*q3_cos + q2_cos*q3_sin)
		self.base_X_RR_foot[2,1] =  (q2_cos*q3_cos - q2_sin*q3_sin)
		self.base_X_RR_foot[2,1] =  0.
		self.base_X_RR_foot[2,3] =  (q2_sin*q3_cos + q2_cos*q3_sin)*L3 + q2_sin*L2

	def update_base_X_LF_edge(self):
		self.base_X_LF_edge_1[0,3] = self.base_X_LF_foot[0,3] + (-LEG_AREA_LX/2)
		self.base_X_LF_edge_1[1,3] = self.base_X_LF_foot[1,3] + (-LEG_AREA_LY/2)
		self.base_X_LF_edge_2[0,3] = self.base_X_LF_foot[0,3] + ( LEG_AREA_LX/2)
		self.base_X_LF_edge_2[1,3] = self.base_X_LF_foot[1,3] + (-LEG_AREA_LY/2)
		self.base_X_LF_edge_3[0,3] = self.base_X_LF_foot[0,3] + ( LEG_AREA_LX/2)
		self.base_X_LF_edge_3[1,3] = self.base_X_LF_foot[1,3] + ( LEG_AREA_LY/2)
		self.base_X_LF_edge_4[0,3] = self.base_X_LF_foot[0,3] + (-LEG_AREA_LX/2)
		self.base_X_LF_edge_4[1,3] = self.base_X_LF_foot[1,3] + ( LEG_AREA_LY/2)
		
	def update_base_X_LM_edge(self):
		self.base_X_LM_edge_1[0,3] = self.base_X_LM_foot[0,3] + (-LEG_AREA_LX/2)
		self.base_X_LM_edge_1[1,3] = self.base_X_LM_foot[1,3] + (-LEG_AREA_LY/2)
		self.base_X_LM_edge_2[0,3] = self.base_X_LM_foot[0,3] + ( LEG_AREA_LX/2)
		self.base_X_LM_edge_2[1,3] = self.base_X_LM_foot[1,3] + (-LEG_AREA_LY/2)
		self.base_X_LM_edge_3[0,3] = self.base_X_LM_foot[0,3] + ( LEG_AREA_LX/2)
		self.base_X_LM_edge_3[1,3] = self.base_X_LM_foot[1,3] + ( LEG_AREA_LY/2)
		self.base_X_LM_edge_4[0,3] = self.base_X_LM_foot[0,3] + (-LEG_AREA_LX/2)
		self.base_X_LM_edge_4[1,3] = self.base_X_LM_foot[1,3] + ( LEG_AREA_LY/2)

	def update_base_X_LR_edge(self):
		self.base_X_LR_edge_1[0,3] = self.base_X_LR_foot[0,3] + (-LEG_AREA_LX/2)
		self.base_X_LR_edge_1[1,3] = self.base_X_LR_foot[1,3] + (-LEG_AREA_LY/2)
		self.base_X_LR_edge_2[0,3] = self.base_X_LR_foot[0,3] + ( LEG_AREA_LX/2)
		self.base_X_LR_edge_2[1,3] = self.base_X_LR_foot[1,3] + (-LEG_AREA_LY/2)
		self.base_X_LR_edge_3[0,3] = self.base_X_LR_foot[0,3] + ( LEG_AREA_LX/2)
		self.base_X_LR_edge_3[1,3] = self.base_X_LR_foot[1,3] + ( LEG_AREA_LY/2)
		self.base_X_LR_edge_4[0,3] = self.base_X_LR_foot[0,3] + (-LEG_AREA_LX/2)
		self.base_X_LR_edge_4[1,3] = self.base_X_LR_foot[1,3] + ( LEG_AREA_LY/2)

	def update_base_X_RF_edge(self):
		self.base_X_RF_edge_1[0,3] = self.base_X_RF_foot[0,3] + (-LEG_AREA_LX/2)
		self.base_X_RF_edge_1[1,3] = self.base_X_RF_foot[1,3] + ( LEG_AREA_LY/2)
		self.base_X_RF_edge_2[0,3] = self.base_X_RF_foot[0,3] + ( LEG_AREA_LX/2)
		self.base_X_RF_edge_2[1,3] = self.base_X_RF_foot[1,3] + ( LEG_AREA_LY/2)
		self.base_X_RF_edge_3[0,3] = self.base_X_RF_foot[0,3] + ( LEG_AREA_LX/2)
		self.base_X_RF_edge_3[1,3] = self.base_X_RF_foot[1,3] + (-LEG_AREA_LY/2)
		self.base_X_RF_edge_4[0,3] = self.base_X_RF_foot[0,3] + (-LEG_AREA_LX/2)
		self.base_X_RF_edge_4[1,3] = self.base_X_RF_foot[1,3] + (-LEG_AREA_LY/2)

	def update_base_X_RM_edge(self):
		self.base_X_RM_edge_1[0,3] = self.base_X_RM_foot[0,3] + (-LEG_AREA_LX/2)
		self.base_X_RM_edge_1[1,3] = self.base_X_RM_foot[1,3] + ( LEG_AREA_LY/2)
		self.base_X_RM_edge_2[0,3] = self.base_X_RM_foot[0,3] + ( LEG_AREA_LX/2)
		self.base_X_RM_edge_2[1,3] = self.base_X_RM_foot[1,3] + ( LEG_AREA_LY/2)
		self.base_X_RM_edge_3[0,3] = self.base_X_RM_foot[0,3] + ( LEG_AREA_LX/2)
		self.base_X_RM_edge_3[1,3] = self.base_X_RM_foot[1,3] + (-LEG_AREA_LY/2)
		self.base_X_RM_edge_4[0,3] = self.base_X_RM_foot[0,3] + (-LEG_AREA_LX/2)
		self.base_X_RM_edge_4[1,3] = self.base_X_RM_foot[1,3] + (-LEG_AREA_LY/2)

	def update_base_X_RR_edge(self):
		self.base_X_RR_edge_1[0,3] = self.base_X_RR_foot[0,3] + (-LEG_AREA_LX/2)
		self.base_X_RR_edge_1[1,3] = self.base_X_RR_foot[1,3] + ( LEG_AREA_LY/2)
		self.base_X_RR_edge_2[0,3] = self.base_X_RR_foot[0,3] + ( LEG_AREA_LX/2)
		self.base_X_RR_edge_2[1,3] = self.base_X_RR_foot[1,3] + ( LEG_AREA_LY/2)
		self.base_X_RR_edge_3[0,3] = self.base_X_RR_foot[0,3] + ( LEG_AREA_LX/2)
		self.base_X_RR_edge_3[1,3] = self.base_X_RR_foot[1,3] + (-LEG_AREA_LY/2)
		self.base_X_RR_edge_4[0,3] = self.base_X_RR_foot[0,3] + (-LEG_AREA_LX/2)
		self.base_X_RR_edge_4[1,3] = self.base_X_RR_foot[1,3] + (-LEG_AREA_LY/2)

## ================================================================================================ ##
## 												TESTING 											##
## ================================================================================================ ##


XH = HomogeneousTransform()
q  = np.zeros(18)
qb = np.zeros(6)

# set legs to default standup position
for i in range(0,6): 	
	q[0+i*3] = 0.
	q[1+i*3] = 0.3381
	q[2+i*3] = -1.852
qb[5] = BODY_HEIGHT

XH.update_robot(qb,q) 	# set initial stance of robot
# q = np.array([0,0.5,1.1])
# print len(q)
XH.update_base_X_LF_foot(q)
# print XH.base_X_LF_foot
# print XH.base_X_LF_foot[:3,:3]