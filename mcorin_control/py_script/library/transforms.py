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

def deg2rad(q):
	return (q*np.pi/180.)

class HomogeneousTransform:
	def __init__(self):
		self.fr_world_X_base = np.identity(4)

		self.fr_world_X_LF_foot = np.identity(4)
		self.fr_world_X_LM_foot = np.identity(4)
		self.fr_world_X_LR_foot = np.identity(4)
		self.fr_world_X_RF_foot = np.identity(4)
		self.fr_world_X_RM_foot = np.identity(4)
		self.fr_world_X_RR_foot = np.identity(4)
		
		## base to hip frame of each leg; one-off
		self.fr_base_X_LF_coxa  = np.identity(4)
		self.fr_base_X_LM_coxa  = np.identity(4)
		self.fr_base_X_LR_coxa  = np.identity(4)
		self.fr_base_X_RF_coxa  = np.identity(4)
		self.fr_base_X_RM_coxa  = np.identity(4)
		self.fr_base_X_RR_coxa  = np.identity(4)

		self.fr_base_X_LF_foot = np.identity(4)
		self.fr_base_X_LM_foot = np.identity(4)
		self.fr_base_X_LR_foot = np.identity(4)
		self.fr_base_X_RF_foot = np.identity(4)
		self.fr_base_X_RM_foot = np.identity(4)
		self.fr_base_X_RR_foot = np.identity(4)

		self.fr_LF_coxa_X_LF_foot = np.identity(4)
		self.fr_LM_coxa_X_LM_foot = np.identity(4)
		self.fr_LR_coxa_X_LR_foot = np.identity(4)
		self.fr_RF_coxa_X_RF_foot = np.identity(4)
		self.fr_RM_coxa_X_RM_foot = np.identity(4)
		self.fr_RR_coxa_X_RR_foot = np.identity(4)

		## CoM frames
		self.fr_base_X_COM = np.identity(4)
		self.fr_LF_COM = np.identity(4)
		self.fr_LM_COM = np.identity(4)
		self.fr_LR_COM = np.identity(4)
		self.fr_RF_COM = np.identity(4)
		self.fr_RM_COM = np.identity(4)
		self.fr_RR_COM = np.identity(4)

		self.fr_LF_coxa_X_coxa_COM	= np.identity(4)
		self.fr_LF_coxa_X_femur_COM	= np.identity(4)
		self.fr_LF_coxa_X_tibia_COM	= np.identity(4)

		## Edges of foothold area - inner bottom to outer bottom; one-off
		self.fr_base_X_LF_edge_1 = np.identity(4)
		self.fr_base_X_LF_edge_2 = np.identity(4)
		self.fr_base_X_LF_edge_3 = np.identity(4)
		self.fr_base_X_LF_edge_4 = np.identity(4)
		self.fr_base_X_LM_edge_1 = np.identity(4)
		self.fr_base_X_LM_edge_2 = np.identity(4)
		self.fr_base_X_LM_edge_3 = np.identity(4)
		self.fr_base_X_LM_edge_4 = np.identity(4)
		self.fr_base_X_LR_edge_1 = np.identity(4)
		self.fr_base_X_LR_edge_2 = np.identity(4)
		self.fr_base_X_LR_edge_3 = np.identity(4)
		self.fr_base_X_LR_edge_4 = np.identity(4)
		self.fr_base_X_RF_edge_1 = np.identity(4)
		self.fr_base_X_RF_edge_2 = np.identity(4)
		self.fr_base_X_RF_edge_3 = np.identity(4)
		self.fr_base_X_RF_edge_4 = np.identity(4)
		self.fr_base_X_RM_edge_1 = np.identity(4)
		self.fr_base_X_RM_edge_2 = np.identity(4)
		self.fr_base_X_RM_edge_3 = np.identity(4)
		self.fr_base_X_RM_edge_4 = np.identity(4)
		self.fr_base_X_RR_edge_1 = np.identity(4)
		self.fr_base_X_RR_edge_2 = np.identity(4)
		self.fr_base_X_RR_edge_3 = np.identity(4)
		self.fr_base_X_RR_edge_4 = np.identity(4)
		
		self._initialize()

	def _initialize(self):
		## Set default values for robot's nominal stance
		q  = np.zeros(18)
		qb = np.zeros(6)

		KDL = kdl.corin_kinematics()
		xp  = KDL.IK([STANCE_WIDTH,0.,-BODY_HEIGHT])
		# set legs to default standup position
		for i in range(0,6): 	
			q[0+i*3] = xp[0]
			q[1+i*3] = xp[1]
			q[2+i*3] = xp[2]
		qb[5] = BODY_HEIGHT

		self.init_update() 		# single update
		self.update_robot(qb,q) # runtime update

	def init_update(self):
		self.update_fr_base_X_LF_coxa(0)
		self.update_fr_base_X_LM_coxa(0)
		self.update_fr_base_X_LR_coxa(0)
		self.update_fr_base_X_RF_coxa(0)
		self.update_fr_base_X_RM_coxa(0)
		self.update_fr_base_X_RR_coxa(0)

		## Edges of leg bounding box - used in motion planning
		# self.update_fr_base_X_LF_edge()
		# self.update_fr_base_X_LM_edge()
		# self.update_fr_base_X_LR_edge()
		# self.update_fr_base_X_RF_edge()
		# self.update_fr_base_X_RM_edge()
		# self.update_fr_base_X_RR_edge()

		# self.fr_coxa_X_coxaCOM()
		# self.fr_femur_X_femurCOM()
		# self.fr_tibia_X_tibiaCOM()

	def update_robot(self,qb,q):
		self.update_base(qb)
		self.update_legs(q)

	def update_base(self,qb):
		self.update_fr_world_X_base(qb)

	def update_legs(self,q):
		## Forward Kinematics using leg joint angles
		self.update_fr_base_X_LF_foot(q)
		self.update_fr_base_X_LM_foot(q)
		self.update_fr_base_X_LR_foot(q)
		self.update_fr_base_X_RF_foot(q)
		self.update_fr_base_X_RM_foot(q)
		self.update_fr_base_X_RR_foot(q)

		## Forward Kinematics using leg joint angles
		self.update_fr_LF_coxa_X_LF_foot(q)
		self.update_fr_LM_coxa_X_LM_foot(q)
		self.update_fr_LR_coxa_X_LR_foot(q)
		self.update_fr_RF_coxa_X_RF_foot(q)
		self.update_fr_RM_coxa_X_RM_foot(q)
		self.update_fr_RR_coxa_X_RR_foot(q)

		self.update_fr_LF_coxa_X_LF_COM(q)
		self.update_fr_base_X_COM()

	def update_fr_base_X_COM(self):
		# TODO: what if base is rotated?
		self.fr_base_X_COM[0:3] = BODY_COM[0] + LEG_MASS*(self.fr_LF_COM[0,3])
		self.fr_base_X_COM[1:3] = BODY_COM[1] + LEG_MASS*(self.fr_LF_COM[1,3])
		self.fr_base_X_COM[2:3] = BODY_COM[2] + LEG_MASS*(self.fr_LF_COM[2,3])

	def update_fr_LF_coxa_X_LF_COM(self,q):
		# TODO: check against solidworks
		q1_sin = np.sin(q[LF_Q1_JOINT])
		q2_sin = np.sin(q[LF_Q2_JOINT])
		q3_sin = np.sin(q[LF_Q3_JOINT])
		q1_cos = np.cos(q[LF_Q1_JOINT])
		q2_cos = np.cos(q[LF_Q2_JOINT])
		q3_cos = np.cos(q[LF_Q3_JOINT])

		self.fr_LF_coxa_X_coxa_COM[0,3] = L1_COM[0]*q1_cos - L1_COM[1]*q1_sin
		self.fr_LF_coxa_X_coxa_COM[1,3] = L1_COM[1]*q1_sin + L1_COM[0]*q1_cos
		self.fr_LF_coxa_X_coxa_COM[2,3] = L1_COM[2]

		self.fr_LF_coxa_X_femur_COM[0,3] = L1*q1_cos + L2_COM[0]*q1_cos*q2_cos - L2_COM[1]*q1_cos*q2_sin + L2_COM[2]*q1_sin
		self.fr_LF_coxa_X_femur_COM[1,3] = L1*q1_sin + L2_COM[0]*q1_sin*q2_cos - L2_COM[1]*q1_sin*q2_sin - L2_COM[2]*q1_cos
		self.fr_LF_coxa_X_femur_COM[2,3] = L2_COM[0]*q2_sin + L2_COM[1]*q2_cos

		self.fr_LF_coxa_X_tibia_COM[0,3] = (L1*q1_cos + L2*q1_cos*q2_cos + L3_COM[0]*q1_cos*q2_cos*q3_cos - L3_COM[0]*q1_cos*q2_sin*q3_sin +
												L3_COM[2]*q1_sin - L3_COM[1]*q1_cos*q2_cos*q3_sin - L3_COM[1]*q1_cos*q2_sin*q3_cos)
		self.fr_LF_coxa_X_tibia_COM[1,3] = (L1*q1_sin + L2*q1_sin*q2_cos + L3_COM[0]*q1_sin*q2_cos*q3_cos - L3_COM[0]*q1_sin*q2_sin*q3_sin
		 										- L3_COM[1]*q1_sin*q2_cos*q3_sin - L3_COM[1]*q1_sin*q2_sin*q3_cos - L3_COM[1]*q1_cos)
		self.fr_LF_coxa_X_tibia_COM[2,3] = (L2*q2_sin + L3_COM[0]*q2_sin*q3_cos + L3_COM[0]*q2_cos*q3_sin + L3_COM[1]*q2_cos*q3_cos - 
												L3_COM[1]*q2_sin*q3_sin)

		self.fr_LF_COM[0,3] = (L1_MASS*self.fr_LF_coxa_X_coxa_COM[0,3] + L2_MASS*self.fr_LF_coxa_X_femur_COM[0,3] + L3_MASS*self.fr_LF_coxa_X_tibia_COM[0,3])/LEG_MASS
		self.fr_LF_COM[1,3] = (L1_MASS*self.fr_LF_coxa_X_coxa_COM[1,3] + L2_MASS*self.fr_LF_coxa_X_femur_COM[1,3] + L3_MASS*self.fr_LF_coxa_X_tibia_COM[1,3])/LEG_MASS
		self.fr_LF_COM[2,3] = (L1_MASS*self.fr_LF_coxa_X_coxa_COM[2,3] + L2_MASS*self.fr_LF_coxa_X_femur_COM[2,3] + L3_MASS*self.fr_LF_coxa_X_tibia_COM[2,3])/LEG_MASS

	def update_fr_world_X_base(self,q):
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

		self.fr_world_X_base[0,0] =  qz_cos*qy_cos
		self.fr_world_X_base[0,1] = -qz_sin*qx_cos + qz_cos*qy_sin*qx_sin
		self.fr_world_X_base[0,2] =  qz_sin*qx_sin + qz_cos*qy_sin*qx_cos
		self.fr_world_X_base[0,3] =  (qz_sin*qx_sin + qz_cos*qy_sin*qx_cos)*tz + (-qz_sin*qx_cos + qz_cos*qy_sin*qx_sin)*ty + qz_cos*qy_cos*tx
		self.fr_world_X_base[1,0] =  qz_sin*qy_cos
		self.fr_world_X_base[1,1] =  qz_cos*qx_cos + qz_sin*qy_sin*qx_sin
		self.fr_world_X_base[1,2] = -qz_cos*qx_sin + qz_sin*qy_sin*qx_cos
		self.fr_world_X_base[1,3] =  (-qz_cos*qx_sin + qz_sin*qy_sin*qx_cos)*tz + (qz_cos*qx_cos + qz_sin*qy_sin*qx_sin)*ty + qz_sin*qy_cos*tx
		self.fr_world_X_base[2,0] = -qy_sin
		self.fr_world_X_base[2,1] =  qy_cos*qx_sin
		self.fr_world_X_base[2,2] =	 qy_cos*qx_cos
		self.fr_world_X_base[2,3] =  qy_cos*qx_cos*tz + qy_cos*qx_sin*ty - qy_sin*tx
		
	def update_fr_base_X_LF_coxa(self,q=None):
		self.fr_base_X_LF_coxa[0,0] =  np.cos(deg2rad(ROT_BASE_X_LF))
		self.fr_base_X_LF_coxa[0,1] = -np.sin(deg2rad(ROT_BASE_X_LF))
		self.fr_base_X_LF_coxa[1,0] =  np.sin(deg2rad(ROT_BASE_X_LF))
		self.fr_base_X_LF_coxa[1,1] =  np.cos(deg2rad(ROT_BASE_X_LF))
		self.fr_base_X_LF_coxa[0,3] =  COXA_X
		self.fr_base_X_LF_coxa[1,3] =  COXA_Y

	def update_fr_base_X_LM_coxa(self,q=None):
		self.fr_base_X_LM_coxa[0,0] =  np.cos(deg2rad(ROT_BASE_X_LM))
		self.fr_base_X_LM_coxa[0,1] = -np.sin(deg2rad(ROT_BASE_X_LM))
		self.fr_base_X_LM_coxa[1,0] =  np.sin(deg2rad(ROT_BASE_X_LM))
		self.fr_base_X_LM_coxa[1,1] =  np.cos(deg2rad(ROT_BASE_X_LM))
		self.fr_base_X_LM_coxa[1,3] =  COXA_Y

	def update_fr_base_X_LR_coxa(self,q=None):
		self.fr_base_X_LR_coxa[0,0] =  np.cos(deg2rad(ROT_BASE_X_LR))
		self.fr_base_X_LR_coxa[0,1] = -np.sin(deg2rad(ROT_BASE_X_LR))
		self.fr_base_X_LR_coxa[1,0] =  np.sin(deg2rad(ROT_BASE_X_LR))
		self.fr_base_X_LR_coxa[1,1] =  np.cos(deg2rad(ROT_BASE_X_LR))
		self.fr_base_X_LR_coxa[0,3] = -COXA_X
		self.fr_base_X_LR_coxa[1,3] =  COXA_Y

	def update_fr_base_X_RF_coxa(self,q=None):
		self.fr_base_X_RF_coxa[0,0] =  np.cos(deg2rad(ROT_BASE_X_RF))
		self.fr_base_X_RF_coxa[0,1] = -np.sin(deg2rad(ROT_BASE_X_RF))
		self.fr_base_X_RF_coxa[1,0] =  np.sin(deg2rad(ROT_BASE_X_RF))
		self.fr_base_X_RF_coxa[1,1] =  np.cos(deg2rad(ROT_BASE_X_RF))
		self.fr_base_X_RF_coxa[0,3] =  COXA_X
		self.fr_base_X_RF_coxa[1,3] = -COXA_Y

	def update_fr_base_X_RM_coxa(self,q=None):
		self.fr_base_X_RM_coxa[0,0] =  np.cos(deg2rad(ROT_BASE_X_RM))
		self.fr_base_X_RM_coxa[0,1] = -np.sin(deg2rad(ROT_BASE_X_RM))
		self.fr_base_X_RM_coxa[1,0] =  np.sin(deg2rad(ROT_BASE_X_RM))
		self.fr_base_X_RM_coxa[1,1] =  np.cos(deg2rad(ROT_BASE_X_RM))
		self.fr_base_X_RM_coxa[1,3] = -COXA_Y

	def update_fr_base_X_RR_coxa(self,q=None):
		self.fr_base_X_RR_coxa[0,0] =  np.cos(deg2rad(ROT_BASE_X_RR))
		self.fr_base_X_RR_coxa[0,1] = -np.sin(deg2rad(ROT_BASE_X_RR))
		self.fr_base_X_RR_coxa[1,0] =  np.sin(deg2rad(ROT_BASE_X_RR))
		self.fr_base_X_RR_coxa[1,1] =  np.cos(deg2rad(ROT_BASE_X_RR))
		self.fr_base_X_RR_coxa[0,3] = -COXA_X
		self.fr_base_X_RR_coxa[1,3] = -COXA_Y

	def update_fr_LF_coxa_X_LF_foot(self,q):
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

		self.fr_LF_coxa_X_LF_foot[0,0] = (q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos
		self.fr_LF_coxa_X_LF_foot[0,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*q1_cos
		self.fr_LF_coxa_X_LF_foot[0,2] = q1_sin
		self.fr_LF_coxa_X_LF_foot[0,3] = q1_cos*(L1 + L3*(q2_cos*q3_cos - q2_sin*q3_sin) + L2*q2_cos)
		self.fr_LF_coxa_X_LF_foot[1,0] = (q2_cos*q3_cos - q2_sin*q3_sin)*q1_sin
		self.fr_LF_coxa_X_LF_foot[1,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*q1_sin
		self.fr_LF_coxa_X_LF_foot[1,2] = -q1_cos
		self.fr_LF_coxa_X_LF_foot[1,3] = q1_sin*(L1 + L3*(q2_cos*q3_cos - q2_sin*q3_sin) + L2*q2_cos)
		self.fr_LF_coxa_X_LF_foot[2,0] = (q2_sin*q3_cos + q2_cos*q3_sin)
		self.fr_LF_coxa_X_LF_foot[2,1] = (q2_cos*q3_cos - q2_sin*q3_sin)
		self.fr_LF_coxa_X_LF_foot[2,3] = L3*(q2_sin*q3_cos + q2_cos*q3_sin) + L2*q2_sin

	def update_fr_LM_coxa_X_LM_foot(self,q):
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

		self.fr_LM_coxa_X_LM_foot[0,0] = (q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos
		self.fr_LM_coxa_X_LM_foot[0,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*q1_cos
		self.fr_LM_coxa_X_LM_foot[0,2] = q1_sin
		self.fr_LM_coxa_X_LM_foot[0,3] = q1_cos*(L1 + L3*(q2_cos*q3_cos - q2_sin*q3_sin) + L2*q2_cos)
		self.fr_LM_coxa_X_LM_foot[1,0] = (q2_cos*q3_cos - q2_sin*q3_sin)*q1_sin
		self.fr_LM_coxa_X_LM_foot[1,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*q1_sin
		self.fr_LM_coxa_X_LM_foot[1,2] = -q1_cos
		self.fr_LM_coxa_X_LM_foot[1,3] = q1_sin*(L1 + L3*(q2_cos*q3_cos - q2_sin*q3_sin) + L2*q2_cos)
		self.fr_LM_coxa_X_LM_foot[2,0] = (q2_sin*q3_cos + q2_cos*q3_sin)
		self.fr_LM_coxa_X_LM_foot[2,1] = (q2_cos*q3_cos - q2_sin*q3_sin)
		self.fr_LM_coxa_X_LM_foot[2,3] = L3*(q2_sin*q3_cos + q2_cos*q3_sin) + L2*q2_sin

	def update_fr_LR_coxa_X_LR_foot(self,q):
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

		self.fr_LR_coxa_X_LR_foot[0,0] = (q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos
		self.fr_LR_coxa_X_LR_foot[0,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*q1_cos
		self.fr_LR_coxa_X_LR_foot[0,2] = q1_sin
		self.fr_LR_coxa_X_LR_foot[0,3] = q1_cos*(L1 + L3*(q2_cos*q3_cos - q2_sin*q3_sin) + L2*q2_cos)
		self.fr_LR_coxa_X_LR_foot[1,0] = (q2_cos*q3_cos - q2_sin*q3_sin)*q1_sin
		self.fr_LR_coxa_X_LR_foot[1,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*q1_sin
		self.fr_LR_coxa_X_LR_foot[1,2] = -q1_cos
		self.fr_LR_coxa_X_LR_foot[1,3] = q1_sin*(L1 + L3*(q2_cos*q3_cos - q2_sin*q3_sin) + L2*q2_cos)
		self.fr_LR_coxa_X_LR_foot[2,0] = (q2_sin*q3_cos + q2_cos*q3_sin)
		self.fr_LR_coxa_X_LR_foot[2,1] = (q2_cos*q3_cos - q2_sin*q3_sin)
		self.fr_LR_coxa_X_LR_foot[2,3] = L3*(q2_sin*q3_cos + q2_cos*q3_sin) + L2*q2_sin

	def update_fr_RF_coxa_X_RF_foot(self,q):
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

		self.fr_RF_coxa_X_RF_foot[0,0] = (q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos
		self.fr_RF_coxa_X_RF_foot[0,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*q1_cos
		self.fr_RF_coxa_X_RF_foot[0,2] = q1_sin
		self.fr_RF_coxa_X_RF_foot[0,3] = q1_cos*(L1 + L3*(q2_cos*q3_cos - q2_sin*q3_sin) + L2*q2_cos)
		self.fr_RF_coxa_X_RF_foot[1,0] = (q2_cos*q3_cos - q2_sin*q3_sin)*q1_sin
		self.fr_RF_coxa_X_RF_foot[1,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*q1_sin
		self.fr_RF_coxa_X_RF_foot[1,2] = -q1_cos
		self.fr_RF_coxa_X_RF_foot[1,3] = q1_sin*(L1 + L3*(q2_cos*q3_cos - q2_sin*q3_sin) + L2*q2_cos)
		self.fr_RF_coxa_X_RF_foot[2,0] = (q2_sin*q3_cos + q2_cos*q3_sin)
		self.fr_RF_coxa_X_RF_foot[2,1] = (q2_cos*q3_cos - q2_sin*q3_sin)
		self.fr_RF_coxa_X_RF_foot[2,3] = L3*(q2_sin*q3_cos + q2_cos*q3_sin) + L2*q2_sin

	def update_fr_RM_coxa_X_RM_foot(self,q):
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

		self.fr_RM_coxa_X_RM_foot[0,0] = (q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos
		self.fr_RM_coxa_X_RM_foot[0,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*q1_cos
		self.fr_RM_coxa_X_RM_foot[0,2] = q1_sin
		self.fr_RM_coxa_X_RM_foot[0,3] = q1_cos*(L1 + L3*(q2_cos*q3_cos - q2_sin*q3_sin) + L2*q2_cos)
		self.fr_RM_coxa_X_RM_foot[1,0] = (q2_cos*q3_cos - q2_sin*q3_sin)*q1_sin
		self.fr_RM_coxa_X_RM_foot[1,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*q1_sin
		self.fr_RM_coxa_X_RM_foot[1,2] = -q1_cos
		self.fr_RM_coxa_X_RM_foot[1,3] = q1_sin*(L1 + L3*(q2_cos*q3_cos - q2_sin*q3_sin) + L2*q2_cos)
		self.fr_RM_coxa_X_RM_foot[2,0] = (q2_sin*q3_cos + q2_cos*q3_sin)
		self.fr_RM_coxa_X_RM_foot[2,1] = (q2_cos*q3_cos - q2_sin*q3_sin)
		self.fr_RM_coxa_X_RM_foot[2,3] = L3*(q2_sin*q3_cos + q2_cos*q3_sin) + L2*q2_sin

	def update_fr_RR_coxa_X_RR_foot(self,q):
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

		self.fr_RR_coxa_X_RR_foot[0,0] = (q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos
		self.fr_RR_coxa_X_RR_foot[0,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*q1_cos
		self.fr_RR_coxa_X_RR_foot[0,2] = q1_sin
		self.fr_RR_coxa_X_RR_foot[0,3] = q1_cos*(L1 + L3*(q2_cos*q3_cos - q2_sin*q3_sin) + L2*q2_cos)
		self.fr_RR_coxa_X_RR_foot[1,0] = (q2_cos*q3_cos - q2_sin*q3_sin)*q1_sin
		self.fr_RR_coxa_X_RR_foot[1,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*q1_sin
		self.fr_RR_coxa_X_RR_foot[1,2] = -q1_cos
		self.fr_RR_coxa_X_RR_foot[1,3] = q1_sin*(L1 + L3*(q2_cos*q3_cos - q2_sin*q3_sin) + L2*q2_cos)
		self.fr_RR_coxa_X_RR_foot[2,0] = (q2_sin*q3_cos + q2_cos*q3_sin)
		self.fr_RR_coxa_X_RR_foot[2,1] = (q2_cos*q3_cos - q2_sin*q3_sin)
		self.fr_RR_coxa_X_RR_foot[2,3] = L3*(q2_sin*q3_cos + q2_cos*q3_sin) + L2*q2_sin


	def update_fr_base_X_LF_foot(self,q):
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

		self.fr_base_X_LF_foot[0,0] = -(q2_cos*q3_cos - q2_sin*q3_sin)*(rZ_sin*q1_sin - rZ_cos*q1_cos)
		self.fr_base_X_LF_foot[0,1] =  (q2_sin*q3_cos + q2_cos*q3_sin)*(rZ_sin*q1_sin - rZ_cos*q1_cos)
		self.fr_base_X_LF_foot[0,2] =  rZ_cos*q1_sin + rZ_sin*q1_cos
		self.fr_base_X_LF_foot[0,3] = -rZ_sin*(q2_cos*q3_cos - q2_sin*q3_sin)*q1_sin*L3 - rZ_sin*q2_cos*q1_sin*L2 + (q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos*rZ_cos*L3 + q2_cos*q1_cos*rZ_cos*L2 - rZ_sin*q1_sin*L1 + q1_cos*rZ_cos*L1 + COXA_X
		self.fr_base_X_LF_foot[1,0] =  (q2_cos*q3_cos - q2_sin*q3_sin)*(rZ_cos*q1_sin + rZ_sin*q1_cos)
		self.fr_base_X_LF_foot[1,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*(rZ_cos*q1_sin + rZ_sin*q1_cos)
		self.fr_base_X_LF_foot[1,2] =  rZ_sin*q1_sin - rZ_cos*q1_cos
		self.fr_base_X_LF_foot[1,3] =  rZ_sin*(q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos*L3 + rZ_sin*q2_cos*q1_cos*L2 + (q2_cos*q3_cos - q2_sin*q3_sin)*rZ_cos*q1_sin*L3 + q2_cos*rZ_cos*q1_sin*L2 + rZ_sin*q1_cos*L1 + rZ_cos*q1_sin*L1 + COXA_Y
		self.fr_base_X_LF_foot[2,0] =  (q2_sin*q3_cos + q2_cos*q3_sin)
		self.fr_base_X_LF_foot[2,1] =  (q2_cos*q3_cos - q2_sin*q3_sin)
		self.fr_base_X_LF_foot[2,1] =  0.
		self.fr_base_X_LF_foot[2,3] =  (q2_sin*q3_cos + q2_cos*q3_sin)*L3 + q2_sin*L2

	def update_fr_base_X_LM_foot(self,q):
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

		self.fr_base_X_LM_foot[0,0] = -(q2_cos*q3_cos - q2_sin*q3_sin)*(rZ_sin*q1_sin - rZ_cos*q1_cos)
		self.fr_base_X_LM_foot[0,1] =  (q2_sin*q3_cos + q2_cos*q3_sin)*(rZ_sin*q1_sin - rZ_cos*q1_cos)
		self.fr_base_X_LM_foot[0,2] =  rZ_cos*q1_sin + rZ_sin*q1_cos
		self.fr_base_X_LM_foot[0,3] = -rZ_sin*(q2_cos*q3_cos - q2_sin*q3_sin)*q1_sin*L3 - rZ_sin*q2_cos*q1_sin*L2 + (q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos*rZ_cos*L3 + q2_cos*q1_cos*rZ_cos*L2 - rZ_sin*q1_sin*L1 + q1_cos*rZ_cos*L1
		self.fr_base_X_LM_foot[1,0] =  (q2_cos*q3_cos - q2_sin*q3_sin)*(rZ_cos*q1_sin + rZ_sin*q1_cos)
		self.fr_base_X_LM_foot[1,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*(rZ_cos*q1_sin + rZ_sin*q1_cos)
		self.fr_base_X_LM_foot[1,2] =  rZ_sin*q1_sin - rZ_cos*q1_cos
		self.fr_base_X_LM_foot[1,3] =  rZ_sin*(q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos*L3 + rZ_sin*q2_cos*q1_cos*L2 + (q2_cos*q3_cos - q2_sin*q3_sin)*rZ_cos*q1_sin*L3 + q2_cos*rZ_cos*q1_sin*L2 + rZ_sin*q1_cos*L1 + rZ_cos*q1_sin*L1 + COXA_Y
		self.fr_base_X_LM_foot[2,0] =  (q2_sin*q3_cos + q2_cos*q3_sin)
		self.fr_base_X_LM_foot[2,1] =  (q2_cos*q3_cos - q2_sin*q3_sin)
		self.fr_base_X_LM_foot[2,1] =  0.
		self.fr_base_X_LM_foot[2,3] =  (q2_sin*q3_cos + q2_cos*q3_sin)*L3 + q2_sin*L2

	def update_fr_base_X_LR_foot(self,q):
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

		self.fr_base_X_LR_foot[0,0] = -(q2_cos*q3_cos - q2_sin*q3_sin)*(rZ_sin*q1_sin - rZ_cos*q1_cos)
		self.fr_base_X_LR_foot[0,1] =  (q2_sin*q3_cos + q2_cos*q3_sin)*(rZ_sin*q1_sin - rZ_cos*q1_cos)
		self.fr_base_X_LR_foot[0,2] =  rZ_cos*q1_sin + rZ_sin*q1_cos
		self.fr_base_X_LR_foot[0,3] = -rZ_sin*(q2_cos*q3_cos - q2_sin*q3_sin)*q1_sin*L3 - rZ_sin*q2_cos*q1_sin*L2 + (q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos*rZ_cos*L3 + q2_cos*q1_cos*rZ_cos*L2 - rZ_sin*q1_sin*L1 + q1_cos*rZ_cos*L1 - COXA_X
		self.fr_base_X_LR_foot[1,0] =  (q2_cos*q3_cos - q2_sin*q3_sin)*(rZ_cos*q1_sin + rZ_sin*q1_cos)
		self.fr_base_X_LR_foot[1,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*(rZ_cos*q1_sin + rZ_sin*q1_cos)
		self.fr_base_X_LR_foot[1,2] =  rZ_sin*q1_sin - rZ_cos*q1_cos
		self.fr_base_X_LR_foot[1,3] =  rZ_sin*(q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos*L3 + rZ_sin*q2_cos*q1_cos*L2 + (q2_cos*q3_cos - q2_sin*q3_sin)*rZ_cos*q1_sin*L3 + q2_cos*rZ_cos*q1_sin*L2 + rZ_sin*q1_cos*L1 + rZ_cos*q1_sin*L1 + COXA_Y
		self.fr_base_X_LR_foot[2,0] =  (q2_sin*q3_cos + q2_cos*q3_sin)
		self.fr_base_X_LR_foot[2,1] =  (q2_cos*q3_cos - q2_sin*q3_sin)
		self.fr_base_X_LR_foot[2,1] =  0.
		self.fr_base_X_LR_foot[2,3] =  (q2_sin*q3_cos + q2_cos*q3_sin)*L3 + q2_sin*L2

	def update_fr_base_X_RF_foot(self,q):
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

		self.fr_base_X_RF_foot[0,0] = -(q2_cos*q3_cos - q2_sin*q3_sin)*(rZ_sin*q1_sin - rZ_cos*q1_cos)
		self.fr_base_X_RF_foot[0,1] =  (q2_sin*q3_cos + q2_cos*q3_sin)*(rZ_sin*q1_sin - rZ_cos*q1_cos)
		self.fr_base_X_RF_foot[0,2] =  rZ_cos*q1_sin + rZ_sin*q1_cos
		self.fr_base_X_RF_foot[0,3] = -rZ_sin*(q2_cos*q3_cos - q2_sin*q3_sin)*q1_sin*L3 - rZ_sin*q2_cos*q1_sin*L2 + (q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos*rZ_cos*L3 + q2_cos*q1_cos*rZ_cos*L2 - rZ_sin*q1_sin*L1 + q1_cos*rZ_cos*L1 + COXA_X
		self.fr_base_X_RF_foot[1,0] =  (q2_cos*q3_cos - q2_sin*q3_sin)*(rZ_cos*q1_sin + rZ_sin*q1_cos)
		self.fr_base_X_RF_foot[1,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*(rZ_cos*q1_sin + rZ_sin*q1_cos)
		self.fr_base_X_RF_foot[1,2] =  rZ_sin*q1_sin - rZ_cos*q1_cos
		self.fr_base_X_RF_foot[1,3] =  rZ_sin*(q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos*L3 + rZ_sin*q2_cos*q1_cos*L2 + (q2_cos*q3_cos - q2_sin*q3_sin)*rZ_cos*q1_sin*L3 + q2_cos*rZ_cos*q1_sin*L2 + rZ_sin*q1_cos*L1 + rZ_cos*q1_sin*L1 - COXA_Y
		self.fr_base_X_RF_foot[2,0] =  (q2_sin*q3_cos + q2_cos*q3_sin)
		self.fr_base_X_RF_foot[2,1] =  (q2_cos*q3_cos - q2_sin*q3_sin)
		self.fr_base_X_RF_foot[2,1] =  0.
		self.fr_base_X_RF_foot[2,3] =  (q2_sin*q3_cos + q2_cos*q3_sin)*L3 + q2_sin*L2

	def update_fr_base_X_RM_foot(self,q):
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

		self.fr_base_X_RM_foot[0,0] = -(q2_cos*q3_cos - q2_sin*q3_sin)*(rZ_sin*q1_sin - rZ_cos*q1_cos)
		self.fr_base_X_RM_foot[0,1] =  (q2_sin*q3_cos + q2_cos*q3_sin)*(rZ_sin*q1_sin - rZ_cos*q1_cos)
		self.fr_base_X_RM_foot[0,2] =  rZ_cos*q1_sin + rZ_sin*q1_cos
		self.fr_base_X_RM_foot[0,3] = -rZ_sin*(q2_cos*q3_cos - q2_sin*q3_sin)*q1_sin*L3 - rZ_sin*q2_cos*q1_sin*L2 + (q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos*rZ_cos*L3 + q2_cos*q1_cos*rZ_cos*L2 - rZ_sin*q1_sin*L1 + q1_cos*rZ_cos*L1
		self.fr_base_X_RM_foot[1,0] =  (q2_cos*q3_cos - q2_sin*q3_sin)*(rZ_cos*q1_sin + rZ_sin*q1_cos)
		self.fr_base_X_RM_foot[1,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*(rZ_cos*q1_sin + rZ_sin*q1_cos)
		self.fr_base_X_RM_foot[1,2] =  rZ_sin*q1_sin - rZ_cos*q1_cos
		self.fr_base_X_RM_foot[1,3] =  rZ_sin*(q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos*L3 + rZ_sin*q2_cos*q1_cos*L2 + (q2_cos*q3_cos - q2_sin*q3_sin)*rZ_cos*q1_sin*L3 + q2_cos*rZ_cos*q1_sin*L2 + rZ_sin*q1_cos*L1 + rZ_cos*q1_sin*L1 - COXA_Y
		self.fr_base_X_RM_foot[2,0] =  (q2_sin*q3_cos + q2_cos*q3_sin)
		self.fr_base_X_RM_foot[2,1] =  (q2_cos*q3_cos - q2_sin*q3_sin)
		self.fr_base_X_RM_foot[2,1] =  0.
		self.fr_base_X_RM_foot[2,3] =  (q2_sin*q3_cos + q2_cos*q3_sin)*L3 + q2_sin*L2

	def update_fr_base_X_RR_foot(self,q):
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

		self.fr_base_X_RR_foot[0,0] = -(q2_cos*q3_cos - q2_sin*q3_sin)*(rZ_sin*q1_sin - rZ_cos*q1_cos)
		self.fr_base_X_RR_foot[0,1] =  (q2_sin*q3_cos + q2_cos*q3_sin)*(rZ_sin*q1_sin - rZ_cos*q1_cos)
		self.fr_base_X_RR_foot[0,2] =  rZ_cos*q1_sin + rZ_sin*q1_cos
		self.fr_base_X_RR_foot[0,3] = -rZ_sin*(q2_cos*q3_cos - q2_sin*q3_sin)*q1_sin*L3 - rZ_sin*q2_cos*q1_sin*L2 + (q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos*rZ_cos*L3 + q2_cos*q1_cos*rZ_cos*L2 - rZ_sin*q1_sin*L1 + q1_cos*rZ_cos*L1 - COXA_X
		self.fr_base_X_RR_foot[1,0] =  (q2_cos*q3_cos - q2_sin*q3_sin)*(rZ_cos*q1_sin + rZ_sin*q1_cos)
		self.fr_base_X_RR_foot[1,1] = -(q2_sin*q3_cos + q2_cos*q3_sin)*(rZ_cos*q1_sin + rZ_sin*q1_cos)
		self.fr_base_X_RR_foot[1,2] =  rZ_sin*q1_sin - rZ_cos*q1_cos
		self.fr_base_X_RR_foot[1,3] =  rZ_sin*(q2_cos*q3_cos - q2_sin*q3_sin)*q1_cos*L3 + rZ_sin*q2_cos*q1_cos*L2 + (q2_cos*q3_cos - q2_sin*q3_sin)*rZ_cos*q1_sin*L3 + q2_cos*rZ_cos*q1_sin*L2 + rZ_sin*q1_cos*L1 + rZ_cos*q1_sin*L1 - COXA_Y
		self.fr_base_X_RR_foot[2,0] =  (q2_sin*q3_cos + q2_cos*q3_sin)
		self.fr_base_X_RR_foot[2,1] =  (q2_cos*q3_cos - q2_sin*q3_sin)
		self.fr_base_X_RR_foot[2,1] =  0.
		self.fr_base_X_RR_foot[2,3] =  (q2_sin*q3_cos + q2_cos*q3_sin)*L3 + q2_sin*L2

	def update_fr_base_X_LF_edge(self):
		self.fr_base_X_LF_edge_1[0,3] = self.fr_base_X_LF_foot[0,3] + (-LEG_AREA_LX/2)
		self.fr_base_X_LF_edge_1[1,3] = self.fr_base_X_LF_foot[1,3] + (-LEG_AREA_LY/2)
		self.fr_base_X_LF_edge_2[0,3] = self.fr_base_X_LF_foot[0,3] + ( LEG_AREA_LX/2)
		self.fr_base_X_LF_edge_2[1,3] = self.fr_base_X_LF_foot[1,3] + (-LEG_AREA_LY/2)
		self.fr_base_X_LF_edge_3[0,3] = self.fr_base_X_LF_foot[0,3] + ( LEG_AREA_LX/2)
		self.fr_base_X_LF_edge_3[1,3] = self.fr_base_X_LF_foot[1,3] + ( LEG_AREA_LY/2)
		self.fr_base_X_LF_edge_4[0,3] = self.fr_base_X_LF_foot[0,3] + (-LEG_AREA_LX/2)
		self.fr_base_X_LF_edge_4[1,3] = self.fr_base_X_LF_foot[1,3] + ( LEG_AREA_LY/2)
		
	def update_fr_base_X_LM_edge(self):
		self.fr_base_X_LM_edge_1[0,3] = self.fr_base_X_LM_foot[0,3] + (-LEG_AREA_LX/2)
		self.fr_base_X_LM_edge_1[1,3] = self.fr_base_X_LM_foot[1,3] + (-LEG_AREA_LY/2)
		self.fr_base_X_LM_edge_2[0,3] = self.fr_base_X_LM_foot[0,3] + ( LEG_AREA_LX/2)
		self.fr_base_X_LM_edge_2[1,3] = self.fr_base_X_LM_foot[1,3] + (-LEG_AREA_LY/2)
		self.fr_base_X_LM_edge_3[0,3] = self.fr_base_X_LM_foot[0,3] + ( LEG_AREA_LX/2)
		self.fr_base_X_LM_edge_3[1,3] = self.fr_base_X_LM_foot[1,3] + ( LEG_AREA_LY/2)
		self.fr_base_X_LM_edge_4[0,3] = self.fr_base_X_LM_foot[0,3] + (-LEG_AREA_LX/2)
		self.fr_base_X_LM_edge_4[1,3] = self.fr_base_X_LM_foot[1,3] + ( LEG_AREA_LY/2)

	def update_fr_base_X_LR_edge(self):
		self.fr_base_X_LR_edge_1[0,3] = self.fr_base_X_LR_foot[0,3] + (-LEG_AREA_LX/2)
		self.fr_base_X_LR_edge_1[1,3] = self.fr_base_X_LR_foot[1,3] + (-LEG_AREA_LY/2)
		self.fr_base_X_LR_edge_2[0,3] = self.fr_base_X_LR_foot[0,3] + ( LEG_AREA_LX/2)
		self.fr_base_X_LR_edge_2[1,3] = self.fr_base_X_LR_foot[1,3] + (-LEG_AREA_LY/2)
		self.fr_base_X_LR_edge_3[0,3] = self.fr_base_X_LR_foot[0,3] + ( LEG_AREA_LX/2)
		self.fr_base_X_LR_edge_3[1,3] = self.fr_base_X_LR_foot[1,3] + ( LEG_AREA_LY/2)
		self.fr_base_X_LR_edge_4[0,3] = self.fr_base_X_LR_foot[0,3] + (-LEG_AREA_LX/2)
		self.fr_base_X_LR_edge_4[1,3] = self.fr_base_X_LR_foot[1,3] + ( LEG_AREA_LY/2)

	def update_fr_base_X_RF_edge(self):
		self.fr_base_X_RF_edge_1[0,3] = self.fr_base_X_RF_foot[0,3] + (-LEG_AREA_LX/2)
		self.fr_base_X_RF_edge_1[1,3] = self.fr_base_X_RF_foot[1,3] + ( LEG_AREA_LY/2)
		self.fr_base_X_RF_edge_2[0,3] = self.fr_base_X_RF_foot[0,3] + ( LEG_AREA_LX/2)
		self.fr_base_X_RF_edge_2[1,3] = self.fr_base_X_RF_foot[1,3] + ( LEG_AREA_LY/2)
		self.fr_base_X_RF_edge_3[0,3] = self.fr_base_X_RF_foot[0,3] + ( LEG_AREA_LX/2)
		self.fr_base_X_RF_edge_3[1,3] = self.fr_base_X_RF_foot[1,3] + (-LEG_AREA_LY/2)
		self.fr_base_X_RF_edge_4[0,3] = self.fr_base_X_RF_foot[0,3] + (-LEG_AREA_LX/2)
		self.fr_base_X_RF_edge_4[1,3] = self.fr_base_X_RF_foot[1,3] + (-LEG_AREA_LY/2)

	def update_fr_base_X_RM_edge(self):
		self.fr_base_X_RM_edge_1[0,3] = self.fr_base_X_RM_foot[0,3] + (-LEG_AREA_LX/2)
		self.fr_base_X_RM_edge_1[1,3] = self.fr_base_X_RM_foot[1,3] + ( LEG_AREA_LY/2)
		self.fr_base_X_RM_edge_2[0,3] = self.fr_base_X_RM_foot[0,3] + ( LEG_AREA_LX/2)
		self.fr_base_X_RM_edge_2[1,3] = self.fr_base_X_RM_foot[1,3] + ( LEG_AREA_LY/2)
		self.fr_base_X_RM_edge_3[0,3] = self.fr_base_X_RM_foot[0,3] + ( LEG_AREA_LX/2)
		self.fr_base_X_RM_edge_3[1,3] = self.fr_base_X_RM_foot[1,3] + (-LEG_AREA_LY/2)
		self.fr_base_X_RM_edge_4[0,3] = self.fr_base_X_RM_foot[0,3] + (-LEG_AREA_LX/2)
		self.fr_base_X_RM_edge_4[1,3] = self.fr_base_X_RM_foot[1,3] + (-LEG_AREA_LY/2)

	def update_fr_base_X_RR_edge(self):
		self.fr_base_X_RR_edge_1[0,3] = self.fr_base_X_RR_foot[0,3] + (-LEG_AREA_LX/2)
		self.fr_base_X_RR_edge_1[1,3] = self.fr_base_X_RR_foot[1,3] + ( LEG_AREA_LY/2)
		self.fr_base_X_RR_edge_2[0,3] = self.fr_base_X_RR_foot[0,3] + ( LEG_AREA_LX/2)
		self.fr_base_X_RR_edge_2[1,3] = self.fr_base_X_RR_foot[1,3] + ( LEG_AREA_LY/2)
		self.fr_base_X_RR_edge_3[0,3] = self.fr_base_X_RR_foot[0,3] + ( LEG_AREA_LX/2)
		self.fr_base_X_RR_edge_3[1,3] = self.fr_base_X_RR_foot[1,3] + (-LEG_AREA_LY/2)
		self.fr_base_X_RR_edge_4[0,3] = self.fr_base_X_RR_foot[0,3] + (-LEG_AREA_LX/2)
		self.fr_base_X_RR_edge_4[1,3] = self.fr_base_X_RR_foot[1,3] + (-LEG_AREA_LY/2)

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
XH.update_fr_base_X_LF_foot(q)
# print XH.fr_base_X_LF_edge_1