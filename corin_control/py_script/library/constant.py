#!/usr/bin/env python

""" Constants for Corin robot
"""

import sys; sys.dont_write_bytecode = True

import numpy as np
import stance_selection

# Main name
ROBOT_NS = "corin"

## TEMPORARY VARIABLES ##
WALL_TRANSITION = False
CHIM_TRANSITION = False

## ================================================================ ##
##                       Robot parameters 	 						##
## ================================================================ ##
# Link Length, Mass
BODY_MASS = 1.48
L1 = 0.060;	L1_MASS = 0.040
L2 = 0.150;	L2_MASS = 0.350
L3 = 0.150;	L3_MASS = 0.116

LEG_MASS = L1_MASS + L2_MASS + L3_MASS

# Link Centre of Mass reference from parent frame
BODY_COM = [0.0, 0.0, 0.0]
L1_COM   = [0.0247, 0.0, 0.0]
L2_COM   = [0.0750, 0.0, 0.0]
L3_COM   = [0.0887, 0.0, 0.0]

# Joint limit
Q1_F_LIM = 1.00     # q1 on front and rear legs
Q1_M_LIM = 0.84     # q1 on middle legs
Q2_A_LIM = 2.22     # q2 on all legs
Q3_A_LIM = 2.79     # q3 on all legs

# Offset from CoB to Leg using Body Frame
COXA_X = 0.115
COXA_Y = 0.09
COXA_Z = 0.0

# rotation from base frame to leg frame (deg)
ROT_BASE_X_LF = 50.
ROT_BASE_X_LM = 90.
ROT_BASE_X_LR = 130.
ROT_BASE_X_RF = -50.
ROT_BASE_X_RM = -90.
ROT_BASE_X_RR = -130.

ROT_BASE_X_LEG = [ROT_BASE_X_LF, ROT_BASE_X_LM, ROT_BASE_X_LR,
				 	ROT_BASE_X_RF,ROT_BASE_X_RM,ROT_BASE_X_RR]
TRN_BASE_X_LEG = [	(COXA_X,COXA_Y) ,(0.,COXA_Y) ,(-COXA_X,COXA_Y),\
					(COXA_X,-COXA_Y),(0.,-COXA_Y),(-COXA_X,-COXA_Y)]

g 	 = 9.81 	# Gravity
M_KC = 16.06 	# MX64 motor torque constant

# Joint numbering used in transforms.py
LF_Q1_JOINT = 0;	LM_Q1_JOINT = 3;	LR_Q1_JOINT = 6;
LF_Q2_JOINT = 1;	LM_Q2_JOINT = 4;	LR_Q2_JOINT = 7;
LF_Q3_JOINT = 2;	LM_Q3_JOINT = 5;	LR_Q3_JOINT = 8;

RF_Q1_JOINT = 9; 	RM_Q1_JOINT = 12;	RR_Q1_JOINT = 15;
RF_Q2_JOINT = 10;	RM_Q2_JOINT = 13;	RR_Q2_JOINT = 16;
RF_Q3_JOINT = 11;	RM_Q3_JOINT = 14;	RR_Q3_JOINT = 17;

## ------------------------ ##
## 		Motion Planning		##
## ------------------------ ##
# Foothold area (m) - MP
LEG_AREA_LX = 0.15
LEG_AREA_LY = 0.15

## ----------------------------	##
## 			Transition			##
## ----------------------------	##


## ================================================================ ##
##                      Controller parameters 	 					##
## ================================================================ ##
CTR_RATE 	= 50 			# controller rate for robot, Hz
CTR_INTV 	= 1./CTR_RATE 	# controller interval for robot, s

## ================================================================ ##
##                       Gait parameters 	 						##
## ================================================================ ##

GAIT_TYPE 	 = 1 		# type 1=wave, 2=ripple, 3=tetrapod, 4=tripod
BODY_HEIGHT  = 0.1		# ori: 0.10, chimney: 0.0
STANCE_WIDTH = 0.21		# ori: 0.21, chimney: 0.27, 0.31 for tripod
STEP_STROKE  = 0.08		# step size, x, 	default 0.07
STEP_HEIGHT  = 0.05		# step height, z 	default 0.05
GAIT_TPHASE	 = 2.0 		# period per gait phase

BASE_MAX_LINEAR_VELOCITY  = 0.04	# maximum base velocity, m/s - walking: 0.025
BASE_MAX_ANGULAR_VELOCITY = 0.02	# maximum base velocity, rad/s

# BASE_SPEED 			= 0.025 		# walking speed in m/s
# DUTY_FACTOR = {'tripod':0.5, 'tetrapod':2./3., 'ripple':3./4., 'wave':5./6. }

## ================================================================ ##
##                  	Stability Parameters 						##
## ================================================================ ## 
SM_MIN = 0.05

## ================================================================ ##
##                  Error Compensation parameters 					##
## ================================================================ ## 
QCOMPENSATION = 0.0#33

## ================================================================ ##
##                  	Inclination parameters 	 					##
## ================================================================ ##

QDEADZONE = 0.087 		# surface deadzone - ignore surface inclination below 5 degrees

## ================================================================ ##
##                       Stance parameters 	 						##
## ================================================================ ##
# Offset for front and rear legs
TETA_F = 40.;	
TETA_R = -TETA_F;
LEG_OFFSET = [TETA_F, 0., TETA_R, -TETA_F, 0., -TETA_R]
### Leg default position, SCS
STANCE_TYPE = "flat" 	# "flat", "chimney", "sideways"
LEG_STANCE  = stance_selection.initial_stance(STANCE_WIDTH,BODY_HEIGHT, STANCE_TYPE, TETA_F, TETA_R)

##########################################################################################################################################

## ================================================================ ##
##                       	Transforms 		 						##
## ================================================================ ##
# TODO: THESE SHOULD NOT BE REQUIRED ANYMORE
FR_base_X_hip = {}
FR_base_X_hip[0] = np.array([ [COXA_X],  [COXA_Y],  [COXA_Z] ])
FR_base_X_hip[1] = np.array([ [0.0]	  ,  [COXA_Y],  [COXA_Z] ])
FR_base_X_hip[2] = np.array([ [-COXA_X], [COXA_Y],  [COXA_Z] ])
FR_base_X_hip[3] = np.array([ [COXA_X],  [-COXA_Y], [COXA_Z] ])
FR_base_X_hip[4] = np.array([ [0.0]	  ,  [-COXA_Y], [COXA_Z] ])
FR_base_X_hip[5] = np.array([ [-COXA_X], [-COXA_Y], [COXA_Z] ])

# transform from base to hip is reversed h_A_e = R(-90)*b_A_e
TF_BASE_X_HIP = np.array([-(5./18.)*np.pi,-np.pi/2.,-(13./18.)*np.pi, (5./18.)*np.pi, np.pi/2., (13./18.)*np.pi])
TF_HIP_X_BASE = np.array([ (5./18.)*np.pi, np.pi/2., (13./18.)*np.pi,-(5./18.)*np.pi,-np.pi/2.,-(13./18.)*np.pi])

# Direction tuple - for use with tf class
ORIGIN = (0,0,0)
X_AXIS = (1,0,0)
Y_AXIS = (0,1,0)
Z_AXIS = (0,0,1)

## ================================================================ ##
##                       	Topics 	 		 						##
## ================================================================ ##

JOINT_NAME 	  = [None]*18
JOINT_NAME[0] = 'lf_q1_joint'
JOINT_NAME[1] = 'lf_q2_joint'
JOINT_NAME[2] = 'lf_q3_joint'
JOINT_NAME[3] = 'lm_q1_joint'
JOINT_NAME[4] = 'lm_q2_joint'
JOINT_NAME[5] = 'lm_q3_joint'
JOINT_NAME[6] = 'lr_q1_joint'
JOINT_NAME[7] = 'lr_q2_joint'
JOINT_NAME[8] = 'lr_q3_joint'
JOINT_NAME[9] = 'rf_q1_joint'
JOINT_NAME[10] = 'rf_q2_joint'
JOINT_NAME[11] = 'rf_q3_joint'
JOINT_NAME[12] = 'rm_q1_joint'
JOINT_NAME[13] = 'rm_q2_joint'
JOINT_NAME[14] = 'rm_q3_joint'
JOINT_NAME[15] = 'rr_q1_joint'
JOINT_NAME[16] = 'rr_q2_joint'
JOINT_NAME[17] = 'rr_q3_joint'

ROBOT_STATE = ['x','y','z','r','p','y']

