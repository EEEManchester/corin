#!/usr/bin/env python
# constant file for mcorin robot

import sys; sys.dont_write_bytecode = True

import numpy as np
import stance_selection

# Main name
ROBOT_NS = "corin"

## ================================================================ ##
##                       Robot parameters 	 						##
## ================================================================ ##
# Link Length, Mass
L1 = 0.060;	LL_MASS_1 = 0.040
L2 = 0.150;	LL_MASS_2 = 0.350
L3 = 0.150;	LL_MASS_3 = 0.116

# Joint limit
Q1_F_LIM = 1.00     # q1 on front and rear legs
Q1_M_LIM = 0.84     # q1 on middle legs
Q2_A_LIM = 2.22     # q2 on all legs
Q3_A_LIM = 2.79     # q3 on all legs

# Offset from CoB to Leg using Body Frame
COXA_X = 0.115
COXA_Y = 0.09
COXA_Z = 0.0

g 	 = 9.81 	# Gravity
M_KC = 16.06 	# MX64 motor torque constant

## ================================================================ ##
##                       Gait parameters 	 						##
## ================================================================ ##
DUTY_FACTOR = {'tripod':0.5, 'tetrapod':2./3., 'ripple':3./4., 'wave':5./6. }

BODY_HEIGHT  		= 0.1           # ori: 0.10, chimney: 0.0
STANCE_WIDTH 		= 0.21          # ori: 0.21, chimney: 0.27, 0.31 for tripod
GAIT_TYPE 			= 3 			# type 1=wave, 2=ripple, 3=tetrapod, 4=tripod
STEP_STROKE 		= 0.08 			# step size, x, 	default 0.07
STEP_HEIGHT 		= 0.1 			# step height, z 	default 0.05
WALKING_SPEED 		= 0.035 		# walking speed in m/s
TRAC_PERIOD			= 1.5			# cycle time for movement
TRAC_INTERVAL 		= 0.005			# intervals for trajectory

## ================================================================ ##
##                  	Compensation parameters	 					##
## ================================================================ ## 
QCOMPENSATION = -0.035
## ================================================================ ##
##                  	Inclination parameters 	 					##
## ================================================================ ##

QDEADZONE = 0.087 		# surface deadzone - ignore surface inclination below 5 degrees

## ================================================================ ##
##                       Stance parameters 	 						##
## ================================================================ ##

### Leg default position, SCS
TETA_F = 0;	TETA_R = -TETA_F;
STANCE_TYPE = "flat" 	# "flat", "chimney", "sideways"

LEG_STANCE = stance_selection.initial_stance(STANCE_WIDTH,BODY_HEIGHT, STANCE_TYPE, TETA_F, TETA_R)

##########################################################################################################################################

## ================================================================ ##
##                       	Transforms 		 						##
## ================================================================ ##
FR_base_X_hip = {}
FR_base_X_hip[0] = np.array([ [COXA_X],  [COXA_Y],  [COXA_Z] ])
FR_base_X_hip[1] = np.array([ [0.0]	  ,  [COXA_Y],  [COXA_Z] ])
FR_base_X_hip[2] = np.array([ [-COXA_X], [COXA_Y],  [COXA_Z] ])
FR_base_X_hip[3] = np.array([ [COXA_X],  [-COXA_Y], [COXA_Z] ])
FR_base_X_hip[4] = np.array([ [0.0]	  ,  [-COXA_Y], [COXA_Z] ])
FR_base_X_hip[5] = np.array([ [-COXA_X], [-COXA_Y], [COXA_Z] ])

# transform from base to hip is reversed h_A_e = R(-90)*b_A_e
TF_BASE_X_HIP = np.array([-(2./9.)*np.pi,-np.pi/2.,-(13./18.)*np.pi, (2./9.)*np.pi, np.pi/2., (13./18.)*np.pi])
TF_HIP_X_BASE = np.array([ (2./9.)*np.pi, np.pi/2., (13./18.)*np.pi,-(2./9.)*np.pi,-np.pi/2.,-(13./18.)*np.pi])

# Direction tuple - for use with tf class
ORIGIN = (0,0,0)
X_AXIS = (1,0,0)
Y_AXIS = (0,1,0)
Z_AXIS = (0,0,1)

## ================================================================ ##
##                       	Topics 	 		 						##
## ================================================================ ##

JOINT_NAME = {}
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

ROBOT_STATE = {}
ROBOT_STATE[0] ='x'
ROBOT_STATE[1] ='y'
ROBOT_STATE[2] ='z'
ROBOT_STATE[3] ='r'
ROBOT_STATE[4] ='p'
ROBOT_STATE[5] ='y'
