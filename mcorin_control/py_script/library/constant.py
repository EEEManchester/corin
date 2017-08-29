#!/usr/bin/env python
# constant file for mcorin robot 

import sys; sys.dont_write_bytecode = True

import numpy as np
from numpy.linalg import inv
from fractions import Fraction
import copy
import transformations as tf

# Main name
ROBOT_NS = "corin"

## ================================================================ ## 
##                       Robot parameters 	 						##
## ================================================================ ## 
#Link Length, Mass
L1 = 0.077;	LL_LENG_1 = 0.077; 	LL_MASS_1 = 0.175
L2 = 0.150;	LL_LENG_2 = 0.150;	LL_MASS_2 = 0.156
L3 = 0.170;	LL_LENG_3 = 0.170;	LL_MASS_3 = 0.054

#Offset from CoB to Leg using Body Frame
COXA_X = 0.125
COXA_Y = 0.075
COXA_Z = 0.0

g 	 = 9.81 	# Gravity
M_KC = 16.06 	# MX64 motor torque constant

## ================================================================ ## 
##                       Gait parameters 	 						##
## ================================================================ ## 
DUTY_FACTOR = {'tripod':0.5, 'tetrapod':2./3., 'ripple':3./4., 'wave':5./6. }

BODY_HEIGHT  		= 0.1 #0.0851
STANCE_WIDTH 		= 0.21 #0.202
GAIT_TYPE 			= 4 			# type 1=wave, 2=ripple, 3=tetrapod, 4=tripod
STEP_STROKE 		= 0.08 			# step size, x, 	default 0.07
STEP_HEIGHT 		= 0.1 			# step height, z 	default 0.05
WALKING_SPEED 		= 0.035 		# walking speed in m/s
TRAC_PERIOD			= 1.5			# cycle time for movement
TRAC_INTERVAL 		= 0.05 			# intervals for trajectory

## ================================================================ ## 
##                  Compensation parameters 	 					##
## ================================================================ ##
QCOMPENSATION = 0.01
## ================================================================ ## 
##                  	Inclination parameters 	 					##
## ================================================================ ##

QDEADZONE = 0.087 		# surface deadzone - ignore surface inclination below 5 degrees

## ================================================================ ## 
##                       Stance parameters 	 						##
## ================================================================ ##

### Leg default position, SCS
TETA_F = -30
TETA_R = 30

LEG_STANCE = {}

# Flat ground stance - original
LEG_STANCE[0] = np.array([ STANCE_WIDTH*np.cos(TETA_F*np.pi/180), STANCE_WIDTH*np.sin(TETA_F*np.pi/180), -BODY_HEIGHT ]) 
LEG_STANCE[1] = np.array([ STANCE_WIDTH, 0, -BODY_HEIGHT]) 
LEG_STANCE[2] = np.array([ STANCE_WIDTH*np.cos(TETA_R*np.pi/180), STANCE_WIDTH*np.sin(TETA_R*np.pi/180), -BODY_HEIGHT ]) 

LEG_STANCE[3] = np.array([ STANCE_WIDTH*np.cos(TETA_F*np.pi/180), STANCE_WIDTH*np.sin(-TETA_F*np.pi/180), -BODY_HEIGHT ])
LEG_STANCE[4] = np.array([STANCE_WIDTH, 0, -BODY_HEIGHT])
LEG_STANCE[5] = np.array([ STANCE_WIDTH*np.cos(TETA_R*np.pi/180), STANCE_WIDTH*np.sin(-TETA_R*np.pi/180), -BODY_HEIGHT ])

# Vertical stance
# V_LS_WIDTH = 0.338#0.3642
# V_LS_HEIGH = 0.0872#0.07

# V_RS_WIDTH = 0.14
# V_RS_HEIGH = -0.0115

# LEG_STANCE.append( np.array([ V_LS_WIDTH*np.cos(TETA_F*np.pi/180), V_LS_WIDTH*np.sin(TETA_F*np.pi/180), V_LS_HEIGH ]) )
# LEG_STANCE.append( np.array([ V_LS_WIDTH, 0, V_LS_HEIGH ]) )
# LEG_STANCE.append( np.array([ V_LS_WIDTH*np.cos(TETA_R*np.pi/180), V_LS_WIDTH*np.sin(TETA_R*np.pi/180), V_LS_HEIGH ]) )

# LEG_STANCE.append( np.array([ V_RS_WIDTH*np.cos(TETA_F*np.pi/180), V_RS_WIDTH*np.sin(-TETA_F*np.pi/180), V_RS_HEIGH ]) ) 
# LEG_STANCE.append( np.array([ V_RS_WIDTH, 0, V_RS_HEIGH]) ) 
# LEG_STANCE.append( np.array([ V_RS_WIDTH*np.cos(TETA_R*np.pi/180), V_RS_WIDTH*np.sin(-TETA_R*np.pi/180), V_RS_HEIGH ]) ) 

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
TF_base_X_hip = np.array([-np.pi/2.,-np.pi/2.,-np.pi/2.,np.pi/2.,np.pi/2.,np.pi/2.]) 	
TF_hip_X_base = np.array([np.pi/2.,np.pi/2.,np.pi/2.,-np.pi/2.,-np.pi/2.,-np.pi/2.])

# Direction tuple - for use with tf class
ORIGIN = (0,0,0)
X_AXIS = (1,0,0)
Y_AXIS = (0,1,0)
Z_AXIS = (0,0,1)

## ================================================================ ## 
##                       	Topics 	 		 						##
## ================================================================ ##
JOINT_TOPICS = {}
JOINT_TOPICS[0] = '/lf_q1_controller'
JOINT_TOPICS[1] = '/lf_q2_controller'
JOINT_TOPICS[2] = '/lf_q3_controller'
JOINT_TOPICS[3] = '/lm_q1_controller'
JOINT_TOPICS[4] = '/lm_q2_controller'
JOINT_TOPICS[5] = '/lm_q3_controller'
JOINT_TOPICS[6] = '/lr_q1_controller'
JOINT_TOPICS[7] = '/lr_q2_controller'
JOINT_TOPICS[8] = '/lr_q3_controller'
JOINT_TOPICS[9] = '/rf_q1_controller'
JOINT_TOPICS[10] = '/rf_q2_controller'
JOINT_TOPICS[11] = '/rf_q3_controller'
JOINT_TOPICS[12] = '/rm_q1_controller'
JOINT_TOPICS[13] = '/rm_q2_controller'
JOINT_TOPICS[14] = '/rm_q3_controller'
JOINT_TOPICS[15] = '/rr_q1_controller'
JOINT_TOPICS[16] = '/rr_q2_controller'
JOINT_TOPICS[17] = '/rr_q3_controller'

##########################################################################################################################################
#### test scripts ###
e_z = np.zeros((3,3)); e_z[2,2] = 1;

rot = np.array([ [0.64],[0.0],[0.] ]) 	# base to world rotation

p_nom = np.array([ [0.0],[2.0],[-3.0] ]) 
x_com = np.array([ [0.0],[0.0],[ 6.0] ]) 
p_nom = 1
i = 0

a = np.array([1,2,3])
b = np.array([5,5,5])

c = np.dot(a,b)

