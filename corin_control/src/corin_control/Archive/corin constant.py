#!/usr/bin/env python

import sys; sys.dont_write_bytecode = True

import numpy as np
from numpy.linalg import inv
from fractions import Fraction
import copy
import transformations as tf


# Transformation parameters
ORIGIN = (0,0,0)
X_AXIS = (1,0,0)
Y_AXIS = (0,1,0)
Z_AXIS = (0,0,1)

## ================================================================ ## 
##                       Robot parameters 	 						##
## ================================================================ ## 
ROBOT_NS = "corin" 		# Main name

# Leg Link Length, Mass
LL_LENG_1 = 0.0445;	LL_MASS_1 = 0.175
LL_LENG_2 = 0.1920;	LL_MASS_2 = 0.156
LL_LENG_3 = 0.1953;	LL_MASS_3 = 0.054
LL_LENG_d = 0.02839;

LL_COM_1 = 0.0580; 	#LL_INT_1 = inertia_tensor(3.20e-5, 5.60e-5, 4.00e-5)
LL_COM_2 = 0.1213;  #LL_INT_2 = inertia_tensor(1.90e-5, 6.80e-5, 7.90e-5)
LL_COM_3 = 0.0958;	#LL_INT_3 = inertia_tensor(1.60e-5, 11.9e-5, 13.0e-5)

g 	 = 9.81 	# Gravity
M_KC = 16.06 	# MX64 motor torque constant

## ================================================================ ## 
##                       Gait parameters 	 						##
## ================================================================ ## 
# Gait parameters (nominal stance)
DUTY_FACTOR = {'tripod':0.5, 'tetrapod':2./3., 'ripple':3./4., 'wave':5./6. }

BODY_HEIGHT  		= 0.0851
STANCE_WIDTH 		= 0.214
GAIT_TYPE 			= 4 					# type 1=wave, 2=ripple, 3=tetrapod, 4=tripod
STEP_STROKE 		= 0.1 					# step size, x, 	default 0.07
STEP_HEIGHT 		= 0.1 					# step height, z 	default 0.05
WALKING_SPEED 		= 0.035 				# walking speed in m/s
TRAC_PERIOD			= 1.5					# cycle time for movement
TRAC_INTERVAL 		= 0.05 					# intervals for trajectory

## ================================================================ ## 
##                  Compensation parameters 	 					##
## ================================================================ ##
QCOMPENSATION = 0.01

## ================================================================ ## 
##                  	Inclination parameters 	 					##
## ================================================================ ##

QDEADZONE = 0.087 		# surface deadzone - ignore surface inclination below 5 degrees

## ================================================================ ## 
##                  	Surface parameters 	 						##
## ================================================================ ##
# Surface normals
LEFT_PLANE_NORMAL	= [0,0,1]
RIGHT_PLANE_NORMAL	= [0.0,0.0,1.0]

#Offset from CoB to Leg using Body Frame
COXA_X = 0.2
COXA_Y = 0.125
COXA_Z = 0.0

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
## added for mcorin
FR_base_X_hip = {}
FR_base_X_hip[0] = np.array([ [COXA_X],  [COXA_Y],  [COXA_Z] ])
FR_base_X_hip[1] = np.array([ [0.0]	  ,  [COXA_Y],  [COXA_Z] ])
FR_base_X_hip[2] = np.array([ [-COXA_X], [COXA_Y],  [COXA_Z] ])
FR_base_X_hip[3] = np.array([ [COXA_X],  [-COXA_Y], [COXA_Z] ])
FR_base_X_hip[4] = np.array([ [0.0]	  ,  [-COXA_Y], [COXA_Z] ])
FR_base_X_hip[5] = np.array([ [-COXA_X], [-COXA_Y], [COXA_Z] ])

# transfoleg_5 from base to hip is reversed h_A_e = R(-90)*b_A_e
TF_base_X_hip = np.array([-np.pi/2.,-np.pi/2.,-np.pi/2.,np.pi/2.,np.pi/2.,np.pi/2.]) 	
TF_hip_X_base = np.array([np.pi/2.,np.pi/2.,np.pi/2.,-np.pi/2.,-np.pi/2.,-np.pi/2.])

## joint topics
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

# JOINT_TOPICS = {}
# JOINT_TOPICS[0] = '/leg_1_q1_controller/command'
# JOINT_TOPICS[1] = '/leg_1_q2_controller/command'
# JOINT_TOPICS[2] = '/leg_1_q3_controller/command'
# JOINT_TOPICS[3] = '/leg_2_q1_controller/command'
# JOINT_TOPICS[4] = '/leg_2_q2_controller/command'
# JOINT_TOPICS[5] = '/leg_2_q3_controller/command'
# JOINT_TOPICS[6] = '/leg_3_q1_controller/command'
# JOINT_TOPICS[7] = '/leg_3_q2_controller/command'
# JOINT_TOPICS[8] = '/leg_3_q3_controller/command'
# JOINT_TOPICS[9] = '/leg_4_q1_controller/command'
# JOINT_TOPICS[10] = '/leg_4_q2_controller/command'
# JOINT_TOPICS[11] = '/leg_4_q3_controller/command'
# JOINT_TOPICS[12] = '/leg_5_q1_controller/command'
# JOINT_TOPICS[13] = '/leg_5_q2_controller/command'
# JOINT_TOPICS[14] = '/leg_5_q3_controller/command'
# JOINT_TOPICS[15] = '/leg_6_q1_controller/command'
# JOINT_TOPICS[16] = '/leg_6_q2_controller/command'
# JOINT_TOPICS[17] = '/leg_6_q3_controller/command'

##########################################################################################################################################
#### test scripts ###
a = np.zeros(3)
b = np.zeros(3)
a = np.array([1,2,3])
b = copy.deepcopy(a)

print 6%2