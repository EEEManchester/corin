#!/usr/bin/env python
import rospy

import numpy
import math	
import constant as UDC
from scipy import linalg


### Parameters for body stance ###
def get_body_height():
	if	rospy.has_param('/' + UDC.ROBOT_NS + '/stance/body_height'):
		BODY_HEIGHT	=	rospy.get_param('/' + UDC.ROBOT_NS + '/stance/body_height')
	else:
		##rospy.logwarn("body height not provided;	using	%.3f" %UDC.BODY_HEIGHT)
		BODY_HEIGHT = UDC.BODY_HEIGHT

	return BODY_HEIGHT

def get_stance_width():
	if	rospy.has_param('/' + UDC.ROBOT_NS +'/stance/stance_width'):
		STANCE_WIDTH	=	rospy.get_param('/' + UDC.ROBOT_NS +'/stance/stance_width')
	else:
		##rospy.logwarn("stance width not provided;	using	%.3f" %UDC.STANCE_WIDTH)
		STANCE_WIDTH = UDC.STANCE_WIDTH

	return STANCE_WIDTH

def get_gait_type():
	if	rospy.has_param('/' + UDC.ROBOT_NS + '/gait/type'):
		GAIT_TYPE	=	rospy.get_param('/' + UDC.ROBOT_NS + '/gait/type')
	else:
		##rospy.logwarn("gait type not provided; using %.1f" %UDC.gait_type)
		GAIT_TYPE = UDC.GAIT_TYPE

	return GAIT_TYPE

def get_step_height():
	if	rospy.has_param('/' + UDC.ROBOT_NS + '/gait/step_height'):
		STEP_HEIGHT	=	rospy.get_param('/' + UDC.ROBOT_NS + '/gait/STEP_HEIGHT')
	else:
		##rospy.logwarn("step height not provided; using %.3f" %UDC.STEP_HEIGHT)
		STEP_HEIGHT = UDC.STEP_HEIGHT

	return STEP_HEIGHT

def get_step_stroke():
	if	rospy.has_param('/' + UDC.ROBOT_NS + '/gait/step_stroke'):
		step_stroke	=	rospy.get_param('/' + UDC.ROBOT_NS + '/gait/step_stroke')
	else:
		step_stroke = UDC.STEP_STROKE

	return float(step_stroke)


### Query parameter server to determine phase (transfer or support)
def set_leg_phase(phase):
	rospy.set_param(UDC.ROBOT_NS + '/leg_phase', phase)

def get_leg_phase():
	if	rospy.has_param(UDC.ROBOT_NS + '/leg_phase'):
		leg_phase =	rospy.get_param(UDC.ROBOT_NS + '/leg_phase')
	else:
		leg_phase = [0,0,0,0,0,0]

	return leg_phase



#####################################################################################################


#####################################################################################################