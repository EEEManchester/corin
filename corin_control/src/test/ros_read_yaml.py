#!/usr/bin/env python

## Function for manipulating the body pose of the hexapod with legs
## fixed to the spot. Initial stance is the nominal stance

import rospy
import numpy as np
import math
import yaml

from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from corin_control import *

def yaml_to_MotionPlan(data):
	
	mplan = MotionPlan()
	mplan.qb.X.t = data['path_x_t']
	mplan.gait_phase = data['gphase']

	for i in range(len(data['path_x_xp'])):
		mplan.qb.X.xp = np.vstack((mplan.qb.X.xp, np.array(data['path_x_xp'][i])))
		mplan.qb.X.xv = np.vstack((mplan.qb.X.xv, np.array(data['path_x_xv'][i])))
		mplan.qb.X.xa = np.vstack((mplan.qb.X.xa, np.array(data['path_x_xa'][i])))

		mplan.qb.W.xp = np.vstack((mplan.qb.W.xp, np.array(data['path_w_xp'][i])))
		mplan.qb.W.xv = np.vstack((mplan.qb.W.xv, np.array(data['path_w_xv'][i])))
		mplan.qb.W.xa = np.vstack((mplan.qb.W.xa, np.array(data['path_w_xa'][i])))

	for i in range(len(data['cob'])):
		mplan.qbp.append(np.array(data['cob'][i]))

	for j in range(6):
		for i in range(len(data['wXf'][j])):
			mplan.f_world_X_foot[j].xp.append(np.array(data['wXf'][j][i]))
		for i in range(len(data['bXf'][j])):
			mplan.f_base_X_foot[j].xp.append(np.array(data['bXf'][j][i]))
		for i in range(len(data['bXn'][j])):
			mplan.f_world_base_X_NRP[j].xp.append(np.array(data['bXn'][j][i]))

	return mplan

if __name__ == "__main__":
	rospy.init_node('main_controller') 		#Initialises node
	rate  = rospy.Rate(1.0/0.05)			# frequency
	
	with open("data.yaml", 'r') as stream:
		data = yaml.load(stream)

	motion_plan = yaml_to_MotionPlan(data)


