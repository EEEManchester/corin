#!/usr/bin/env python

""" Main file for Corin """
__version__ = '1.0'
__author__  = 'Wei Cheah'

import sys
import rospy
import numpy as np
import robot_controller as Control_Framework

if __name__ == "__main__":

	manager = Control_Framework.RobotController(False)
	rospy.loginfo('Robot Ready!')
	
	motion = None
	if len(sys.argv) > 1: 
		if sys.argv[1] == 'transition':
			motion = 'transition'
			rospy.set_param('/corin/plan_path', True)
		elif sys.argv[1] == 'forward':
			motion = 'forward'
			rospy.set_param('/corin/plan_path', True)
		elif sys.argv[1] == 'bodypose':
			motion = 'bodypose'
			rospy.set_param('/corin/bodypose', True)
		elif sys.argv[1] == 'chimney':
			motion = 'chimney'
			rospy.set_param('/corin/plan_path', True)

	# rospy.set_param('/corin/bodypose', True)
	# rospy.set_param('/corin/plan_path', True)

	## WIP: The following does not work
	# rospy.set_param('/corin/walk_front', True)
	# rospy.set_param('/corin/walk_left', True)
	# rospy.set_param('/corin/walk_right', True)
	# rospy.set_param('/corin/walk_back', True)
	# rospy.set_param('/corin/rotate', True)

	## Run continuously
	# while not rospy.is_shutdown():
	# if motion is not None:
	manager.action_interface(motion)
