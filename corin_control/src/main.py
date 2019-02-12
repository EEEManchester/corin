#!/usr/bin/env python

""" Main file for Corin """ 
__version__ = '1.0'
__author__  = 'Wei Cheah'

import rospy
import robot_controller as Control_Framework

if __name__ == "__main__":

	# manager = Control_Framework.CorinManager(True)
	manager = Control_Framework.RobotController(True)
	rospy.loginfo('Robot Ready!')
	
	# rospy.set_param('/corin/bodypose', True)
	rospy.set_param('/corin/plan_path', True)

	## WIP: The following does not work
	# rospy.set_param('/corin/walk_front', True)
	# rospy.set_param('/corin/walk_left', True)
	# rospy.set_param('/corin/walk_right', True)
	# rospy.set_param('/corin/walk_back', True)
	# rospy.set_param('/corin/rotate', True)
	
	## Run continuously
	# while not rospy.is_shutdown():
	manager.action_interface()
	