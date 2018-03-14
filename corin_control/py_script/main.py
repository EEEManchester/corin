#!/usr/bin/env python

## Main file for Corin
import rospy
import robot_controller as Control_Framework

if __name__ == "__main__":

	manager = Control_Framework.CorinManager(True)

	raw_input('ROBOT READY!')
	
	# rospy.set_param('walkforward', True)
	while not rospy.is_shutdown():
		
		manager.action_interface()
