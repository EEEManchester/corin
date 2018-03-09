#!/usr/bin/env python

## Main file for Corin
import rospy
import robot_controller as Control_Framework

if __name__ == "__main__":

	manager = Control_Framework.CorinManager(True)

	raw_input('ROBOT READY!')
	while not rospy.is_shutdown():
		# rospy.set_param('walkforward', True)
		manager.action_interface()
