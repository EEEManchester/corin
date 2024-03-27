#!/usr/bin/env python

""" Main file for Corin """
__version__ = '1.0'
__author__  = 'Wei Cheah'

import rospy
import robot_controller as Control_Framework

if __name__ == "__main__":

	manager = Control_Framework.CorinManager(True)

	# raw_input('ROBOT READY!')
	rospy.loginfo('Robot Ready!')
	# rospy.set_param('bodypose', True)
	# rospy.set_param('walkleft', True)
	# rospy.set_param('walkright', True)
	# rospy.set_param('walkback', True)
	# rospy.set_param('walkforward', True)
	# rospy.set_param('rotate', True)

	while not rospy.is_shutdown():

		# manager.action_interface()
		# rospy.set_param('bodypose', True)

		manager.action_interface()
		rospy.set_param('walkforward', True)

		manager.action_interface()
		rospy.set_param('walkleft', True)

		# manager.action_interface()
		# rospy.set_param('bodypose', True)

		manager.action_interface()
		rospy.set_param('walkback', True)

		manager.action_interface()
		rospy.set_param('walkright', True)

		manager.action_interface()
		rospy.set_param('rotate', True)

	# test
