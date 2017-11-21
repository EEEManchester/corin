#!/usr/bin/env python

## Main file for Corin
import rospy

import controller_walking as Control_Framework


if __name__ == "__main__":

	manager = Control_Framework.CorinManager(True)

	while not rospy.is_shutdown():
		rospy.set_param('walkforward', True)
		manager.action_interface()
