#!/usr/bin/env python

""" Main file for Corin """ 
__version__ = '1.0'
__author__  = 'Wei Cheah'

import rospy
import robot_controller as Control_Framework

if __name__ == "__main__":

	manager = Control_Framework.CorinManager(True)

	rospy.loginfo('Robot Ready!')
	
	# rospy.set_param('/corin/bodypose', True)
	# rospy.set_param('/corin/walk_front', True)
	# rospy.set_param('/corin/walk_left', True)
	# rospy.set_param('/corin/walk_right', True)
	# rospy.set_param('/corin/walk_back', True)
	# rospy.set_param('/corin/rotate', True)
	# rospy.set_param('/corin/g2c_transition', True)
	
	rospy.set_param('/corin/plan_path', True)

	while not rospy.is_shutdown():
		manager.action_interface()
		# rospy.set_param('walkleft', True)
	
	# ## Demo - Chimney ========================================== ##
	# rospy.set_param('/corin/g2c_transition', True)
	# manager.action_interface()
	# rospy.set_param('/corin/walk_front', True)
	# manager.action_interface()
	# rospy.set_param('/corin/walk_up', True)
	# manager.action_interface()
	# rospy.set_param('/corin/walk_front', True)
	# manager.action_interface()
	# rospy.set_param('/corin/walk_down', True)
	# manager.action_interface()
	# rospy.set_param('/corin/c2g_transition', True)
	# manager.action_interface()

	## Demo - Wall ========================================== ##
	# raw_input('ROBOT READY!')
	# rospy.set_param('/corin/g2w_transition', True)
	# manager.action_interface()
	# rospy.set_param('/corin/walk_front', True)
	# manager.action_interface()
	# rospy.set_param('/corin/w2g_transition', True)
	# manager.action_interface()