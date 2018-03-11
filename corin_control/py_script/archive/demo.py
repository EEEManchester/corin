#!/usr/bin/env python

## Actions for corin stitched together - 
## Does not fully work, not exactly sure why
import rospy

import controller_walking as Control_Framework


if __name__ == "__main__":

	rospy.init_node('demo') 		#Initialises node

	# raw_input('Start demo!')
	print 'Bodypose!'
	rospy.set_param('bodypose',True)
	rospy.sleep(20)
	print 'Going forward'
	rospy.set_param('walkforward',True)
	rospy.sleep(16)
	print 'Going backwards'
	rospy.set_param('walkback',True)
	rospy.sleep(16)
	print 'Resetting'
	rospy.set_param('reset',True)
	rospy.sleep(3)
	rospy.set_param('reset',True)
