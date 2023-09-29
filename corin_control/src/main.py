#!/usr/bin/env python

""" Main file for Corin """
__version__ = '1.0'
__author__ = 'Wei Cheah'

import sys
import rospy
import numpy as np
import robot_controller as Control_Framework

if __name__ == "__main__":

    print('Initialising Robot ....')
    manager = Control_Framework.RobotController(False)
    rospy.loginfo('Robot Ready!')

    motion = None
    if len(sys.argv) == 2:
        if sys.argv[1] == 'bodypose':
            motion = 'bodypose'
            rospy.set_param('/corin/bodypose', True)
        else:
            # Plan from grid map
            motion = sys.argv[1]
            rospy.set_param('/corin/walk', True)

    elif len(sys.argv) == 3:
        # Loads motion plan
        motion = sys.argv[2]
        rospy.set_param('/corin/load_plan', True)
        pass

    ## Run continuously
    # while not rospy.is_shutdown():
    # if motion is not None:
    manager.action_interface(motion)
