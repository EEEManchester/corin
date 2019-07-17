#!/usr/bin/env python

""" Control interface for control of Corin 
	TODO: use action server or rostopic inside of parameter server
"""
import rospy

# from library import *
from corin_control import *
class ControlInterface:
	def __init__(self):
		self.mode  = 1 								# 1: pose, 2: walk
		self.reset_flag = False

		self.__initialise__()

	def __initialise__(self):
		rospy.set_param('/corin/execute', 0) 		# execute motion selected

		rospy.set_param('stop_flag', False)			# emergency stop
		rospy.set_param('/corin/reset',False)		# move to ground position with legs up
		rospy.set_param('/corin/bodypose', False)	# perform the bodypose movement
		rospy.set_param('/corin/walk', False)

		# TBC
		rospy.set_param('/corin/load_plan', False)
		rospy.set_param('/corin/rotate', False)


	#corin performs bodypose
	def bodypose(self, x_cob, w_cob):
		self.mode = 4

		## Chimney Demo
		if (STANCE_TYPE == "chimney"):
			# up/down
			x_cob = np.vstack((x_cob,np.array([0.0, 0.0, 0.05])))
			# x_cob = np.vstack((x_cob,np.array([0.0, 0.0, BODY_HEIGHT])))
			# x_cob = np.vstack((x_cob,np.array([0.0, 0.0, 0.06])))
			# x_cob = np.vstack((x_cob,np.array([0.0, 0.0, BODY_HEIGHT])))

			# front/back
			x_cob = np.vstack((x_cob,np.array([0.025, 0.0, 0.05])))
			x_cob = np.vstack((x_cob,np.array([-0.025, 0.0, 0.05])))
			x_cob = np.vstack((x_cob,np.array([0.0, 0.0, 0.05])))

			# left/right
			x_cob = np.vstack((x_cob,np.array([0.0, 0.025, 0.05])))
			x_cob = np.vstack((x_cob,np.array([0.0, -0.025, 0.05])))
			x_cob = np.vstack((x_cob,np.array([0.0, 0.0, 0.05])))
			x_cob = np.vstack((x_cob,np.array([0.0, 0.0, 0.0])))

		## Sideways Demo
		elif (STANCE_TYPE == "sideways"):

			x_cob = np.vstack((x_cob,np.array([ 0.025, 0., 0.])))
			x_cob = np.vstack((x_cob,np.array([-0.025, 0., 0.])))
			x_cob = np.vstack((x_cob,np.array([ 0.0  , 0., 0. ])))

			## 90 degrees
			# x_cob = np.vstack((x_cob,np.array([0.0,-0.05, BODY_HEIGHT])))
			# x_cob = np.vstack((x_cob,np.array([0.0, 0.0  , BODY_HEIGHT ])))
			
			# x_cob = np.vstack((x_cob,np.array([0.0, 0.0, BODY_HEIGHT+0.05])))
			# x_cob = np.vstack((x_cob,np.array([0.0, 0.0, BODY_HEIGHT])))
			# x_cob = np.vstack((x_cob,np.array([0.0, 0.0, BODY_HEIGHT ])))

			## 30 and 60 deg
			x_cob = np.vstack((x_cob,np.array([0.0,-0.025, 0.])))
			x_cob = np.vstack((x_cob,np.array([0.0, 0.025, 0.])))
			x_cob = np.vstack((x_cob,np.array([0.0, 0.0  , 0. ])))

			x_cob = np.vstack((x_cob,np.array([0.0, 0.0, 0.+0.025])))
			x_cob = np.vstack((x_cob,np.array([0.0, 0.0, 0.-0.025])))
			x_cob = np.vstack((x_cob,np.array([0.0, 0.0, 0.])))

			# left/right & up/down
			# x_cob = np.vstack((x_cob,np.array([0.0, 0.025, BODY_HEIGHT+0.025])))
			# x_cob = np.vstack((x_cob,np.array([0.0,-0.025, BODY_HEIGHT-0.025])))
			# x_cob = np.vstack((x_cob,np.array([0.0, 0.025, BODY_HEIGHT+0.025])))
			# x_cob = np.vstack((x_cob,np.array([0.0,-0.025, BODY_HEIGHT-0.025])))
			# x_cob = np.vstack((x_cob,np.array([0.0, 0.0 , BODY_HEIGHT ])))
			# # forward/backwards
			# x_cob = np.vstack((x_cob,np.array([ 0.025, 0., BODY_HEIGHT])))
			# x_cob = np.vstack((x_cob,np.array([-0.025, 0., BODY_HEIGHT])))
			# x_cob = np.vstack((x_cob,np.array([ 0.025, 0., BODY_HEIGHT])))
			# x_cob = np.vstack((x_cob,np.array([-0.025, 0., BODY_HEIGHT])))
			# x_cob = np.vstack((x_cob,np.array([ 0.0, 0., BODY_HEIGHT])))

		## Full bodypose demo
		elif (STANCE_TYPE == "flat"):
			
			x_cob = np.vstack((x_cob,np.array([0. , -0.04, 0.])))
			x_cob = np.vstack((x_cob,np.array([0.03,  0.04, 0.])))
			x_cob = np.vstack((x_cob,np.array([-0.03, 0.04, 0.])))
			x_cob = np.vstack((x_cob,np.array([-0.03, -0.04, 0.])))
			x_cob = np.vstack((x_cob,np.array([0.,  0.0, 0.1])))
			x_cob = np.vstack((x_cob,np.array([0.,  0.0, 0.])))

			w_cob = np.vstack((w_cob,np.array([-0.22, -0.075, 0.])))
			w_cob = np.vstack((w_cob,np.array([0.22, -0.075, 0.])))
			w_cob = np.vstack((w_cob,np.array([0.22,  0.075, 0.])))
			w_cob = np.vstack((w_cob,np.array([-0.22,  0.075, 0.])))
			w_cob = np.vstack((w_cob,np.array([0.,  0.0, 0.05])))
			w_cob = np.vstack((w_cob,np.array([0.,  0.0, 0.])))

			## Simple motions
			# x_cob = np.vstack((x_cob,np.array([0.,  0.0, 0.])))
			# x_cob = np.vstack((x_cob,np.array([0.,  0.0, 0.])))
			# w_cob = np.vstack((w_cob,np.array([0.2,  0.0, 0.])))
			# w_cob = np.vstack((w_cob,np.array([0.4,  0.0, 0.])))

		return x_cob, w_cob, self.mode, 'walk'
		
	def walk(self):
		self.mode = 5
		
		return None, None, self.mode, 'walk'

	def load_plan(self):
		self.mode = 6
		
		return None, None, self.mode, 'plan'
		
	def rotate(self, x_cob, w_cob):
		self.mode = 2
		w_cob = np.vstack((w_cob,np.array([0.,0.,1.])))
		return x_cob, w_cob, self.mode, 'walk'

	def reset(self, x_cob, w_cob):
		self.mode = 3
		self.reset_flag = True
		return x_cob, w_cob, self.mode, 'walk'

	def action_to_take(self):
		""" Checks parameter server to identify action to take """

		## Define Variables ##
		x_cob = np.array([.0,.0,.0])
		w_cob = np.array([.0,.0,.0])

		if (rospy.get_param('/corin/bodypose')==True):
			rospy.set_param('/corin/bodypose',False)
			return self.bodypose(x_cob, w_cob)

		elif (rospy.get_param('/corin/walk')==True):
			rospy.set_param('/corin/walk',False)
			return self.walk()

		elif (rospy.get_param('/corin/rotate')==True):
			rospy.set_param('/corin/rotate',False)
			return self.rotate(x_cob, w_cob)

		elif (rospy.get_param('/corin/load_plan')==True):
			rospy.set_param('/corin/load_plan',False)
			return self.load_plan()

		elif (rospy.get_param('/corin/reset')==True):		# Command Prompt: rosparam set reset True
			rospy.set_param('/corin/reset',False)
			return self.reset(x_cob, w_cob)
		else:
			return None