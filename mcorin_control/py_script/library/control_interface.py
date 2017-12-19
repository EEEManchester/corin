#!/usr/bin/env python

## Control interface for tablet control of Corin
import rospy
import numpy as np

from constant import *

class control_interface:
	def __init__(self):
		self.x_com = np.array([.0,.0,BODY_HEIGHT])
		self.w_com = np.array([.0,.0,.0])
		self.mode  = 1 								# 1: pose, 2: walk
		self.reset_flag = False

		self.reset_parameters()

	def reset_variables(self):
		self.x_com = np.array([.0,.0,BODY_HEIGHT])
		self.w_com = np.array([.0,.0,.0])

	def reset_parameters(self):
		rospy.set_param('stop_flag', False)		# emergency stop (working)
		rospy.set_param('reset',False)			# move to ground position with legs up (working)
		rospy.set_param('gaitdemo',False)		# to implement gait demo later
		rospy.set_param('bodypose', False)		# perform the bodypose movement (working)
		rospy.set_param('walkforward', False)	# walk forward ~0. 4 metres (working)
		rospy.set_param('walkright', False)		# walk right ~0.4 metres (working)
		rospy.set_param('walkback', False)		# Walk backwards ~0.4 metres (Working)
		rospy.set_param('walkleft', False)		# walk left ~0.4 metres (working)

	#corin performs bodypose
	def Bodypose(self):
		self.reset_variables()
		self.mode = 1

		## Chimney Demo
		if (STANCE_TYPE == "chimney"):
			# up/down
			self.x_com = np.vstack((self.x_com,np.array([0.0, 0.0, 0.06])))
			# self.x_com = np.vstack((self.x_com,np.array([0.0, 0.0, -0.03])))
			self.x_com = np.vstack((self.x_com,np.array([0.0, 0.0, BODY_HEIGHT])))
			self.x_com = np.vstack((self.x_com,np.array([0.0, 0.0, 0.06])))
			self.x_com = np.vstack((self.x_com,np.array([0.0, 0.0, BODY_HEIGHT])))
			
			# front/back
			# self.x_com = np.vstack((self.x_com,np.array([0.03, 0.0, 0.0])))
			# self.x_com = np.vstack((self.x_com,np.array([-0.03, 0.0, 0.0])))
			# self.x_com = np.vstack((self.x_com,np.array([0.0, 0.0, BODY_HEIGHT])))

		## Sideways Demo
		elif (STANCE_TYPE == "sideways"):
			# left/right & up/down
			self.x_com = np.vstack((self.x_com,np.array([0.0, 0.03, BODY_HEIGHT+0.03])))
			self.x_com = np.vstack((self.x_com,np.array([0.0,-0.03, BODY_HEIGHT-0.03])))
			self.x_com = np.vstack((self.x_com,np.array([0.0, 0.03, BODY_HEIGHT+0.03])))
			self.x_com = np.vstack((self.x_com,np.array([0.0,-0.03, BODY_HEIGHT-0.03])))
			self.x_com = np.vstack((self.x_com,np.array([0.0, 0.0 , BODY_HEIGHT ])))
			# forward/backwards
			self.x_com = np.vstack((self.x_com,np.array([ 0.03, 0., BODY_HEIGHT])))
			self.x_com = np.vstack((self.x_com,np.array([-0.03, 0., BODY_HEIGHT])))
			self.x_com = np.vstack((self.x_com,np.array([ 0.03, 0., BODY_HEIGHT])))
			self.x_com = np.vstack((self.x_com,np.array([-0.03, 0., BODY_HEIGHT])))
			self.x_com = np.vstack((self.x_com,np.array([ 0.0, 0., BODY_HEIGHT])))

		## Full bodypose demo
		elif (STANCE_TYPE == "flat"):
			self.x_com = np.vstack((self.x_com,np.array([0.03, -0.04, BODY_HEIGHT])))
			self.w_com = np.vstack((self.w_com,np.array([-0.22, -0.075, 0.])))
			self.x_com = np.vstack((self.x_com,np.array([0.03,  0.04, BODY_HEIGHT])))
			self.w_com = np.vstack((self.w_com,np.array([0.22, -0.075, 0.])))
			self.x_com = np.vstack((self.x_com,np.array([-0.03, 0.04, BODY_HEIGHT])))
			self.w_com = np.vstack((self.w_com,np.array([0.22,  0.075, 0.])))
			self.x_com = np.vstack((self.x_com,np.array([-0.03, -0.04, BODY_HEIGHT])))
			self.w_com = np.vstack((self.w_com,np.array([-0.22,  0.075, 0.])))
			self.x_com = np.vstack((self.x_com,np.array([0.00, 0.00, BODY_HEIGHT])))
			self.w_com = np.vstack((self.w_com,np.array([0.00, 0.00, -0.2])))
			self.x_com = np.vstack((self.x_com,np.array([0.00, 0.00, 0.22])))
			self.w_com = np.vstack((self.w_com,np.array([0.,0.,0.12])))
			self.x_com = np.vstack((self.x_com,np.array([0.,  0.0, BODY_HEIGHT])))
			self.w_com = np.vstack((self.w_com,np.array([0.,  0.0, 0.])))

	def WalkForward(self):
		self.reset_variables()
		self.mode = 2

		self.x_com = np.vstack((self.x_com,np.array([0.2, 0.0, BODY_HEIGHT])))
		self.w_com = np.vstack((self.w_com,np.array([0.,0.,0.])))

	def WalkBack(self):
		self.reset_variables()
		self.mode = 2

		self.x_com = np.vstack((self.x_com,np.array([-0.2, 0.0, BODY_HEIGHT])))
		self.w_com = np.vstack((self.w_com,np.array([0.,0.,0.])))

	def WalkRight(self):
		self.reset_variables()
		self.mode = 2

		self.x_com = np.vstack((self.x_com,np.array([0.0, -0.2, BODY_HEIGHT])))
		self.w_com = np.vstack((self.w_com,np.array([0.,0.,0.])))

	def WalkLeft(self):
		self.reset_variables()
		self.mode = 2

		self.x_com = np.vstack((self.x_com,np.array([0.0, 0.2, BODY_HEIGHT])))
		self.w_com = np.vstack((self.w_com,np.array([0.,0.,0.])))

	def Reset(self):
		self.reset_variables()
		self.mode = 3
		self.reset_flag = True

	def action_to_take(self):
		if (rospy.get_param('bodypose')==True):
			rospy.set_param('bodypose',False)
			self.Bodypose()
			return (self.x_com, self.w_com, self.mode)

		elif (rospy.get_param('walkforward')==True):
			rospy.set_param('walkforward',False)
			self.WalkForward()
			return (self.x_com, self.w_com, self.mode)

		elif (rospy.get_param('walkback')==True):	#command prompt: rosparam set walkback True
			rospy.set_param('walkback',False)
			self.WalkBack()
			return (self.x_com, self.w_com, self.mode)

		elif (rospy.get_param('walkleft')==True): 	#command: set rosparam walkleft True
			rospy.set_param('walkleft',False)
			self.WalkLeft()
			return (self.x_com, self.w_com, self.mode)

		elif (rospy.get_param('walkright')==True):
			rospy.set_param('walkright',False)
			self.WalkRight()
			return (self.x_com, self.w_com, self.mode)

		elif (rospy.get_param('reset')==True):		# Command Prompt: rosparam set reset True
			rospy.set_param('reset',False)
			self.Reset()
			return (self.x_com, self.w_com, self.mode)
		else:
			return None
