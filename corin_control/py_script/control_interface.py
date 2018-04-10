#!/usr/bin/env python

""" Control interface for control of Corin 
	TODO: use action server or rostopic inside of parameter server
"""
import rospy

from library import *

class ControlInterface:
	def __init__(self):
		self.mode  = 1 								# 1: pose, 2: walk
		self.reset_flag = False

		self.__initialise__()

	def __initialise__(self):
		rospy.set_param('stop_flag', False)			# emergency stop (working)
		rospy.set_param('/corin/reset',False)		# move to ground position with legs up (working)
		rospy.set_param('/corin/bodypose', False)	# perform the bodypose movement (working)
		rospy.set_param('/corin/walk_front', False)	# walk forward ~0. 4 metres (working)
		rospy.set_param('/corin/walk_right', False)	# walk right ~0.4 metres (working)
		rospy.set_param('/corin/walk_back', False)	# Walk backwards ~0.4 metres (Working)
		rospy.set_param('/corin/walk_left', False)	# walk left ~0.4 metres (working)
		rospy.set_param('/corin/rotate', False)
		rospy.set_param('/corin/wall_transition', False)
		rospy.set_param('/corin/chimney_transition', False)

	#corin performs bodypose
	def bodypose(self, x_cob, w_cob):
		self.mode = 1

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
			# for i in range(0,5):
			# 	x_cob = np.vstack((x_cob,np.zeros(3,1)))

			# x_cob = np.vstack((x_cob,np.array([0. , -0.04, 0.])))
			# x_cob = np.vstack((x_cob,np.array([0.03,  0.04, 0.])))
			# x_cob = np.vstack((x_cob,np.array([-0.03, 0.04, 0.])))
			# x_cob = np.vstack((x_cob,np.array([-0.03, -0.04, 0.])))
			# x_cob = np.vstack((x_cob,np.array([0.00, 0.00, 0.])))
			# x_cob = np.vstack((x_cob,np.array([0.00, 0.0, 0.12])))
			x_cob = np.vstack((x_cob,np.array([0.,  0.0, 0.])))

			# w_cob = np.vstack((w_cob,np.array([-0.22, -0.075, 0.])))
			# w_cob = np.vstack((w_cob,np.array([0.22, -0.075, 0.])))
			# w_cob = np.vstack((w_cob,np.array([0.22,  0.075, 0.])))
			# w_cob = np.vstack((w_cob,np.array([-0.22,  0.075, 0.])))
			# w_cob = np.vstack((w_cob,np.array([0.00, 0.00, -0.15])))
			# w_cob = np.vstack((w_cob,np.array([0.,0.,0.12])))
			w_cob = np.vstack((w_cob,np.array([0.2,  0.0, 0.])))

		return x_cob, w_cob, self.mode
		
	def walk_front(self, x_cob, w_cob):
		self.mode  = 2
		x_cob = np.vstack((x_cob,np.array([0.4, 0., 0.])))
		# x_cob = np.vstack((x_cob,np.array([0.3, 0., 0.])))
		w_cob = np.vstack((w_cob,np.array([0.25, 0., 0.])))
		# w_cob = np.vstack((w_cob,np.array([0.15, 0., 0.])))
		return x_cob, w_cob, self.mode

	def walk_back(self, x_cob, w_cob):
		self.mode  = 2
		x_cob = np.vstack((x_cob,np.array([-0.4, 0.0, 0.])))
		return x_cob, w_cob, self.mode

	def walk_right(self, x_cob, w_cob):
		self.mode  = 2
		x_cob = np.vstack((x_cob,np.array([0.0, -0.25, 0.])))
		w_cob = np.vstack((w_cob,np.array([0.1, 0., 0.])))
		x_cob = np.vstack((x_cob,np.array([0.0, -0.5, 0.])))
		w_cob = np.vstack((w_cob,np.array([0.1, 0., 0.])))
		return x_cob, w_cob, self.mode

	def walk_left(self, x_cob, w_cob):
		self.mode  = 2
		x_cob = np.vstack((x_cob,np.array([0.0, 0.11, 0.])))
		return x_cob, w_cob, self.mode

	def rotate(self, x_cob, w_cob):
		self.mode = 2
		w_cob = np.vstack((w_cob,np.array([0.,0.,1.])))
		return x_cob, w_cob, self.mode

	def reset(self, x_cob, w_cob):
		self.mode = 3
		self.reset_flag = True
		return x_cob, w_cob, self.mode

	def action_to_take(self):
		""" Checks parameter server to identify action to take """

		## Define Variables ##
		x_cob = np.array([.0,.0,.0])
		w_cob = np.array([.0,.0,.0])

		if (rospy.get_param('/corin/bodypose')==True):
			rospy.set_param('/corin/bodypose',False)
			return self.bodypose(x_cob, w_cob)

		elif (rospy.get_param('/corin/walk_front')==True):
			rospy.set_param('/corin/walk_front',False)
			return self.walk_front(x_cob, w_cob)

		elif (rospy.get_param('/corin/walk_back')==True):	
			rospy.set_param('/corin/walk_back',False)
			return self.walk_back(x_cob, w_cob)

		elif (rospy.get_param('/corin/walk_left')==True): 	
			rospy.set_param('/corin/walk_left',False)
			return self.walk_left(x_cob, w_cob)

		elif (rospy.get_param('/corin/walk_right')==True):
			rospy.set_param('/corin/walk_right',False)
			return self.walk_right(x_cob, w_cob)

		elif (rospy.get_param('/corin/rotate')==True):
			rospy.set_param('/corin/rotate',False)
			return self.rotate(x_cob, w_cob)

		elif (rospy.get_param('/corin/wall_transition')==True):
			rospy.set_param('/corin/wall_transition',False)
			return self.wall_transition(x_cob, w_cob)

		elif (rospy.get_param('/corin/chimney_transition')==True):
			rospy.set_param('/corin/chimney_transition',False)
			return self.chimney_transition(x_cob, w_cob)

		elif (rospy.get_param('/corin/reset')==True):		# Command Prompt: rosparam set reset True
			rospy.set_param('/corin/reset',False)
			return self.reset(x_cob, w_cob)
		else:
			return None

	def wall_transition(self, x_cob, w_cob):
		""" wall transition base trajectory """
		## TODO: set somewhere else
		self.mode = 2
		tran_y = 0.2
		tran_z = 0.3

		for q in range(10,91,10):
			qr = np.deg2rad(q)
			xd = np.array([0.0, (1.-np.cos(qr))*tran_y, (np.sin(qr))*tran_z])
			
			x_cob = np.vstack(( x_cob, xd ))
			w_cob = np.vstack(( w_cob, np.array([qr,0.0,0.0]) ))
			
		# Plot.plot_3d(x_cob[:,0],x_cob[:,1],x_cob[:,2])
		return x_cob, w_cob, self.mode

	def chimney_transition(self, x_cob, w_cob):
		self.mode  = 2
		## TODO: set somewhere else
		for i in range(0,6):
			x_cob = np.vstack((x_cob,np.array([0., 0., 0.])))
		
		return x_cob, w_cob, self.mode