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
		rospy.set_param('/corin/execute', 0) 		# execute motion selected

		rospy.set_param('stop_flag', False)			# emergency stop
		rospy.set_param('/corin/reset',False)		# move to ground position with legs up
		rospy.set_param('/corin/bodypose', False)	# perform the bodypose movement
		rospy.set_param('/corin/walk_front', False)	# walk forward
		rospy.set_param('/corin/walk_right', False)	# walk right 
		rospy.set_param('/corin/walk_back', False)	# Walk backwards
		rospy.set_param('/corin/walk_left', False)	# walk left 
		rospy.set_param('/corin/walk_up', False)	# walk up
		rospy.set_param('/corin/walk_down', False)	# walk down
		rospy.set_param('/corin/rotate', False)
		
		rospy.set_param('/corin/g2w_transition', False)
		rospy.set_param('/corin/w2g_transition', False)
		
		rospy.set_param('/corin/g2c_transition', False)
		rospy.set_param('/corin/c2g_transition', False)

		rospy.set_param('/corin/plan_path', False)

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

			x_cob = np.vstack((x_cob,np.array([0. , -0.04, 0.])))
			x_cob = np.vstack((x_cob,np.array([0.03,  0.04, 0.])))
			x_cob = np.vstack((x_cob,np.array([-0.03, 0.04, 0.])))
			x_cob = np.vstack((x_cob,np.array([-0.03, -0.04, 0.])))
			# x_cob = np.vstack((x_cob,np.array([0.00, 0.00, 0.])))
			# x_cob = np.vstack((x_cob,np.array([0.00, 0.0, 0.12])))
			x_cob = np.vstack((x_cob,np.array([0.,  0.0, 0.])))

			w_cob = np.vstack((w_cob,np.array([-0.22, -0.075, 0.])))
			w_cob = np.vstack((w_cob,np.array([0.22, -0.075, 0.])))
			w_cob = np.vstack((w_cob,np.array([0.22,  0.075, 0.])))
			w_cob = np.vstack((w_cob,np.array([-0.22,  0.075, 0.])))
			# w_cob = np.vstack((w_cob,np.array([0.00, 0.00, -0.15])))
			# w_cob = np.vstack((w_cob,np.array([0.,0.,0.12])))
			w_cob = np.vstack((w_cob,np.array([0.,  0.0, 0.])))

		return x_cob, w_cob, self.mode, 'walk'
		
	def walk_front(self, x_cob, w_cob):
		self.mode  = 2
		x_cob = np.vstack((x_cob,np.array([0.15, 0., 0.])))
		# x_cob = np.vstack((x_cob,np.array([0.3, 0., 0.])))
		# w_cob = np.vstack((w_cob,np.array([0.25, 0., 0.])))
		# w_cob = np.vstack((w_cob,np.array([0.15, 0., 0.])))
		return x_cob, w_cob, self.mode, 'walk'

	def walk_back(self, x_cob, w_cob):
		self.mode  = 2
		x_cob = np.vstack((x_cob,np.array([-0.4, 0.0, 0.])))
		return x_cob, w_cob, self.mode, 'walk'

	def walk_right(self, x_cob, w_cob):
		self.mode  = 2
		x_cob = np.vstack((x_cob,np.array([0.0, -0.2, 0.])))
		w_cob = np.vstack((w_cob,np.array([0.1, 0., 0.])))
		x_cob = np.vstack((x_cob,np.array([0.0, -0.4, 0.])))
		w_cob = np.vstack((w_cob,np.array([0.1, 0., 0.])))
		return x_cob, w_cob, self.mode, 'walk'

	def walk_left(self, x_cob, w_cob):
		self.mode  = 2
		x_cob = np.vstack((x_cob,np.array([0.0, 0.11, 0.])))
		return x_cob, w_cob, self.mode, 'walk'

	def walk_up(self, x_cob, w_cob):
		self.mode  = 2
		x_cob = np.vstack((x_cob,np.array([0., 0., 0.6])))
		return x_cob, w_cob, self.mode, 'walk'

	def walk_down(self, x_cob, w_cob):
		self.mode  = 2
		x_cob = np.vstack((x_cob,np.array([0., 0., -0.3])))
		return x_cob, w_cob, self.mode, 'walk'

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

		elif (rospy.get_param('/corin/walk_up')==True): 	
			rospy.set_param('/corin/walk_up',False)
			return self.walk_up(x_cob, w_cob)

		elif (rospy.get_param('/corin/walk_down')==True):
			rospy.set_param('/corin/walk_down',False)
			return self.walk_down(x_cob, w_cob)

		elif (rospy.get_param('/corin/rotate')==True):
			rospy.set_param('/corin/rotate',False)
			return self.rotate(x_cob, w_cob)

		elif (rospy.get_param('/corin/g2w_transition')==True):
			rospy.set_param('/corin/g2w_transition',False)
			return self.gnd_X_wall_transition(x_cob, w_cob)

		elif (rospy.get_param('/corin/w2g_transition')==True):
			rospy.set_param('/corin/w2g_transition',False)
			return self.wall_X_gnd_transition(x_cob, w_cob)

		elif (rospy.get_param('/corin/g2c_transition')==True):
			rospy.set_param('/corin/g2c_transition',False)
			return self.gnd_X_chimney_transition(x_cob, w_cob)

		elif (rospy.get_param('/corin/c2g_transition')==True):
			rospy.set_param('/corin/c2g_transition',False)
			return self.chimney_X_gnd_transition(x_cob, w_cob)

		elif (rospy.get_param('/corin/plan_path')==True):
			rospy.set_param('/corin/plan_path',False)
			return self.plan_path(x_cob, w_cob)

		elif (rospy.get_param('/corin/reset')==True):		# Command Prompt: rosparam set reset True
			rospy.set_param('/corin/reset',False)
			return self.reset(x_cob, w_cob)
		else:
			return None

	def gnd_X_wall_transition(self, x_cob, w_cob):
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
		return x_cob, w_cob, self.mode, 'g2w_transition'

	def wall_X_gnd_transition(self, x_cob, w_cob):
		""" wall transition base trajectory """
		## TODO: set somewhere else
		self.mode = 2
		tran_y = 0.2
		tran_z = 0.3

		# overwrite initial robot position
		x_cob = np.zeros(3)
		w_cob = np.zeros(3)

		for q in range(80,-1,-10):
			qr = np.deg2rad(q)
			# xd = np.array([0.0, (1-np.cos(qr))*tran_y, (np.sin(qr))*tran_z])
			xd = np.array([0.0, (np.cos(qr))*tran_y, (1-np.sin(qr))*tran_z])
			
			x_cob = np.vstack(( x_cob, xd ))
			w_cob = np.vstack(( w_cob, np.array([qr-np.pi/2,0.0,0.0]) ))
			
		# Plot.plot_3d(x_cob[:,0],x_cob[:,1],x_cob[:,2])
		return -x_cob, w_cob, self.mode, 'w2g_transition'

	def gnd_X_chimney_transition(self, x_cob, w_cob):
		self.mode  = 2
		## TODO: set somewhere else

		## Forward translation
		# x_cob = np.vstack((x_cob,np.array([0.5, 0., 0.])))
		## Vertical translation
		for i in range(0,6):
			x_cob = np.vstack((x_cob,np.array([0., 0., i*0.001/6])))
		
		return x_cob, w_cob, self.mode, 'g2c_transition'

	def chimney_X_gnd_transition(self, x_cob, w_cob):
		self.mode  = 2
		## TODO: set somewhere else
		x_cob = np.vstack((x_cob,np.array([0.7, 0., 0.])))

		# for i in range(0,6):
		# 	x_cob = np.vstack((x_cob,np.array([0., 0., 0.])))
		
		return x_cob, w_cob, self.mode, 'c2g_transition'

	def plan_path(self, x_cob, w_cob):
		self.mode = 4
		return x_cob, w_cob, self.mode, 'walk'