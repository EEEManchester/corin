
#!/usr/bin/env python

## Indexing for leg starts with 0, 1 .... 5
import sys; sys.dont_write_bytecode = True

import rospy
import math
import numpy as np
from scipy import linalg
import transformations as tf

from constant import *
import gait_class as Gaitgenerator
import param_gait
import kdl
import plotgraph as Plot

from std_msgs.msg import Float64
from control_msgs.msg import FollowJointTrajectoryGoal
import actionlib
import tf as RTF

from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTrajectoryControllerState

# from lcorin_control.srv import *

# =======================================#
import pspline_generator as Pspline
import bspline_generator as Bspline

from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from trajectory_msgs.msg import JointTrajectoryPoint

from mcorin_control.srv import JointState as JointStateSrv
from mcorin_control.srv import Imu as ImuSrv

class RobotState:
	def __init__(self):
		self.qc  = JointState() 					# JointState class for service call joint updates
		self.imu = Imu()
		self.Leg = {} 								# leg class
		self.x_spline = JointTrajectoryPoint() 		# CoM trajectory
		self.w_spline = JointTrajectoryPoint() 		# Base angular trajectory

		self.world_X_base = FrameClass('twist') 			# FrameClass for world to base transformation
		self.bspline = Bspline.SplineGenerator() 	# bSplineClass for spline generation

		self.pose = np.zeros((3,1))								# euler angle rotation x,y,z
		self.fr_world_X_base = tf.rotation_zyx(np.zeros(3)) 	# SO(3) rotation using pose

		self.invalid = False 			# robot state: invalid - do not do anything
		self.suspend = False 			# robot state: suspend support (TODO)

		self.gaitgen = Gaitgenerator.GaitClass(3) 		# gait class

		self.active_legs  = 6 				# robot state: number of legs selected to be active		
		self.phase_change = False 			# robot state: phase changed, enables transfer trajectory to be generated
		self.reset_state  = True 			# robot state: reset whole of robot state

		self.support_mode = False 			# puts robot into support mode

		self.cstate = [None]*6 			# foot contact state
		self.cforce = [None]*18			# foot contact forces

		self._initialise()

	def _initialise(self):
		for i in range(6):
			self.Leg[i] = LegClass(i)

	def updateState(self):
		## update using topics - more elegant as rospy is multi-threaded
		self.update_LegState()
		# self.update_ImuState()

	def update_LegState(self):
		# offset to deal with extra joints
		if (len(self.qc.name)==18):
			offset = 0
		else:
			offset = 1
		# update leg states and check if boundary exceeded
		for i in range(0,self.active_legs):

			bstate = self.Leg[i].updateJointState(self.qc.position[offset+i*3:offset+(i*3)+3], self.reset_state)
			self.Leg[i].updateForceState(self.cstate[i], self.cforce[i*3:(i*3)+3])

			if (bstate==True and self.gaitgen.cs[i]==0 and self.support_mode==False):
				self.suspend = True
				print 'suspended'

		# clear reset state
		self.reset_state = False if self.reset_state == True else False

	def update_ImuState(self):
		## quaternion to euler transformation
		neuler = RTF.transformations.euler_from_quaternion([self.imu.orientation.x, self.imu.orientation.y, self.imu.orientation.z, self.imu.orientation.w])

		## update variables
		self.world_X_base.es.wp = self.world_X_base.ds.wp - self.world_X_base.cs.wp 

	def generateSpline(self, leg_no, start, end, qsurface, phase, reflex=False, ctime=2.0):
		j = leg_no
		self.Leg[j].spline = self.bspline.generate_leg_spline(start.flatten(), end.flatten(), qsurface, phase, reflex, ctime)
		self.Leg[j].spline_counter = 1
		self.Leg[j].spline_length  = len(self.Leg[j].spline.time_from_start[0])


class Joint_class:
	#common base class for Joint
	def __init__(self, num):
		self.name 	= np.array([ 'leg_' + str(num) + '_q1', 'leg_' + str(num) + '_q2', 'leg_' + str(num) + '_q3'])
		self.qpc 	= np.zeros(4) 		# joint current position
		self.qpd 	= np.zeros(4)		# joint desired position
		self.qpe 	= np.zeros(4)		# joint position error
		self.qvc	= np.zeros(4)		# joint current velocity
		self.qvd	= np.zeros(4)		# joint desired velocity
		self.qac 	= np.zeros(4) 		# joint current acceleration
		self.qad 	= np.zeros(4) 		# joint desired acceleration
		self.qtc 	= np.zeros(4)		# joint current effort
		self.qtd 	= np.zeros(4)		# joint desired effort

	def update_state(self, jointState):
		self.qpc = np.array(jointState.m_state.actual.positions)
		self.qpe = np.array(jointState.m_state.error.positions)

		self.qvc = np.array(jointState.m_state.actual.velocities)
		self.qtc = np.array(jointState.m_state.actual.effort)

	def state_display(self):
		print 'q position: ', self.qpc[1:4]
		print 'q p  error: ', self.qpe[1:4]
		print 'q velocity: ', self.qvc[1:4]
		print 'q  effort : ', self.qtc[1:4]

class TwistClass:
	def __init__(self):
		# linear components
		self.xp = np.zeros((3,1))
		self.xv = np.zeros((3,1))
		self.xa = np.zeros((3,1))
		# angular components
		self.wp = np.zeros((3,1))
		self.wv = np.zeros((3,1))
		self.wa = np.zeros((3,1))


class FrameClass:
	def __init__(self, stype):
		if (stype == 'twist'):
			self.cs = TwistClass()
			self.ds = TwistClass()
			self.es = TwistClass()
		elif (stype == 'force'):
			self.cs = np.zeros((3,1))
			self.ds = np.zeros((3,1))
			self.es = np.zeros((3,1))

class LegClass:
	#common base class for Leg
	def __init__(self, name):
		self.number = name
		self.Joint 	= Joint_class(self.number)

		# kinematic library functions
		self.KL 	= kdl.corin_kinematics()
		# self.DL 	= kdl.corin_dynamics()

		# transfer phase variables
		self.goal 	= FollowJointTrajectoryGoal()
		self.spline = JointTrajectoryPoint()
		self.spline_counter = 1
		self.spline_length  = 0

		self.feedback_state	= 0 				# 0 = idle, 1 = command received (executing), 2 = command completed 
		
		self.cstate = 0 	# foot contact state: 1 or 0

		# transformation frames
		self.hip_X_ee 	= FrameClass('twist')
		self.base_X_ee 	= FrameClass('twist')
		
		self.foot_XF_ee = FrameClass('force')
		self.hip_XF_ee 	= FrameClass('force')
		self.base_XF_ee	= FrameClass('force')

		self.tf_base_X_hip 	= tf.rotation_matrix(TF_base_X_hip[self.number],Z_AXIS)[:3, :3]

		self.AEP = np.zeros((3,1)) 		# Anterior Extreme Position for leg wrt base frame
		self.PEP = np.zeros((3,1))		# Posterior Extreme Position for leg wrt base frame
		self.REP = np.zeros((3,1)) 		# nominal stance for leg wrt base frame
		
		self.hip_AEP = np.zeros((3,1)) 	# for storing transfer AEP

		self.leg_positions() 			# set rest position of legs

		self.qsurface = np.array([0.,0.,0.]) 		# surface orientation in leg frame, (roll, pitch, yaw)
		self.qsnorm   = np.array([[0.],[0.],[1.]]) 	# surface normal

		self.phase_change = False 		# Flag for enabling trajectory to be generated at transfer
		
		print 'Leg ', self.number , ' initialised'

	## Transformation functions
	def base_X_hip_ee(self, base_X_ee_xp):
		if (self.number < 3):
			hip_X_ee_xp = np.dot( tf.rotation_matrix(-np.pi/2.0,Z_AXIS)[:3, :3], (-FR_base_X_hip[self.number] + base_X_ee_xp))
		elif (self.number >=3):
			hip_X_ee_xp = np.dot( tf.rotation_matrix(np.pi/2.0,Z_AXIS)[:3, :3], (-FR_base_X_hip[self.number] + base_X_ee_xp))
		return hip_X_ee_xp

	def hip_X_base_ee(self, hip_X_ee_xp):
		if (self.number < 3):
			base_X_ee_xp = FR_base_X_hip[self.number] + np.dot( tf.rotation_matrix(np.pi/2.0,Z_AXIS)[:3, :3], hip_X_ee_xp)
		elif (self.number >=3):
			base_X_ee_xp = FR_base_X_hip[self.number] + np.dot( tf.rotation_matrix(-np.pi/2.0,Z_AXIS)[:3, :3], hip_X_ee_xp)
		return base_X_ee_xp

	## Update functions
	def updateJointState(self, jointState, resetState):

		## ********************************	Joint Angle  ******************************** ##
		## Error Compensation
		q_compensated = (jointState[0], jointState[1]+QCOMPENSATION, jointState[2]) 		# offset q2 by 0.01 - gravity

		## current
		self.hip_X_ee.cs.xp  = self.KL.FK(q_compensated)
		self.base_X_ee.cs.xp = self.hip_X_base_ee(self.hip_X_ee.cs.xp)		

		## error
		self.hip_X_ee.es.xp  = self.hip_X_ee.ds.xp  - self.hip_X_ee.cs.xp 
		self.base_X_ee.es.xp = self.base_X_ee.ds.xp - self.base_X_ee.cs.xp

		## check work envelope
		bound_exceed = self.boundary_limit()

		## state reset
		if (resetState):
			self.hip_X_ee.ds.xp  = self.hip_X_ee.cs.xp
			self.base_X_ee.ds.xp = self.base_X_ee.cs.xp
			# if (self.number==0): 
			# 	print 'state reset'
			# 	print 'bcs: ', self.base_X_ee.cs.xp.transpose()
			# 	print 'bds: ', self.base_X_ee.ds.xp.transpose()
			# 	print 'hcs: ', self.hip_X_ee.cs.xp.transpose()
			# 	print 'hds: ', self.hip_X_ee.ds.xp.transpose()

		return bound_exceed

	def updateForceState(self, cState, cForce):
		## ********************************	Contact Force  ******************************** ##
		self.cstate = cState
		self.foot_XF_ee.cs = np.reshape(np.array(cForce),(3,1))

		## TODO: TRANSFORM FROM FOOT FRAME TO HIP, BASE, WORLD FRAME

	def boundary_limit(self):
		bound_violate = False
		BOUND_FACTOR  = 1.8

		# vector to aep and current position wrt nominal point
		vec_nom_X_aep = self.hip_AEP - np.reshape(LEG_STANCE[self.number],(3,1))
		vec_nom_X_ee  = self.hip_X_ee.cs.xp - np.reshape(LEG_STANCE[self.number],(3,1))
		
		# magnitude of current point
		mag_nom_X_ee  = np.dot(vec_nom_X_aep.flatten(), vec_nom_X_ee.flatten())
		
		# angle between hip frame and AEP
		vec_ang = np.arctan2(vec_nom_X_aep.item(1), vec_nom_X_aep.item(0))

		# ellipse boundary
		if (mag_nom_X_ee > 0.):
			try:
				# ellipse major, minor radius, rotation
				a  = np.sqrt(vec_nom_X_aep.item(0)**2 + vec_nom_X_aep.item(1)**2)
				b  = STEP_STROKE/2.
				qr = vec_ang
				x  = vec_nom_X_ee.item(0)
				y  = vec_nom_X_ee.item(1)

				r_state = ((x*np.cos(qr)+y*np.sin(qr))**2)/(a**2) + ((x*np.sin(qr)-y*np.cos(qr))**2)/(b**2)
				
				if (BOUND_FACTOR < r_state):
					bound_violate = True
					# print self.number, ' ellipse boundary violated ', r_state
			except:
				pass

		# circle boundary
		else:
			r_state = (vec_nom_X_ee.item(0)**2 + vec_nom_X_ee.item(1)**2)/(STEP_STROKE/2.)**2
			
			if (BOUND_FACTOR < r_state):
				bound_violate = True
				# print self.number, ' circle boundary violated ', r_state
		
		return bound_violate

	## Kinematic functions
	def getJointKinematic(self,velocity):
		self.Joint.qpd = self.KL.IK(self.hip_X_ee.ds.xp)
		
		# checks if there's need to compute joint velocity
		if (not self.KL.singularity_check(self.Joint.qpd)):
			if (velocity==True):
				self.Joint.qvd, self.Joint.qad = self.KL.joint_speed(self.Joint.qpd, self.hip_X_ee.ds.xv, self.hip_X_ee.ds.xa)
			return True
		else:
			print 'Singularity detected'
			return False

	## Convenience functions
	def pointToArray(self):
		# if (self.number==0): print 'spline count: ', self.spline_counter, '\t', self.spline_length
		i = self.spline_counter
		self.hip_X_ee.ds.xp = np.array([self.spline.positions[0][i],  self.spline.positions[1][i],  self.spline.positions[2][i]])
		self.hip_X_ee.ds.xv = np.array([self.spline.velocities[0][i], self.spline.velocities[1][i], self.spline.velocities[2][i]])
		self.hip_X_ee.ds.xa = np.array([self.spline.accelerations[0][i], self.spline.accelerations[1][i], self.spline.accelerations[2][i]])

	## Configure leg positions
	def leg_positions(self):
		if (self.number < 3):
			self.REP = FR_base_X_hip[self.number] + np.dot( tf.rotation_matrix(np.pi/2.0,Z_AXIS)[:3, :3], np.reshape(LEG_STANCE[self.number],(3,1)) )

		elif (self.number >=3):
			self.REP = FR_base_X_hip[self.number] + np.dot( tf.rotation_matrix(-np.pi/2.0,Z_AXIS)[:3, :3], np.reshape(LEG_STANCE[self.number],(3,1)) )

	def update_REP(self, bodypose, base_X_surface):
		
		# update only if surface orientation above deadzone
		if (abs(self.qsurface.item(0)) > QDEADZONE or abs(self.qsurface.item(1)) > QDEADZONE):
			self.REP = FR_base_X_hip[self.number] + self.KL.nominal_stance(bodypose, base_X_surface, self.qsurface)
			# print 'updating REP for leg ', self.number
			# if (self.number == 0):
			