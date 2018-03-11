#!/usr/bin/env python

## Main controller for the robot
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'library'))
sys.dont_write_bytecode = True

import time
import warnings
import numpy as np

## Personal libraries
from constant import *
import control_interface 					# action selection from ROS parameter server
import robot_class 							# class for robot states and function
import gait_class as Gaitgenerator			# class for gait coordinator
import path_generator as Pathgenerator 		# generates path from via points
import transformations as tf 				# SE(3) transformation library
import plotgraph as Plot 					# library for plotting 
import TrajectoryPoints

## ROS messages
import rospy
from sensor_msgs.msg import Imu 						# sub msg for IMU
from sensor_msgs.msg import JointState 					# sub msg for joint states
from trajectory_msgs.msg import JointTrajectoryPoint	# pub msg for Robotis joints
from std_msgs.msg import Float64 						# pub msg for Gazebo joints
from std_msgs.msg import ByteMultiArray 				# foot contact state
from std_msgs.msg import Float32MultiArray				# foot contact force

## Robotis ROS msgs for joint control
from robotis_controller_msgs.msg import SyncWriteMultiFloat

class CorinManager:
	def __init__(self, initialise):
		rospy.init_node('main_controller') 		# Initialises node
		self.robot_ns 	= ROBOT_NS 				# Robot namespace
		self.rate 	  	= rospy.Rate(CTR_RATE)	# Controller rate

		self.Robot 		= robot_class.RobotState() 					# robot class
		self.Action		= control_interface.control_interface()		# control action class

		self.interface 	= self.interface_selection()		# interface to control: simulation or Robotis hardware

		## TODO: think about how to proceess this
		self.lin_v = np.zeros(3) 			# linear velocity vector
		self.ang_v = np.zeros(3) 			# angular velocity vector
		self.ang_p = np.zeros(3) 			# angular position vector
		# self.snorm = np.array([0.,0.,1.])	# surface normal

		self.point 	   	= JointTrajectoryPoint() 		# for logging and assist publishing trajectory points
		self.sp_state  	= JointState() 					# LOGGING

		self.resting  	= False 	# Flag indicating robot standing or resting
		self.on_start 	= False 	# variable for resetting to leg suspended in air

		if (initialise):
			self._start() 				# initialise publishers and subscribers
			rospy.sleep(0.5)
			if rospy.has_param(ROBOT_NS + '/standing'):
				if (rospy.get_param(ROBOT_NS + '/standing')==True):
					pass
			else:
				self._default_pose() 		# move robot to default position

			# update and reset states
			self.Robot.reset_state = True
			self.Robot.suspend = False
			self.Robot.updateState()

			for j in range(0,self.Robot.active_legs):
				self.Robot.Leg[j].feedback_state = 2 	# trajectory completed

	def interface_selection(self):
		""" Identifies interface to control  				"""
		"""	Output: interface: simulation or robotis motors """
		return 'simulation' if rospy.has_param('/gazebo/auto_disable_bodies') else 'robotis'

	def joint_state_callback(self, msg): 		# robot joint state callback
		self.Robot.qc = msg

	def imu_callback(self, msg): 				# imu callback
		self.Robot.imu = msg

	def contact_state_callback(self, msg): 		# foot contact binary state
		self.Robot.cstate = msg.data

	def contact_force_callback(self, msg): 		# foot contact force
		self.Robot.cforce = msg.data

	def _start(self):
		#######################################
		## initialise publishers/subscribers ##
		#######################################

		##***************** PUBLISHERS ***************##
		self.sp_pub_ = rospy.Publisher('/corin/setpoint_states', JointState, queue_size=1)	# LOGGING publisher

		## Hardware Specific Publishers ##
		if (self.robot_ns == 'corin' and (self.interface == 'simulation' or self.interface == 'robotis')):
			## Publish to Gazebo
			if (self.interface == 'simulation'):
				self.qpub_ = {}
				for i in range(0,18):
					self.qpub_[i] = rospy.Publisher(self.robot_ns + '/' + JOINT_NAME[i] + '/command', Float64, queue_size=1)

			## Publish to Dynamixel motors
			elif (self.interface == 'robotis'):
				self.sync_qpub_  = rospy.Publisher('/robotis/sync_write_multi_float', SyncWriteMultiFloat, queue_size=1)

			print ">> INITIALISED JOINT TOPICS"
		else:
			# shutdown if wrong hardware selected
			rospy.logerr("INVALID HARDWARE INTERFACE SELECTED!")
			rospy.signal_shutdown("INVALID HARDWARE INTERFACE SELECTED - SHUTTING DOWN")

		##***************** SUBSCRIBERS ***************##
		# self.imu_sub_	 = rospy.Subscriber(self.robot_ns + '/imu/trunk/data', Imu, self.imu_callback, queue_size=1)
		self.cstate_sub_ = rospy.Subscriber(self.robot_ns + '/contact_state', ByteMultiArray, self.contact_state_callback, queue_size=1)
		self.cforce_sub_ = rospy.Subscriber(self.robot_ns + '/contact_force', Float32MultiArray, self.contact_force_callback, queue_size=1)

		## Hardware Specific Subscribers ##
		if (self.interface == 'simulation'):
			self.joint_sub_  = rospy.Subscriber(self.robot_ns + '/joint_states', JointState, self.joint_state_callback, queue_size=5)
		elif (self.interface == 'robotis'):
			self.joint_sub_  = rospy.Subscriber('robotis/present_joint_states', JointState, self.joint_state_callback, queue_size=5)

	# routine to stand up: using current state move leg up to air, then step up
	def _air_suspend_legs(self):
		self.Robot.updateState() 		# get current state

		# Variables
		fix_stance = 0.18;
		fix_height = 0.01
		leg_stance = {}
		# stance for air suspension
		leg_stance[0] = np.array([ fix_stance*np.cos(TETA_F*np.pi/180), fix_stance*np.sin(TETA_F*np.pi/180), fix_height ])
		leg_stance[1] = np.array([ fix_stance, 0, fix_height])
		leg_stance[2] = np.array([ fix_stance*np.cos(TETA_R*np.pi/180), fix_stance*np.sin(TETA_R*np.pi/180), fix_height ])

		leg_stance[3] = np.array([ fix_stance*np.cos(TETA_F*np.pi/180), fix_stance*np.sin(-TETA_F*np.pi/180), fix_height ])
		leg_stance[4] = np.array([ fix_stance, 0, fix_height])
		leg_stance[5] = np.array([ fix_stance*np.cos(TETA_R*np.pi/180), fix_stance*np.sin(-TETA_R*np.pi/180), fix_height ])

		# generate spline to legs up position
		for j in range(0,self.Robot.active_legs):
			# self.Robot.Leg[j].hip_X_ee.ds.xp = LEG_STANCE[j] + np.array([0.0,0.0,BODY_HEIGHT+fix_height]) 	# put leg on final ground position first
			self.Robot.Leg[j].hip_X_ee.ds.xp = leg_stance[j] 	# place leg in fixed position in air
			self.Robot.generateSpline(j, self.Robot.Leg[j].hip_X_ee.cs.xp, self.Robot.Leg[j].hip_X_ee.ds.xp,
											self.Robot.Leg[j].qsurface, 1, False, TRAC_PERIOD)
			
		# move leg into position
		for npoint in range(0, int(TRAC_PERIOD/CTR_INTV)):
			for j in range(0,self.Robot.active_legs):
				# set cartesian position for joint kinematics
				self.Robot.Leg[j].pointToArray();
				self.Robot.Leg[j].spline_counter += 1
				self.task_X_joint(j)

			# publish appended joint angles if motion valid
			self.publish_topics()
			self.rate.sleep()

	def _default_pose(self, stand_state=0):

		## moves leg into air
		if (self.on_start == False):
			print 'Resetting stance....'
			self._air_suspend_legs()
			self.on_start = True

		rospy.sleep(0.5)
		# get current state
		for i in range(0,3):
			self.Robot.updateState()

		print 'Moving to nominal stance....'

		# generate spline to standup position
		for j in range(0,self.Robot.active_legs):
			self.Robot.Leg[j].hip_X_ee.ds.xp = LEG_STANCE[j]
			self.Robot.generateSpline(j, self.Robot.Leg[j].hip_X_ee.cs.xp, self.Robot.Leg[j].hip_X_ee.ds.xp,
											self.Robot.Leg[j].qsurface, stand_state, False, TRAC_PERIOD)

		## move leg into position
		# already in stand up position, shuffle in tripod fashion
		if (stand_state):
			for npoint in range(0, int(TRAC_PERIOD/CTR_INTV)):
				for j in range(0,self.Robot.active_legs):
					# set cartesian position for joint kinematics
					if (j%2 == 0): 	# even number legs
						self.Robot.Leg[j].pointToArray();
						self.Robot.Leg[j].spline_counter += 1
						self.task_X_joint(j)
					else: 			# odd number legs
						self.Robot.Leg[j].hip_X_ee.ds.xp = self.Robot.Leg[j].hip_X_ee.cs.xp
						self.task_X_joint(j)

				self.publish_topics()
				self.rate.sleep()

			self.Robot.updateState()
			for npoint in range(0, int(TRAC_PERIOD/CTR_INTV)):
				for j in range(0,self.Robot.active_legs):
					# set cartesian position for joint kinematics
					if (j%2 == 1): 	# even number legs
						self.Robot.Leg[j].pointToArray();
						self.Robot.Leg[j].spline_counter += 1
						self.task_X_joint(j)
					else: 			# odd number legs
						self.Robot.Leg[j].hip_X_ee.ds.xp = self.Robot.Leg[j].hip_X_ee.cs.xp
						self.task_X_joint(j)

				self.publish_topics()
				self.rate.sleep()

		## belly on ground, move all together
		else:
			for npoint in range(0, int(TRAC_PERIOD/CTR_INTV)):
				for j in range(0,self.Robot.active_legs):
					# set cartesian position for joint kinematics
					self.Robot.Leg[j].pointToArray();
					self.Robot.Leg[j].spline_counter += 1
					self.task_X_joint(j)

				# publish appended joint angles if motion valid
				self.publish_topics()
				self.rate.sleep()

	def _Get_to_Stance(self):
		dir_uv = np.array([0.,0.,0.]) 	# zero vector direction
		spline_count = 0 				# current spline count for transfer phase legs
		self.Robot.suspend = True 		# suspend robot for zero trunk movement

		# Get leg phases
		for j in range(0,6):
			# check leg transfer status, if true
			if (self.Robot.gaitgen.cs[j] == 1):
				# get number of points yet to execute
				spline_count   = self.Robot.Leg[j].spline_length - self.Robot.Leg[j].spline_counter

		# loop only required via points
		for sc in range(0, spline_count):
			# send command to robot
			self.execution(dir_uv)
			rospy.sleep(CTR_INTV)

	def task_X_joint(self, j):
		## Leg task to joint space
		try:
			if (self.Robot.Leg[j].getJointKinematic(True)):
				for z in range(0,3):
					self.point.positions.append( self.Robot.Leg[j].Joint.qpd.item(z) )
					self.point.velocities.append( self.Robot.Leg[j].Joint.qvd.item(z) )
					self.point.accelerations.append( self.Robot.Leg[j].Joint.qad.item(z) )
					self.point.effort.append( 0.0 )
			else:
				self.Robot.invalid = True
				raise ValueError
		except Exception, e:
			print 'Error - singularity in leg, ', j

	def clear_point(self):
		self.point = JointTrajectoryPoint()

	def publish_topics(self):
		""" Publish joint position to joint controller topics and
			setpoint topic (for logging all setpoint states) 		"""

		## Publish joint position to robot	
		if (not self.Robot.invalid):
			self.Robot.active_legs = 6

			if (self.interface == 'simulation'):
				for n in range(0,self.Robot.active_legs*3):
					qp = Float64()
					qp.data = self.point.positions[n]
					self.qpub_[n].publish(qp)

					self.sp_state.position.append(self.point.positions[n]) 	# LOGGING
					self.sp_state.velocity.append(self.point.velocities[n])	# LOGGING

			elif (self.interface == 'robotis'):
				dqp = SyncWriteMultiFloat()
				dqp.item_name = str("goal_position") 	# goal_position
				dqp.data_length = 4

				for n in range(0,self.Robot.active_legs*3): 	# loop for each joint
					dqp.joint_name.append(str(JOINT_NAME[n])) 	# joint name
					dqp.value.append(self.point.positions[n])	

					self.sp_state.position.append(self.point.positions[n]) 	# LOGGING
					self.sp_state.velocity.append(self.point.velocities[n])	# LOGGING

				self.sync_qpub_.publish(dqp)

		## Publish setpoints to logging topic
		self.sp_pub_.publish(self.sp_state)
		self.clear_point()

	def alternate_phase(self):
		# change gait phase
		self.Robot.gaitgen.change_phase()

		# update robot leg phase_change
		for i in range(0,6):
			if (self.Robot.gaitgen.cs[i] == 1):
				self.Robot.Leg[i].phase_change = False

		self.Robot.suspend = False


	def execution(self, dir_uv, i=0):
		"""  """

		## update robot state
		self.Robot.updateState()
		self.Robot.world_X_base.cs.wp = self.Robot.world_X_base.ds.wp

		## task space mapping (change in variable name for convenience)
		cv = self.lin_v 	# linear velocity
		wv = self.ang_v		# angular velocity
		wp = self.ang_p		# angular orientation

		## Update robot's desired pose
		self.Robot.fr_world_X_base 	  = tf.rotation_zyx(wp)
		self.Robot.world_X_base.ds.wp = wp

		## generate trajectory for leg in transfer phase
		for j in range (0, self.Robot.active_legs):

			# transfer phase
			if (self.Robot.gaitgen.cs[j] == 1 and self.Robot.Leg[j].phase_change==False):

				self.Robot.Leg[j].feedback_state = 1 	# leg put into execution mode

				## 'Foothold Selection' Algorithm: 1) Update REP 2) Get_to_Stance

				# Update REP - MOCKED DATASET HERE
				base_X_surface = 0.32
				bodypose = np.array([0.,0.,BODY_HEIGHT, 0.,0.,0.])
				self.Robot.Leg[j].update_REP(bodypose, base_X_surface)

				# change in CoB in next phase
				e_z = tf.SO3_selection(np.array([0.,0.,1.]) , 'z')

				# unstack next CoB location, determine foot z position in that CoB location
				try:
					nex_com = self.Robot.x_spline.xp[int(i+TRAC_PERIOD/CTR_INTV)]
				except:
					nex_com = self.Robot.x_spline.xp[-1]

				## calculate AEP with CoB height change
				# condition occurs when leg against wall
				if (self.Robot.Leg[j].REP.item(2) > 0.):
					# get current CoB
					cur_com = self.Robot.x_spline.xp[int(i)]
					# change in vertical height between current and next CoB, in R3 representation
					d_h = np.dot(e_z, (nex_com - cur_com).reshape(3,1) )
					self.Robot.Leg[j].AEP = (self.Robot.Leg[j].REP + dir_uv*STEP_STROKE/2. + d_h)

				# normal walking condition
				else:
					# change in vertical height between next CoB and REP, in R3 representation
					d_h = np.dot(e_z, nex_com.reshape(3,1) + self.Robot.Leg[j].REP)
					self.Robot.Leg[j].AEP = (self.Robot.Leg[j].REP + dir_uv*STEP_STROKE/2. - d_h)

				self.Robot.Leg[j].base_X_ee.ds.xp = np.dot(tf.rotation_zyx(-1.*self.Robot.world_X_base.cs.wp), self.Robot.Leg[j].AEP)

				# Base to hip frame conversion
				self.Robot.Leg[j].hip_X_ee.ds.xp = self.Robot.Leg[j].base_X_hip_ee(self.Robot.Leg[j].base_X_ee.ds.xp)
				self.Robot.Leg[j].hip_AEP = self.Robot.Leg[j].hip_X_ee.ds.xp 											# set only during transfer phase

				## generate spline for transfer phase leg
				self.Robot.generateSpline(j, self.Robot.Leg[j].hip_X_ee.cs.xp, self.Robot.Leg[j].hip_X_ee.ds.xp,
											self.Robot.Leg[j].qsurface, 1, False, TRAC_PERIOD, CTR_INTV)

				# set flag that phase has changed
				self.Robot.Leg[j].phase_change = True


		## trajectory points for all legs
		for j in range (0, self.Robot.active_legs):

			# transfer phase
			if (self.Robot.gaitgen.cs[j] == 1 and self.Robot.Leg[j].feedback_state==1):

				# unstack next point in individual leg spline to be processed
				try:
					self.Robot.Leg[j].pointToArray()
					self.Robot.Leg[j].spline_counter += 1
				except:
					self.Robot.Leg[j].feedback_state = 2 	# trajectory completed
					self.Robot.suspend = True 				# stops robot from moving i.e. stops support stance until rest of legs finish
					pass

			## support phase
			elif (self.Robot.gaitgen.cs[j] == 0 and self.Robot.suspend == False):
				## Ideal trajectory: doesn't use current leg states
				# prob: IMU feedback would cause change here, increase leg height etc
				# prob: use leg current state, but shouldn't drift

				## determine foot velocity using CoB velocity; world_X_base cause rotation
				self.Robot.Leg[j].base_X_ee.ds.xv = np.dot( tf.rotation_zyx(self.Robot.world_X_base.cs.wp).transpose() , \
									-( cv-np.cross((np.dot(tf.rotation_zyx(self.Robot.world_X_base.cs.wp), self.Robot.Leg[j].base_X_ee.ds.xp)).transpose(),wv) ).transpose() )

				## base_X_feet position: integrate base_X_feet velocity
				self.Robot.Leg[j].base_X_ee.ds.xp = self.Robot.Leg[j].base_X_ee.ds.xv*CTR_INTV + self.Robot.Leg[j].base_X_ee.ds.xp #+ self.Robot.Leg[j].base_X_ee.es.xp

				## base to hip transform
				self.Robot.Leg[j].hip_X_ee.ds.xp = self.Robot.Leg[j].base_X_hip_ee(self.Robot.Leg[j].base_X_ee.ds.xp)
				self.Robot.Leg[j].hip_X_ee.ds.xv = np.dot(self.Robot.Leg[j].tf_base_X_hip, self.Robot.Leg[j].base_X_ee.ds.xv)

				# if (j==0):
				# 	print 'bds: ', np.round(self.Robot.Leg[j].base_X_ee.ds.xp.flatten(),4)
				# 	print 'bcs: ', np.round(self.Robot.Leg[j].base_X_ee.cs.xp.flatten(),4)
				# 	print 'hds: ', np.round(self.Robot.Leg[j].hip_X_ee.ds.xp.flatten(),4)
				# 	print 'hcs: ', np.round(self.Robot.Leg[j].hip_X_ee.cs.xp.flatten(),4)
			
			## Task to joint space
			self.task_X_joint(j)

		if (self.Robot.invalid == True):
			print 'Error Occured'
			# raw_input('Error occured')

		# ends gait phase early if transfer phase completes
		transfer_total = 0; 	leg_complete   = 0;

		for j in range(0,6):
			if (self.Robot.gaitgen.cs[j]==1):
				transfer_total += 1
				if (self.Robot.Leg[j].feedback_state == 2):
					leg_complete += 1

		# triggers only when all transfer phase legs complete and greater than zero
		# > 0: prevents trigger during all leg support
		if (transfer_total == leg_complete and transfer_total > 0 and leg_complete > 0):
			self.alternate_phase()

		# publish appended joint angles if motion valid
		self.publish_topics()
		# print '--------------------------------'

	def trajectory_tracking(self, x_cob, w_cob=0):
		""" Generates body trajectory from given via points and
			cycles through the trajectory 							"""
		""" Input: 	1) x_com -> 2D array of linear positions
					2) w_com -> 2D array of angular orientations
			Output: None											"""

		## define variables ##
		PathGenerator = Pathgenerator.PathGenerator() 	# path generator for robot's base
		cob_X_desired = np.zeros(3) 	# cob linear location
		cob_W_desired = np.zeros(3) 	# cob angular location

		# produce interpolation points
		# self.Robot.x_spline, self.Robot.w_spline = PathGenerator.generate_path(x_cob, w_cob, CTR_INTV)
		coba = PathGenerator.generate_path(x_cob, w_cob, CTR_INTV)
		self.Robot.x_spline = coba.X 	# TODO: change this
		# Plot.plot_2d(self.Robot.x_spline.t, self.Robot.x_spline.xp)
		
		# Set all legs to support mode for bodyposing, prevent AEP from being set
		if (self.Robot.support_mode == True):
			for j in range(0,6):
				self.Robot.gaitgen.cs[j] = 0

		# cycle through trajectory points until complete
		i = 0
		while (i != len(coba.X.t) and not rospy.is_shutdown()):
			print 'counting: ', i, len(coba.X.t)

			## suppress trajectory counter as body support suspended
			if (self.Robot.suspend == True):
				i -= 1

			## Extract linear task space position, velocity, acceleration
			## TODO: all six instances required by execution
			cp 			= coba.X.xp[i]
			self.lin_v 	= coba.X.xv[i]
			ca 			= coba.X.xa[i]
			ct 			= coba.X.t[i]
			
			self.ang_p 	= coba.W.xp[i]
			self.ang_v	= coba.W.xv[i]

			## LOGGING: initialise variable and set bodypose ##
			self.sp_state = JointState()
			for j in range(6):
				self.sp_state.name.append(ROBOT_STATE[j])
				if (j<3):
					self.sp_state.position.append(coba.X.xp[i].item(j))
				else:
					self.sp_state.position.append(coba.W.xp[i].item(j-3))
			for j in range(0,18):
				self.sp_state.name.append(JOINT_NAME[j])

			# horizontal plane instantenous velocity magnitude
			mag_v  = np.sqrt(self.lin_v.item(0)**2 + self.lin_v.item(1)**2)

			# horizontal plane: unit vector
			dir_uv = np.nan_to_num(np.array([ [self.lin_v.item(0)/mag_v], [self.lin_v.item(1)/mag_v], [0.0] ]))
			
			# Tracking desired translations - TODO: include orientation 
			cob_X_desired += self.lin_v*CTR_INTV
			cob_W_desired += self.ang_v*CTR_INTV

			# Next CoB point
			# if (self.Robot.x_spline.t[i] == PathGenerator.t_com[PathGenerator.count]):
			# 	PathGenerator.count += 1

			# send command to body controller
			self.execution(dir_uv, i)
			self.rate.sleep()

			i += 1

		print 'Trajectory executed'
		print 'Desired Goal: ', coba.X.xp[-1]
		print 'Tracked Goal: ', cob_X_desired
		self._Get_to_Stance() 				# put transfer legs onto ground

	def action_interface(self):
		""" Interface for commanding robot """

		self.Robot.updateState()
		# print 'ds: ', np.round(self.Robot.Leg[0].hip_X_ee.ds.xp.flatten(),3)
		# print 'cs: ', np.round(self.Robot.Leg[0].hip_X_ee.cs.xp.flatten(),3)
		
		## Define variables
		data = None; x_cob = None; w_cob = None; mode = None;

		## check action to take
		data = self.Action.action_to_take()

		if (data is not None):
			## data mapping - for convenience
			x_cob, w_cob, mode = data

			## stand up if at rest
			if ( (mode == 1 or mode == 2) and self.resting == True):
				print 'Going to standup'
				self._default_pose()

			## condition for support (1), walk (2), reset (3)
			if (mode == 1):
				print 'support mode'
				self.Robot.reset_state = True
				self.Robot.suspend = False 		# clear suspension flag
				self.Robot.support_mode = True

				self.trajectory_tracking(x_cob, w_cob)
				# self._default_pose()
				# self._default_pose(1) 	# required for resetting stance for walking

			elif (mode == 2):
				print 'walk mode'
				self.Robot.support_mode = False

				self.alternate_phase()
				self.trajectory_tracking(x_cob, w_cob)

			elif (mode == 3):
				if (self.resting == False): 	# rest robot
					print 'Rest'
					self._air_suspend_legs()
					self.resting = True
				elif (self.resting == True): 	# standup from rest
					print 'Standup'
					self._default_pose()
					self.resting = False

		else:
			# print 'standy'
			# print self.Robot.cforce[0], self.Robot.cforce[1], self.Robot.cforce[2]
			# print self.Robot.Leg[0].hip_XF_ee.cs
			rospy.sleep(0.5)
