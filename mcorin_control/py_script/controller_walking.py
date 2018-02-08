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
import control_interface 								# action selection from ROS parameter server
import robot_class 										# class for robot states and function
import gait_class as Gaitgenerator						# class for gait coordinator
import path_generator as Pathgenerator 					# generates path from via points
import transformations as tf 							# SE(3) transformation library
# import pspline_generator as Pspline 					# polynomial spline generator 
# import bspline_generator as Bspline 					# basis spline generator
# import param_gait										# class for setting gait parameters in RT 
import plotgraph as Plot 								# library for plotting 

## ROS messages
import rospy
from sensor_msgs.msg import Imu 						# sub msg for IMU
from sensor_msgs.msg import JointState 					# sub msg for joint states
from trajectory_msgs.msg import JointTrajectoryPoint	# pub msg for Robotis joints
from std_msgs.msg import Float64 						# pub msg for Gazebo joints
from std_msgs.msg import ByteMultiArray 				# foot contact state
from std_msgs.msg import Float32MultiArray				# foot contact force

## Robotis ROS msgs for joint control
from robotis_controller_msgs.msg import SyncWriteItem 	# pub msg for Robotis joints
from robotis_controller_msgs.msg import SyncWriteMulti 	# pub msg for Robotis joints

## Operator control files
from sensor_msgs.msg import Joy
import connex

def rad2raw(radian):
	## convert joint position to dxl raw value
	value_of_0_radian_position_      = 2048
	value_of_min_radian_position_    = 0
	value_of_max_radian_position_    = 4095
	min_radian_                      = -3.14159265
	max_radian_                      =  3.14159265

	if (radian > 0):
		value = (radian * (value_of_max_radian_position_ - value_of_0_radian_position_) / max_radian_) + value_of_0_radian_position_;
	elif (radian < 0):
		value = (radian * (value_of_min_radian_position_ - value_of_0_radian_position_) / min_radian_) + value_of_0_radian_position_;
	else:
		value = 2048;

	return int(value)

class CorinManager:
	def __init__(self, initialise):
		rospy.init_node('main_controller') 		#Initialises node
		self.robot_ns = ROBOT_NS
		self.interval = TRAC_INTERVAL
		self.rate 	  = rospy.Rate(1.0/TRAC_INTERVAL)	# frequency

		self.cp = 0.0						# magnitude of linear translation in horizontal plane

		self.lin_v = np.zeros(3) 			# linear velocity vector
		self.ang_v = np.zeros(3) 			# angular velocity vector
		self.ang_p = np.zeros(3) 			# angular position vector
		# self.snorm = np.array([0.,0.,1.])	# surface normal

		self.h_v  = np.zeros(2)				# linear translation vector in horizontal plane
		self.zc_v = np.zeros(2) 			# zero crossing vector

		self.Robot 	  = robot_class.RobotState() 		# robot class
		self.planner  = Pathgenerator.PathGenerator() 	# trajectory following
		# self.b_spline = Bspline.SplineGenerator() 	# B-spline class
		# self.p_spline = Pspline.SplineGenerator() 	# polynomial spline class
		# self.connex   = connex.Connexion() 			# connex hardware controller

		self.control_mode = 1 							# control mode: 1-autonomous, 0-manual control
		self.hardware 	  = self.hardware_selection()	# returns interface to simulation or Robotis hardware

		self.point 	   = JointTrajectoryPoint() 		# for logging and assist publishing trajectory points

		self.sp_state  = JointState() 					# LOGGING
		self.track_com = np.zeros(3) 					# tracking com location

		self.resting = False 			# Flag indicating robot standing or resting
		self.ui_control	= control_interface.control_interface()

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

	def hardware_selection(self):
		# always default to simulation
		if rospy.has_param('/gazebo/auto_disable_bodies'):
			return 'simulation'
		else:
			return 'robotis'

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

		if (self.robot_ns == 'corin' and (self.hardware == 'simulation' or self.hardware == 'robotis')):
			## Publish to Gazebo
			if (self.hardware == 'simulation'):
				self.qpub_ = {}
				for i in range(0,18):
					self.qpub_[i] = rospy.Publisher(self.robot_ns + '/' + JOINT_NAME[i] + '/command', Float64, queue_size=1)

			## Publish to Dynamixel motors
			elif (self.hardware == 'robotis'):
				self.sync_qpub_  = rospy.Publisher('/robotis/sync_write_multi', SyncWriteMulti, queue_size=1)

			print ">> INITIALISED JOINT TOPICS"
		else:
			# shutdown if wrong hardware selected
			rospy.logerr("INVALID HARDWARE INTERFACE SELECTED!")
			rospy.signal_shutdown("INVALID HARDWARE INTERFACE SELECTED - SHUTTING DOWN")

		##***************** SUBSCRIBERS ***************##
		if (self.hardware == 'simulation'):
			self.joint_sub_  = rospy.Subscriber(self.robot_ns + '/joint_states', JointState, self.joint_state_callback, queue_size=5)
		elif (self.hardware == 'robotis'):
			self.joint_sub_  = rospy.Subscriber('robotis/present_joint_states', JointState, self.joint_state_callback, queue_size=5)
		# self.imu_sub_	 = rospy.Subscriber(self.robot_ns + '/imu/trunk/data', Imu, self.imu_callback, queue_size=1)
		self.cstate_sub_ = rospy.Subscriber(self.robot_ns + '/contact_state', ByteMultiArray, self.contact_state_callback, queue_size=1)
		self.cforce_sub_ = rospy.Subscriber(self.robot_ns + '/contact_force', Float32MultiArray, self.contact_force_callback, queue_size=1)

		# if (self.control_mode == 0): 	# manual control mode
		# 	self.connex_sub = rospy.Subscriber('/spacenav/joy', Joy, self.connex_callback, queue_size=1 )

		self.on_start = False 	# variable for resetting to leg suspended in air

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
		leg_stance[4] = np.array([fix_stance, 0, fix_height])
		leg_stance[5] = np.array([ fix_stance*np.cos(TETA_R*np.pi/180), fix_stance*np.sin(-TETA_R*np.pi/180), fix_height ])

		# generate spline to legs up position
		for j in range(0,self.Robot.active_legs):
			# self.Robot.Leg[j].hip_X_ee.ds.xp = LEG_STANCE[j] + np.array([0.0,0.0,BODY_HEIGHT+fix_height]) 	# put leg on final ground position first
			self.Robot.Leg[j].hip_X_ee.ds.xp = leg_stance[j] 	# place leg in fixed position in air
			self.Robot.generateSpline(j, self.Robot.Leg[j].hip_X_ee.cs.xp, self.Robot.Leg[j].hip_X_ee.ds.xp,
											self.Robot.Leg[j].qsurface, 1, False, TRAC_PERIOD)
			
		# move leg into position
		for npoint in range(0, int(TRAC_PERIOD/TRAC_INTERVAL)):
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
			for npoint in range(0, int(TRAC_PERIOD/TRAC_INTERVAL)):
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
			for npoint in range(0, int(TRAC_PERIOD/TRAC_INTERVAL)):
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
			for npoint in range(0, int(TRAC_PERIOD/TRAC_INTERVAL)):
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
		mag_v  = 0.1 					# fake for robot to execute
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
			self.execution(mag_v, dir_uv)
			rospy.sleep(self.interval)

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
		if (not self.Robot.invalid):
			self.Robot.active_legs = 6

			if (self.hardware == 'simulation'):
				for n in range(0,self.Robot.active_legs*3):
					qp = Float64()
					qp.data = self.point.positions[n]
					self.qpub_[n].publish(qp)

					self.sp_state.position.append(self.point.positions[n]) 	# LOGGING
					self.sp_state.velocity.append(self.point.velocities[n])	# LOGGING

			elif (self.hardware == 'robotis'):
				dqp = SyncWriteMulti()
				dqp.item_name = str("goal_position") 	# goal_position
				dqp.data_length = 4

				for n in range(0,self.Robot.active_legs*3): 	# loop for each joint
					dqp.joint_name.append(str(JOINT_NAME[n])) 				# joint name
					dqp.value.append(rad2raw(self.point.positions[n]))		# position - convert to raw data

					self.sp_state.position.append(self.point.positions[n]) 	# LOGGING
					self.sp_state.velocity.append(self.point.velocities[n])	# LOGGING

				self.sync_qpub_.publish(dqp)

			# print '==================================='
		
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


	def execution(self, mag_v, dir_uv, i=0):
		## update robot state
		self.Robot.updateState()
		self.Robot.world_X_base.cs.wp = self.Robot.world_X_base.ds.wp

		## Execute when there is change i.e. instanteneous velocity != 0
		if (mag_v > 0):

			## Determine time interval
			interval = self.interval

			## task space mapping (change in variable name for convenience)
			cv = self.lin_v
			wv = self.ang_v
			wp = self.ang_p

			## Update robot's desired pose
			self.Robot.fr_world_X_base 	  = tf.rotation_zyx(wp)
			self.Robot.world_X_base.ds.wp = wp

			## generate trajectory for leg in transfer phase
			for j in range (0, self.Robot.active_legs):

				# transfer phase
				if (self.Robot.gaitgen.cs[j] == 1 and self.Robot.Leg[j].phase_change==False):

					self.Robot.Leg[j].feedback_state = 1 	# leg put into execution mode

					## Operator mode: determine next foothold location base on forward/backwards direction of motion
					if (self.control_mode == 0):
						self.Robot.Leg[j].AEP = (self.Robot.Leg[j].REP + dir_uv*STEP_STROKE/2.)
						self.Robot.Leg[j].base_X_ee.ds.xp = np.dot(tf.rotation_zyx(-1.*self.Robot.world_X_base.cs.wp), self.Robot.Leg[j].AEP)

					## Autonomous mode
					elif (self.control_mode == 1):
						# 'Foothold Selection' Algorithm: 1) Update REP 2) Get_to_Stance

						# Update REP - MOCKED DATASET HERE
						base_X_surface = 0.32
						bodypose = np.array([0.,0.,BODY_HEIGHT, 0.,0.,0.])
						self.Robot.Leg[j].update_REP(bodypose, base_X_surface)

						# change in CoM in next phase
						e_z = tf.SO3_selection(np.array([0.,0.,1.]) , 'z')

						# unstack next CoM location, determine foot z position in that CoM location
						try:
							nex_com = self.Robot.x_spline.xp[int(i+TRAC_PERIOD/TRAC_INTERVAL)]
						except:
							nex_com = self.Robot.x_spline.xp[-1]

						## calculate AEP with CoM height change
						# condition occurs when leg against wall
						if (self.Robot.Leg[j].REP.item(2) > 0.):
							# get current CoM
							cur_com = self.Robot.x_spline.xp[int(i)]
							# change in vertical height between current and next CoM, in R3 representation
							d_h = np.dot(e_z, (nex_com - cur_com).reshape(3,1) )
							self.Robot.Leg[j].AEP = (self.Robot.Leg[j].REP + dir_uv*STEP_STROKE/2. + d_h)

						# normal walking condition
						else:
							# change in vertical height between next CoM and REP, in R3 representation
							d_h = np.dot(e_z, nex_com.reshape(3,1) + self.Robot.Leg[j].REP)
							self.Robot.Leg[j].AEP = (self.Robot.Leg[j].REP + dir_uv*STEP_STROKE/2. - d_h)

						self.Robot.Leg[j].base_X_ee.ds.xp = np.dot(tf.rotation_zyx(-1.*self.Robot.world_X_base.cs.wp), self.Robot.Leg[j].AEP)

					# Base to hip frame conversion
					self.Robot.Leg[j].hip_X_ee.ds.xp = self.Robot.Leg[j].base_X_hip_ee(self.Robot.Leg[j].base_X_ee.ds.xp)
					self.Robot.Leg[j].hip_AEP = self.Robot.Leg[j].hip_X_ee.ds.xp 											# set only during transfer phase

					## generate spline for transfer phase leg
					self.Robot.generateSpline(j, self.Robot.Leg[j].hip_X_ee.cs.xp, self.Robot.Leg[j].hip_X_ee.ds.xp,
												self.Robot.Leg[j].qsurface, 1, False, TRAC_PERIOD)

					# set flag that phase has changed
					self.Robot.Leg[j].phase_change = True


			## trajectory points for all legs
			for j in range (0, self.Robot.active_legs):

				# transfer phase
				if (self.Robot.gaitgen.cs[j] == 1 and self.Robot.Leg[j].feedback_state==1):

					# unstack next point in individual leg spline to be processed
					try:
						self.Robot.Leg[j].pointToArray()

						# Operator mode
						if (self.control_mode == 0):
							self.Robot.Leg[j].spline_counter = np.int((self.cp/(STEP_STROKE*self.Robot.gaitgen.gdic['dphase']))*41)
						# Autonomous mode
						elif (self.control_mode == 1):
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

					## determine foot velocity using CoM velocity; world_X_base cause rotation
					self.Robot.Leg[j].base_X_ee.ds.xv = np.dot( tf.rotation_zyx(self.Robot.world_X_base.cs.wp).transpose() , \
										-( cv-np.cross((np.dot(tf.rotation_zyx(self.Robot.world_X_base.cs.wp), self.Robot.Leg[j].base_X_ee.ds.xp)).transpose(),wv) ).transpose() )

					## base_X_feet position: integrate base_X_feet velocity
					self.Robot.Leg[j].base_X_ee.ds.xp = self.Robot.Leg[j].base_X_ee.ds.xv*interval + self.Robot.Leg[j].base_X_ee.ds.xp #+ self.Robot.Leg[j].base_X_ee.es.xp

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

		else:
			print 'mag_v skipped'

		# print '--------------------------------'
	def trajectory_tracking(self, x_com, w_com=0):

		self.planner.heading = (1,0,0)

		# produce interpolation points
		self.Robot.x_spline, self.Robot.w_spline = self.planner.generate_path(x_com, w_com)
		self.planner.get_path_details() 	# for information about trajectory

		## This segment to check output only
		# p_spline = Pspline.SplineGenerator()
		# nx, nv, na, nt  = p_spline.generate_body_spline(xn_com, tn_com, 2)
		# wx, wv, wa, wt  = p_spline.generate_body_spline(wn_com, tw_com, 2)
		# Plot.plot_3d(nx[:,0],nx[:,1],nx[:,2])
		# Plot.plot_2d(nx[:,0],nx[:,2])
		# Plot.plot_2d(nt, nx)
		# Plot.plot_2d(wt, wx)
		
		i = 1 					# counter for number of points in robot's trajectory
		self.planner.count = 1 	# counter for CoM point

		# Set all legs to support mode for bodyposing, prevent AEP from being set
		if (self.Robot.support_mode == True):
			for j in range(0,6):
				self.Robot.gaitgen.cs[j] = 0

		# Choose shortest trajectory to follow
		if (len(self.Robot.x_spline.t) > len(self.Robot.w_spline.t)):
			tlen  = len(self.Robot.w_spline.t)
			tinst = self.Robot.x_spline.t
		else:
			tlen  = len(self.Robot.x_spline.t)
			tinst = self.Robot.w_spline.t
		print 'tlen: ', len(self.Robot.x_spline.t), len(self.Robot.w_spline.t)

		# cycle until trajectory complete
		while (i != tlen):
			print 'counting: ', i, tlen
			# suppress trajectory counter as body support suspended
			if (self.Robot.suspend == True):
				i -= 1

			## Determine time interval
			self.interval = tinst[i] - tinst[i-1]

			## Extract linear task space position, velocity, acceleration
			cp 			= self.Robot.x_spline.xp[i]
			self.lin_v 	= self.Robot.x_spline.xv[i]
			ca 			= self.Robot.x_spline.xa[i]
			ct 			= self.Robot.x_spline.t[i]

			self.ang_p 	= self.Robot.w_spline.xp[i]
			self.ang_v	= self.Robot.w_spline.xv[i]

			## LOGGING: initialise variable and set bodypose
			self.sp_state = JointState()
			for j in range(6):
				self.sp_state.name.append(ROBOT_STATE[j])
				if (j<3):
					self.sp_state.position.append(self.Robot.x_spline.xp[i].item(j))
				else:
					self.sp_state.position.append(self.Robot.w_spline.xp[i].item(j-3))
			for j in range(0,18):
				self.sp_state.name.append(JOINT_NAME[j])

			# horizontal plane instantenous velocity magnitude
			mag_v  = np.sqrt(self.lin_v.item(0)**2 + self.lin_v.item(1)**2)

			# horizontal plane: unit vector
			dir_uv = np.nan_to_num(np.array([ [self.lin_v.item(0)/mag_v], [self.lin_v.item(1)/mag_v], [0.0] ]))
			mag_v  = 1 	# forces controller to always execute
			# tracking translations
			self.track_com = np.array([ self.lin_v.item(0), self.lin_v.item(1), self.lin_v.item(2) ])*self.interval + self.track_com

			# Next CoM point
			if (self.Robot.x_spline.t[i] == self.planner.t_com[self.planner.count]):
				self.planner.count += 1

			# send command to body controller
			self.execution(mag_v, dir_uv, i)
			self.rate.sleep()

			i += 1

		print 'Trajectory executed'
		print 'Desired Goal: ', self.Robot.x_spline.xp[tlen-1]
		print 'Tracked Goal: ', self.track_com
		self._Get_to_Stance() 				# put transfer legs onto ground

	def action_interface(self):
		self.Robot.updateState()
		# print 'ds: ', np.round(self.Robot.Leg[0].hip_X_ee.ds.xp.flatten(),3)
		# print 'cs: ', np.round(self.Robot.Leg[0].hip_X_ee.cs.xp.flatten(),3)
		# initialise variables
		data = None; x_com = None; w_com = None; mode = None;

		# check action to take
		data = self.ui_control.action_to_take()
		if (data != None):
			## data mapping - for convenience
			x_com, w_com, mode = data

			## stand up if at rest
			if ( (mode == 1 or mode == 2) and self.resting == True):
				print 'Going to standup'
				self._default_pose()

			## condition for support (1), walk (2), reset (3)
			if (mode == 1):
				print 'support mode'
				self.Robot.reset_state = True
				self.Robot.suspend = False 		# clear suspension flag
				self.planner  = Pathgenerator.PathGenerator()
				self.planner.gait = self.Robot.gaitgen.gdic
				self.Robot.support_mode = True

				self.trajectory_tracking(x_com, w_com)
				# self._default_pose()
				# self._default_pose(1) 	# required for resetting stance for walking

			elif (mode == 2):
				print 'walk mode'
				self.planner  = Pathgenerator.PathGenerator()
				self.planner.gait = self.Robot.gaitgen.gdic
				self.Robot.support_mode = False

				self.alternate_phase()
				self.trajectory_tracking(x_com, w_com)

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
