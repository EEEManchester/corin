#!/usr/bin/env python

## Main controller for the robot
import rospy
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'library'))
sys.dont_write_bytecode = True

import time
from fractions import Fraction
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import copy
import warnings

## personal libraries
from constant import *
import robot_class
import gait_class as Gaitgenerator
import param_gait
import path_generator as Pathgenerator
import plotgraph as Plot
import transformations as tf
import pspline_generator as Pspline
import bspline_generator as Bspline

from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from std_msgs.msg import ByteMultiArray 	# foot contact state
from std_msgs.msg import Float32MultiArray	# foot contact force

## arebgun ROS msgs
from dynamixel_msgs.msg import PositionVelocity

## Robotis ROS msgs for joint control
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from robotis_controller_msgs.msg import SyncWriteItem
from robotis_controller_msgs.msg import SyncWriteMulti

## User control files
from sensor_msgs.msg import Joy
import connex
import control_interface

def point_to_array(cpoints, i, select=0):
	## Extract linear task space position, velocity, acceleration
	cp = np.array([cpoints.positions[0][i],  cpoints.positions[1][i],  cpoints.positions[2][i]])
	cv = np.array([cpoints.velocities[0][i], cpoints.velocities[1][i], cpoints.velocities[2][i]])
	ca = np.array([cpoints.accelerations[0][i], cpoints.accelerations[1][i], cpoints.accelerations[2][i]])
	ct = cpoints.time_from_start[0][i]

	if (select == 1):
		return cp
	else:
		return cp, cv, ca, ct

def rad2raw(radian):

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

def rads2raw(rads):
	velocity_to_value_ratio_ = 1.0/(0.229*2*3.14159265/60.0)

	value = int(round(abs(rads) * velocity_to_value_ratio_))
	if (value == 0):
		value = 1
	return value # int(math.ceil(abs(rads) * velocity_to_value_ratio_))

def raw2rads(raw):
	velocity_to_value_ratio_ = 1.0/(0.229*2*3.14159265/60.0)

	return (rads / velocity_to_value_ratio_)

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
		self.b_spline = Bspline.SplineGenerator() 		# B-spline class
		self.p_spline = Pspline.SplineGenerator() 		# polynomial spline class
		self.connex   = connex.Connexion() 				# connex hardware controller
		self.planner  = Pathgenerator.PathGenerator() 	# trajectory following

		self.control_mode = 1 				# control mode: 1-autonomous, 0-manual control
		self.hardware = 'simulation'		# options: 'simulation', 'real', 'robotis'

		self.point = JointTrajectoryPoint()
		self.point.time_from_start = rospy.Duration(TRAC_INTERVAL)

		if (initialise):
			self._start()
			rospy.sleep(0.5)
			self._default_pose()

		self.com_tracking = np.zeros(3)

		self.resting = False 		# Flag indicating robot standing or resting
		self.ui_control	= control_interface.control_interface()

	def joint_state_callback(self, msg):
		self.Robot.qc = msg

	def imu_callback(self, msg):
		self.Robot.imu = msg

	def contact_state_callback(self, msg):
		self.Robot.cstate = msg.data

	def contact_force_callback(self, msg):
		self.Robot.cforce = msg.data

	def _start(self):
		#######################################
		## initialise publishers/subscribers ##
		#######################################

		##***************** PUBLISHERS ***************##
		self.trajectory_pub = rospy.Publisher(self.robot_ns + '/setpoint', JointTrajectoryPoint, queue_size=1)
		self.phase_pub 		= rospy.Publisher(self.robot_ns + '/phase', JointTrajectoryPoint, queue_size=1)
		self.control_mode_	= rospy.Publisher('/robotis/set_control_mode', String, queue_size=1)
		self.sync_qpub_  	= rospy.Publisher('/robotis/sync_write_multi', SyncWriteMulti, queue_size=1)

		self.qpub = {}

		if (self.robot_ns == 'corin' and (self.hardware == 'simulation' or self.hardware == 'real' or self.hardware == 'robotis')):
			print 'initializing topics'

			for i in range(0,18):
				if (self.hardware == 'simulation'):
					## Publish to Gazebo
					self.qpub[i] = rospy.Publisher(self.robot_ns + '/' + JOINT_TOPICS[i] + '/command', Float64, queue_size=1)

				elif (self.hardware == 'real'):
					## Publish to Dynamixel motors
					self.qpub[i] = rospy.Publisher(self.robot_ns + '/' + JOINT_TOPICS[i] + '/command_pv', PositionVelocity, queue_size=1)

		else:
			# shutdown if wrong hardware selected
			rospy.logerr("Invalid Hardware Interface Selected!")
			rospy.signal_shutdown("Invalid Hardware Interface Selected - shutting down")

		##***************** SUBSCRIBERS ***************##
		if (self.hardware == 'simulation'):
			self.joint_sub_  = rospy.Subscriber(self.robot_ns + '/joint_states', JointState, self.joint_state_callback, queue_size=5)
		elif (self.hardware == 'robotis'):
			self.joint_sub_  = rospy.Subscriber('robotis/present_joint_states', JointState, self.joint_state_callback, queue_size=5)
		# self.imu_sub_	 = rospy.Subscriber(self.robot_ns + '/imu/trunk/data', Imu, self.imu_callback, queue_size=1)
		self.cstate_sub_ = rospy.Subscriber(self.robot_ns + '/contact_state', ByteMultiArray, self.contact_state_callback, queue_size=1)
		self.cforce_sub_ = rospy.Subscriber(self.robot_ns + '/contact_force', Float32MultiArray, self.contact_force_callback, queue_size=1)

		if (self.control_mode == 0): 	# manual control mode
			self.connex_sub = rospy.Subscriber('/spacenav/joy', Joy, self.connex_callback, queue_size=1 )

		self.on_start = False

	# routine to stand up: using current state move leg up to air, then step up
	def _air_suspend_legs(self):
		self.Robot.updateState() 		# get current state

		# generate spline to legs up position
		for j in range(0,self.Robot.active_legs):
			self.Robot.Leg[j].hip_X_ee.ds.xp = LEG_STANCE[j] + np.array([0.0,0.0,BODY_HEIGHT+0.01]) 	# put leg on final ground position first
			self.Robot.generateSpline(j, self.Robot.Leg[j].hip_X_ee.cs.xp, self.Robot.Leg[j].hip_X_ee.ds.xp,
											self.Robot.Leg[j].qsurface, 1, False, TRAC_PERIOD)
			if (j==3):
				print self.Robot.Leg[j].hip_X_ee.ds.xp
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

		if (self.on_start == False):
			self._air_suspend_legs() 	# moves leg into air
			self.on_start = True

		rospy.sleep(0.5)
		# get current state
		for i in range(0,3):
			self.Robot.updateState()

		print 'moving to nominal stance'

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

		# update and reset states
		self.Robot.reset_state = True
		self.Robot.suspend = False
		self.Robot.updateState()

		for j in range(0,self.Robot.active_legs):
			self.Robot.Leg[j].feedback_state = 2 	# trajectory completed

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
		## Task to joint space
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
		self.point.time_from_start = rospy.Duration(TRAC_INTERVAL)

	def publish_topics(self):
		if (not self.Robot.invalid):
			self.Robot.active_legs = 6

			if (self.hardware == 'simulation' or self.hardware == 'real'):
				## TEMP
				dqp = SyncWriteMulti()
				dqp.item_name = str("profile_velocity") 	# goal_position
				dqp.data_length = 8

				for n in range(0,self.Robot.active_legs*3):
					## Gazebo
					if (self.hardware == 'simulation'):
						qp = Float64()
						qp.data = self.point.positions[n]
						self.qpub[n].publish(qp)

						## TEMP
						# if (n<9 and n>5):
						dqp.joint_name.append(str(JOINT_NAME[n])) 				# joint name
						dqp.value.append(rads2raw(self.point.velocities[n]))	# velocity
						dqp.value.append(rad2raw(self.point.positions[n]))		# position

					## Dynamixel - arebgun library
					elif (self.hardware == 'real'):
						data = PositionVelocity()
						data.position = self.point.positions[n]
						data.velocity = self.point.velocities[n]
						self.qpub[n].publish(data)

				self.sync_qpub_.publish(dqp) ## TEMP

			elif (self.hardware == 'robotis'):
				dqp = SyncWriteMulti()
				dqp.item_name = str("profile_velocity") 	# goal_position
				dqp.data_length = 8

				for n in range(0,self.Robot.active_legs*3): 	# loop for each joint
					dqp.joint_name.append(str(JOINT_NAME[n])) 				# joint name
					dqp.value.append(rads2raw(self.point.velocities[n]))	# velocity
					dqp.value.append(rad2raw(self.point.positions[n]))		# position

				self.sync_qpub_.publish(dqp)

			# print '==================================='

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

			## clear point list
			point = JointTrajectoryPoint()
			point.time_from_start = rospy.Duration(TRAC_INTERVAL)

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
							nex_com = point_to_array(self.Robot.x_spline, int(i+TRAC_PERIOD/TRAC_INTERVAL), 1)
						except:
							last_val = len(self.Robot.x_spline.time_from_start[0]) - 1
							nex_com = point_to_array(self.Robot.x_spline, last_val, 1)

						## calculate AEP with CoM height change
						# condition occurs when leg against wall
						if (self.Robot.Leg[j].REP.item(2) > 0.):
							# get current CoM
							cur_com = point_to_array(self.Robot.x_spline, int(i), 1)
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
		xn_com, wn_com, tn_com, twn_com, xlen, wlen = self.planner.generate_path(x_com, w_com)

		self.planner.get_path_details() 	# for information about trajectory

		## This segment to check output only
		p_spline = Pspline.SplineGenerator()
		nx, nv, na, nt  = p_spline.generate_body_spline(xn_com, tn_com, 2)
		wx, wv, wa, wt  = p_spline.generate_body_spline(wn_com, twn_com, 2)
		# Plot.plot_3d(nx[:,0],nx[:,1],nx[:,2])
		# Plot.plot_2d(nx[:,0],nx[:,2])
		# Plot.plot_2d(nt, nx)
		# Plot.plot_2d(wt, wx)

		self.planner.count = 1 	# counter for CoM point

		## create spline from CoM via points
		self.Robot.x_spline = self.p_spline.generate_body_spline(xn_com, tn_com, 0)
		self.Robot.w_spline = self.p_spline.generate_body_spline(wn_com, twn_com, 0)

		i = 1 	# counter for number of points in robot's trajectory

		# Set all legs to support mode for bodyposing, prevent AEP from being set
		if (self.Robot.support_mode == True):
			for j in range(0,6):
				self.Robot.gaitgen.cs[j] = 0

		# cycle until trajectory complete
		while (i != xlen):
			# suppress trajectory counter as body support suspended
			if (self.Robot.suspend == True):
				i -= 1

			## Determine time interval
			self.interval = self.Robot.x_spline.time_from_start[0][i] - self.Robot.x_spline.time_from_start[0][i-1]

			## Extract linear task space position, velocity, acceleration
			cp, self.lin_v, ca, ct = point_to_array(self.Robot.x_spline, i)

			self.ang_p, self.ang_v, wa, wt = point_to_array(self.Robot.w_spline, i)

			# horizontal plane instantenous velocity magnitude
			mag_v  = np.sqrt(self.lin_v.item(0)**2 + self.lin_v.item(1)**2)

			# horizontal plane: unit vector
			dir_uv = np.nan_to_num(np.array([ [self.lin_v.item(0)/mag_v], [self.lin_v.item(1)/mag_v], [0.0] ]))
			mag_v  = 1 	# forces controller to always execute
			# tracking translations
			self.com_tracking = np.array([ self.lin_v.item(0), self.lin_v.item(1), self.lin_v.item(2) ])*self.interval + self.com_tracking

			# Next CoM point
			if (self.Robot.x_spline.time_from_start[0][i] == self.planner.t_com[self.planner.count]):
				self.planner.count += 1

			# send command to robot
			self.execution(mag_v, dir_uv, i)
			self.rate.sleep()
			# rospy.sleep(self.interval)
			i += 1

		print 'Trajectory executed'
		print 'Desired Goal: ', point_to_array(self.Robot.x_spline, xlen-1, 1)
		print 'Tracked Goal: ', self.com_tracking
		self._Get_to_Stance() 				# put transfer legs onto ground

	def action_interface(self):
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
				self.Robot.suspend = False 		# clear suspension flag
				self.planner  = Pathgenerator.PathGenerator()
				self.planner.gait = self.Robot.gaitgen.gdic
				self.Robot.support_mode = True

				self.trajectory_tracking(x_com, w_com)
				self._default_pose()
				self._default_pose(1) 	# required for resetting stance for walking

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
