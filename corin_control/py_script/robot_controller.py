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
import tf as RTF 										# ROS transform library

## Robotis ROS msgs for joint control
from robotis_controller_msgs.msg import SyncWriteMultiFloat

def r3_X_m(r3a):
	""" convert 3D angular vector in R^(3x1) to SE(3) """
	out = np.eye(4)
	out[0:3,0:3] = tf.rotation_xyz(r3a)
	return out

def v3_X_m(v3a):
	""" convert 3D linear vector in R^(3x1) to SE(3) """
	out = np.eye(4)
	out[0:3,3] = v3a.flatten()
	return out

def mX(a, b, c=None, d=None, e=None):
	""" multiplies 2D array in matrix style """
	## TODO: include check on array size
	if (c is None):
		return np.dot(a,b)
	elif (d is None):
		return np.dot(np.dot(a,b), c)
	elif (e is None):
		return np.dot(np.dot(a,b), np.dot(c,d))
	elif (e is not None):
		return np.dot(np.dot(np.dot(a,b), np.dot(c,d)), e)

class CorinManager:
	def __init__(self, initialise):
		rospy.init_node('main_controller') 		# Initialises node
		self.robot_ns 	= ROBOT_NS 				# Robot namespace
		self.rate 	  	= rospy.Rate(CTR_RATE)	# Controller rate

		self.Robot 		= robot_class.RobotState() 					# robot class
		self.Action		= control_interface.control_interface()		# control action class

		self.point 	   	= JointTrajectoryPoint() 		# for logging and assist publishing trajectory points
		self.sp_state  	= JointState() 					# LOGGING

		self.interface 	= None		# interface to control: simulation or Robotis hardware
		self.resting  	= False 	# Flag indicating robot standing or resting
		self.on_start 	= False 	# variable for resetting to leg suspended in air

		self._initialise_()


	def joint_state_callback(self, msg): 		# robot joint state callback
		self.Robot.qc = msg

	def imu_callback(self, msg): 				# imu callback
		rpy = RTF.transformations.euler_from_quaternion([self.imu.orientation.x, self.imu.orientation.y, self.imu.orientation.z, self.imu.orientation.w])
		self.Robot.imu = rpy

	def contact_state_callback(self, msg): 		# foot contact binary state
		self.Robot.cstate = msg.data

	def contact_force_callback(self, msg): 		# foot contact force
		self.Robot.cforce = msg.data

	def _initialise_(self):
		""" Initialises robot and classes """

		## Identifies interface to control: simulation or robotis motors
		self.interface = 'simulation' if rospy.has_param('/gazebo/auto_disable_bodies') else 'robotis'

		## set up publisher and subscriber topics
		self._initialise_topics()

		## moves robot to nominal position if haven't done so
		if rospy.has_param(ROBOT_NS + '/standing'):
			if (rospy.get_param(ROBOT_NS + '/standing')==True):
				pass
		else:
			self._default_pose() 		# move robot to default position
		
		## update and reset robot states
		self.Robot.reset_state = True
		self.Robot.suspend = False
		self.Robot.updateState()

		for j in range(0,self.Robot.active_legs):
			self.Robot.Leg[j].feedback_state = 2 	# trajectory completed

	def _initialise_topics(self):
		""" Initialises publishers and subscribers """

		##***************** PUBLISHERS ***************##
		self.setpoint_pub_ = rospy.Publisher('/corin/setpoint_states', JointState, queue_size=1)	# LOGGING publisher

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

		rospy.sleep(0.5) # sleep for short while for topics to be initiated properly

	def _air_suspend_legs(self):
		""" routine to move legs from current position to in the air """

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
			qd = self.Robot.task_X_joint()

			# publish appended joint angles if motion valid
			self.publish_topics(qd)

	def _default_pose(self, stand_state=0):
		""" moves robot to nominal stance (default pose) """

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
					else: 			# odd number legs
						self.Robot.Leg[j].hip_X_ee.ds.xp = self.Robot.Leg[j].hip_X_ee.cs.xp
				qd = self.Robot.task_X_joint()
				self.publish_topics(qd)

			self.Robot.updateState()
			for npoint in range(0, int(TRAC_PERIOD/CTR_INTV)):
				for j in range(0,self.Robot.active_legs):
					# set cartesian position for joint kinematics
					if (j%2 == 1): 	# even number legs
						self.Robot.Leg[j].pointToArray();
						self.Robot.Leg[j].spline_counter += 1
					else: 			# odd number legs
						self.Robot.Leg[j].hip_X_ee.ds.xp = self.Robot.Leg[j].hip_X_ee.cs.xp

				qd = self.Robot.task_X_joint()
				self.publish_topics(qd)

		## belly on ground, move all together
		else:
			for npoint in range(0, int(TRAC_PERIOD/CTR_INTV)):
				for j in range(0,self.Robot.active_legs):
					# set cartesian position for joint kinematics
					self.Robot.Leg[j].pointToArray();
					self.Robot.Leg[j].spline_counter += 1

				# convert and publish joint angles if motion valid
				qd = self.Robot.task_X_joint()
				self.publish_topics(qd)

	def _Get_to_Stance(self):
		""" Executes remaining leg trajectories in transfer phase """
		""" Checks remaining points on trajectory and finish off  """ 

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
			for j in range(0,6):
				# transfer phase
				if (self.Robot.gaitgen.cs[j] == 1 and self.Robot.Leg[j].feedback_state==1):

					# unstack next point in individual leg spline to be processed
					try:
						self.Robot.Leg[j].pointToArray()
						self.Robot.Leg[j].spline_counter += 1
					except:
						self.Robot.Leg[j].feedback_state = 2 	# trajectory completed
						self.Robot.suspend = True 				# stops robot from moving i.e. stops support stance until rest of legs finish
						
			## Task to joint space
			qd = self.Robot.task_X_joint()

			self.publish_topics(qd)
			rospy.sleep(CTR_INTV)

	def task_X_joint(self, j):
		""" Convert leg task space to joint space 	"""
		""" Input: j -> leg number 					"""
		## TODO: should be Robot function
		## TODO: include checks - full trajectory check

		try:
			if (self.Robot.Leg[j].tf_task_X_joint()):
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

	def publish_topics(self, q):
		""" Publish joint position to joint controller topics and
			setpoint topic (for logging all setpoint states) 				"""
		""" Input: q -> Joint setpoints (position, velocity, acceleration) 	"""

		## Publish joint position to robot if valid
		if (not self.Robot.invalid):
			self.Robot.active_legs = 6

			if (self.interface == 'simulation'):
				for n in range(0,self.Robot.active_legs*3):
					qp = Float64()
					# qp.data = self.point.positions[n]
					qp.data = q.xp[n]
					self.qpub_[n].publish(qp)

					# self.sp_state.position.append(self.point.positions[n]) 	# LOGGING
					# self.sp_state.velocity.append(self.point.velocities[n])	# LOGGING

			elif (self.interface == 'robotis'):
				dqp = SyncWriteMultiFloat()
				dqp.item_name 	= str("goal_position") 	# register to start first write
				dqp.data_length = 4 					# size of register

				for n in range(0,self.Robot.active_legs*3): 	# loop for each joint
					dqp.joint_name.append(str(JOINT_NAME[n])) 	# joint name
					dqp.value.append(self.point.positions[n])	# data

					self.sp_state.position.append(self.point.positions[n]) 	# LOGGING
					self.sp_state.velocity.append(self.point.velocities[n])	# LOGGING

				self.sync_qpub_.publish(dqp)

		## Publish setpoints to logging topic
		self.setpoint_pub_.publish(self.sp_state)
		self.rate.sleep()

		self.clear_point()

	def alternate_phase(self):
		# change gait phase
		self.Robot.gaitgen.change_phase()

		# update robot leg phase_change
		for i in range(0,6):
			if (self.Robot.gaitgen.cs[i] == 1):
				self.Robot.Leg[i].phase_change = False

		self.Robot.suspend = False

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

		# Trajectory for robot's base
		# x_spline, w_spline = PathGenerator.generate_path(x_cob, w_cob, CTR_INTV)
		coba = PathGenerator.generate_path(x_cob, w_cob, CTR_INTV) 	# think of another name for 'coba'
		# Plot.plot_2d(x_spline.t, x_spline.xp)
		
		# Set all legs to support mode for bodyposing, prevent AEP from being set
		if (self.Robot.support_mode == True):
			for j in range(0,6):
				self.Robot.gaitgen.cs[j] = 0

		# cycle through trajectory points until complete
		i = 1 	# skip first point since spline has zero initial differential conditions
		while (i != len(coba.X.t) and not rospy.is_shutdown()):
			# print 'counting: ', i, len(coba.X.t)

			## suppress trajectory counter as body support suspended
			if (self.Robot.suspend == True):
				i -= 1

			## Variable mapping for base linear and angular time, position, velocity, acceleration
			ct	= coba.X.t[i]; 		wt	= coba.W.t[i]; 
			cp	= coba.X.xp[i];		wp	= coba.W.xp[i];
			cv	= coba.X.xv[i];		wv	= coba.W.xv[i];
			ca	= coba.X.xa[i];		wa	= coba.W.xa[i];

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

			## Tracking desired translations
			if (self.Robot.suspend != True):
				cob_X_desired += cv*CTR_INTV
				cob_W_desired += wv*CTR_INTV

			# horizontal plane instantenous velocity magnitude
			# TODO: need to consider vertical - dependant on surface normal (motion primitive used)
			mag_v  = np.sqrt(cv.item(0)**2 + cv.item(1)**2) 	

			# horizontal plane: unit vector
			dir_uv = np.nan_to_num(np.array([ [cv.item(0)/mag_v], [cv.item(1)/mag_v], [0.0] ]))

			# Next CoB point - may change if legs are suspended
			# if (coba.X.t[i] == PathGenerator.t_com[PathGenerator.count]): 	# pre-computed in advance
			# 	PathGenerator.count += 1

			## ============================================================================================================================== ##
			## Execution of command ##
			## ==================== ##

			## update robot state
			self.Robot.updateState()
			self.Robot.world_X_base.cs.wp = self.Robot.world_X_base.ds.wp 	# overwrite - TC: use IMU data
			
			self.Robot.XHc.world_X_base = self.Robot.XHd.world_X_base 		# overwrite - TC: use IMU data
			self.Robot.XHc.base_X_world = self.Robot.XHd.base_X_world

			## Update robot's desired pose
			# self.Robot.fr_world_X_base 	  = tf.rotation_zyx(wp)
			self.Robot.world_X_base.ds.wp = wp
			self.Robot.XHd.update_world_X_base(np.concatenate((np.zeros(3),wp)))

			# print j, ' bXf ', tf.rotation_zyx(-1.*self.Robot.world_X_base.cs.wp)
			# print j, ' bXf ', self.Robot.XHc.base_X_world

			## generate trajectory for leg in transfer phase
			for j in range (0, self.Robot.active_legs):

				# Transfer phase
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
						cp_n = (coba.X.xp[int(i+TRAC_PERIOD/CTR_INTV)]).reshape(3,1)
					except:
						cp_n = (coba.X.xp[-1]).reshape(3,1)

					## calculate AEP with CoB height change
					# condition occurs when leg against wall
					if (self.Robot.Leg[j].REP.item(2) > 0.):
						# get current CoB
						cur_com = (coba.X.xp[int(i)]).reshape(3,1)
						# change in vertical height between current and next CoB, in v3 representation
						d_h = np.dot(e_z, (cp_n - cp) )
						self.Robot.Leg[j].AEP = (self.Robot.Leg[j].REP + dir_uv*STEP_STROKE/2. + d_h)

					# normal walking condition
					else:
						# change in vertical height between next CoB and REP, in R^(3x1) representation - extract vertical component only
						v3_delta_height = mX(e_z, cp_n + self.Robot.Leg[j].REP)
						
						# update AEP
						self.Robot.Leg[j].AEP = self.Robot.Leg[j].REP + dir_uv*STEP_STROKE/2. - v3_delta_height
						self.Robot.Leg[j].XHc.base_X_AEP = mX(v3_X_m(dir_uv*STEP_STROKE/2. - v3_delta_height), self.Robot.Leg[j].XHc.base_X_NRP)
						
					# set base_X_foot position based on AEP
					self.Robot.Leg[j].base_X_ee.ds.xp = mX(tf.rotation_zyx(-1.*self.Robot.world_X_base.cs.wp), self.Robot.Leg[j].AEP)
					self.Robot.Leg[j].XHd.base_X_foot = mX(self.Robot.XHc.base_X_world, self.Robot.Leg[j].XHc.base_X_AEP)

					# Base to hip frame conversion
					self.Robot.Leg[j].hip_X_ee.ds.xp = self.Robot.Leg[j].base_X_hip_ee(self.Robot.Leg[j].base_X_ee.ds.xp)
					self.Robot.Leg[j].hip_AEP = self.Robot.Leg[j].hip_X_ee.ds.xp 											# set only during transfer phase

					self.Robot.Leg[j].XHd.coxa_X_foot = mX(self.Robot.Leg[j].XHc.coxa_X_base, self.Robot.Leg[j].XHd.base_X_foot)
					self.Robot.Leg[j].XHd.coxa_X_AEP  = self.Robot.Leg[j].XHd.coxa_X_foot

					## generate spline for transfer phase leg - 
					svalid = self.Robot.Leg[j].generateSpline(self.Robot.Leg[j].XHc.coxa_X_foot[0:3,3], self.Robot.Leg[j].XHd.coxa_X_foot[0:3,3],
																self.Robot.Leg[j].qsurface, False, TRAC_PERIOD, CTR_INTV)

					# svalid = self.Robot.Leg[j].generateSpline(self.Robot.Leg[j].hip_X_ee.cs.xp, self.Robot.Leg[j].hip_X_ee.ds.xp,
					# 											self.Robot.Leg[j].qsurface, False, TRAC_PERIOD, CTR_INTV)

					if (svalid is False):
						self.Robot.invalid = True
					else:	
						# set flag that phase has changed
						self.Robot.Leg[j].phase_change = True


			## Comput task space foot position for all legs
			for j in range (0, self.Robot.active_legs):

				# Transfer phase
				if (self.Robot.gaitgen.cs[j] == 1 and self.Robot.Leg[j].feedback_state==1):

					# unstack next point in individual leg spline to be processed
					try:
						self.Robot.Leg[j].pointToArray()
						self.Robot.Leg[j].spline_counter += 1
					except:
						self.Robot.Leg[j].feedback_state = 2 	# trajectory completed
						self.Robot.suspend = True 				# stops robot from moving i.e. stops support stance until rest of legs finish

				## Support phase
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
				# self.task_X_joint(j)

			qd = self.Robot.task_X_joint()	
			
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
			self.publish_topics(qd)

			## ============================================================================================================================== ##
			

			i += 1

		print 'Trajectory executed'
		print 'Desired Goal: ', coba.X.xp[-1]
		print 'Tracked Goal: ', cob_X_desired
		self._Get_to_Stance() 				# put transfer legs onto ground

	def action_interface(self):
		""" Interface for commanding robot """

		## update robot state prior to starting action
		self.Robot.updateState()
		
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
			## Standby ##
			rospy.sleep(0.5)
