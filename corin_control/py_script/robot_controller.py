#!/usr/bin/env python

""" Main controller for the robot """
__version__ = '1.0'
__author__ = 'Wei Cheah'

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
import gait_class 							# class for gait coordinator
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
# import tf as RTF 										# ROS transform library

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
	print out
	return out

def mX(a, b, c=None, d=None, e=None):
	""" multiplies 2D array in matrix style """
	
	try:
		if (c is None):
			return np.dot(a,b)
		elif (d is None):
			return np.dot(np.dot(a,b), c)
		elif (e is None):
			return np.dot(np.dot(a,b), np.dot(c,d))
		elif (e is not None):
			return np.dot(np.dot(np.dot(a,b), np.dot(c,d)), e)
	except Exception, e:
		print e
		return None

def mC(a, b):
	""" cross product of two vectors """

	try:
		return (np.cross(a.transpose(),b.transpose())).reshape(3,1)
	except Exception, e:
		print e
		return None
#aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
class CorinManager:
	def __init__(self, initialise=False):
		rospy.init_node('CorinController') 		# Initialises node
		self.rate 	  = rospy.Rate(CTR_RATE)	# Controller rate

		self.Robot 	= robot_class.RobotState() 				# robot class
		self.Action	= control_interface.control_interface()	# control action class
		# self.Gait = gait_class.GaitClass(GAIT_TYPE) 		# gait class

		self.interface = None	# interface to control: simulation or Robotis hardware
		self.resting   = False 	# Flag indicating robot standing or resting
		self.on_start  = False 	# variable for resetting to leg suspended in air

		self._initialise_()

	def joint_state_callback(self, msg):
		""" robot joint state callback """
		self.Robot.qc = msg

	def imu_callback(self, msg):
		""" imu callback """
		self.Robot.imu = msg

	def contact_state_callback(self, msg):
		""" foot contact binary state """
		self.Robot.cstate = msg.data

	def contact_force_callback(self, msg):
		""" foot contact force """
		self.Robot.cforce = msg.data

	def _initialise_(self):
		""" Initialises robot and classes """

		## Identifies interface to control: simulation or robotis motors
		self.interface = 'simulation' if rospy.has_param('/gazebo/auto_disable_bodies') else 'robotis'

		## set up publisher and subscriber topics
		self.__initialise_topics()

		## moves robot to nominal position if haven't done so
		if rospy.has_param(ROBOT_NS + '/standing'):
			if (rospy.get_param(ROBOT_NS + '/standing')==True):
				pass
		else:
			self._default_pose() 		# move robot to default position
		
		## update and reset robot states
		self.Robot.reset_state = True
		self.Robot.suspend = False
		self.Robot.update_state()

		for j in range(0,self.Robot.active_legs):
			self.Robot.Leg[j].feedback_state = 2 	# trajectory completed


	def __initialise_topics(self):
		""" Initialises publishers and subscribers """

		##***************** PUBLISHERS ***************##
		self.setpoint_pub_ = rospy.Publisher('/corin/setpoint_states', JointState, queue_size=1)	# LOGGING publisher

		## Hardware Specific Publishers ##
		if (ROBOT_NS == 'corin' and (self.interface == 'simulation' or self.interface == 'robotis')):
			## Publish to Gazebo
			if (self.interface == 'simulation'):
				self.qpub_ = {}
				for i in range(0,18):
					self.qpub_[i] = rospy.Publisher(ROBOT_NS + '/' + JOINT_NAME[i] + '/command', Float64, queue_size=1)

			## Publish to Dynamixel motors
			elif (self.interface == 'robotis'):
				self.sync_qpub_  = rospy.Publisher('/robotis/sync_write_multi_float', SyncWriteMultiFloat, queue_size=1)

			print ">> INITIALISED JOINT TOPICS"
		else:
			# shutdown if wrong hardware selected
			rospy.logerr("INVALID HARDWARE INTERFACE SELECTED!")
			rospy.signal_shutdown("INVALID HARDWARE INTERFACE SELECTED - SHUTTING DOWN")

		##***************** SUBSCRIBERS ***************##
		self.imu_sub_	 = rospy.Subscriber(ROBOT_NS + '/imu/base/data', Imu, self.imu_callback, queue_size=1)
		self.cstate_sub_ = rospy.Subscriber(ROBOT_NS + '/contact_state', ByteMultiArray, self.contact_state_callback, queue_size=1)
		self.cforce_sub_ = rospy.Subscriber(ROBOT_NS + '/contact_force', Float32MultiArray, self.contact_force_callback, queue_size=1)

		## Hardware Specific Subscribers ##
		if (self.interface == 'simulation'):
			self.joint_sub_  = rospy.Subscriber(ROBOT_NS + '/joint_states', JointState, self.joint_state_callback, queue_size=5)
		elif (self.interface == 'robotis'):
			self.joint_sub_  = rospy.Subscriber('robotis/present_joint_states', JointState, self.joint_state_callback, queue_size=5)

		rospy.sleep(0.5) # sleep for short while for topics to be initiated properly

	def _air_suspend_legs(self):
		""" routine to move legs from current position to in the air """
		## TODO: move elsewhere
		self.Robot.update_state() 		# get current state

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
			self.Robot.generate_spline(j, self.Robot.Leg[j].hip_X_ee.cs.xp, self.Robot.Leg[j].hip_X_ee.ds.xp,
											self.Robot.Leg[j].qsurface, 1, False, TRAC_PERIOD)
			
		# move leg into position
		for npoint in range(0, int(TRAC_PERIOD/CTR_INTV)):
			for j in range(0,self.Robot.active_legs):
				# set cartesian position for joint kinematics
				self.Robot.Leg[j].update_from_spline();
				self.Robot.Leg[j].spline_counter += 1
			qd = self.Robot.task_X_joint()

			# publish appended joint angles if motion valid
			self.publish_topics(qd)

	def _default_pose(self, stand_state=0):
		""" moves robot to nominal stance (default pose) """
		## TODO: move elsewhere
		## moves leg into air
		if (self.on_start == False):
			print 'Resetting stance....'
			self._air_suspend_legs()
			self.on_start = True

		rospy.sleep(0.5)
		# get current state
		for i in range(0,3):
			self.Robot.update_state()

		print 'Moving to nominal stance....'

		# generate spline to standup position
		for j in range(0,self.Robot.active_legs):
			self.Robot.Leg[j].hip_X_ee.ds.xp = LEG_STANCE[j]
			self.Robot.generate_spline(j, self.Robot.Leg[j].hip_X_ee.cs.xp, self.Robot.Leg[j].hip_X_ee.ds.xp,
											self.Robot.Leg[j].qsurface, stand_state, False, TRAC_PERIOD)

		## move leg into position
		# already in stand up position, shuffle in tripod fashion
		if (stand_state):
			for npoint in range(0, int(TRAC_PERIOD/CTR_INTV)):
				for j in range(0,self.Robot.active_legs):
					# set cartesian position for joint kinematics
					if (j%2 == 0): 	# even number legs
						self.Robot.Leg[j].update_from_spline();
						self.Robot.Leg[j].spline_counter += 1
					else: 			# odd number legs
						self.Robot.Leg[j].hip_X_ee.ds.xp = self.Robot.Leg[j].hip_X_ee.cs.xp
				qd = self.Robot.task_X_joint()
				self.publish_topics(qd)

			self.Robot.update_state()
			for npoint in range(0, int(TRAC_PERIOD/CTR_INTV)):
				for j in range(0,self.Robot.active_legs):
					# set cartesian position for joint kinematics
					if (j%2 == 1): 	# even number legs
						self.Robot.Leg[j].update_from_spline();
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
					self.Robot.Leg[j].update_from_spline();
					self.Robot.Leg[j].spline_counter += 1

				# convert and publish joint angles if motion valid
				qd = self.Robot.task_X_joint()
				self.publish_topics(qd)

	def complete_transfer_trajectory(self):
		""" Executes remaining leg trajectories in transfer phase """
		""" Checks remaining points on trajectory and finish off  """ 

		dir_uv = np.array([0.,0.,0.]) 	# zero vector direction
		spline_count = 0 				# current spline count for transfer phase legs
		self.Robot.suspend = True 		# suspend robot for zero trunk movement

		# Get leg phases
		for j in range(0,6):
			# check leg transfer status, if true
			if (self.Robot.Gait.cs[j] == 1):
				# get number of points yet to execute
				spline_count = self.Robot.Leg[j].spline_length - self.Robot.Leg[j].spline_counter

		try:
			# loop only required via points
			for sc in range(0, spline_count):
				for j in range(0,6):
					# transfer phase
					if (self.Robot.Gait.cs[j] == 1 and self.Robot.Leg[j].feedback_state==1):
						# unstack next point in individual leg spline to be processed
						try:
							self.Robot.Leg[j].update_from_spline()
							self.Robot.Leg[j].spline_counter += 1
						except:
							self.Robot.Leg[j].feedback_state = 2 	# trajectory completed
							self.Robot.suspend = True 				# stops robot from moving i.e. stops support stance until rest of legs finish
							
				## Task to joint space
				qd = self.Robot.task_X_joint()

				self.publish_topics(qd)
				rospy.sleep(CTR_INTV)
			return True
		except:
			print "Returning legs to ground failed"
			return False

	def publish_topics(self, q, q_log=None):
		""" Publish joint position to joint controller topics and
			setpoint topic (for logging all setpoint states) 				"""
		""" Input: q -> Joint setpoints (position, velocity, acceleration) 	"""

		## Publish joint position to robot if valid
		if (self.Robot.invalid is not True):
			self.Robot.active_legs = 6

			if (self.interface == 'simulation'):
				for n in range(0,self.Robot.active_legs*3):
					qp = Float64()
					qp.data = q.xp[n]
					# Publish joint angles individually
					self.qpub_[n].publish(qp)

			elif (self.interface == 'robotis'):
				dqp = SyncWriteMultiFloat()
				dqp.item_name 	= str("goal_position") 	# register to start first write
				dqp.data_length = 4 					# size of register

				for n in range(0,self.Robot.active_legs*3): 	# loop for each joint
					dqp.joint_name.append(str(JOINT_NAME[n])) 	# joint name
					dqp.value.append(q.xp[n])					# joint angle

				# Publish all joint angles together
				self.sync_qpub_.publish(dqp)

		## Publish setpoints to logging topic
		if (q_log is not None):
			self.setpoint_pub_.publish(q_log)

		self.rate.sleep()

	def alternate_phase(self):
		# change gait phase
		self.Robot.Gait.change_phase()

		# update robot leg phase_change
		for i in range(0,6):
			if (self.Robot.Gait.cs[i] == 1):
				self.Robot.Leg[i].phase_change = False

		self.Robot.suspend = False
		# print 'Gait phase changed!'
		# raw_input('gait continue')

	def trajectory_tracking(self, x_cob, w_cob=0):
		""" Generates body trajectory from given via points and
			cycles through the trajectory 							"""
		""" Input: 	1) x_com -> 2D array of linear positions
					2) w_com -> 2D array of angular orientations
			Output: None											"""

		## TEMP
		self.Robot.update_state()

		## define variables ##
		PathGenerator = Pathgenerator.PathGenerator() 	# path generator for robot's base
		cob_X_desired = np.zeros((3,1)) 	# cob linear location
		cob_W_desired = np.zeros((3,1)) 	# cob angular location

		# Trajectory for robot's base
		# x_spline, w_spline = PathGenerator.generate_path(x_cob, w_cob, CTR_INTV)
		base_path = PathGenerator.generate_path(x_cob, w_cob, CTR_INTV) 	# think of another name for 'base_path'
		Plot.plot_2d(base_path.W.t, base_path.W.xp)
		
		# Set all legs to support mode for bodyposing, prevent AEP from being set
		if (self.Robot.support_mode == True):
			for j in range(0,6):
				self.Robot.Gait.cs[j] = 0
				# self.Gait.cs[j] = 0

		# cycle through trajectory points until complete
		i = 1 	# skip first point since spline has zero initial differential conditions
		while (i != len(base_path.X.t) and not rospy.is_shutdown()):
			# print 'counting: ', i, len(base_path.X.t)

			## suppress trajectory counter as body support suspended
			if (self.Robot.suspend == True):
				i -= 1

			cp = base_path.X.xp[i];	wp = base_path.W.xp[i]
			cv = base_path.X.xv[i];	wv = base_path.W.xv[i]
			ca = base_path.X.xa[i];	wa = base_path.W.xa[i]
			## Variable mapping to R^(3x1) for base linear and angular time, position, velocity, acceleration
			v3cp = base_path.X.xp[i].reshape(3,1);	v3wp = base_path.W.xp[i].reshape(3,1);
			v3cv = base_path.X.xv[i].reshape(3,1);	v3wv = base_path.W.xv[i].reshape(3,1);
			v3ca = base_path.X.xa[i].reshape(3,1);	v3wa = base_path.W.xa[i].reshape(3,1);

			## Tracking desired translations
			if (self.Robot.suspend != True):
				cob_X_desired += v3cv*CTR_INTV
				cob_W_desired += v3wv*CTR_INTV

			# horizontal plane instantenous velocity magnitude
			# TODO: need to consider vertical - dependant on surface normal (motion primitive used)
			mag_v  = np.sqrt(v3cv.item(0)**2 + v3cv.item(1)**2) 	

			# horizontal plane: unit vector
			dir_uv = np.nan_to_num(np.array([ [v3cv.item(0)/mag_v], [v3cv.item(1)/mag_v], [0.0] ]))

			# Next CoB point - may change if legs are suspended
			# if (base_path.X.t[i] == PathGenerator.t_com[PathGenerator.count]): 	# pre-computed in advance
			# 	PathGenerator.count += 1

			## ============================================================================================================================== ##
			## Execution of command ##
			## ==================== ##
			
			## update robot state
			self.Robot.update_state()
			for j in range(0,6):
				self.Robot.Leg[j].hip_X_ee.cs.xp  = self.Robot.Leg[j].hip_X_ee.ds.xp.copy()  #
				self.Robot.Leg[j].base_X_ee.cs.xp = self.Robot.Leg[j].base_X_ee.ds.xp.copy() #

			self.Robot.world_X_base.cs.wp = self.Robot.world_X_base.ds.wp.copy() 	# TC: use IMU data
			self.Robot.fr_world_X_base 	  = tf.rotation_zyx(wp)
			self.Robot.world_X_base.ds.wp = wp.copy()

			## overwrite - TC: use IMU data
			self.Robot.XHc.world_X_base = self.Robot.XHd.world_X_base.copy()
			self.Robot.XHc.base_X_world = self.Robot.XHd.base_X_world.copy()

			## Update robot's desired pose & twist
			# self.Robot.XHd.update_world_X_base(np.vstack((np.zeros((3,1)),v3wp)))
			self.Robot.XHd.update_world_X_base(np.vstack((v3cp,v3wp)))
			self.Robot.V6d.world_X_base = np.vstack((v3cv,v3wv))
			self.Robot.A6d.world_X_base = np.vstack((v3ca,v3wa))
			
			## generate trajectory for leg in transfer phase
			for j in range (0, self.Robot.active_legs):

				# Transfer phase
				if (self.Robot.Gait.cs[j] == 1 and self.Robot.Leg[j].phase_change==False):
					self.Robot.Leg[j].feedback_state = 1 	# leg put into execution mode

					## 'Foothold Selection' Algorithm: 1) Update REP 2) Get_to_Stance

					# Update REP - MOCKED DATASET HERE
					base_X_surface = 0.32
					bodypose = np.array([0.,0.,BODY_HEIGHT, 0.,0.,0.])
					self.Robot.Leg[j].update_NRP(bodypose, base_X_surface)

					# change in CoB in next phase
					e_z = tf.SO3_selection(np.array([0.,0.,1.]) , 'z')

					# unstack next CoB location, determine foot z position in that CoB location
					try:
						v3np = (base_path.X.xp[int(i+TRAC_PERIOD/CTR_INTV)]).reshape(3,1)
					except:
						v3np = (base_path.X.xp[-1]).reshape(3,1)

					## calculate AEP with CoB height change
					# condition occurs when leg against wall
					if (self.Robot.Leg[j].XHd.base_X_NRP[2,3] > 0.):
						# change in vertical height between current and next CoB, in v3 representation
						d_h = np.dot(e_z, (v3np - v3cp) )
						# self.Robot.Leg[j].AEP = (self.Robot.Leg[j].REP + dir_uv*STEP_STROKE/2. + d_h)
						## TODO: check following
						v3_delta_height = mX(e_z, v3np - v3cp)
						self.Robot.Leg[j].XHc.world_base_X_AEP = mX( v3_X_m(dir_uv*STEP_STROKE/2. + v3_delta_height), 
																			self.Robot.Leg[j].XHd.world_base_X_NRP)

					# normal walking condition
					else:
						# change in vertical height between next CoB and REP, in R^(3x1) representation - extract vertical component only
						v3_delta_height = mX(e_z, v3np + self.Robot.Leg[j].XHd.world_base_X_NRP[:3,3:4])
						
						# update AEP
						# self.Robot.Leg[j].XHd.base_X_AEP = mX(v3_X_m(dir_uv*STEP_STROKE/2. - v3_delta_height), self.Robot.Leg[j].XHd.base_X_NRP)
						self.Robot.Leg[j].XHd.world_base_X_AEP = mX(v3_X_m(dir_uv*STEP_STROKE/2. - v3_delta_height), 
																	self.Robot.Leg[j].XHd.world_base_X_NRP)

					# set base_X_foot position based on AEP - base_X_world rotation used so that foot position remains in world frame
					# orientation for base_X_foot should remain the same as before?
					# self.Robot.Leg[j].XHd.base_X_foot = mX(self.Robot.XHc.base_X_world, self.Robot.Leg[j].XHd.world_base_X_AEP)
					self.Robot.Leg[j].XHd.base_X_foot[:3,3:4] = mX(self.Robot.XHc.base_X_world[:3,:3], 
																	self.Robot.Leg[j].XHd.world_base_X_AEP[:3,3:4])

					# Base to hip frame conversion (rotational components here is incorrect)
					self.Robot.Leg[j].XHd.coxa_X_foot = mX(self.Robot.Leg[j].XHc.coxa_X_base, self.Robot.Leg[j].XHd.base_X_foot)
					self.Robot.Leg[j].XHd.coxa_X_AEP  = self.Robot.Leg[j].XHd.coxa_X_foot.copy()

					## generate spline for transfer phase leg - 
					svalid = self.Robot.Leg[j].generate_spline(self.Robot.Leg[j].XHc.coxa_X_foot[0:3,3], self.Robot.Leg[j].XHd.coxa_X_foot[0:3,3],
																self.Robot.Leg[j].qsurface, False, TRAC_PERIOD, CTR_INTV)
					if (j==1):
						print 'sp : ', np.round(self.Robot.Leg[j].XHc.base_X_foot[0:3,3].flatten(),4)
						print 'ep : ', np.round(self.Robot.Leg[j].XHd.base_X_foot[0:3,3].flatten(),4)
					if (svalid is False):
						self.Robot.invalid = True
					else:	
						# set flag that phase has changed
						self.Robot.Leg[j].phase_change = True

			## Compute task space foot position for all legs
			for j in range (0, self.Robot.active_legs):

				# Transfer phase
				if (self.Robot.Gait.cs[j] == 1 and self.Robot.Leg[j].feedback_state==1):
					# update leg position from generated leg spline
					if (self.Robot.Leg[j].update_from_spline() is False):
						# stops robot from moving i.e. stops support stance until rest of legs finish
						self.Robot.suspend = True

				## Support phase - TODO
				elif (self.Robot.Gait.cs[j] == 0 and self.Robot.suspend == False):
					## Ideal trajectory: doesn't use current leg states
					# prob: IMU feedback would cause change here, increase leg height etc
					# prob: use leg current state, but shouldn't drift

					## determine foot velocity using CoB velocity; world_X_base cause rotation
					self.Robot.Leg[j].base_X_ee.ds.xv = mX( tf.rotation_zyx(self.Robot.world_X_base.cs.wp).transpose() , \
										-( cv-np.cross((mX(tf.rotation_zyx(self.Robot.world_X_base.cs.wp), self.Robot.Leg[j].base_X_ee.ds.xp)).transpose(),wv) ).transpose() )

					## base_X_feet position: integrate base_X_feet velocity
					self.Robot.Leg[j].base_X_ee.ds.xp = self.Robot.Leg[j].base_X_ee.ds.xv*CTR_INTV + self.Robot.Leg[j].base_X_ee.ds.xp
					
					## base to hip transform
					self.Robot.Leg[j].hip_X_ee.ds.xp = self.Robot.Leg[j].base_X_hip_ee(self.Robot.Leg[j].base_X_ee.ds.xp)
					self.Robot.Leg[j].hip_X_ee.ds.xv = np.dot(self.Robot.Leg[j].tf_base_X_hip, self.Robot.Leg[j].base_X_ee.ds.xv)

					## ================================================================================================================== ##
					## determine foot velocity using CoB velocity; world_X_base cause rotation

					v3_world_fr_base_X_foot = ( mX(self.Robot.XHc.world_X_base[:3,:3] , self.Robot.Leg[j].XHd.base_X_foot[:3,3:4]) )
					self.Robot.Leg[j].V6d.world_X_foot[:3] 	= -( v3cv - mC(v3_world_fr_base_X_foot, v3wv) )
					self.Robot.Leg[j].V6d.base_X_foot[:3]	= mX(self.Robot.XHc.base_X_world[:3,:3], self.Robot.Leg[j].V6d.world_X_foot[:3])

					## base_X_feet position: integrate base_X_feet velocity
					self.Robot.Leg[j].XHd.base_X_foot[:3,3:4] += self.Robot.Leg[j].V6d.base_X_foot[:3]*CTR_INTV

					## base to hip transform
					self.Robot.Leg[j].XHd.coxa_X_foot = mX(self.Robot.Leg[j].XHc.coxa_X_base, self.Robot.Leg[j].XHd.base_X_foot)
					self.Robot.Leg[j].V6d.coxa_X_foot[:3] = mX(self.Robot.Leg[j].XHc.coxa_X_base[:3,:3],self.Robot.Leg[j].V6d.base_X_foot[:3])

					if (j==1 or j==0):
						print 'old: ', j, ' ', np.round(self.Robot.Leg[j].base_X_ee.ds.xp.flatten(),4)

					# 	print 'DwXf: ', np.round(v3_world_fr_base_X_foot.flatten(),5)
					# 	print 'DbXf: ', np.round(self.Robot.Leg[j].XHd.base_X_foot[:3,3:4].flatten(),5)
						print 'cXf: ', j, ' ', np.round(self.Robot.Leg[j].XHd.base_X_foot[:3,3:4].flatten(),4)
					# 	print 'CbXf: ', np.round(self.Robot.Leg[j].XHc.base_X_foot[:3,3:4].flatten(),5)
					# 	print 'CcXf: ', np.round(self.Robot.Leg[j].XHc.coxa_X_foot[:3,3:4].flatten(),4)
					# 	print '------------------------------------------------------------------------'
			
			## Task to joint space
			qd = self.Robot.task_X_joint()	
			
			if (self.Robot.invalid == True):
				print 'Error Occured'

			# ends gait phase early if transfer phase completes - TODO: ANOTHER WAY?
			transfer_total = 0; 	
			leg_complete   = 0;

			for j in range(0,6):
				if (self.Robot.Gait.cs[j]==1):
					transfer_total += 1
					if (self.Robot.Leg[j].feedback_state == 2):
						leg_complete += 1

			# triggers only when all transfer phase legs complete and greater than zero
			# > 0: prevents trigger during all leg support
			if (transfer_total == leg_complete and transfer_total > 0 and leg_complete > 0):
				self.alternate_phase()

			## LOGGING: initialise variable and set respective data ##
			qlog 		  = JointState()
			qlog.name 	  = ROBOT_STATE + JOINT_NAME
			qlog.position = v3cp.flatten().tolist() + v3wp.flatten().tolist() + qd.xp.tolist()
			qlog.velocity = v3cv.flatten().tolist() + v3wv.flatten().tolist() + qd.xv.tolist()

			# publish appended joint angles if motion valid
			self.publish_topics(qd, qlog)

			## ============================================================================================================================== ##
			i += 1

		# Finish off transfer legs trajectory onto ground
		self.complete_transfer_trajectory()

		print 'Trajectory executed'
		print 'Desired Goal: ', base_path.X.xp[-1]
		print 'Tracked Goal: ', np.round(cob_X_desired.flatten(),4)
		

	def action_interface(self):
		""" Interface for commanding robot """

		## update robot state prior to starting action
		self.Robot.update_state()
		
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
				self.Robot.support_mode = True
				self.Robot.reset_state  = True
				self.Robot.suspend = False 		# clear suspension flag
				
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
