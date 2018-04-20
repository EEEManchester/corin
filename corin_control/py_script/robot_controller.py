#!/usr/bin/env python

""" Main controller for the robot """
__version__ = '1.0'
__author__  = 'Wei Cheah'

import sys; sys.dont_write_bytecode = True

## Personal libraries
from library import *			# library modules to include
from motion_planning import *	# library modules on motion planning
import control_interface 		# action selection from ROS parameter server

## ROS messages & libraries
import rospy
from sensor_msgs.msg import Imu 			# sub msg for IMU
from sensor_msgs.msg import JointState 		# sub msg for joint states
from std_msgs.msg import Float64 			# pub msg for Gazebo joints
from std_msgs.msg import ByteMultiArray 	# foot contact state
from std_msgs.msg import Float32MultiArray	# foot contact force
from std_msgs.msg import String 			# ui control
import tf 		 							# ROS transform library

## Services
from corin_control.srv import UiState 	# NOT USED

## Robotis ROS msgs for joint control
from robotis_controller_msgs.msg import SyncWriteMultiFloat

#####################################################################

class CorinManager:
	## TEMP VARIABLES
	T_GND_X_WALL = False
	T_GND_X_CHIM = False
	T_WALL_X_GND = False
	T_CHIM_X_GND = False
	W_GND 	= False
	W_WALL = False
	W_CHIM = False

	def __init__(self, initialise=False):
		rospy.init_node('CorinController') 		# Initialises node
		self.rate 	  = rospy.Rate(CTR_RATE)	# Controller rate

		self.Robot 	= robot_class.RobotState() 				# robot class
		self.Action	= control_interface.ControlInterface()	# control action class
		self.Map 	= grid_planner.GridPlanner((1.02,1.02))	# map class 
		# self.Gait = gait_class.GaitClass(GAIT_TYPE) 		# gait class

		self.resting   = False 			# Flag indicating robot standing or resting
		self.on_start  = False 			# variable for resetting to leg suspended in air
		self.interface = "rviz"			# interface to control: gazebo, rviz or robotis hardware
		self.control_mode = "fast" 	# run controller in various mode: 1) normal, 2) fast

		self.ui_state = "hold" 		# user interface for commanding motions

		self.__initialise__()

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

	def ui_callback(self, msg):
		""" user interface control state """
		self.ui_state = msg.data.lower()
		

	def __initialise__(self):
		""" Initialises robot and classes """

		## Identify interface automatically to control: simulation or robotis motors
		# self.interface = 'gazebo' if rospy.has_param('/gazebo/auto_disable_bodies') else 'robotis'

		## set up publisher and subscriber topics
		self.__initialise_topics__()
		# self.__initialise_service__()

		## initialises robot transform
		self.robot_broadcaster.sendTransform( (0.,0.,BODY_HEIGHT), (0.,0.,0.,1.), 
													rospy.Time.now(), "trunk", "world");
		## setup RVIZ JointState topic
		if (self.interface == 'rviz'):
			rospy.set_param(ROBOT_NS + '/standing', True)
			qd = self.Robot.task_X_joint()
			for i in range(0,5):
				self.publish_topics(qd)
				self.rate.sleep()

		## moves robot to nominal position if haven't done so
		try:
			if (rospy.get_param(ROBOT_NS + '/standing') is False):
				raise Exception
		except Exception, e:
			self.default_pose()

		## update and reset robot states
		self.Robot.suspend = False
		self.Robot.update_state(reset=True,control_mode=self.control_mode)

		for j in range(0,self.Robot.active_legs):
			self.Robot.Leg[j].feedback_state = 2 	# trajectory completed

	def __initialise_topics__(self):
		""" Initialises publishers and subscribers """

		##***************** PUBLISHERS ***************##
		self.setpoint_pub_ = rospy.Publisher('/corin/setpoint_states', JointState, queue_size=1)	# LOGGING publisher
		
		## Hardware Specific Publishers ##
		if (ROBOT_NS == 'corin' and (self.interface == 'gazebo' or 
									 self.interface == 'rviz' or
									 self.interface == 'robotis')):
			## Publish to Gazebo
			if (self.interface == 'gazebo'):
				self.joint_pub_ = {}
				for i in range(0,18):
					self.joint_pub_[i] = rospy.Publisher(ROBOT_NS + '/' + JOINT_NAME[i] + '/command', Float64, queue_size=1)

			## Publish to RVIZ (JointState)
			elif (self.interface == 'rviz'):
				self.joint_pub_ = rospy.Publisher(ROBOT_NS + '/joint_states', JointState, queue_size=1)

			## Publish to Dynamixel motors
			elif (self.interface == 'robotis'):
				self.joint_pub_  = rospy.Publisher('/robotis/sync_write_multi_float', SyncWriteMultiFloat, queue_size=1)

			print ">> INITIALISED JOINT TOPICS"
		else:
			# shutdown if wrong hardware selected
			rospy.logerr("INVALID HARDWARE INTERFACE SELECTED!")
			rospy.signal_shutdown("INVALID HARDWARE INTERFACE SELECTED - SHUTTING DOWN")

		## Motion planning and preview for RVIZ ##
		self.map_pub_  = rospy.Publisher(ROBOT_NS + '/point_cloud', PointCloud2, queue_size=1)
		self.path_pub_ = rospy.Publisher(ROBOT_NS + '/path', Path, queue_size=1)
		self.mark_pub_ = rospy.Publisher(ROBOT_NS + '/footholds', MarkerArray, queue_size=1)	# marker array
		self.robot_broadcaster = tf.TransformBroadcaster()	# Transform for robot pose

		##***************** SUBSCRIBERS ***************##
		## Robot State ##
		self.imu_sub_	 = rospy.Subscriber(ROBOT_NS + '/imu/base/data', Imu, self.imu_callback, queue_size=1)
		self.cstate_sub_ = rospy.Subscriber(ROBOT_NS + '/contact_state', ByteMultiArray, self.contact_state_callback, queue_size=1)
		self.cforce_sub_ = rospy.Subscriber(ROBOT_NS + '/contact_force', Float32MultiArray, self.contact_force_callback, queue_size=1)

		## Hardware Specific Subscribers ##
		if (self.interface == 'gazebo' or self.interface == 'rviz'):
			self.joint_sub_  = rospy.Subscriber(ROBOT_NS + '/joint_states', JointState, self.joint_state_callback, queue_size=5)
		elif (self.interface == 'robotis'):
			self.joint_sub_  = rospy.Subscriber('robotis/present_joint_states', JointState, self.joint_state_callback, queue_size=5)

		## User Interface
		self.ui_control_ = rospy.Subscriber(ROBOT_NS + '/ui_execute', String, self.ui_callback, queue_size=1)

		# Sleep for short while for topics to be initiated properly
		rospy.sleep(0.5)

	# def handle_ui_state(self,req):
	# 	""" Service handler for user interface """
	# 	self.ui_state = str(req.state.data)
	# 	msg = String()
	# 	msg.data = "Success"
	# 	return msg

	# def __initialise_service__(self):
	# 	self.ui_service = rospy.Service(ROBOT_NS + '/set_ui_state', UiState, self.handle_ui_state)

	def publish_topics(self, q, q_log=None):
		""" Publish joint position to joint controller topics and
			setpoint topic (for logging all setpoint states) 		"""
		""" Input: 	1) q -> Joint setpoints (position, velocity, 
							acceleration) 	
					2) q_log -> JointState msg with base and
								joint setpoints for logging			"""

		## Publish joint position to robot if valid
		if (self.Robot.invalid is not True):
			self.Robot.active_legs = 6

			if (self.interface == 'gazebo'):
				for n in range(0,self.Robot.active_legs*3):
					qp = Float64()
					qp.data = q.xp[n]
					# Publish joint angles individually
					if (self.control_mode is "normal"):
						self.joint_pub_[n].publish(qp)
					elif (self.control_mode is "fast"):
						pass

			elif (self.interface == 'rviz'):
				dqp = JointState()
				dqp.header.stamp = rospy.Time.now()
				for n in range(0,self.Robot.active_legs*3): 	# loop for each joint
					dqp.name.append(str(JOINT_NAME[n])) 	# joint name
					dqp.position.append(q.xp[n])			# joint angle
				self.joint_pub_.publish(dqp)
				
			elif (self.interface == 'robotis'):
				dqp = SyncWriteMultiFloat()
				dqp.item_name 	= str("goal_position") 	# register to start first write
				dqp.data_length = 4 					# size of register

				for n in range(0,self.Robot.active_legs*3): 	# loop for each joint
					dqp.joint_name.append(str(JOINT_NAME[n])) 	# joint name
					dqp.value.append(q.xp[n])					# joint angle

				self.joint_pub_.publish(dqp)

		qb = self.Robot.P6c.world_X_base.copy()
		self.robot_broadcaster.sendTransform( (qb[0],qb[1],qb[2]), 
												tf.transformations.quaternion_from_euler(qb[3],	qb[4], qb[5]), 
												rospy.Time.now(), "trunk", "world");

		## Publish setpoints to logging topic
		if (q_log is not None):
			self.setpoint_pub_.publish(q_log)

		## Runs controller at desired rate for normal control mode
		if (self.control_mode is "normal" or self.interface is 'robotis'):
			self.rate.sleep()

	def default_pose(self, stand_state=0):
		""" Moves robot to nominal stance (default pose) 		 """
		""" Input: 1) stand_state -> indicates if robot standing """

		## Moves leg into air
		if (self.on_start == False):
			print 'Resetting stance....'
			setpoints = Routine.air_suspend_legs()
			self.on_start = True if self.leg_level_controller(setpoints) else False

		rospy.sleep(0.5)
		
		print 'Moving to nominal stance....'
		if (stand_state):
			setpoints = Routine.shuffle_legs()
		else:
			setpoints = (range(0,6), LEG_STANCE, [0]*6, 2)
		self.leg_level_controller(setpoints)
		rospy.set_param(ROBOT_NS + '/standing', True)

	def complete_transfer_trajectory(self):
		""" Executes remaining leg trajectories in transfer phase """
		""" Checks remaining points on trajectory and finish off  """ 

		## Define Variables ##
		sc_new = 0	# current spline count for transfer phase legs
		sc_max = 0	# maximum spline count among the transfer phase legs
		self.Robot.suspend = True 		# suspend robot for zero trunk movement

		# Check for legs that are in transfer phase
		for j in range(0,6):
			if (self.Robot.Gait.cs[j] == 1):
				# get number of points yet to execute
				sc_new = self.Robot.Leg[j].spline_length - self.Robot.Leg[j].spline_counter
				sc_max = sc_new if (sc_new > sc_max) else sc_max

		try:
			# loop only required via points for legs in transfer phase
			for sc in range(0, sc_max):
				for j in range(0,6):
					if (self.Robot.Gait.cs[j] == 1 and self.Robot.Leg[j].feedback_state==1):
						try:
							self.Robot.Leg[j].update_from_spline()
						except:
							self.Robot.Leg[j].feedback_state = 2
							
				## Task to joint space
				qd = self.Robot.task_X_joint()
				self.publish_topics(qd)
				
			return True
		except Exception, e:
			print "Error: ", e
			print "Returning legs to ground failed"
			return False


	def leg_level_controller(self, setpoints):
		""" Generates and execute trajectory by specified desired leg position 	"""
		""" Input: 	setpoints -> tuple of 4 items:-
							- nleg -> list of corresponding legs to move
							- leg_stance -> list of 3D positions for each leg 
											expressed wrt leg frame
							- leg_phase -> type of trajectory
							- period -> timing of leg execution
			Output: flag -> True: execution complete, False: error occured 		"""

		self.Robot.update_state(control_mode=self.control_mode) 		# get current state

		## Define Variables ##
		te = 0 		# end time of motion
		td = 0 		# duration of trajectory
		tv = False 	# flag if period is an array
		nleg, leg_stance, leg_phase, period = setpoints

		## Check if time intervals is used - set trajectory duration and end time
		try:
			len(period)
			tv = True
			te = np.amax(period[3])
		except TypeError:
			td = te = period
			period = [[0.,2.]]*6 	# change period from int to list
		
		# Number of points based on duration of trajectory
		npc = int(te/CTR_INTV+1)
		
		try:
			## Generate spline for each leg
			for i in range(len(nleg)):
				j = nleg[i]
				td = period[j][1]-period[j][0] if tv else td

				self.Robot.Leg[j].XHd.coxa_X_foot[0:3,3] = leg_stance[j]
				svalid = self.Robot.Leg[j].generate_spline('leg', np.array([0.,0.,1.]), leg_phase[j], False, td, CTR_INTV)
				if (svalid is False):
					self.Robot.invalid = True
					raise Exception, "Trajectory Invalid"

			## Unstack trajectory and execute
			for p in range(0, npc):
				ti = p*CTR_INTV 	# current time in seconds
				for i in range(len(nleg)):
					j = nleg[i]
					# identify start and end of transfer trajectory
					if (ti >= period[j][0] and ti <= period[j][1]):
						self.Robot.Leg[j].update_from_spline(); 	# set cartesian position for joint kinematics

				qd = self.Robot.task_X_joint()
				self.publish_topics(qd)

		except Exception, e:
			print "Exception raised: ", e
			return False

		return True

	def foothold_selection(self, base_path):
		""" Computes foothold based on generated path 	"""
		""" Input: 1) path -> linear and angular path 
			Output: Foothold array 						"""

		def compute_ground_footholds():
			""" Computes NRP position for ground footholds """
			## TODO: this should also hold for normal ground walking
			world_ground_X_base = XHd.world_X_base.copy()
			world_ground_X_base[:2,3:4] = np.zeros((2,1))
			world_ground_X_femur = mX(world_ground_X_base, Leg[j].base_X_femur)

			hy = world_ground_X_femur[2,3] - L3 - 0. 		# h_femur_X_tibia
			yy = np.sqrt(L2**2 - hy**2) 					# world horizontal distance from femur to foot
			by = np.cos(v3wp[0])*(COXA_Y + L1) 				# world horizontal distance from base to femur 
			sy = by + yy									# y_base_X_foot - leg frame
			py = sy*np.sin(np.deg2rad(ROT_BASE_X_LEG[j]+LEG_OFFSET[j])) 	# y_base_X_foot - world frame

			Leg[j].world_base_X_NRP[1,3] = py
			# if (j==4):
			# 	print j, ' cp : ', np.round(v3cp.flatten(),4)
			# 	print j, ' wp : ', np.round(v3wp.flatten(),4)
			# 	print j, ' hy : ', np.round(hy,4)
			# 	print j, 'bXF : ', np.round(Leg[j].base_X_femur[:3,3],4)
			# 	print j, ' yy : ', np.round(yy,4)
			# 	print j, ' by : ', np.round(by,4)
			# 	print j, ' sy : ', np.round(sy,4)
			# 	print j, ' py : ', np.round(py,4)
			# 	print j, ' bXN: ', np.round(Leg[j].world_base_X_NRP[:3,3],4)

		def compute_wall_footholds():
			""" Computes NRP position for wall footholds """ 
			## TODO: make it such that this is only computed once

			h_init = 0.15 	# initial height for wall contact foothold
			d_wall = 0.32 	# initial distance from base to wall for wall walking
			d_chim = 0.36 	# initial distance from base to wall for chimney walking
			v3cp_base = self.Robot.P6c.world_X_base[0:3].copy()	# previous CoB position
			v3wp_base = self.Robot.P6c.world_X_base[3:6].copy()	# previous CoB orientation
			initial_foothold = [np.zeros(3)]*6
			final_foothold 	 = [np.zeros(3)]*6

			if (self.T_GND_X_WALL is True):
				for ji in range(0,3):
					final_foothold[ji] = np.array([v3cp_base[0]+ Leg[j].base_X_NRP[0,3], v3cp_base[1]+d_wall, 0.69])
					
			elif (self.T_WALL_X_GND is True):
				for ji in range(0,3):
					final_foothold[ji] = np.array([v3cp_base[0]+ Leg[j].base_X_NRP[0,3], d_wall, h_init])	
				
			elif (self.T_GND_X_CHIM is True):
				for ji in range(0,3):
					final_foothold[ji] = np.array([v3cp[0]+ Leg[j].base_X_NRP[0,3], v3cp_base[1]+ d_chim, BODY_HEIGHT])
				
				for ji in range(3,6):
					final_foothold[ji] = np.array([v3cp[0]+ Leg[j].base_X_NRP[0,3], v3cp_base[1]+-d_chim, BODY_HEIGHT])
				

			for ji in range(0,6):
				# set initial foothold and overwrite y & z-component
				initial_foothold[ji] = self.Robot.Leg[ji].XHc.world_X_foot[:3,3].copy()
				initial_foothold[ji][1] = final_foothold[ji][1]

				if (self.T_GND_X_WALL is True):
					initial_foothold[ji][2] = h_init

			delta_foothold   = final_foothold[j] - initial_foothold[j] 	# difference
			delta_foothold_n = delta_foothold/len(base_path.X.t) 		# change per interval

			## New NRP position for instant i
			if (self.T_GND_X_WALL is True or self.T_WALL_X_GND is True):
				world_X_NRP = initial_foothold[j] + (delta_foothold_n*i)
			elif (self.T_GND_X_CHIM is True):
				world_X_NRP = final_foothold[j].copy()

			Leg[j].world_base_X_NRP[:3,3] =  world_X_NRP - v3cp.flatten()

			# if (j == 1):
			# 	print '--------------------------------------------'
			# 	print 'if : ', np.round(initial_foothold[j],6)
			# 	print 'ff : ', np.round(final_foothold[1],4)
			# 	print 'd  : ', np.round(delta_foothold,4)
			# 	print 'wXN: ', np.round(world_X_NRP,4)
				
		## Define Variables ##
		i = 0
		XHd  = robot_transforms.HomogeneousTransform()
		Leg  = [None]*6
		Gait = gait_class.GaitClass(GAIT_TYPE)	# gait class
		gphase_intv  = [] 						# intervals in which gait phase changes
		world_X_base = []
		world_X_footholds = [None]*6
		base_X_footholds  = [None]*6
		world_base_X_NRP  = [None]*6

		Gait.cs = list(self.Robot.Gait.cs) 	# update local gait current state 
		Gait.gphase = self.Robot.Gait.gphase
		v3cp_prev = self.Robot.P6c.world_X_base[0:3].copy()	# previous CoB position
		v3wp_prev = self.Robot.P6c.world_X_base[3:6].copy()	# previous CoB orientation
		world_X_base.append(self.Robot.P6c.world_X_base.flatten())

		## Instantiate leg transformation class & append initial footholds
		for j in range (0, self.Robot.active_legs):
			Leg[j] = robot_transforms.ArrayHomogeneousTransform(j)
			Leg[j].duplicate(self.Robot.Leg[j].XHd)

			world_X_footholds[j] = MarkerList()
			world_X_footholds[j].t.append(0.)
			world_X_footholds[j].xp.append(self.Robot.Leg[j].XHc.world_X_foot[:3,3:4])

			base_X_footholds[j] = MarkerList()
			base_X_footholds[j].t.append(0.)
			base_X_footholds[j].xp.append(self.Robot.Leg[j].XHc.base_X_foot[:3,3:4])

			world_base_X_NRP[j] = MarkerList()
			world_base_X_NRP[j].t.append(0.)
			world_base_X_NRP[j].xp.append(self.Robot.Leg[j].XHc.world_base_X_NRP[:3,3:4])

		## Returns as footholds remain fixed for full support mode
		if (self.Robot.support_mode is True):
			return world_X_base, world_X_footholds, base_X_footholds, world_base_X_NRP
		
		## Cycle through trajectory
		while (i != len(base_path.X.t) and not rospy.is_shutdown()):
			# print i, ' Gait phase ', Gait.cs
			bound_exceed = False

			# cycles through one gait phase
			for m in range(0,int(GAIT_TPHASE*CTR_RATE)+1):
				## Variable mapping to R^(3x1) for base linear and angular time, position, velocity, acceleration
				try:
					v3cp = self.Robot.P6c.world_X_base[0:3] + base_path.X.xp[i].reshape(3,1)
					v3wp = self.Robot.P6c.world_X_base[3:6] + base_path.W.xp[i].reshape(3,1)
					# v3wp = base_path.W.xp[i].reshape(3,1)

					P6d_world_X_base = np.vstack((v3cp,v3wp))

					## update robot's pose
					XHd.update_world_X_base(P6d_world_X_base)
					world_X_base_rot = XHd.world_X_base.copy()
					world_X_base_rot[:3,3:4] = np.zeros((3,1))

					for j in range (0, self.Robot.active_legs):
						## Legs in support phase:
						if (Gait.cs[j] == 0):
							Leg[j].base_X_foot = mX(XHd.base_X_world, Leg[j].world_X_foot)
							Leg[j].coxa_X_foot = mX(Leg[j].coxa_X_base, Leg[j].base_X_foot)

							Leg[j].world_base_X_foot = mX(world_X_base_rot, Leg[j].base_X_foot)
							bound_exceed = self.Robot.Leg[j].check_boundary_limit(Leg[j].world_base_X_foot,
																					Leg[j].world_base_X_NRP)
							
							if (bound_exceed == True):
								# print 'bound exceed on ', j, ' at ', i, i*CTR_INTV
								break

					if (bound_exceed is True):
						## To break through two layers of loops
						break
					else:	
						i += 1

				except IndexError:
					## Trajectory finished, skip support
					print 'Finishing foothold planning at ', i
					break

			## Stack to array next CoB location
			world_X_base.append(P6d_world_X_base.reshape(1,6))
			gphase_intv.append(i)

			## Set foothold for legs in transfer phase
			for j in range (0, self.Robot.active_legs):
				if (Gait.cs[j] == 1 and i <= len(base_path.X.t)):

					## 1) Update NRP
					if (self.T_GND_X_WALL is True):
						## Identify sides for ground or wall contact based on body roll
						delta_w = base_path.W.xp[-1] - base_path.W.xp[0]
						if (delta_w[0] > 0.):
							## Right side ground contact, Left side wall contact
							if (j >= 3):
								compute_ground_footholds()
							else:
								compute_wall_footholds()
						else:
							## Left side in ground contact, Right side wall contact
							if (j >= 3):
								compute_wall_footholds()
							else:
								compute_ground_footholds()
					
					elif (self.T_WALL_X_GND is True):
						if (j >= 3):
							compute_ground_footholds()
						else:
							compute_wall_footholds()

					elif (self.W_WALL is True):
						# SE(3) with linear components and only yaw rotation
						XHy_world_X_base = mX(v3_X_m(P6d_world_X_base[:3]), r3_X_m(np.array([P6d_world_X_base[3],
																					0.,P6d_world_X_base[5]])))

						Leg[j].world_X_NRP = np.dot(XHy_world_X_base, Leg[j].base_X_NRP)
						Leg[j].world_base_X_NRP[:3,3:4] = mX((XHy_world_X_base[:3,:3]), Leg[j].base_X_NRP[:3,3:4])

					else:
						Leg[j].update_world_base_X_NRP(P6d_world_X_base)
					
					## 2) Compute magnitude & vector direction
					wall_trans = True if (self.T_GND_X_WALL is True or self.T_WALL_X_GND is True) else False
					chim_trans = True if (self.T_GND_X_CHIM is True or self.T_CHIM_X_GND is True) else False

					# get surface normal
					## TODO: Change to world_X_NRP
					snorm = self.Map.get_cell_snorm(Leg[j].world_base_X_NRP[0:3,3], 
													wall_trans,
													chim_trans,
													self.W_WALL,
													self.W_CHIM)

					v3_dv = (v3cp - v3cp_prev).flatten() 			# direction vector from previous to current CoB
					v3_pv = v3_dv - (np.dot(v3_dv,snorm))*snorm 	# project direction vector onto plane
					m1_dv = np.linalg.norm(v3_pv) 					# magnitude of direction vector
					v3_uv = np.nan_to_num(v3_pv/m1_dv) 				# unit vector direction

					## TEMP: overwrite last transfer phase on base spline
					if (i == len(base_path.X.t)):
						print 'last ', j
						v3_uv = np.zeros(3)

					## 3) Compute AEP wrt base and world frame					
					Leg[j].world_base_X_AEP[:3,3] = Leg[j].world_base_X_NRP[:3,3] + (v3_uv*STEP_STROKE/2.)
					Leg[j].base_X_AEP[:3,3:4] = mX(XHd.base_X_world[:3,:3], Leg[j].world_base_X_AEP[:3,3:4])
					Leg[j].base_X_NRP[:3,3:4] = mX(XHd.base_X_world[:3,:3], Leg[j].world_base_X_NRP[:3,3:4])

					Leg[j].world_X_foot = mX(XHd.world_X_base, Leg[j].base_X_AEP)
					
					## Get cell height in (x,y) location of world_X_foot
					cell_h = np.array([0.,0.,0.])			# TODO: unstack height from map
					## SIM DATA
					if (self.T_GND_X_WALL is True or self.T_WALL_X_GND is True or self.W_WALL is True):
						## TODO: temporary setting this side height to be equiv. of wall
						if (j < 3):
							cell_h = np.array([0.,0.,1.])
					

					## Cell height above threshold gets ignored as this requires advanced motions
					if (cell_h.item(2) < 0.1 and chim_trans is False and self.W_CHIM is False):
						Leg[j].world_X_foot[2,3] = cell_h.item(2) 	# set z-component to cell height

					## Check if foothold valid for chimney transition
					elif (self.T_GND_X_CHIM is True):

						## SIM DATA
						# if (Leg[j].world_X_foot[0,3]>0.3):
						cell_h[2] = -0.1

						dh = Leg[j].world_X_foot[2,3] - cell_h.item(2)
						# if (j==0):
						# 	print 'before: ', j, np.round(Leg[j].world_X_foot[:3,3],4)
						# 	print np.round(Leg[j].world_base_X_NRP[:3,3],4)
						if (dh > 0.001):
							compute_wall_footholds()
							## Recompute AEP wrt base and world frame					
							# Leg[j].world_base_X_AEP[:3,3] = Leg[j].world_base_X_NRP[:3,3].copy()
							Leg[j].world_base_X_AEP[:3,3] = Leg[j].world_base_X_NRP[:3,3] + (v3_uv*STEP_STROKE/2.)

							Leg[j].base_X_AEP[:3,3:4] = mX(XHd.base_X_world[:3,:3], Leg[j].world_base_X_AEP[:3,3:4])
							Leg[j].base_X_NRP[:3,3:4] = mX(XHd.base_X_world[:3,:3], Leg[j].world_base_X_NRP[:3,3:4])
							
							Leg[j].world_X_foot = mX(XHd.world_X_base, Leg[j].base_X_AEP)

						# if (j==0):
						# 	print 'after: ', j, np.round(Leg[j].world_X_foot[:3,3],4)
						# 	print np.round(Leg[j].world_base_X_NRP[:3,3],4)
							# print np.round(Leg[j].world_base_X_AEP[:3,3],4)

					elif (self.T_CHIM_X_GND is True):
						
						## SIM DATA
						if (Leg[j].world_X_foot[0,3]>0.86):
							cell_h[2] = 0.0
						else:
							cell_h[2] = -0.1

						if (cell_h[2] > -0.01):
							# set to default ground NRP
							KDL = kdl.KDL()
							Leg[j].update_base_X_NRP(KDL.leg_IK(LEG_STANCE[j])) 	
							Leg[j].world_base_X_NRP[:3,3:4] = mX(XHd.world_X_base[:3,:3], Leg[j].base_X_NRP[:3,3:4])

							# recompute AEP
							Leg[j].world_base_X_AEP[:3,3] = Leg[j].world_base_X_NRP[:3,3] + (v3_uv*STEP_STROKE/2.)
							Leg[j].base_X_AEP[:3,3:4] = mX(XHd.base_X_world[:3,:3], Leg[j].world_base_X_AEP[:3,3:4])
						
							Leg[j].world_X_foot = mX(XHd.world_X_base, Leg[j].base_X_AEP)

						# if (j==0):
						# 	print j, Leg[j].world_X_foot[0,3]
						# 	print np.round(Leg[j].world_base_X_AEP[:3,3],4)
					else:
						pass

					## Recompute base_X_AEP based on cell height
					Leg[j].base_X_AEP = mX(XHd.base_X_world, v3_X_m(Leg[j].world_X_foot[:3,3]))

					## Stack to array
					world_X_footholds[j].t.append(i*CTR_INTV)
					world_X_footholds[j].xp.append(Leg[j].world_X_foot[:3,3:4].copy())
					
					world_base_X_NRP[j].t.append(i*CTR_INTV)
					world_base_X_NRP[j].xp.append(Leg[j].world_base_X_NRP[:3,3:4].copy())

					base_X_footholds[j].t.append(i*CTR_INTV)
					base_X_footholds[j].xp.append(Leg[j].base_X_AEP[:3,3:4].copy())
					
					if (j==1):
					# 	print '--------------------------------------------'
					# 	print 'snorm ', snorm
						# print 'v3pv: ', v3_pv.flatten()
						# print 'v3uv: ', v3_uv.flatten()
						print 'bXN:  ', np.round(Leg[j].base_X_NRP[:3,3],4)
						# print 'wXf:  ', np.round(Leg[j].world_X_foot[:3,3], 4)
						# print 'wbXN: ', np.round(Leg[j].world_base_X_NRP[:3,3],4)
						# print 'wbXA: ', np.round(Leg[j].world_base_X_AEP[:3,3],4)
						# print 'wXN:  ', XHd.world_X_base[:3,3] + Leg[j].world_base_X_NRP[:3,3]
						# print 'wXA:  ', XHd.world_X_base[:3,3] + Leg[j].world_base_X_AEP[:3,3]
						# print 'wXb:  ', np.round(XHd.base_X_world,4)
						

			## Alternate gait phase
			Gait.change_phase()
			v3cp_prev = v3cp.copy()
			v3wp_prev = v3wp.copy()

		# Replace last footholds with default ground NRP
		if (self.T_WALL_X_GND is True or self.T_CHIM_X_GND is True):
			KDL = kdl.KDL()
			for j in range(0,6):
				Leg[j].update_base_X_NRP(KDL.leg_IK(LEG_STANCE[j]))
				base_X_footholds[j].xp[-1] = Leg[j].base_X_NRP[:3,3:4].copy()
				world_base_X_NRP[j].xp[-1] = Leg[j].base_X_NRP[:3,3:4].copy()
				world_X_footholds[j].xp[-1] = mX(XHd.world_X_base, Leg[j].base_X_NRP)[:3,3:4]

		## Smoothen base path
		# print gphase_intv
		glength = int(GAIT_TPHASE*CTR_RATE)+1 
		PathGenerator = Pathgenerator.PathGenerator()

		for i in range(1,len(gphase_intv)-1):
			# Identify phases to extend
			if (gphase_intv[i] - gphase_intv[i-1] - glength < 0):
				# print 'Extend', gphase_intv[i]*CTR_INTV, gphase_intv[i-1]*CTR_INTV
				## Set initial and final position/orientation
				## TODO: INCLUDE VELOCITY N ACCELERATION IN INTERPOLATION
				xn_cob = base_path.X.xp[gphase_intv[i-1]]
				xn_cob = np.vstack((xn_cob, base_path.X.xp[gphase_intv[i]]))

				wn_cob = base_path.W.xp[gphase_intv[i-1]]
				wn_cob = np.vstack((wn_cob, base_path.W.xp[gphase_intv[i]]))
				
				## Generate new segment
				new_segment = PathGenerator.generate_base_path(xn_cob, wn_cob, CTR_INTV, np.array([0.,GAIT_TPHASE]))
				
				## Append into existing trajectory
				# base_path.insert(gphase_intv[i-1],gphase_intv[i],new_segment)

		# Plot.plot_2d(base_path.X.t, base_path.X.xp)
		# print np.round(world_X_footholds[0].xp,3)
		return world_X_base, world_X_footholds, base_X_footholds, world_base_X_NRP

	def trajectory_tracking(self, x_cob, w_cob=0):
		""" Generates body trajectory from given via points and
			cycles through the trajectory 							"""
		""" Input: 	1) x_com -> 2D array of linear positions
					2) w_com -> 2D array of angular orientations
			Output: None											"""

		## Define Variables ##
		PathGenerator = Pathgenerator.PathGenerator() 	# path generator for robot's base
		cob_X_desired = np.zeros((3,1)) 	# cob linear location
		cob_W_desired = np.zeros((3,1)) 	# cob angular location
		wXbase_offset = self.Robot.P6c.world_X_base.copy()

		# Set all legs to support mode for bodyposing, prevent AEP from being set
		if (self.Robot.support_mode == True):
			self.Robot.Gait.support_mode()
			PathGenerator.V_MAX = PathGenerator.W_MAX = 0.4
		else:
			self.Robot.Gait.walk_mode()

		# Trajectory for robot's base
		base_path = PathGenerator.generate_base_path(x_cob, w_cob, CTR_INTV)
		# Plot.plot_2d(base_path.X.t, base_path.X.xp)
		# Plot.plot_2d(base_path.W.t, base_path.W.xp)

		## Plan foothold for robot - world_X_base, world_X_footholds not used
		world_X_base, world_X_footholds, base_X_footholds, w_base_X_NRP = self.foothold_selection(base_path)

		## Publish multiple times to ensure it is published 
		for c in range(0,3):
			self.robot_broadcaster.sendTransform( (wXbase_offset[0],wXbase_offset[1],wXbase_offset[2]), 
													tf.transformations.quaternion_from_euler(wXbase_offset[3].copy(), 
																								wXbase_offset[4].copy(), 
																								wXbase_offset[5].copy()), 
													rospy.Time.now(), "trunk", "world") ;
			self.path_pub_.publish(array_to_path(base_path, rospy.Time.now(), "world", wXbase_offset))
			self.mark_pub_.publish(list_to_marker_array(world_X_footholds, rospy.Time.now(), "world"))
			if (self.interface == 'rviz'):
				self.joint_pub_.publish(array_to_joint_states(self.Robot.qc.position, rospy.Time.now(), ""))
			rospy.sleep(0.2)

		## User input
		print '>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>'
		print 'Execute Path? '
		self.ui_state = 'hold'

		# cycle through trajectory points until complete
		i = 1 	# skip first point since spline has zero initial differential conditions
		while (i != len(base_path.X.t) and not rospy.is_shutdown()):

			## User Interface for commanding robot motion state
			# while (self.ui_state == 'hold' or self.ui_state == 'pause'):
			# 	# loop until instructed to start or cancel
			# 	if (rospy.is_shutdown()):
			# 		break
			# 	rospy.sleep(0.15)
			# else:
			# 	if (self.ui_state == 'play'):
			# 		pass
			# 	else:
			# 		print 'Motion Cancelled!'
			# 		return False

			# print 'counting: ', i, len(base_path.X.t)
			
			## suppress trajectory counter as body support suspended
			if (self.Robot.suspend == True):
				i -= 1

			########################################################

			## Variable mapping to R^(3x1) for base linear and angular time, position, velocity, acceleration
			## Next Point along base trajectory
			v3cp = wXbase_offset[0:3] + base_path.X.xp[i].reshape(3,1);
			v3cv = base_path.X.xv[i].reshape(3,1);
			v3ca = base_path.X.xa[i].reshape(3,1);

			v3wp = wXbase_offset[3:6] + base_path.W.xp[i].reshape(3,1);
			v3wv = base_path.W.xv[i].reshape(3,1);
			v3wa = base_path.W.xa[i].reshape(3,1);

			## Next CoB location - NOT USED ATM
			# try:
			# 	v3cp_n = wXbase_offset[0:3] + base_path.X.xp[i+int(CTR_RATE*GAIT_TPHASE)].reshape(3,1);
			# 	v3wp_n = wXbase_offset[3:6] + base_path.W.xp[i+int(CTR_RATE*GAIT_TPHASE)].reshape(3,1);
				
			# except:
			# 	v3cp_n = wXbase_offset[0:3] + base_path.X.xp[-1].reshape(3,1);
			# 	v3wp_n = wXbase_offset[3:6] + base_path.W.xp[-1].reshape(3,1);

			## Tracking desired translations
			if (self.Robot.suspend != True):
				cob_X_desired += v3cv*CTR_INTV
				cob_W_desired += v3wv*CTR_INTV

			## ============================================================================================================================== ##
			## Execution of command ##
			## ==================== ##
			
			## Update robot state
			self.Robot.update_state(control_mode=self.control_mode)
			
			#########################################################################
			## overwrite - TC: use IMU data
			self.Robot.XHc.world_X_base = self.Robot.XHd.world_X_base.copy()
			self.Robot.XHc.base_X_world = self.Robot.XHd.base_X_world.copy()
			self.Robot.P6c.world_X_base = self.Robot.P6d.world_X_base.copy()
			for j in range(0, self.Robot.active_legs):
				self.Robot.Leg[j].P6_world_X_base = self.Robot.P6c.world_X_base.copy()

			#########################################################################

			## Update robot's desired position, velocity & acceleration to next point on spline
			self.Robot.P6d.world_X_base = np.vstack((v3cp,v3wp))
			self.Robot.V6d.world_X_base = np.vstack((v3cv,v3wv)) 			# not used atm
			self.Robot.A6d.world_X_base = np.vstack((v3ca,v3wa)) 			# not used atm
			self.Robot.XHd.update_world_X_base(self.Robot.P6d.world_X_base)

			## Generate trajectory for legs in transfer phase
			for j in range (0, self.Robot.active_legs):
				# Transfer phase
				if (self.Robot.Gait.cs[j] == 1 and self.Robot.Leg[j].transfer_phase_change==False):
					self.Robot.Leg[j].feedback_state = 1 	# set leg to execution mode

					## Using planned foothold arrays
					try:
						self.Robot.Leg[j].XHd.world_X_foot = v3_X_m(world_X_footholds[j].xp.pop(1))
						self.Robot.Leg[j].XHd.base_X_foot  = v3_X_m(base_X_footholds[j].xp.pop(1))
						self.Robot.Leg[j].XHd.world_base_X_NRP = v3_X_m(w_base_X_NRP[j].xp.pop(1))
						self.Robot.Leg[j].XHc.world_base_X_NRP = self.Robot.Leg[j].XHd.world_base_X_NRP.copy()
					except IndexError:
						## TODO: plan on the fly. Currently set to default position
						print 'Leg: ', j, ' No further foothold planned!'
						self.Robot.Leg[j].XHd.base_X_foot = self.Robot.Leg[j].XHd.base_X_NRP.copy()

					## Compute average surface normal from cell surface normal at both footholds
					wall_trans = True if (self.T_GND_X_WALL is True or self.T_WALL_X_GND is True) else False
					snorm_1 = self.Map.get_cell_snorm(self.Robot.Leg[j].XHc.world_X_foot[0:3,3], 
														wall_trans, self.T_GND_X_CHIM, self.W_WALL, self.W_CHIM)
					snorm_2 = self.Map.get_cell_snorm(self.Robot.Leg[j].XHd.world_X_foot[0:3,3], 
														wall_trans, self.T_GND_X_CHIM, self.W_WALL, self.W_CHIM)
					w_snorm = (snorm_1 + snorm_2)/2.

					## Set bodypose in leg class
					self.Robot.Leg[j].XH_world_X_base = self.Robot.XHd.world_X_base.copy()
					
					## Generate transfer spline
					svalid = self.Robot.Leg[j].generate_spline('world', w_snorm, 1, False, GAIT_TPHASE, CTR_INTV)
					
					## Update NRP
					self.Robot.Leg[j].XHd.base_X_NRP[:3,3] = mX(self.Robot.XHd.base_X_world[:3,:3], 
																self.Robot.Leg[j].XHc.world_base_X_NRP[:3,3])
					if (j == 0):
						print 'bXN: ', np.round(self.Robot.Leg[j].XHd.base_X_NRP[:3,3],4)
						print 'wXN: ', np.round(self.Robot.Leg[j].XHc.world_base_X_NRP[:3,3],4)
						# print j, ' Xc: ', np.round(self.Robot.Leg[j].XHc.coxa_X_foot[0:3,3],4)
						# print j, ' Xd: ', np.round(self.Robot.Leg[j].XHd.coxa_X_foot[0:3,3],4)
						# print j, ' Wn: ', np.round(world_norm,4)
						# print j, ' Ln: ', np.round(self.Robot.Leg[j].qsurface,4)
						# print 'wXf: ', np.round(self.Robot.Leg[j].XHd.world_X_foot[0:3,3],4)

					if (svalid is False):
						# set invalid if trajectory unfeasible for leg's kinematic
						self.Robot.invalid = True
					else:	
						# set flag that phase has changed
						self.Robot.Leg[j].transfer_phase_change = True

			# self.Robot.XHd.update_world_X_base(np.vstack((v3cp,v3wp)))

			## Compute task space foot position for all legs
			for j in range (0, self.Robot.active_legs):

				## Transfer phase
				if (self.Robot.Gait.cs[j] == 1 and self.Robot.Leg[j].feedback_state==1):
					## Update leg position from generated leg spline
					if (self.Robot.Leg[j].update_from_spline() is False):
						## Stops body from moving until transfer phase completes
						self.Robot.suspend = True

				## Support phase
				elif (self.Robot.Gait.cs[j] == 0 and self.Robot.suspend == False):
					## Determine foot position wrt base & coxa - REQ: world_X_foot position
					self.Robot.Leg[j].XHd.base_X_foot = mX(self.Robot.XHd.base_X_world, self.Robot.Leg[j].XHc.world_X_foot)
					self.Robot.Leg[j].XHd.coxa_X_foot = mX(self.Robot.Leg[j].XHd.coxa_X_base, self.Robot.Leg[j].XHd.base_X_foot)

			## Task to joint space
			qd = self.Robot.task_X_joint()	
			
			if (self.Robot.invalid == True):
				print 'Error Occured, robot invalid!'
				## TODO: recovery routine
				break
			else:
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
					self.Robot.alternate_phase()
					

				## LOGGING: initialise variable and set respective data ##
				qlog 		  = JointState()
				qlog.name 	  = ROBOT_STATE + JOINT_NAME
				qlog.position = v3cp.flatten().tolist() + v3wp.flatten().tolist() + qd.xp.tolist()
				qlog.velocity = v3cv.flatten().tolist() + v3wv.flatten().tolist() + qd.xv.tolist()
				
				# publish appended joint angles if motion valid
				self.publish_topics(qd, qlog)
				
				i += 1
			## ============================================================================================================================== ##
			
		if (self.Robot.invalid is False):
			# Finish off transfer legs trajectory onto ground
			self.complete_transfer_trajectory()
			self.Robot.update_state(control_mode=self.control_mode)
			
			print 'Trajectory executed'
			print 'Desired Goal: ', np.round(base_path.X.xp[-1],4), np.round(base_path.W.xp[-1],4)
			print 'Tracked Goal: ', np.round(cob_X_desired.flatten(),4), np.round(cob_W_desired.flatten(),4)

			return True
		else:
			print 'Motion invalid, exiting!'
			return False

	def action_interface(self):
		""" Interface for commanding robot """
		## TODO: alternative for this - using action server

		## update robot state prior to starting action
		self.Robot.update_state(control_mode=self.control_mode)
		
		## check action to take
		data = self.Action.action_to_take()

		if (data is not None):
			
			## Clear visualization components ##
			clear_marker = MarkerArray()
			mark = Marker()
			mark.action = 3
			clear_marker.markers.append(mark)
			clear_path = Path()
			clear_path.header.frame_id = 'world'
			self.mark_pub_.publish(clear_marker)
			self.path_pub_.publish(clear_path)

			## Data mapping - for convenience
			x_cob, w_cob, mode, motion_prim = data

			## TEMP: Check motion primitive
			self.T_GND_X_CHIM = self.T_CHIM_X_GND = False
			self.T_GND_X_WALL = self.T_WALL_X_GND = False

			if (motion_prim == 'g2w_transition'):
				self.T_GND_X_WALL = True
				self.W_WALL = True
				self.W_GND  = False
			elif (motion_prim == 'w2g_transition'):
				self.T_WALL_X_GND = True
				self.W_WALL = False
				self.W_GND  = True
			elif (motion_prim == 'g2c_transition'):
				self.T_GND_X_CHIM = True
				self.W_CHIM = True
				self.W_GND  = False
			elif (motion_prim == 'c2g_transition'):
				self.T_CHIM_X_GND = True
				self.W_CHIM = False
				self.W_GND  = True
			# print 'prim: ', self.W_CHIM
			## Stand up if at rest
			if ( (mode == 1 or mode == 2) and self.resting == True):
				print 'Going to standup'
				self.default_pose()

			## condition for support (1), walk (2), reset (3)
			if (mode == 1):
				print 'Support mode'
				prev_suspend = self.Robot.suspend
				self.Robot.support_mode = True
				self.Robot.suspend = False 		# clear suspension flag
				success = self.trajectory_tracking(x_cob, w_cob)
				self.Robot.suspend = prev_suspend
				## Move back to nominal position
				# self.default_pose()
				# self.default_pose(1) 	# required for resetting stance for walking

			elif (mode == 2):
				print 'walk mode'
				self.Robot.support_mode = False
				
				success = self.trajectory_tracking(x_cob, w_cob)
				self.Robot.alternate_phase()

			elif (mode == 3):
				if (self.resting == False): 	# rest robot
					print 'Rest'
					self.routine_air_suspend_legs()
					self.resting = True

				elif (self.resting == True): 	# standup from rest
					print 'Standup'
					self.default_pose()
					self.resting = False

		else:
			rospy.sleep(0.5)
