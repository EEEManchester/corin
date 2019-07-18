#!/usr/bin/env python

""" Main controller for the robot """
__version__ = '1.0'
__author__  = 'Wei Cheah'

import sys; sys.dont_write_bytecode = True

## Personal libraries
from corin_control import *			# library modules to include
# from motion_planning import *	# library modules on motion planning
import control_interface 		# action selection from ROS parameter server
from rviz_visual import *
from grid_planner_core.grid_map import GridMap
from grid_planner_core.numpy_to_rosmsg import *
from grid_planner_core.nominal_planner import PathPlanner

## ROS messages & libraries
import rospy
from sensor_msgs.msg import Imu 			# sub msg for IMU
from sensor_msgs.msg import JointState 		# sub msg for joint states
from std_msgs.msg import Float64 			# pub msg for Gazebo joints
from std_msgs.msg import ByteMultiArray 	# foot contact state
from std_msgs.msg import Float32MultiArray	# foot contact force
from std_msgs.msg import String 			# ui control
import tf 		 							# ROS transform library
from geometry_msgs.msg import Vector3Stamped # Force sensor readings
from geometry_msgs.msg import Vector3 		# debugging
# from geometry_msgs.msg import PolygonStamped
# from geometry_msgs.msg import Point32
# from geometry_msgs.msg import Polygon

## Services
from service_handler import ServiceHandler
# from corin_control.srv import UiState 	# NOT USED
from corin_msgs.srv import RigidBody # compute CRBI & CoM
from grid_planner_msgs.srv import PlanPath

## Robotis ROS msgs for joint control
from robotis_controller_msgs.msg import SyncWriteMultiFloat

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

# from grid_planner_msgs.msg import MotionPlan as RosMotionPlan
from corin_msgs.msg import LoggingState

#####################################################################

class CorinManager:
	def __init__(self, initialise=False):
		rospy.init_node('CorinController') 		# Initialises node
		self.rate 	  = rospy.Rate(CTR_RATE)	# Controller rate

		self.Action	= control_interface.ControlInterface()	# control action class
		self.Robot 	= robot_class.RobotState() 				# robot class
		self.GridMap   = None#GridMap()
		self.ForceDist = QPForceDistribution()

		self.resting   = False 		# Flag indicating robot standing or resting
		self.on_start  = False 		# variable for resetting to leg suspended in air
		self.interface = "rviz"		# interface to control: 'rviz', 'gazebo' or 'robotis'
		self.control_rate = "normal" 	# run controller in either: 1) normal, or 2) fast
		self.control_loop = "open" 	# run controller in open or closed loop

		self.ui_state = "hold" 		# user interface for commanding motions
		self.MotionPlan = MotionPlan()
		self.Visualizer = RvizVisualise() 	# visualisation for rviz

		self.Planner = None#PathPlanner(self.GridMap)

		self.__initialise__()

	def joint_state_callback(self, msg):
		""" robot joint state callback """
		self.Robot.qc = msg

	def imu_callback(self, msg):
		""" imu callback """
		if self.IMU == "LORD":
			# only for LORD IMU (to put the orientation from NED to the correct frame)
			o = msg.orientation
			q0 = np.array([o.w, o.x, o.y, o.z])
			q = quaternion_multiply(np.array([0, 1, 0, 0]), q0)
			msg.orientation = Quaternion(q[1], q[2], q[3], q[0])

		self.Robot.imu = msg
		self.Robot.estimate_state()

		# Hassan - debugging
		if(self.Robot.state_estimator.calibrated):
			try:
				self.pub_imu_v.publish(Vector3(*(1.0*self.Robot.state_estimator.v).tolist()))
				self.pub_imu_r.publish(Vector3(*(1.0*self.Robot.state_estimator.r).tolist()))
				self.pub_imu_q.publish(Vector3(*self.Robot.state_estimator.get_fixed_angles().tolist()))
				self.pub_imu_bf.publish(Vector3(*self.Robot.state_estimator.bf.tolist()))
				self.pub_imu_bw.publish(Vector3(*self.Robot.state_estimator.bw.tolist()))
			except ROSException, e:
				rospy.logerror(e)

	def contact_force_callback_0(self, msg):
		data = msg.vector
		self.Robot.cforce[0:3] = np.array([data.x, data.y, data.z]) 
		
	def contact_force_callback_1(self, msg):
		data = msg.vector
		self.Robot.cforce[3:6] = np.array([data.x, data.y, data.z]) 

	def contact_force_callback_2(self, msg):
		data = msg.vector
		self.Robot.cforce[6:9] = np.array([data.x, data.y, data.z]) 

	def contact_force_callback_3(self, msg):
		data = msg.vector
		self.Robot.cforce[9:12] = np.array([data.x, data.y, data.z]) 

	def contact_force_callback_4(self, msg):
		data = msg.vector
		self.Robot.cforce[12:15] = np.array([data.x, data.y, data.z]) 

	def contact_force_callback_5(self, msg):
		data = msg.vector
		self.Robot.cforce[15:18] = np.array([data.x, data.y, data.z]) 

	def contact_force_callback(self, msg):
		""" Gazebo foot contact force """
		self.Robot.cforce = msg.data

	def ui_callback(self, msg):
		""" user interface control state """
		self.ui_state = msg.data.lower()

	def robot_state_callback(self, msg):
		""" robot state from gazebo """

		idx = msg.name.index(ROBOT_NS)
		rpy = np.array(euler_from_quaternion([msg.pose[idx].orientation.w, msg.pose[idx].orientation.x,
												msg.pose[idx].orientation.y, msg.pose[idx].orientation.z], 'sxyz'))
		# self.Robot.P6c.world_X_base[0] = msg.pose[idx].position.x + self.Robot.P6c.world_X_base_offset.item(0)
		# self.Robot.P6c.world_X_base[1] = msg.pose[idx].position.y + self.Robot.P6c.world_X_base_offset.item(1)
		# self.Robot.P6c.world_X_base[2] = msg.pose[idx].position.z
		# self.Robot.P6c.world_X_base[3] = rpy.item(0)
		# self.Robot.P6c.world_X_base[4] = rpy.item(1)
		# self.Robot.P6c.world_X_base[5] = rpy.item(2)

		# self.Robot.cb_V6c.world_X_base[0] = msg.twist[idx].linear.x
		# self.Robot.cb_V6c.world_X_base[1] = msg.twist[idx].linear.y
		# self.Robot.cb_V6c.world_X_base[2] = msg.twist[idx].linear.z
		# self.Robot.cb_V6c.world_X_base[3] = msg.twist[idx].angular.x
		# self.Robot.cb_V6c.world_X_base[4] = msg.twist[idx].angular.y
		# self.Robot.cb_V6c.world_X_base[5] = msg.twist[idx].angular.z

	def __initialise__(self):
		""" Initialises robot and classes """

		## Setup hardware
		self.__initialise_imu__()

		## set up publisher and subscriber topics
		self.__initialise_topics__()
		self.__initialise_services__()

		## initialises robot transform
		self.Visualizer.robot_broadcaster.sendTransform( (0.,0.,BODY_HEIGHT), (0.,0.,0.,1.),
													rospy.Time.now(), "trunk", "world");
		## setup RVIZ JointState topic
		if (self.interface == 'rviz'):
			rospy.set_param(ROBOT_NS + '/standing', True)
			qd, err_list = self.Robot.task_X_joint()
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
		self.Robot.update_state(reset=True,control_mode=self.control_rate)

		for j in range(0,self.Robot.active_legs):
			self.Robot.Leg[j].feedback_state = 2 	# trajectory completed

		## Override control flag states
		# if (self.interface == 'rviz'):
		# 	self.control_loop = 'open'

		## Set map to that available in service
		print '==============================================='
		try:
			mapname = rospy.get_param('/GridMap/map_name')
		except Exception, e:
			mapname = 'flat'
		# print 'Grid Map set to : ', mapname
		self.GridMap  = GridMap(mapname)
		self.Planner = PathPlanner(self.GridMap)

	def __initialise_topics__(self):
		""" Initialises publishers and subscribers """

		# print 'Setting up Topics: ', 
		##***************** PUBLISHERS ***************##
		self.log_sp_pub_ = rospy.Publisher(ROBOT_NS + '/log_setpoints', LoggingState, queue_size=1)	# LOGGING setpoint publisher
		self.log_ac_pub_ = rospy.Publisher(ROBOT_NS + '/log_actual', LoggingState, queue_size=1)		# LOGGING actual publisher
		self.log_er_pub_ = rospy.Publisher(ROBOT_NS + '/log_error', LoggingState, queue_size=1)		# LOGGING error publisher

		self.stability_pub_ = rospy.Publisher(ROBOT_NS + '/stability_margin', Float64, queue_size=1) # Stability margin publisher
		self.cstate_pub_  	= rospy.Publisher(ROBOT_NS + '/contact_state', ByteMultiArray, queue_size=1)

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

		else:
			# shutdown if wrong hardware selected
			rospy.logerr("INVALID HARDWARE INTERFACE SELECTED!")
			rospy.signal_shutdown("INVALID HARDWARE INTERFACE SELECTED - SHUTTING DOWN")

		##***************** SUBSCRIBERS ***************##
		## Robot State ##
		self.imu_sub_	 = rospy.Subscriber(ROBOT_NS + '/imu/base/data', Imu, self.imu_callback, queue_size=1)
		
		## Hardware Specific Subscribers ##
		if (self.interface == 'robotis'):
			self.joint_sub_  = rospy.Subscriber('robotis/present_joint_states', JointState, self.joint_state_callback, queue_size=5)
			self.cforce_sub_ = {}
			self.cforce_sub_[0] = rospy.Subscriber('/force_sensor_' + str(0), Vector3Stamped, self.contact_force_callback_0, queue_size=1)
			self.cforce_sub_[1] = rospy.Subscriber('/force_sensor_' + str(1), Vector3Stamped, self.contact_force_callback_1, queue_size=1)
			self.cforce_sub_[2] = rospy.Subscriber('/force_sensor_' + str(2), Vector3Stamped, self.contact_force_callback_2, queue_size=1)
			self.cforce_sub_[3] = rospy.Subscriber('/force_sensor_' + str(3), Vector3Stamped, self.contact_force_callback_3, queue_size=1)
			self.cforce_sub_[4] = rospy.Subscriber('/force_sensor_' + str(4), Vector3Stamped, self.contact_force_callback_4, queue_size=1)
			self.cforce_sub_[5] = rospy.Subscriber('/force_sensor_' + str(5), Vector3Stamped, self.contact_force_callback_5, queue_size=1)
		else:
			self.joint_sub_  = rospy.Subscriber(ROBOT_NS + '/joint_states', JointState, self.joint_state_callback, queue_size=5)
			if (self.interface == 'gazebo'):
				self.robot_sub_ = rospy.Subscriber('/gazebo/model_states', ModelStates, self.robot_state_callback, queue_size=5)
				self.cforce_sub_ = rospy.Subscriber(ROBOT_NS + '/contact_force', Float32MultiArray, self.contact_force_callback, queue_size=1)

		## User Interface
		self.ui_control_ = rospy.Subscriber(ROBOT_NS + '/ui_execute', String, self.ui_callback, queue_size=1)

		# Sleep for short while for topics to be initiated properly
		rospy.sleep(0.5)
		# print 'Completed'

	def __initialise_services__(self):
		""" Initialises services used in manager """

		# print 'Setting up Services: ', 		
		self.grid_serv_ = ServiceHandler('/GridMap/query_map', PlanPath)
		# self.rbim_serv_ = ServiceHandler('/Corin/get_rigid_body_matrix', RigidBody)
		# print 'Completed'

	def __initialise_imu__(self):
		""" Selects IMU used and initialises parameters """

		if self.interface == "robotis":
			self.IMU = "LORD"
		else:
			self.IMU = "gazebo"

		if self.IMU == "LORD":
			self.Robot.state_estimator.IMU_r = 0.001*np.array([-123, 5.25, 24.5]) # Body frame origin relative to IMU frame, described in the IMU frame
			self.Robot.state_estimator.IMU_R = np.array([[-1, 0, 0],
															[0, 1, 0],
															[0, 0, -1]])	# Rotation matrix mapping vectors from IMU frame to body frame
		elif self.IMU == "ODROID":
			self.Robot.state_estimator.IMU_r = 0.001*np.array([-4, -8, -27]) #np.zeros(3)
			self.Robot.state_estimator.IMU_R = np.eye(3)

		else:
			# Gazebo
			self.Robot.state_estimator.Qf = 0.1 * np.eye(3)
			self.Robot.state_estimator.Qw = 0.01 * np.eye(3)
			self.Robot.state_estimator.IMU_r = 0.001*np.array([0, 0, 0]) #np.zeros(3)
			self.Robot.state_estimator.IMU_R = np.eye(3)

		# For debugging
		self.pub_imu_v = rospy.Publisher('imu_v', Vector3, queue_size=1)
		self.pub_imu_r = rospy.Publisher('imu_r', Vector3, queue_size=1)
		self.pub_imu_q = rospy.Publisher('imu_q', Vector3, queue_size=1)
		self.pub_imu_bf = rospy.Publisher('imu_bf', Vector3, queue_size=1)
		self.pub_imu_bw = rospy.Publisher('imu_bw', Vector3, queue_size=1)

	def publish_topics(self, q, qlog=None, q_trac=None):
		""" Publish joint position to joint controller topics and
			setpoint topic (for logging all setpoint states) 		"""
		""" Input: 	1) q -> Joint setpoints (position, velocity,
							acceleration)
					2) qlog -> JointState msg with base and
								joint setpoints for logging			"""

		## Publish joint position to robot if valid
		if (self.Robot.invalid is not True):
			self.Robot.active_legs = 6

			if (self.interface == 'gazebo'):
				for n in range(0,self.Robot.active_legs*3):
					qp = Float64()
					qp.data = q.xp[n]
					# Publish joint angles individually
					if (self.control_rate is "normal"):
						self.joint_pub_[n].publish(qp)
					elif (self.control_rate is "fast"):
						pass
				
			elif (self.interface == 'rviz'):
				self.Visualizer.publish_robot_pose(self.Robot.P6d.world_X_base)
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

		self.Visualizer.publish_support_polygon(self.Robot.SM.convex_hull)
		self.Visualizer.publish_com(self.Robot.Rbdl.com + self.Robot.P6c.world_X_base[:3])
		self.stability_pub_.publish(self.Robot.SM.min)

		## Publish setpoints to logging topic
		# if (qlog is not None):
		# 	self.log_sp_pub_.publish(qlog)
		# if self.interface != 'rviz' and 
		if qlog is not None:
			qlog_ac, qlog_er = self.set_log_actual()
			self.log_sp_pub_.publish(qlog)
			self.log_ac_pub_.publish(qlog_ac)
			self.log_er_pub_.publish(qlog_er)
			cstate_msg = ByteMultiArray(data = self.Robot.cstate)
			self.cstate_pub_.publish(cstate_msg)

		## Runs controller at desired rate for normal control mode
		if (self.control_rate is "normal" or self.interface is 'robotis'):
			self.rate.sleep()

	def default_pose(self, stand_state=0, leg_stance=None):
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
			# Resets leg in standup position to nominal stance
			setpoints = Routine.shuffle_legs(leg_stance)
		else:
			# Stands up from sit down position
			leg_stance = self.Robot.set_leg_stance(STANCE_WIDTH, BODY_HEIGHT, self.Robot.stance_offset, 'flat')
			setpoints = (range(0,6), leg_stance, [0]*6, GAIT_TPHASE)

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
				qd, err_list = self.Robot.task_X_joint()
				self.publish_topics(qd)

			return True
		except Exception, e:
			print colored("ERROR: %s" %e, 'red')
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

		print colored("INFO: Leg-level controller", 'green')
		self.Robot.update_state(control_mode=self.control_rate) 		# get current state

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
			period = [[0.,4.]]*6 	# change period from int to list

		# Number of points based on duration of trajectory
		npc = int(te/CTR_INTV+1)

		## Generate spline for each leg
		for i in range(len(nleg)):
			j = nleg[i]
			td = period[j][1]-period[j][0] if tv else td

			self.Robot.Leg[j].XHd.coxa_X_foot[0:3,3] = leg_stance[j]
			# svalid = self.Robot.Leg[j].generate_spline('leg', np.array([0.,0.,1.]), np.array([0.,0.,1.]), leg_phase[j], False, td, CTR_INTV)
			svalid = self.Robot.Leg[j].generate_spline('leg', np.array([0.,0.,1.]), np.array([0.,0.,1.]), 1, False, GAIT_TPHASE, CTR_INTV)
			if (svalid is False):
				self.Robot.invalid = True
				raise Exception, "Trajectory Invalid"

		## Unstack trajectory and execute
		for p in range(0, npc-1):
			ti = p*CTR_INTV 	# current time in seconds
			for i in range(len(nleg)):
				j = nleg[i]
				# identify start and end of transfer trajectory
				if (ti >= period[j][0] and ti <= period[j][1]):
					self.Robot.Leg[j].update_from_spline(); 	# set cartesian position for joint kinematics

			qd, err_list = self.Robot.task_X_joint()
			self.publish_topics(qd)

		# except Exception, e:
		# 	print "Exception raised: ", e
		# 	return False

		return True

	def path_tracking(self, x_cob, w_cob=0):
		""" Generates body trajectory from given via points and
			cycles through the trajectory 							"""
		""" Input: 	1) x_com -> 2D array of linear positions
					2) w_com -> 2D array of angular orientations
			Output: None											"""

		## Define Variables ##
		PathGenerator = Pathgenerator.PathGenerator() 	# path generator for robot's base

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
		# mplan = self.Map.foothold_planner(base_path, self.Robot)
		motion_plan = MotionPlan()
		motion_plan.qb = base_path
		self.main_controller(motion_plan)

	def action_interface(self, motion):
		""" Interface for commanding robot """
		## TODO: alternative for this - using action server

		## update robot state prior to starting action
		self.Robot.Gait.support_mode()
		self.Robot.update_stability_region()
		self.Robot.update_state(control_mode=self.control_rate)
		self.Robot.Gait.walk_mode()

		## check action to take - rosparam server
		data = self.Action.action_to_take()
		# data = None
		if (data is not None):
			self.Visualizer.clear_visualisation()
			
			## Data mapping - for convenience
			x_cob, w_cob, mode, motion_prim = data
			# mode = 2 	# HARDCODED 
			## Stand up if at rest
			if ( (mode == 1 or mode == 2) and self.resting == True):
				print colored("INFO: Standing up", 'green')
				self.default_pose()

			## condition for support (1), walk (2), reset (3)
			if (mode == 3):
				if (self.resting == False): 	# rest robot
					print colored("INFO: Rest", 'green')
					self.routine_air_suspend_legs()
					self.resting = True

				elif (self.resting == True): 	# standup from rest
					print colored("INFO: Stand", 'green')
					self.default_pose()
					self.resting = False

			elif (mode == 4):
				print colored("INFO: Bodypose", 'green')
				prev_suspend = self.Robot.suspend
				self.Robot.support_mode = True
				self.Robot.suspend = False 		# clear suspension flag
				self.path_tracking(x_cob, w_cob)

				self.Robot.suspend = prev_suspend
				## Move back to nominal position
				# self.default_pose()
				# self.default_pose(1) 	# required for resetting stance for walking

			elif (mode == 5):
				""" Local planning and execution """
				print colored("INFO: Walk", 'green')
				self.Robot.support_mode = False
			
				if motion == 'forward':
					map_offset = (0.33, 0.39)
					d_travel = np.array([0.2,0.,0.,0.,0.,0.]).reshape(6,1)
				elif motion == 'transition':
					map_offset = (0.33, 0.39)
					d_travel = np.array([0.,0.21,0.,0.,0.,0.]).reshape(6,1)
				elif motion == 'chimney':
					map_offset = (0.33, 0.39)
					d_travel = np.array([0.93,0.,0.,0.,0.,0.]).reshape(6,1)
					base_height = 0.0
				elif motion == 'taros':
					map_offset = (0.33, 0.39)
					d_travel = np.array([1.95,0.,0.,0.,0.,0.]).reshape(6,1)
				else:
					map_offset = (0.33, 0.39)
					d_travel = np.zeros(6).reshape(6,1)

				# self.Robot.Fault.status = False
				self.Robot.Gait.support_mode()
				if self.Robot.Fault.status:
					xb = self.Robot.Fault.get_fault_pose().flatten()
					self.Robot.P6c.world_X_base = np.array([map_offset[0], map_offset[1], 
															xb[2], xb[3], xb[4], 0.]).reshape(6,1)
					self.Robot.P6c.world_X_base_offset = np.array([map_offset[0], map_offset[1],
																0.,0.,0.,0.]).reshape(6,1)
					self.Robot.P6d.world_X_base = self.Robot.P6c.world_X_base.copy()
					self.Robot.XHc.update_world_X_base(self.Robot.P6c.world_X_base)
					
					self.Robot.init_fault_stance()
					
				else:
					self.Robot.P6c.world_X_base = np.array([map_offset[0], map_offset[1], 
															BODY_HEIGHT, 0., 0., 0.]).reshape(6,1)
					self.Robot.P6c.world_X_base_offset = np.array([map_offset[0], map_offset[1],
																0.,0.,0.,0.]).reshape(6,1)
					self.Robot.P6d.world_X_base = self.Robot.P6c.world_X_base.copy()
					self.Robot.XHc.update_world_X_base(self.Robot.P6c.world_X_base)
					self.Robot.init_robot_stance()
				self.Robot.Gait.walk_mode()
				if self.interface != "robotis":
					qd, tXj_error = self.Robot.task_X_joint()
					for i in range(0,10):
						self.publish_topics(qd)
				
				ps = self.Robot.P6c.world_X_base
				pf = self.Robot.P6c.world_X_base + d_travel
				# Use Cheah2019 grid planning service if available 
				if (self.grid_serv_.available):
					if (self.GridMap.get_index_exists(ps[0:2]) and self.GridMap.get_index_exists(pf[0:2])):
						start = Pose()
						goal  = Pose()
						start.position.x = ps[0]
						start.position.y = ps[1]
						goal.position.x  = pf[0]
						goal.position.y  = pf[1]
						try:
							print 'Requesting Planning service'
							serial_plan = self.grid_serv_.call(start, goal, String())
							motion_plan = planpath_to_motionplan(serial_plan)
						except:
							motion_plan = None
							print 'Planning Service Failed!'

					else:
						print "Start or End goal out of bounds!"
				# Use reactive planning
				else:
					motion_plan = self.Planner.motion_planning(ps, pf, self.Robot)

				if motion_plan is not None:
					if (self.main_controller(motion_plan)):
						self.Robot.alternate_phase()
					else:
						print 'Planning Failed'	
				else:
					print 'Error: No motion plan! Exiting....'

			elif (mode == 6):
				# Execute motion plan from ros_grid_planner
				print 'Importing motion plan to execute'
				# filename = 'wall_transition.yaml'
				if motion == 'chimney_nom':
					filename = 'chimney_nom.csv'
					mapname  = 'chimney_corner_053'

				rospy.wait_for_service(ROBOT_NS + '/import_motion_plan', 1.0)
				try:
					path_planner = rospy.ServiceProxy(ROBOT_NS + '/import_motion_plan', PlanPath)
					motion_plan  = planpath_to_motionplan( path_planner(Pose(), Pose(), String(filename)) )
					print 'Motion plan service imported!'
					self.GridMap.set_map(mapname)
				except:
					print 'Motion plan service call error!'
				# motion_plan = planpath_to_motionplan( path_planner(Pose(), Pose()) )
				# motion_plan = planpath_to_motionplan(plan_generat)
				# Plot.plot_2d(motion_plan.qb.X.t, motion_plan.qb.X.xp)
				# print 'bias by ', motion_plan.qb_offset
				# self.Robot.P6c.world_X_base = np.array([motion_plan.qb.X.xp[0],
				# 										motion_plan.qb.W.xp[0]]).reshape(6,1)
				self.Robot.P6c.world_X_base = motion_plan.qb_offset.copy()
				self.Robot.P6d.world_X_base = self.Robot.P6c.world_X_base.copy()
				self.Robot.P6c.world_X_base_offset = np.zeros((6,1))#self.Robot.P6c.world_X_base.copy()

				self.Robot.XHc.update_world_X_base(self.Robot.P6c.world_X_base)
				self.Robot.XHd.update_world_X_base(self.Robot.P6d.world_X_base)
				# print np.round(self.Robot.XHd.world_X_base,3)
				# Update to new current foot position
				leg_stance = [None]*6
				for j in range(0,6):
					# print np.round(motion_plan.f_world_X_foot[j].xp[0],3)
					self.Robot.Leg[j].XHd.world_X_foot = v3_X_m(motion_plan.f_world_X_foot[j].xp[0])
					self.Robot.Leg[j].XHd.base_X_foot = mX(self.Robot.XHd.base_X_world, self.Robot.Leg[j].XHd.world_X_foot)
					self.Robot.Leg[j].XHd.coxa_X_foot = mX(self.Robot.Leg[j].XHd.coxa_X_base, self.Robot.Leg[j].XHd.base_X_foot)
					self.Robot.Leg[j].XHc.world_X_foot = self.Robot.Leg[j].XHd.world_X_foot.copy()
					self.Robot.Leg[j].XHc.base_X_foot = self.Robot.Leg[j].XHd.base_X_foot.copy()
					self.Robot.Leg[j].XHc.coxa_X_foot = self.Robot.Leg[j].XHd.coxa_X_foot.copy()
					self.Robot.Leg[j].snorm = self.GridMap.get_cell('norm', self.Robot.Leg[j].XHc.world_X_foot[0:3,3])
					leg_stance[j] = self.Robot.Leg[j].XHd.coxa_X_foot[0:3,3]
					# print j, ' bXf: ', np.round(self.Robot.Leg[j].XHd.base_X_foot[:3,3],3)
				self.Robot.stance_offset = (40, -40)
				self.Robot._initialise(leg_stance)
				self.Robot.qc.position = self.Robot.qd

				# print 'moving to default'
				# self.default_pose(0, leg_stance)
				if motion_plan is not None:
					if (self.main_controller(motion_plan)):
						self.Robot.alternate_phase()
					else:
						print 'Planning Failed'	
				else:
					print 'Error: No motion plan! Exiting....'

		else:
			self.Robot.Gait.support_mode()
			self.main_controller()

	def set_log_setpoint(self, v3cp, v3cv, v3ca, v3wp, v3wv, v3wa, qd, effort=None, forces=None):
		""" Sets the variables for logging """

		qlog = LoggingState()
		qlog.positions = v3cp.flatten().tolist() + v3wp.flatten().tolist() + qd.xp.tolist()
		qlog.velocities = v3cv.flatten().tolist() + v3wv.flatten().tolist() + qd.xv.tolist()
		qlog.accelerations = v3ca.flatten().tolist() + v3wa.flatten().tolist() + qd.xa.tolist()
		qlog.effort = effort
		qlog.forces = forces
		
		qlog.qp_sum_forces = self.ForceDist.sum_forces.flatten().tolist()
		qlog.qp_sum_moments = self.ForceDist.sum_moments.flatten().tolist()
		qlog.qp_desired_forces = self.ForceDist.desired_forces.flatten().tolist()
		qlog.qp_desired_moments = self.ForceDist.desired_moments.flatten().tolist()

		error_forces = self.ForceDist.desired_forces - self.ForceDist.sum_forces
		error_moment = self.ForceDist.desired_moments - self.ForceDist.sum_moments
		
		self.qlog_setpoint = qlog

		return qlog

	def set_log_actual(self):

		cforce_actaul = [0]*18
		cforce_error  = [0]*18
		for j in range(0,6):
			cforce_actaul[j*3:j*3+3] = self.Robot.Leg[j].F6c.world_X_foot[:3]
			cforce_error[j*3:j*3+3]  = abs(self.Robot.Leg[j].F6d.world_X_foot[:3] - self.Robot.Leg[j].F6c.world_X_foot[:3])

		qlog_ac = LoggingState()
		qlog_ac.positions  = self.Robot.P6c.world_X_base[0:3].flatten().tolist() + self.Robot.P6c.world_X_base[3:6].flatten().tolist() + list(self.Robot.qc.position)
		qlog_ac.velocities = self.Robot.V6c.world_X_base[0:3].flatten().tolist() + self.Robot.V6c.world_X_base[3:6].flatten().tolist() + list(self.Robot.qc.velocity)
		
		qlog_ac.effort = list(self.Robot.qc.effort)
		qlog_ac.forces = cforce_actaul

		qlog_er = LoggingState()
		qlog_er.forces = cforce_error

		return qlog_ac, qlog_er

	def get_snorm(self, p, j):
		## surface normal for chimney
		# wall_outer = -1.1 	# chimney width 0.72m
		wall_outer = -0.995 	# chimney width 0.62m
		# wall_outer = -0.93 	# chimney width 0.535m
		wall_inner = -0.4
		if (p[1] < wall_outer and j >= 3):
			snorm = np.array([0.,1.,0.])
		elif (p[1] > wall_outer and j >= 3):
			snorm = np.array([-1.,0.,0.])
		elif (p[1] <= wall_inner and j < 3):
			snorm = np.array([0.,-1.,0.])
		elif (p[1] > wall_inner and j < 3):
			snorm = np.array([1.,0.,0.])
		else:
			print j, p

		## surface normal for convex wall
		# if (j >= 3):
		# 	snorm = np.array([0.,0.,1.])
		# elif (p[1] <= -0.4 and j < 3):
		# 	snorm = np.array([0.,-1.,0.])
		# elif (p[1] > -0.4 and j < 3):
		# 	snorm = np.array([1.,0.,0.])
		# else:
		# 	print j, p

		## surface normal for concave wall
		# if (j < 3):
		# 	snorm = np.array([0.,0.,1.])
		# elif (p[0] <= 0.62 and j >= 3):
		# 	snorm = np.array([0.,1.,0.])
		# elif (p[0] > 0.62 and j >= 3):
		# 	snorm = np.array([-1.,0.,0.])
		# else:
		# 	print j, p

		return snorm