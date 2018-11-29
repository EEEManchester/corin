#!/usr/bin/env python

""" Main controller for the robot """
__version__ = '1.0'
__author__  = 'Wei Cheah'

import sys; sys.dont_write_bytecode = True
from itertools import cycle

## Personal libraries
from corin_control import *			# library modules to include
# from motion_planning import *	# library modules on motion planning
import control_interface 		# action selection from ROS parameter server
from rviz_visual import *
# from grid_planner import grid_map
from grid_planner_core.grid_map import GridMap
from grid_planner_core.numpy_to_rosmsg import *

## ROS messages & libraries
import rospy
from sensor_msgs.msg import Imu 					# sub msg for IMU
from sensor_msgs.msg import JointState 		# sub msg for joint states
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float64 					# pub msg for Gazebo joints
from std_msgs.msg import ByteMultiArray 	# foot contact state
from std_msgs.msg import Float32MultiArray	# foot contact force
from std_msgs.msg import String 			# ui control
import tf 		 							# ROS transform library
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Polygon

## Services
from service_handler import ServiceHandler
# from corin_control.srv import UiState 	# NOT USED
from corin_msgs.srv import RigidBody # compute CRBI & CoM
from corin_msgs.srv import PlanPath

## Robotis ROS msgs for joint control
from robotis_controller_msgs.msg import SyncWriteMultiFloat

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

from corin_msgs.msg import MotionPlan as RosMotionPlan
from corin_msgs.msg import LoggingState
#####################################################################

class CorinManager:
	def __init__(self, initialise=False):
		rospy.init_node('CorinController') 		# Initialises node
		self.rate 	  = rospy.Rate(CTR_RATE)	# Controller rate

		self.Action	= control_interface.ControlInterface()	# control action class	
		self.Robot 	= robot_class.RobotState() 				# robot class
		self.GridMap   = GridMap()
		self.ForceDist = QPForceDistribution()

		self.resting   = False 		# Flag indicating robot standing or resting
		self.on_start  = False 		# variable for resetting to leg suspended in air
		self.interface = "rviz"		# interface to control: gazebo, rviz or robotis hardware
		self.control_rate = "fast" 	# run controller in various mode: 1) normal, 2) fast
		self.control_loop = "open" 	# run controller in open or closed loop

		self.ui_state = "hold" 		# user interface for commanding motions
		self.MotionPlan = MotionPlan()
		self.Visualizer = RvizVisualise() 	# visualisation for rviz

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
		
	def robot_state_callback(self, msg):
		""" robot state from gazebo """

		idx = msg.name.index(ROBOT_NS)
		rpy = np.array(euler_from_quaternion([msg.pose[idx].orientation.w, msg.pose[idx].orientation.x,
												msg.pose[idx].orientation.y, msg.pose[idx].orientation.z], 'sxyz'))
		self.Robot.P6c.world_X_base[0] = msg.pose[idx].position.x + self.Robot.P6c.world_X_base_offset.item(0)
		self.Robot.P6c.world_X_base[1] = msg.pose[idx].position.y + self.Robot.P6c.world_X_base_offset.item(1)
		self.Robot.P6c.world_X_base[2] = msg.pose[idx].position.z
		self.Robot.P6c.world_X_base[3] = rpy.item(0)
		self.Robot.P6c.world_X_base[4] = rpy.item(1)
		self.Robot.P6c.world_X_base[5] = rpy.item(2)

	def __initialise__(self):
		""" Initialises robot and classes """

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

		## set control flag states
		if (self.interface == 'rviz'):
			self.control_loop = 'open'

		## Set map to that available in service
		try:
			self.GridMap.set_map(rospy.get_param('GridMap/map_name'))
		except Exception, e:
			print 'Grid Map has not been set'

	def __initialise_topics__(self):
		""" Initialises publishers and subscribers """

		##***************** PUBLISHERS ***************##
		self.setpoint_pub_ = rospy.Publisher('/corin/setpoint_states', JointState, queue_size=1)	# LOGGING publisher
		self.trajectory_pub_ = rospy.Publisher('/corin/motion_plan', RosMotionPlan, queue_size=1)	# LOGGING publisher
		self.log_pub_ = rospy.Publisher('/corin/log_states', LoggingState, queue_size=1)	# LOGGING publisher
		self.support_poly_pub_ = rospy.Publisher('/corin/support_polygon', PolygonStamped, queue_size=1) # Support polygon publisher
		self.stability_pub_ = rospy.Publisher('/corin/stability_margin', Float64, queue_size=1) # Support polygon publisher

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

		##***************** SUBSCRIBERS ***************##
		## Robot State ##
		self.imu_sub_	 = rospy.Subscriber(ROBOT_NS + '/imu/base/data', Imu, self.imu_callback, queue_size=1)
		self.cstate_sub_ = rospy.Subscriber(ROBOT_NS + '/contact_state', ByteMultiArray, self.contact_state_callback, queue_size=1)
		self.cforce_sub_ = rospy.Subscriber(ROBOT_NS + '/contact_force', Float32MultiArray, self.contact_force_callback, queue_size=1)

		## Hardware Specific Subscribers ##
		if (self.interface == 'robotis'):
			self.joint_sub_  = rospy.Subscriber('robotis/present_joint_states', JointState, self.joint_state_callback, queue_size=5)
		else:
			self.joint_sub_  = rospy.Subscriber(ROBOT_NS + '/joint_states', JointState, self.joint_state_callback, queue_size=5)
			if (self.interface == 'gazebo'):
				self.robot_sub_ = rospy.Subscriber('/gazebo/model_states', ModelStates, self.robot_state_callback, queue_size=5)

		## User Interface
		self.ui_control_ = rospy.Subscriber(ROBOT_NS + '/ui_execute', String, self.ui_callback, queue_size=1)

		# Sleep for short while for topics to be initiated properly
		rospy.sleep(0.5)

	def __initialise_services__(self):
		""" Initialises services used in manager """

		self.grid_serv_ = ServiceHandler('GridMap/query_map', PlanPath)
		self.rbim_serv_ = ServiceHandler('/corin/get_rigid_body_matrix', RigidBody)
		
	def publish_topics(self, q, q_log=None, q_trac=None):
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
					if (self.control_rate is "normal"):
						self.joint_pub_[n].publish(qp)
					elif (self.control_rate is "fast"):
						pass
				self.rstate_cs_pub_.publish(array_to_pose(self.Robot.P6c.world_X_base))
				self.rstate_ds_pub_.publish(array_to_pose(self.Robot.P6d.world_X_base))

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

		self.Visualizer.publish_robot(self.Robot.P6d.world_X_base)
		self.stability_pub_.publish(self.Robot.SM.min)
		
		## Publish setpoints to logging topic
		if (q_log is not None):
			self.setpoint_pub_.publish(q_log)
			self.trajectory_pub_.publish(q_trac)

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
			setpoints = Routine.shuffle_legs(leg_stance)
		else:
			print 'moving here'
			setpoints = (range(0,6), leg_stance, [0]*6, 2)
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

		self.Robot.update_state(control_mode=self.control_rate) 		# get current state

		## Define Variables ##
		te = 0 		# end time of motion
		td = 0 		# duration of trajectory
		tv = False 	# flag if period is an array
		nleg, leg_stance, leg_phase, period = setpoints
		print 'in here'
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
		
		# try:
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
		for p in range(0, npc):
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

	def main_controller(self, motion_plan):

		## Variables ##
		cob_X_desired = np.zeros((3,1)) 	# cob linear location
		cob_W_desired = np.zeros((3,1)) 	# cob angular location
		wXbase_offset = self.Robot.P6c.world_X_base.copy()
		
		## Variable mapping from motion plan
		base_path = motion_plan.qb
		world_X_base = motion_plan.qbp
		gait_phase 	 = motion_plan.gait_phase 
		w_base_X_NRP = motion_plan.f_world_base_X_NRP
		world_X_footholds = motion_plan.f_world_X_foot
		base_X_footholds  = motion_plan.f_base_X_foot
		
		## Publish multiple times to ensure it is published 
		for c in range(0,3):
			self.Visualizer.publish_robot(wXbase_offset)
			self.Visualizer.publish_path(world_X_base)
			self.Visualizer.publish_footholds(world_X_footholds)

			if (self.interface == 'rviz'):
				self.joint_pub_.publish(array_to_joint_states(self.Robot.qc.position, rospy.Time.now(), ""))
			rospy.sleep(0.2)
		# Plot.plot_2d(base_path.X.t, base_path.X.xp)
		# Plot.plot_2d(base_path.W.t, base_path.W.xp)
		## User input
		
		print '>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>'
		print 'Execute Path? '
		if (self.interface == 'gazebo'):
			raw_input('cont')
			self.ui_state = 'play'#'hold'
		else:
			self.ui_state = 'hold'

		try:
			gait_stack = cycle(gait_phase)
			self.Robot.Gait.cs = next(gait_stack)
		except:
			pass
		## Remove first foothold
		for j in range(0,6):
			world_X_footholds[j].xp.pop(0)
		
		# cycle through trajectory points until complete
		i = 1 	# skip first point since spline has zero initial differential conditions
		while (i != len(base_path.X.t) and not rospy.is_shutdown()):
			
			# User Interface for commanding robot motion state
			while (self.ui_state == 'hold' or self.ui_state == 'pause'):
				# loop until instructed to start or cancel
				if (rospy.is_shutdown()):
					break
				rospy.sleep(0.15)
			else:
				if (self.ui_state == 'play'):
					pass
				else:
					print 'Motion Cancelled!'
					return False

			# print 'counting: ', i, len(base_path.X.t)

			## Update robot state
			self.Robot.update_state(control_mode=self.control_rate)
			#########################################################################
			## overwrite - TC: use IMU data
			self.Robot.XHc.world_X_base = self.Robot.XHd.world_X_base.copy()
			self.Robot.XHc.base_X_world = self.Robot.XHd.base_X_world.copy()
			
			if (self.interface == 'rviz'):
				self.Robot.P6c.world_X_base = self.Robot.P6d.world_X_base.copy()
			elif (self.interface != 'rviz' and self.control_loop == 'close'):
				P6e_world_X_base = self.Robot.P6d.world_X_base - self.Robot.P6c.world_X_base
					
			for j in range(0, self.Robot.active_legs):
				self.Robot.Leg[j].P6_world_X_base = self.Robot.P6c.world_X_base.copy()
			
			#########################################################################
			self.Robot.suspend = False
			## suppress trajectory counter as body support suspended
			if (self.Robot.suspend == True):
				i -= 1

			########################################################

			## Variable mapping to R^(3x1) for base linear and angular time, position, velocity, acceleration
			## Next Point along base trajectory
			# v3cp = wXbase_offset[0:3] + base_path.X.xp[i].reshape(3,1);
			v3cp = base_path.X.xp[i].reshape(3,1);
			v3cv = base_path.X.xv[i].reshape(3,1);
			v3ca = base_path.X.xa[i].reshape(3,1);

			# v3wp = wXbase_offset[3:6] + base_path.W.xp[i].reshape(3,1);
			v3wp = base_path.W.xp[i].reshape(3,1);
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
						self.Robot.Leg[j].XHd.world_X_foot = v3_X_m(world_X_footholds[j].xp.pop(0))
						self.Robot.Leg[j].XHd.base_X_foot  = v3_X_m(base_X_footholds[j].xp.pop(0))
						self.Robot.Leg[j].XHd.world_base_X_NRP = v3_X_m(w_base_X_NRP[j].xp.pop(0))
						self.Robot.Leg[j].XHc.world_base_X_NRP = self.Robot.Leg[j].XHd.world_base_X_NRP.copy()

						## Set bodypose in leg class
						self.Robot.Leg[j].XH_world_X_base = self.Robot.XHd.world_X_base.copy()

					except IndexError:
						## TODO: plan on the fly. Currently set to default position
						print 'Leg: ', j, ' No further foothold planned!', i
						gait_tphase = 1.0
						iahead = int(gait_tphase/CTR_INTV)
						## MODIFICATION FOR CHIMNEY_11.CSV
						if i==601:
							iahead = int(gait_tphase/CTR_INTV)*12
							gait_tphase = 12.0
							print 'tphase ', 12
						else:
							gait_tphase = 1.0
						self.Robot.Leg[j].XH_world_X_base = transform_world_X_base(np.array([base_path.X.xp[i+iahead],
																							 base_path.W.xp[i+iahead]]).reshape((6,1)))
						self.Robot.Leg[j].XHd.base_X_foot = mX(np.linalg.inv(self.Robot.Leg[j].XH_world_X_base), 
																self.Robot.Leg[j].XHd.world_X_foot)
						# self.Robot.Leg[j].XHd.base_X_foot = self.Robot.Leg[j].XHd.base_X_NRP.copy()

					## Compute average surface normal from cell surface normal at both footholds
					# sn1 = self.GridMap.get_cell('norm', self.Robot.Leg[j].XHc.world_X_foot[0:3,3], j)
					# sn2 = self.GridMap.get_cell('norm', self.Robot.Leg[j].XHd.world_X_foot[0:3,3], j)
					# print j, np.round(self.Robot.Leg[j].XHc.world_X_foot[0:3,3],3)
					# print j, np.round(self.Robot.Leg[j].XHd.world_X_foot[0:3,3],3)
					sn1 = self.get_snorm(self.Robot.Leg[j].XHc.world_X_foot[0:3,3], j)
					sn2 = self.get_snorm(self.Robot.Leg[j].XHd.world_X_foot[0:3,3], j)
					# print j, sn1, sn2

					## Generate transfer spline
					svalid = self.Robot.Leg[j].generate_spline('world', sn1, sn2, 1, False, gait_tphase, CTR_INTV)
					
					## Update NRP
					self.Robot.Leg[j].XHd.base_X_NRP[:3,3] = mX(self.Robot.XHd.base_X_world[:3,:3], 
																self.Robot.Leg[j].XHc.world_base_X_NRP[:3,3])
					self.Robot.Leg[j].XHd.world_base_X_AEP[:3,3] = mX(self.Robot.XHd.world_X_base[:3,:3], 
																		self.Robot.Leg[j].XHd.base_X_foot[:3,3])
					print 'base: ', np.round(self.Robot.XHd.world_X_base[:3,3],4)
					t_world_X_hip = mX(self.Robot.XHd.world_X_base, 
													self.Robot.Leg[j].XHd.base_X_coxa)
					leg_d = t_world_X_hip[:3,3] - self.Robot.Leg[j].XHd.world_X_foot[:3,3]
					print 'delta ', np.round(leg_d,3)
					## Identify knee up/down - first, checks only if body roll above threshold
					if ((self.Robot.P6d.world_X_base[3] > ROLL_TH and j >= 3) or (self.Robot.P6d.world_X_base[3] < -ROLL_TH and j < 3)):
						x_offset = np.array([COXA_X,0,0]).reshape((3,1)) 	# offset so that search at front of robot
						wclear = self.GridMap.get_median_width(self.Robot.P6d.world_X_base[:3] + x_offset)
						# change to knee down if width clearance below threshold
						if (wclear < WALL_WIDTH_NARROW):
							print j, ' Changing to knee down ', wclear
							if (self.Robot.Leg[j].KDL.knee_up == True):
								print 'Initiate change of knee config. routine'
							self.Robot.Leg[j].KDL.knee_up = False
						else:
							self.Robot.Leg[j].KDL.knee_up = True
					
					if (svalid is False):
						# set invalid if trajectory unfeasible for leg's kinematic
						self.Robot.invalid = True
						print 'Leg Transfer Trajectory Invalid'
					else:	
						# set flag that phase has changed
						self.Robot.Leg[j].transfer_phase_change = True

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
					if (self.control_loop == "open"):
						self.Robot.Leg[j].XHd.base_X_foot = mX(self.Robot.XHd.base_X_world, self.Robot.Leg[j].XHc.world_X_foot)
					
					elif (self.control_loop == "close"):
						## Closed-loop control on bodypose. Move by sum of desired pose and error 
						comp_world_X_base = transform_world_X_base(self.Robot.P6d.world_X_base + K_BP*P6e_world_X_base)
						comp_base_X_world = np.linalg.inv(comp_world_X_base)
						self.Robot.Leg[j].XHd.base_X_foot = mX(comp_base_X_world, self.Robot.Leg[j].XHc.world_X_foot)
							
					self.Robot.Leg[j].XHd.coxa_X_foot = mX(self.Robot.Leg[j].XHd.coxa_X_base, self.Robot.Leg[j].XHd.base_X_foot)
					# print j, ' bXw: ', np.round(self.Robot.XHd.base_X_world,3)
					# print j, ' wXf: ', np.round(self.Robot.Leg[j].XHc.world_X_foot[0:3,3],3)
					# print j, ' bXf: ', np.round(self.Robot.Leg[j].XHd.base_X_foot[0:3,3],3)
					# print j, ' cXf: ', np.round(self.Robot.Leg[j].XHd.coxa_X_foot[0:3,3],3)
					# print '========================================================='
			
			## Set stability polygon
			support_polygon = Polygon()
			foothold_list = []
			for j in range(0,6):
				if (self.Robot.Gait.cs[j]==0):
					foothold_list.append(self.Robot.Leg[j].XHd.world_X_foot[0:3,3])
					# Point3D = Point32()
					# Point3D.x = self.Robot.Leg[j].XHd.world_X_foot[0,3]
					# Point3D.y = self.Robot.Leg[j].XHd.world_X_foot[1,3]
					# Point3D.z = self.Robot.Leg[j].XHd.world_X_foot[2,3]
			self.Visualizer.publish_support_polygon(foothold_list)
			# 		support_polygon.points.append(Point3D)
			# sp_data = PolygonStamped()
			# sp_data.header.frame_id = 'world'
			# sp_data.polygon = support_polygon
			# self.support_poly_pub_.publish(sp_data)

			## Task to joint space
			qd, tXj_error = self.Robot.task_X_joint()	

			if (self.Robot.invalid == True):
				print 'Error Occured, robot invalid! ', tXj_error
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
				print i
				if (self.Robot.Gait.cs==[0,0,0,0,0,0] and i%50==0):
					transfer_total = 1
					leg_complete = 1
				if (transfer_total == leg_complete and transfer_total > 0 and leg_complete > 0):
					## FORCED CHANGED FOR wp=0.62 (Chimney_10.csv)
					# if i==300:
					# 	self.Robot.Gait.phases = []
					# 	self.Robot.Gait.phases.append([0,0,0,0,0,1])
					# 	self.Robot.Gait.phases.append([0,0,0,1,0,0])
					# 	self.Robot.Gait.phases.append([0,0,1,0,0,0])
					# 	self.Robot.Gait.phases.append([0,1,0,0,0,0])
					# 	self.Robot.Gait.phases.append([1,0,0,0,0,0])
					# 	self.Robot.Gait.phases.append([0,0,0,0,1,0])
					# elif i==600:
					# 	self.Robot.Gait.phases = []
					# 	self.Robot.Gait.phases.append([0,0,0,0,0,0])
					# 	self.Robot.Gait.phases.append([0,0,0,0,1,0])
					# 	self.Robot.Gait.phases.append([0,0,0,1,0,0])
					# 	self.Robot.Gait.phases.append([0,0,1,0,0,0])
					# 	self.Robot.Gait.phases.append([0,1,0,0,0,0])
					# 	self.Robot.Gait.phases.append([1,0,0,0,0,0])
					# elif i==2100:
					# 	self.Robot.Gait.phases = []
					# 	self.Robot.Gait.phases.append([0,0,0,0,0,1])
					# 	self.Robot.Gait.phases.append([0,0,0,0,1,0])
					# 	self.Robot.Gait.phases.append([0,0,0,1,0,0])
					# 	self.Robot.Gait.phases.append([0,0,1,0,0,0])
					# 	self.Robot.Gait.phases.append([0,1,0,0,0,0])
					# 	self.Robot.Gait.phases.append([1,0,0,0,0,0])
					## FORCED CHANGED FOR wp=0.64 (Chimney_11.csv)
					# if i==300:
					# 	self.Robot.Gait.phases = []
					# 	self.Robot.Gait.phases.append([0,0,0,0,0,1])
					# 	self.Robot.Gait.phases.append([0,0,0,0,0,0])
					# 	self.Robot.Gait.phases.append([0,0,0,1,0,0])
					# 	self.Robot.Gait.phases.append([0,0,1,0,0,0])
					# 	self.Robot.Gait.phases.append([0,1,0,0,0,0])
					# 	self.Robot.Gait.phases.append([1,0,0,0,0,0])
					# elif i==600:
					# 	self.Robot.Gait.phases = []
					# 	self.Robot.Gait.phases.append([0,0,0,0,1,0])
					# elif i==900:
					# 	self.Robot.Gait.np = 5
					# 	self.Robot.Gait.phases = []
					# 	self.Robot.Gait.phases.append([0,0,0,0,0,0])
					# 	self.Robot.Gait.phases.append([0,0,0,0,1,0])
					# 	self.Robot.Gait.phases.append([0,0,0,1,0,0])
					# 	self.Robot.Gait.phases.append([0,0,1,0,0,0])
					# 	self.Robot.Gait.phases.append([0,1,0,0,0,0])
					# 	self.Robot.Gait.phases.append([1,0,0,0,0,0])
					# elif i==2100:
					# 	self.Robot.Gait.phases = []
					# 	self.Robot.Gait.phases.append([0,0,0,0,0,1])
					# 	self.Robot.Gait.phases.append([0,0,1,0,0,0])
					# 	self.Robot.Gait.phases.append([0,0,0,0,1,0])
					# 	self.Robot.Gait.phases.append([0,0,0,1,0,0])
					# 	self.Robot.Gait.phases.append([0,1,0,0,0,0])
					# 	self.Robot.Gait.phases.append([1,0,0,0,0,0])
					## FORCED CHANGED FOR wp=0.66 (Chimney_12.csv)
					if i==600:
						self.Robot.Gait.phases = []
						self.Robot.Gait.phases.append([0,0,0,0,1,0])
					elif i==1200:
						self.Robot.Gait.np = 5
						self.Robot.Gait.phases = []
						self.Robot.Gait.phases.append([0,0,1,0,0,0])
						self.Robot.Gait.phases.append([0,1,0,0,0,0])
						self.Robot.Gait.phases.append([1,0,0,0,0,0])
						self.Robot.Gait.phases.append([0,0,0,0,0,0])
						self.Robot.Gait.phases.append([0,0,0,0,1,0])
						self.Robot.Gait.phases.append([0,0,0,1,0,0])
					elif i==2100:
						self.Robot.Gait.phases = []
						self.Robot.Gait.phases.append([0,0,0,0,1,0])
						self.Robot.Gait.phases.append([0,0,0,0,0,1])
						self.Robot.Gait.phases.append([0,0,0,1,0,0])
						self.Robot.Gait.phases.append([0,0,1,0,0,0])
						self.Robot.Gait.phases.append([0,1,0,0,0,0])
						self.Robot.Gait.phases.append([1,0,0,0,0,0])
					try:
						self.Robot.alternate_phase(next(gait_stack))
					except:
						self.Robot.alternate_phase()
					print i, self.Robot.Gait.cs, np.round(np.rad2deg(v3wp[2]).flatten(),4)#np.round(v3cp.flatten(),4)
					print '======================================================='

				## LOGGING: initialise variable and set respective data ##
				qlog 		  = JointState()
				qlog.name 	  = ROBOT_STATE + list(JOINT_NAME)
				qlog.position = v3cp.flatten().tolist() + v3wp.flatten().tolist() + qd.xp.tolist()
				qlog.velocity = v3cv.flatten().tolist() + v3wv.flatten().tolist() + qd.xv.tolist()
				
				qtrac = JointTrajectoryPoint()
				qtrac.positions = v3cp.flatten().tolist() + v3wp.flatten().tolist() + qd.xp.tolist()
				qtrac.velocities = v3cv.flatten().tolist() + v3wv.flatten().tolist() + qd.xv.tolist()
				qtrac.accelerations = v3ca.flatten().tolist() + v3wa.flatten().tolist() + qd.xa.tolist()
				
				mp = RosMotionPlan(setpoint = qtrac, gait_phase = self.Robot.Gait.cs)

				logstate = LoggingState()
				logstate.positions = v3cp.flatten().tolist() + v3wp.flatten().tolist() + qd.xp.tolist()
				logstate.velocities = v3cv.flatten().tolist() + v3wv.flatten().tolist() + qd.xv.tolist()
				logstate.accelerations = v3ca.flatten().tolist() + v3wa.flatten().tolist() + qd.xa.tolist()
				## Compute CRBI and CoM, then compute foot force distribution and joint torque
				# if (self.rbim_serv_.available):
				# 	try:
				# 		resp1 = self.rbim_serv_.call(mp)
				# 		rbim_valid = True
						# self.Robot.update_com_crbi(resp1.CoM, resp1.CRBI)
				# 	except:
				# 		rbim_valid = False
				## Next two lines bypasses service call
				if (True):
					rbim_valid = True
					if (rbim_valid):
						
						p_foot = []
						pfoot_log = []
						for j in range(0,6):
							if (self.Robot.Gait.cs[j] == 0):
								world_CoM_X_foot = mX( self.Robot.XHd.world_X_base[:3,:3], 
									  					(-self.Robot.P6c.base_X_CoM[:3].flatten()+self.Robot.Leg[j].XHd.base_X_foot[:3,3]) )
								p_foot.append(world_CoM_X_foot.copy())
								pfoot_log += world_CoM_X_foot.flatten().tolist()
							else:
								pfoot_log += np.zeros(3).flatten().tolist()
						# Gravity vector wrt base orientation
						gv = mX(rotation_zyx(self.Robot.P6d.world_X_base[3:6]),np.array([0,0,G]))

						foot_force = self.ForceDist.resolve_force(gv, v3ca, v3wa, p_foot, 
																	self.Robot.P6c.base_X_CoM[:3], 
																	self.Robot.CRBI, self.Robot.Gait.cs )
						joint_torque = self.Robot.force_to_torque(foot_force)

						## update logging parameters
						qlog.effort = np.zeros(6).flatten().tolist() + joint_torque
						qtrac.effort = np.zeros(6).flatten().tolist() + joint_torque
						mp = RosMotionPlan(setpoint = qtrac, gait_phase = self.Robot.Gait.cs)

						# logstate.effort = np.zeros(6).flatten().tolist() + joint_torque
						logstate.effort = np.zeros(6).flatten().tolist() + pfoot_log
						logstate.forces = np.zeros(6).flatten().tolist() + foot_force.flatten().tolist()
						logstate.qp_sum_forces  = self.ForceDist.sum_forces.flatten().tolist()
						logstate.qp_sum_moments = self.ForceDist.sum_moment.flatten().tolist()
						logstate.qp_desired_forces  = self.ForceDist.d_forces.flatten().tolist()
						logstate.qp_desired_moments = self.ForceDist.d_moment.flatten().tolist()
						self.log_pub_.publish(logstate)
				# print logstate.effort
				# print pfoot_log
				# publish appended joint angles if motion valid
				self.publish_topics(qd, qlog, mp)
				qd_prev = JointTrajectoryPoints(18,(qd.t, qd.xp, qd.xv, qd.xa))

				i += 1
			## ============================================================================================================================== ##
			
		if (self.Robot.invalid is False):
			# Finish off transfer legs trajectory onto ground
			# self.complete_transfer_trajectory()
			self.Robot.update_state(control_mode=self.control_rate)
			
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
		self.Robot.update_state(control_mode=self.control_rate)
		
		## check action to take
		data = self.Action.action_to_take()
		
		if (data is not None):
			self.Visualizer.clear_visualisation()

			## Data mapping - for convenience
			x_cob, w_cob, mode, motion_prim = data
			mode = 5
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
				self.path_tracking(x_cob, w_cob)
				
				self.Robot.suspend = prev_suspend
				## Move back to nominal position
				# self.default_pose()
				# self.default_pose(1) 	# required for resetting stance for walking

			elif (mode == 2):
				print 'walk mode'
				self.Robot.support_mode = False
				## TODO: Shouldnt' have to call motion planner
				motion_plan = self.Map.generate_motion_plan(self.Robot, path=(x_cob,w_cob))

				if (motion_plan is not None):
					if (self.main_controller(motion_plan)):
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

			elif (mode == 4):
				""" Plan path & execute """
				print 'Planning path...'
				self.Robot.support_mode = False

				ps = (10,13); pf = (20,13)	# Short straight Line
				# ps = (10,14); pf = (10,20)	# G2W - Left side up
				# ps = (10,13); pf = (10,6)	# G2W - Right side up
				# ps = (10,13); pf = (25,21)	# G2W - Left side up
				# ps = (10,13); pf = (40,13)	# full wall or chimney 
				# ps = (10,13); pf = (20,13) 	# wall and chimney demo
				# ps = (10,15); pf = (150,10) 	# IROS demo
				# ps = (10,15); pf = (17,12) 	# IROS - past chimney
				# ps = (10,12); pf = (20,12)

				## Set robot to starting position in default configuration
				self.Robot.P6c.world_X_base = np.array([ps[0]*self.GridMap.resolution,
														ps[1]*self.GridMap.resolution,
														BODY_HEIGHT,
														0.,0.,0.]).reshape(6,1)
				self.Robot.P6c.world_X_base_offset = np.array([ps[0]*self.GridMap.resolution,
																ps[1]*self.GridMap.resolution,
																0.,0.,0.,0.]).reshape(6,1)
				self.Robot.P6d.world_X_base = self.Robot.P6c.world_X_base.copy()
				self.Robot.XHc.update_world_X_base(self.Robot.P6c.world_X_base)

				self.Robot._initialise()
	
				if (self.grid_serv_.available):
					if (self.GridMap.get_index_exists(ps) and self.GridMap.get_index_exists(pf)):
						start = Pose()
						goal  = Pose()
						start.position.x = ps[0]*self.GridMap.resolution
						start.position.y = ps[1]*self.GridMap.resolution
						goal.position.x = pf[0]*self.GridMap.resolution
						goal.position.y = pf[1]*self.GridMap.resolution
						print start.position.x, start.position.y
						try:	
							print 'Requesting Planning service'
							path_generat = self.grid_serv_.call(start, goal)
							plan_exist = True
						except:
							plan_exist = False
						
						if (plan_exist):
							motion_plan  = planpath_to_motionplan(path_generat)
							if (motion_plan is not None):
								# print len(motion_plan.qb.X.t)
								if (self.main_controller(motion_plan)):
									self.Robot.alternate_phase()
					else:
						print "Start or End goal out of bounds!"
				else:
					print 'Planning service unavailable, exiting...'

			elif (mode == 5):
				rospy.wait_for_service('import_csv')
				try:
					path_planner = rospy.ServiceProxy('import_csv', PlanPath)
					plan_generat = path_planner(Pose(), Pose())
					print 'succeed'
				except:
					print 'CSVImport Service Failed'
				motion_plan = planpath_to_motionplan( path_planner(Pose(), Pose()) )
				# Plot.plot_2d(motion_plan.qb.X.t, motion_plan.qb.X.xp)
				
				self.Robot.P6c.world_X_base = np.array([motion_plan.qb.X.xp[0], 
														motion_plan.qb.W.xp[0]]).reshape(6,1)
				self.Robot.P6d.world_X_base = self.Robot.P6c.world_X_base.copy()
				self.Robot.P6c.world_X_base_offset = self.Robot.P6c.world_X_base.copy()
				
				self.Robot.XHc.update_world_X_base(self.Robot.P6c.world_X_base)
				self.Robot.XHd.update_world_X_base(self.Robot.P6d.world_X_base)
				
				# Compute foot position
				leg_stance = [None]*6
				for j in range(0,6):
					# print np.round(motion_plan.f_world_X_foot[j].xp[0],3)
					self.Robot.Leg[j].XHd.world_X_foot = v3_X_m(motion_plan.f_world_X_foot[j].xp[0])
					self.Robot.Leg[j].XHd.base_X_foot = mX(self.Robot.XHd.base_X_world, self.Robot.Leg[j].XHd.world_X_foot)
					self.Robot.Leg[j].XHd.coxa_X_foot = mX(self.Robot.Leg[j].XHd.coxa_X_base, self.Robot.Leg[j].XHd.base_X_foot)
					self.Robot.Leg[j].XHc.base_X_foot = self.Robot.Leg[j].XHd.base_X_foot.copy()
					self.Robot.Leg[j].XHc.coxa_X_foot = self.Robot.Leg[j].XHd.coxa_X_foot.copy()
					leg_stance[j] = self.Robot.Leg[j].XHd.coxa_X_foot[0:3,3]
				
				self.Robot.stance_offset = (40, -40)
				self.Robot._initialise(leg_stance)
				self.Robot.qc.position = self.Robot.qd
				# print 'moving to default'
				# self.default_pose(0, leg_stance)

				if (self.main_controller(motion_plan)):
					self.Robot.alternate_phase()
		else:
			rospy.sleep(0.5)


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