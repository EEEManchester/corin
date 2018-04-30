#!/usr/bin/env python

""" Main controller for the robot """
__version__ = '1.0'
__author__  = 'Wei Cheah'

import sys; sys.dont_write_bytecode = True

## Personal libraries
from library import *			# library modules to include
from motion_planning import *	# library modules on motion planning
import control_interface 		# action selection from ROS parameter server
from rviz_visual import *

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
		self.Map 	= grid_planner.GridPlanner('wall_demo_02')	# map class 
		# self.Gait = gait_class.GaitClass(GAIT_TYPE) 		# gait class

		self.resting   = False 			# Flag indicating robot standing or resting
		self.on_start  = False 			# variable for resetting to leg suspended in air
		self.interface = "rviz"			# interface to control: gazebo, rviz or robotis hardware
		self.control_mode = "fast" 	# run controller in various mode: 1) normal, 2) fast

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
		

	def __initialise__(self):
		""" Initialises robot and classes """

		## Identify interface automatically to control: simulation or robotis motors
		# self.interface = 'gazebo' if rospy.has_param('/gazebo/auto_disable_bodies') else 'robotis'

		## set up publisher and subscriber topics
		self.__initialise_topics__()
		self.__initialise_service__()

		## initialises robot transform
		self.Visualizer.robot_broadcaster.sendTransform( (0.,0.,BODY_HEIGHT), (0.,0.,0.,1.), 
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

	def handle_ui_state(self,req):
		""" Service handler for user interface """
		
		cmd = str(req.state.data)
		if (cmd == "go_final_pose"):
			if (self.interface != 'robotis'):
				print 'Going to final pose'
				self.pose_switch('final')

		elif (cmd == "go_initial_pose"):
			if (self.interface != 'robotis'):
				print 'Going to initial pose'
				self.pose_switch('initial')

		msg = String()
		msg.data = "Success"
		return msg

	def __initialise_service__(self):
		self.ui_service = rospy.Service(ROBOT_NS + '/set_ui_state', UiState, self.handle_ui_state)

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

		self.Visualizer.publish_robot(self.Robot.P6d.world_X_base)

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
		mplan = self.Map.foothold_planner(base_path, self.Robot)

		self.main_controller(mplan)

	def main_controller(self, motion_plan):

		## Variables ##
		cob_X_desired = np.zeros((3,1)) 	# cob linear location
		cob_W_desired = np.zeros((3,1)) 	# cob angular location
		wXbase_offset = self.Robot.P6c.world_X_base.copy()
		# print wXbase_offset
		# print motion_plan.qb_bias
		# print motion_plan.qb.X.xp
		## Variable mapping from motion plan
		base_path = motion_plan.qb
		world_X_base = motion_plan.qbp
		w_base_X_NRP = motion_plan.f_world_base_X_NRP
		world_X_footholds = motion_plan.f_world_X_foot
		base_X_footholds  = motion_plan.f_base_X_foot
		# print np.round(world_X_footholds[0].xp,4)
		## Publish multiple times to ensure it is published 
		for c in range(0,3):
			self.Visualizer.publish_robot(wXbase_offset)
			self.Visualizer.publish_path(base_path, wXbase_offset)
			self.Visualizer.publish_footholds(world_X_footholds)

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
			self.Robot.update_state(control_mode=self.control_mode)
			#########################################################################
			## overwrite - TC: use IMU data
			self.Robot.XHc.world_X_base = self.Robot.XHd.world_X_base.copy()
			self.Robot.XHc.base_X_world = self.Robot.XHd.base_X_world.copy()
			self.Robot.P6c.world_X_base = self.Robot.P6d.world_X_base.copy()
			for j in range(0, self.Robot.active_legs):
				self.Robot.Leg[j].P6_world_X_base = self.Robot.P6c.world_X_base.copy()
			#########################################################################

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
						print 'Leg: ', j, ' No further foothold planned!', i
						self.Robot.Leg[j].XHd.base_X_foot = self.Robot.Leg[j].XHd.base_X_NRP.copy()

					## Compute average surface normal from cell surface normal at both footholds
					wall_trans = True if (self.T_GND_X_WALL is True or self.T_WALL_X_GND is True) else False
					snorm_1 = self.Map.get_cell('norm', self.Robot.Leg[j].XHc.world_X_foot[0:3,3], j)
					snorm_2 = self.Map.get_cell('norm', self.Robot.Leg[j].XHd.world_X_foot[0:3,3], j)
					w_snorm = (snorm_1 + snorm_2)/2.

					## Set bodypose in leg class
					self.Robot.Leg[j].XH_world_X_base = self.Robot.XHd.world_X_base.copy()
					
					## Generate transfer spline
					svalid = self.Robot.Leg[j].generate_spline('world', w_snorm, 1, False, GAIT_TPHASE, CTR_INTV)
					
					## Update NRP
					self.Robot.Leg[j].XHd.base_X_NRP[:3,3] = mX(self.Robot.XHd.base_X_world[:3,:3], 
																self.Robot.Leg[j].XHc.world_base_X_NRP[:3,3])
					# if (j == 5):
						# print 'bXN: ', np.round(self.Robot.Leg[j].XHd.base_X_NRP[:3,3],4)
						# print 'wXN: ', np.round(self.Robot.Leg[j].XHc.world_base_X_NRP[:3,3],4)
						# print j, ' Xc: ', np.round(self.Robot.Leg[j].XHc.coxa_X_foot[0:3,3],4)
						# print j, ' Xd: ', np.round(self.Robot.Leg[j].XHd.coxa_X_foot[0:3,3],4)
						# print j, ' Wn: ', np.round(w_snorm,4)
						# print j, ' Ln: ', np.round(self.Robot.Leg[j].qsurface,4)
						# print 'wXf: ', np.round(self.Robot.Leg[j].XHd.world_X_foot[:3,3],4)

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
				# if (j==4):
				# 	print 'bXf: ', np.round(self.Robot.Leg[j].XHd.base_X_foot[:3,3],4)

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
					print i, self.Robot.Gait.cs

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
			
			self.Visualizer.clear_visualisation()

			## Data mapping - for convenience
			x_cob, w_cob, mode, motion_prim = data

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
				
				motion_plan = self.Map.generate_motion_plan(self.Robot, path=(x_cob,w_cob))
				if (motion_plan is not None):
					success = self.main_controller(motion_plan)

				self.Robot.suspend = prev_suspend
				## Move back to nominal position
				# self.default_pose()
				# self.default_pose(1) 	# required for resetting stance for walking

			elif (mode == 2):
				print 'walk mode'
				self.Robot.support_mode = False
				
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
				
				## Initial Map
				# Straight Line
				# ps = (12,17); pf = (12,20)
				# g2w transition
				# ps = (12,17); pf = (6,17) # Left side up
				# ps = (12,17); pf = (18,21)	# right side up

				## Second map
				# ps = (13,12); pf = (16,12)	# Straight Line
				ps = (13,12); pf = (16,15)	# G2W - Right side up

				## Set robot to starting position in default configuration
				self.Robot.P6c.world_X_base = np.array([ps[0]*self.Map.resolution,
														ps[1]*self.Map.resolution,
														BODY_HEIGHT,
														0.,0.,0.]).reshape(6,1)
				self.Robot.P6d.world_X_base = self.Robot.P6c.world_X_base.copy()
				self.Robot.XHc.update_world_X_base(self.Robot.P6c.world_X_base)

				self.Robot._initialise()
				
				motion_plan = self.Map.generate_motion_plan(self.Robot, start=ps, end=pf)

				if (motion_plan is not None):
					if (self.main_controller(motion_plan)):
						self.Robot.alternate_phase()

		else:
			rospy.sleep(0.5)

	def pose_switch(self, setpoint):
		""" Moves the robot into a pose immediately """
		
		# Set counter and gait phases to start or end position
		if (setpoint == 'initial'):
			i = 0
			self.Robot.Gait.np = self.MotionPlan.gait_in
			self.Robot.Gait.cs = self.Robot.Gait.phase[self.Robot.Gait.np]
		else:
			i = -1
			self.Robot.Gait.np = self.MotionPlan.gait_fn
			self.Robot.Gait.cs = self.Robot.Gait.phase[self.Robot.Gait.np]
		
		# Set bodypose
		v3cp = self.MotionPlan.qb_bias[0:3] + self.MotionPlan.qb.X.xp[i].reshape(3,1);
		v3wp = self.MotionPlan.qb_bias[3:6] + self.MotionPlan.qb.W.xp[i].reshape(3,1);

		self.Robot.P6d.world_X_base = np.vstack((v3cp,v3wp))
		self.Robot.P6c.world_X_base = np.vstack((v3cp,v3wp))

		self.Robot.XHd.update_world_X_base(self.Robot.P6d.world_X_base)
		self.Robot.XHc.update_world_X_base(self.Robot.P6c.world_X_base)

		## Set leg states
		for j in range(0,6):

			self.Robot.Leg[j].XHd.world_X_foot = v3_X_m(self.MotionPlan.f_world_X_foot[j].xp[i])
			self.Robot.Leg[j].XHc.world_X_foot = self.Robot.Leg[j].XHd.world_X_foot.copy()
			self.Robot.Leg[j].XHd.world_base_X_NRP = v3_X_m(self.MotionPlan.f_world_base_X_NRP[j].xp[i])
			self.Robot.Leg[j].XHc.world_base_X_NRP = self.Robot.Leg[j].XHd.world_base_X_NRP.copy()

			self.Robot.Leg[j].XHd.base_X_foot = mX(self.Robot.XHd.base_X_world, self.Robot.Leg[j].XHc.world_X_foot)
			self.Robot.Leg[j].XHc.base_X_foot = self.Robot.Leg[j].XHd.base_X_foot.copy()
			self.Robot.Leg[j].XHd.coxa_X_foot = mX(self.Robot.Leg[j].XHd.coxa_X_base, self.Robot.Leg[j].XHd.base_X_foot)
			self.Robot.Leg[j].XHc.coxa_X_foot = self.Robot.Leg[j].XHd.coxa_X_foot.copy()

		qd = self.Robot.task_X_joint()
		
		if (self.Robot.invalid == True):
			print 'Robot Configuration Invalid!'
		else:
			self.publish_topics(qd)