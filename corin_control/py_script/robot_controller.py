#!/usr/bin/env python

""" Main controller for the robot """
__version__ = '1.0'
__author__  = 'Wei Cheah'

import sys; sys.dont_write_bytecode = True

## Personal libraries
from library import *			# library modules to include 
import control_interface 		# action selection from ROS parameter server

## ROS messages & libraries
import rospy
from sensor_msgs.msg import Imu 						# sub msg for IMU
from sensor_msgs.msg import JointState 					# sub msg for joint states
from std_msgs.msg import Float64 						# pub msg for Gazebo joints
from std_msgs.msg import ByteMultiArray 				# foot contact state
from std_msgs.msg import Float32MultiArray				# foot contact force
# import tf as RTF 										# ROS transform library

## Robotis ROS msgs for joint control
from robotis_controller_msgs.msg import SyncWriteMultiFloat

#####################################################################

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
		self.control_mode = "normal" # run controller in various mode: 1) normal, 2) fast, 3) simfast

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

	def __initialise__(self):
		""" Initialises robot and classes """

		## Identifies interface to control: simulation or robotis motors
		self.interface = 'simulation' if rospy.has_param('/gazebo/auto_disable_bodies') else 'robotis'

		## set up publisher and subscriber topics
		self.__initialise_topics__()

		## moves robot to nominal position if haven't done so
		try:
			if (rospy.get_param(ROBOT_NS + '/standing') is False):
				raise Exception
		except Exception, e:
			self.default_pose()

		## update and reset robot states
		self.Robot.suspend = False
		self.Robot.update_state(reset=True)

		for j in range(0,self.Robot.active_legs):
			self.Robot.Leg[j].feedback_state = 2 	# trajectory completed

	def __initialise_topics__(self):
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

	def publish_topics(self, q, q_log=None):
		""" Publish joint position to joint controller topics and
			setpoint topic (for logging all setpoint states) 		"""
		""" Input: 	1) q -> Joint setpoints (position, velocity, 
							acceleration) 	
					2) q_log -> JointState msg with base and joint
								setpoints for logging purposes		"""

		## Publish joint position to robot if valid
		if (self.Robot.invalid is not True):
			self.Robot.active_legs = 6

			if (self.interface == 'simulation'):
				for n in range(0,self.Robot.active_legs*3):
					qp = Float64()
					qp.data = q.xp[n]
					# Publish joint angles individually
					if (self.control_mode is "normal" or self.control_mode is "fast"):
						self.qpub_[n].publish(qp)
					elif (self.control_mode is "simfast"):
						pass

			elif (self.interface == 'robotis'):
				dqp = SyncWriteMultiFloat()
				dqp.item_name 	= str("goal_position") 	# register to start first write
				dqp.data_length = 4 					# size of register

				for n in range(0,self.Robot.active_legs*3): 	# loop for each joint
					dqp.joint_name.append(str(JOINT_NAME[n])) 	# joint name
					dqp.value.append(q.xp[n])					# joint angle

				# Publish all joint angles together
				if (self.control_mode is "normal" or self.control_mode is "fast"):
					self.sync_qpub_.publish(dqp)
				elif (self.control_mode is "simfast"):
					pass

		## Publish setpoints to logging topic
		if (q_log is not None):
			self.setpoint_pub_.publish(q_log)

		## Runs controller at desired rate for normal control mode
		if (self.control_mode is "normal"):
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
		dir_uv = np.array([0.,0.,0.]) 	# zero vector direction
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
				svalid = self.Robot.Leg[j].generate_spline(self.Robot.Leg[j].XHc.coxa_X_foot[0:3,3], self.Robot.Leg[j].XHd.coxa_X_foot[0:3,3],
															self.Robot.Leg[j].qsurface, leg_phase[j], False, td, CTR_INTV)
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

		## Define Variables ##
		i = 0
		Gait = gait_class.GaitClass(GAIT_TYPE)	# gait class
		skip_support = False
		world_X_footholds = [None]*6
		base_X_footholds  = [None]*6

		XHd = robot_transforms.HomogeneousTransform()
		Leg = [None]*6

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

		## Cycle through trajectory
		while (i != len(base_path.X.t) and not rospy.is_shutdown()):
			print i, ' Gait phase ', Gait.cs
			bound_exceed = False
			# n = i
			# cycles through one gait phase
			for m in range(0,int(GAIT_PHASE*CTR_RATE)):
				# print i, len(base_path.X.t)

				## Variable mapping to R^(3x1) for base linear and angular time, position, velocity, acceleration
				try:
					v3cp = self.Robot.P6c.world_X_base[0:3] + base_path.X.xp[i].reshape(3,1)
					v3wp = self.Robot.P6c.world_X_base[3:6] + base_path.W.xp[i].reshape(3,1)

					## update robot's pose
					XHd.update_world_X_base(np.vstack((v3cp,v3wp)))
					temp_mx = XHd.world_X_base.copy()
					temp_mx[:3,3:4] = np.zeros((3,1))

					for j in range (0, self.Robot.active_legs):
						## Legs in support phase:
						if (Gait.cs[j] == 0):
							Leg[j].base_X_foot = mX(XHd.base_X_world, Leg[j].world_X_foot)
							Leg[j].coxa_X_foot = mX(Leg[j].coxa_X_base, Leg[j].base_X_foot)
							Leg[j].world_base_X_foot = mX(temp_mx, Leg[j].base_X_foot)

							bound_exceed = self.Robot.Leg[j].check_boundary_limit(Leg[j].world_base_X_foot)
							
							if (bound_exceed == True):
								print 'bound exceed on ', j, ' at ', i, i*CTR_INTV
								break

					if (bound_exceed is True):
						## To break through two layers of loops
						break
					else:	
						i += 1

				except IndexError:
					## Trajectory finished, skip support
					skip_support = True
					print 'breaking now at ', i
					break

			## Set leg states
			for j in range (0, self.Robot.active_legs):
				## Legs in transfer phase
				if (Gait.cs[j] == 1 and i <= len(base_path.X.t)):
					## compute NRP

					## compute AEP wrt base and world frame & update 'current' foot position
					## to new position which is after transfer phase
					Leg[j].world_X_foot = mX(XHd.world_X_base, Leg[j].world_base_X_AEP)

					## stack to array
					# footholds[j].append(Leg[j].world_X_foot[:3,3])
					world_X_footholds[j].t.append(i*CTR_INTV)
					world_X_footholds[j].xp.append(Leg[j].world_X_foot[:3,3:4])
					
					base_X_footholds[j].t.append(i*CTR_INTV)
					base_X_footholds[j].xp.append(Leg[j].world_base_X_AEP[:3,3:4])

			## Alternate gait phase
			Gait.change_phase()
			# print v3cp.flatten()
		# print footholds[0].t
		# print np.round(footholds[0].xp,4)
		# print footholds[2].t
		# print footholds[2].xp
		# raw_input('bo')
		return world_X_footholds, base_X_footholds

	def trajectory_tracking(self, x_cob, w_cob=0):
		""" Generates body trajectory from given via points and
			cycles through the trajectory 							"""
		""" Input: 	1) x_com -> 2D array of linear positions
					2) w_com -> 2D array of angular orientations
			Output: None											"""

		## Define Variables ##
		PathGenerator = Pathgenerator.PathGenerator() 	# path generator for robot's base
		# PathGenerator.gait = self.Robot.Gait.gdic
		cob_X_desired = np.zeros((3,1)) 	# cob linear location
		cob_W_desired = np.zeros((3,1)) 	# cob angular location
		wXbase_offset = self.Robot.P6c.world_X_base.copy()
		print 'wXbase: ', np.round(wXbase_offset.flatten(),4)
		# Trajectory for robot's base
		base_path = PathGenerator.generate_base_path(x_cob, w_cob, CTR_INTV) 
		# Plot.plot_2d(base_path.W.t, base_path.W.xp)

		# Set all legs to support mode for bodyposing, prevent AEP from being set
		if (self.Robot.support_mode == True):
			for j in range(0,6):
				self.Robot.Gait.cs[j] = 0
		else:
			## Plan foothold for robot
			world_X_footholds, base_X_footholds = self.foothold_selection(base_path)
			print base_X_footholds[1].t
		print 'start: wXf : ', np.round(self.Robot.Leg[4].XHc.world_X_foot[:3,3],4)

		raw_input('Accept & start execution?')
		
		# cycle through trajectory points until complete
		i = 1 	# skip first point since spline has zero initial differential conditions
		while (i != len(base_path.X.t) and not rospy.is_shutdown()):
			print 'counting: ', i, len(base_path.X.t)

			## suppress trajectory counter as body support suspended
			if (self.Robot.suspend == True):
				i -= 1

			########################################################
			
			## Variable mapping to R^(3x1) for base linear and angular time, position, velocity, acceleration
			v3cp = wXbase_offset[0:3] + base_path.X.xp[i].reshape(3,1);
			v3cv = base_path.X.xv[i].reshape(3,1);
			v3ca = base_path.X.xa[i].reshape(3,1);

			v3wp = wXbase_offset[3:6] + base_path.W.xp[i].reshape(3,1);
			v3wv = base_path.W.xv[i].reshape(3,1);
			v3wa = base_path.W.xa[i].reshape(3,1);

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
			self.Robot.update_state(control_mode=self.control_mode)

			###########################################################################################

			## overwrite - TC: use IMU data
			self.Robot.XHc.world_X_base = self.Robot.XHd.world_X_base.copy()
			self.Robot.XHc.base_X_world = self.Robot.XHd.base_X_world.copy()
			self.Robot.P6c.world_X_base = np.array([v3cp,v3wp]).reshape(6,1)

			## Update robot's desired pose & twist
			self.Robot.XHd.update_world_X_base(np.vstack((v3cp,v3wp)))

			self.Robot.V6d.world_X_base = np.vstack((v3cv,v3wv))
			self.Robot.A6d.world_X_base = np.vstack((v3ca,v3wa))
			
			## generate trajectory for leg in transfer phase
			for j in range (0, self.Robot.active_legs):

				# Transfer phase
				if (self.Robot.Gait.cs[j] == 1 and self.Robot.Leg[j].transfer_phase_change==False):
					self.Robot.Leg[j].feedback_state = 1 	# leg put into execution mode

					## ====================================================================================================== ##
					## Reactive 'Foothold Selection' Algorithm: 1) Update REP 2) Get_to_Stance

					# Update REP - MOCKED DATASET HERE
					base_X_surface = 0.32
					bodypose = np.array([0.,0.,BODY_HEIGHT, 0.,0.,0.])
					self.Robot.Leg[j].update_NRP(bodypose, base_X_surface)

					# unstack next CoB location, determine foot z position in that CoB location
					try:
						v3np = (base_path.X.xp[int(i+GAIT_PHASE*CTR_RATE)]).reshape(3,1)
					except:
						v3np = (base_path.X.xp[-1]).reshape(3,1)

					## ====================================================================================================== ##
					## Using planned foothold arrays
					
					try:
						# self.Robot.Leg[j].XHd.world_X_foot = v3_X_m(world_X_footholds[j].xp.pop(1))
						# self.Robot.Leg[j].XHd.base_X_foot  = mX(self.Robot.XHd.base_X_world, self.Robot.Leg[j].XHd.world_X_foot)
						self.Robot.Leg[j].XHd.base_X_foot  = v3_X_m(base_X_footholds[j].xp.pop(1))
					except IndexError:
						print 'Leg: ', j, ' No further foothold planned!'


#					## ====================================================================================================== ##
					# Base to hip frame conversion (rotational components here is incorrect)
					self.Robot.Leg[j].XHd.coxa_X_foot = mX(self.Robot.Leg[j].XHc.coxa_X_base, self.Robot.Leg[j].XHd.base_X_foot)
					self.Robot.Leg[j].XHd.coxa_X_AEP  = self.Robot.Leg[j].XHd.coxa_X_foot.copy()

					## generate spline for transfer phase leg - 
					svalid = self.Robot.Leg[j].generate_spline(self.Robot.Leg[j].XHc.coxa_X_foot[0:3,3], self.Robot.Leg[j].XHd.coxa_X_foot[0:3,3],
																self.Robot.Leg[j].qsurface, 1, False, GAIT_PHASE, CTR_INTV)
					if (j==0):
					# 	print np.round(self.Robot.XHc.base_X_world[:3,:3],4)
					# 	print np.round(self.Robot.Leg[j].XHd.world_base_X_NRP[:3,3:4].flatten(),4)
					# 	print np.round(self.Robot.Leg[j].XHd.world_base_X_AEP[:3,3:4].flatten(),4)
					# 	print np.round(self.Robot.Leg[j].XHc.base_X_foot[0:3,3],4)
						print np.round(self.Robot.Leg[j].XHd.base_X_foot[0:3,3],4)
						# print np.round(base_X_foot[0:3,3],4)
						# raw_input('cont')
					## TODO: update world_X_foot position?
					self.Robot.Leg[j].XHd.world_X_foot[:3,3:4] = self.Robot.Leg[j].XHd.world_base_X_AEP[:3,3:4] + v3np

					if (svalid is False):
						# set invalid if trajectory unfeasible for leg's kinematic
						self.Robot.invalid = True
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
					self.Robot.Leg[j].XHd.base_X_foot = mX(self.Robot.XHd.base_X_world, self.Robot.Leg[j].XHc.world_X_foot)
					self.Robot.Leg[j].XHd.coxa_X_foot = mX(self.Robot.Leg[j].XHd.coxa_X_base, self.Robot.Leg[j].XHd.base_X_foot)
			
			print 'bXw : ', np.round(self.Robot.XHd.base_X_world[:3,3],4)
			print 'wXf : ', np.round(self.Robot.Leg[4].XHc.world_X_foot[:3,3],4)
			print 'bXf : ', np.round(self.Robot.Leg[4].XHd.base_X_foot[:3,3],4)
			# print self.Robot.Leg[2].XHd.world_X_foot

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
				self.Robot.alternate_phase()

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
		self.Robot.update_state(control_mode=self.control_mode)
		self.Robot.alternate_phase()
		print 'Trajectory executed'
		print 'Desired Goal: ', np.round(base_path.X.xp[-1],4), np.round(base_path.W.xp[-1],4)
		print 'Tracked Goal: ', np.round(cob_X_desired.flatten(),4), np.round(cob_W_desired.flatten(),4)
		

	def action_interface(self):
		""" Interface for commanding robot """
		## TODO: alternative for this - using action server

		## update robot state prior to starting action
		self.Robot.update_state(control_mode=self.control_mode)
		
		## check action to take
		data = self.Action.action_to_take()

		if (data is not None):
			## data mapping - for convenience
			x_cob, w_cob, mode = data

			## stand up if at rest
			if ( (mode == 1 or mode == 2) and self.resting == True):
				print 'Going to standup'
				self.default_pose()

			## condition for support (1), walk (2), reset (3)
			if (mode == 1):
				print 'support mode'
				self.Robot.support_mode = True
				# self.Robot.reset_state  = True 	# is this required?
				self.Robot.suspend = False 		# clear suspension flag
				
				self.trajectory_tracking(x_cob, w_cob)
				# self.default_pose()
				# self.default_pose(1) 	# required for resetting stance for walking

			elif (mode == 2):
				print 'walk mode'
				self.Robot.support_mode = False

				self.trajectory_tracking(x_cob, w_cob)

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
			## Standby ##
			rospy.sleep(0.5)
