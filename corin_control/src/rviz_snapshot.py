#!/usr/bin/env python

""" Snapshot of motion plan imported from csv """

import rospy
from corin_control import *			# library modules to include
from rviz_visual import *
from corin_msgs.msg import LoggingState

import csv
import numpy as np

class RvizSnapshot:
	def __init__(self):
		rospy.init_node('CorinController') 		# Initialises node
		self.rate 	  = rospy.Rate(CTR_RATE)	# Controller rate

		self.CoB = []
		self.footholds = [None]*6
		self.snorm = np.zeros((18,1))
		self.Visualizer = RvizVisualise()

		self.Robot = robot_class.RobotState()
		self.ForceDist = QPForceDistribution()

		self.joint_states = []

		self.__initialise_variables__()
		self.__initialise_topics__()
		
	def __initialise_topics__(self):
		self.joint_pub_ = rospy.Publisher(ROBOT_NS + '/joint_states', JointState, queue_size=1)
		self.log_pub_ = rospy.Publisher('/corin/log_states', LoggingState, queue_size=1)	# LOGGING publisher

	def __initialise_variables__(self):
		for j in range(0,6):
			self.footholds[j] = MarkerList()

	def publish(self, CoB, qp):
		self.Visualizer.publish_robot(CoB)

		dqp = JointState()
		dqp.header.stamp = rospy.Time.now()
		for n in range(0,18): 	# loop for each joint
			dqp.name.append(str(JOINT_NAME[n])) # joint name
			dqp.position.append(qp[n])			# joint angle
		self.joint_pub_.publish(dqp)

		# rospy.sleep(0.4)

	def load_file(self, filename):
		print 'Reading file: ', filename
		
		with open(filename, 'rb') as csvfile:
			motion_data = csv.reader(csvfile, delimiter=',')
			counter = 0
			for row in motion_data:
				# Append for CoB
				self.CoB.append( np.asarray(map(lambda x: float(x), row[0:6])) )
				# Append for footholds
				# self.footholds.append( map(lambda x: float(x), row[6:25]) )
				for j in range(0,6):
					self.footholds[j].t.append(counter)
					# self.footholds[j].xp.append( np.asarray(map(lambda x: float(x), row[6+j*3:6+(j*3)+3])) )
					mod_foothold = np.asarray(map(lambda x: float(x), row[6+j*3:6+(j*3)+3]))# + np.array([0.,0.,0.05])
					self.footholds[j].xp.append(mod_foothold)
				
				self.joint_states.append( np.asarray(map(lambda x: float(x), row[24:42])) )
				
				counter += 1
			
			# print self.CoB
			# print '======================'
			# print (self.footholds[0].xp)
		try:
			if not self.joint_states[0]:
				self.joint_states = []
		except:
			pass

	def visualise_motion_plan(self):

		self.Visualizer.publish_robot(self.CoB[0])
		self.Visualizer.publish_path(self.CoB)
		self.Visualizer.publish_footholds(self.footholds)

		if self.joint_states:
			for xi in range(0,3):
				self.publish(self.CoB[0], self.joint_states[0])
				rospy.sleep(0.5)

	def cycle_snapshot(self):

		i = 0
		while (i != len(self.CoB) and not rospy.is_shutdown()):

			self.Robot.P6d.world_X_base = self.CoB[i].reshape((6,1))
			# self.Robot.P6d.world_X_base[5] = 0
			self.Robot.XHd.update_world_X_base(self.Robot.P6d.world_X_base)
			
			# self.Visualizer.publish_robot((self.CoB[i]))

			qp = []
			phip_X_foot = [] 	# TEMP
			pworld_X_foot = []	# TEMP

			err_list = [0]*6
			for j in range(0,6):
				# Transform world_X_foot to hip_X_foot
				world_base_X_foot = self.footholds[j].xp[i] - self.CoB[i][0:3]
				# base_X_foot = mX(np.transpose(rotation_zyx(self.CoB[i][3:6])), world_base_X_foot)
				# hip_X_foot = mX(self.Robot.Leg[j].XHc.coxa_X_base, v3_X_m(base_X_foot))[:3,3]
				self.Robot.Leg[j].XHd.base_X_foot[:3,3] = mX(self.Robot.XHd.base_X_world[:3,:3], world_base_X_foot)
				self.Robot.Leg[j].XHd.coxa_X_foot[:3,3] = mX(self.Robot.Leg[j].XHc.coxa_X_base, 
																v3_X_m(self.Robot.Leg[j].XHd.base_X_foot[:3,3]))[:3,3]

				error, qpd, qvd, qad = self.Robot.Leg[j].tf_task_X_joint(self.Robot.Leg[j].XHd.coxa_X_foot[:3,3])

				if (error == 0):
					for z in range(0,3):
						qp.append(qpd.item(z))

						phip_X_foot.append(self.Robot.Leg[j].XHd.coxa_X_foot[:3,3].item(z))
						pworld_X_foot.append(self.footholds[j].xp[i].item(z))
				else:
					print 'Error on leg ', j

			## Set some variables to default values
			v3ca = v3wa = np.array([0,0,0])
			self.Robot.Gait.cs = [0]*6
			self.Robot.qd = qp

			p_foot = []
			for j in range(0,6):
				## Append for legs in support mode only
				if (self.Robot.Gait.cs[j] == 0):
					world_CoM_X_foot = mX( self.Robot.XHd.world_X_base[:3,:3], 
						  					(-self.Robot.P6c.base_X_CoM[:3].flatten()+self.Robot.Leg[j].XHd.base_X_foot[:3,3]) )
					p_foot.append(world_CoM_X_foot.copy())
					
			## Gravity vector wrt base orientation 
			# gv = mX(self.Robot.XHd.world_X_base[:3,:3], np.array([0,0,G]))
			gv = np.array([0,0,G])
			
			## Set surface normal vector for each contact wrt world frame according to path
			for j in range(0,3):
				if (abs(self.footholds[j].xp[i].item(1))<0.4):
					self.snorm[j*3:j*3+3] = np.array([1,0,0]).reshape((3,1))
				else:
					self.snorm[j*3:j*3+3] = np.array([0,-1,0]).reshape((3,1))
					# hardcode for LR leg normal at final instance 
					if (j==2 and i==len(self.CoB)-1):
						self.snorm[j*3:j*3+3] = np.array([1,0,0]).reshape((3,1))

			for j in range(3,6):
				if (abs(self.footholds[j].xp[i].item(1))<1.12): # wide: 1.12; narrow: 1.02
					self.snorm[j*3:j*3+3] = np.array([-1,0,0]).reshape((3,1))	# chimney wall
					# self.snorm[j*3:j*3+3] = np.array([0,0,1]).reshape((3,1)) 	# wall walking ground
				else:
					self.snorm[j*3:j*3+3] = np.array([0,1,0]).reshape((3,1))
			
			## Resolve forces and compute joint torque
			foot_force = self.ForceDist.resolve_force(gv, v3ca, v3wa, p_foot, 
														self.Robot.P6c.base_X_CoM[:3], 
														self.Robot.CRBI, self.Robot.Gait.cs, self.snorm )
			joint_torque = self.Robot.force_to_torque(-foot_force)
			
			for xi in range(0,2):
				self.publish(self.CoB[i], qp)
			rospy.sleep(0.1)

			fforce_local = np.zeros((18,1))
			fforce_hip   = np.zeros((18,1))

			for j in range(0,6):
				self.Robot.Leg[j].XHd.update_base_X_foot(qp[j*3:j*3+3])
				R_world_X_foot = np.transpose(mX(self.Robot.XHd.world_X_base[:3,:3], self.Robot.Leg[j].XHd.base_X_foot[:3,:3]))
				fforce_local[j*3:j*3+3] = mX(R_world_X_foot, foot_force[j*3:j*3+3])

				## Force in hip frame -  for logging
				# fforce_hip[j*3:j*3+3] = self.Robot.Leg[j].F6d.coxa_X_foot[:3]
			
			## Visualise foot force
			for v in range(0,2):
				self.Visualizer.publish_foot_force(self.Robot.Gait.cs, fforce_local)
			# print np.round(phip_X_foot,3)
			## Data Logging
			logstate = LoggingState()
			logstate.positions = self.CoB[i].tolist() + qp
			logstate.velocities = [0]*6 + pworld_X_foot
			logstate.accelerations = [0]*6 + phip_X_foot
			logstate.effort = np.zeros(6).flatten().tolist() + joint_torque
			logstate.forces = np.zeros(6).flatten().tolist() + foot_force.flatten().tolist()
			# logstate.forces = np.zeros(6).flatten().tolist() + fforce_hip.flatten().tolist()
			logstate.qp_sum_forces  = self.ForceDist.sum_forces.flatten().tolist()
			logstate.qp_sum_moments = self.ForceDist.sum_moment.flatten().tolist()
			logstate.qp_desired_forces  = self.ForceDist.d_forces.flatten().tolist()
			# logstate.qp_desired_moments = self.ForceDist.d_moment.flatten().tolist()
			logstate.qp_desired_moments = self.snorm.flatten().tolist()
			self.log_pub_.publish(logstate)

			# rospy.sleep(0.05)
			i += 1
			
			# print np.round(self.CoB[i][5]*180./np.pi,3)
			# raw_input('continue')

	def cycle_states(self):
		""" Publishes joint states directly """

		i = 0
		while (i != len(self.CoB) and not rospy.is_shutdown()):
			
			for xi in range(0,2):
				self.publish(self.CoB[i], self.joint_states[i])
			rospy.sleep(0.1)
			i += 1
			# raw_input('cont')
			rospy.sleep(0.5)

if __name__ == "__main__":

	rviz = RvizSnapshot()

	# rviz.load_file('chimney_cheight_0d1.csv')
	# rviz.load_file('wall_highRes_convex.csv')
	# rviz.load_file('wall_medRes_concave.csv')
	rviz.load_file('wall_medRes_convex.csv')
	
	rviz.visualise_motion_plan()
	raw_input('Start motion!')
	for i in range(0,5):
		# rviz.cycle_snapshot()
		rviz.cycle_states()