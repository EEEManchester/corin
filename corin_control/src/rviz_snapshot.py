#!/usr/bin/env python

""" Snapshot of motion plan imported from csv """

import rospy
from corin_control import *			# library modules to include
from rviz_visual import *

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

		self.__initialise_variables__()
		self.__initialise_topics__()
		
	def __initialise_topics__(self):
		self.joint_pub_ = rospy.Publisher(ROBOT_NS + '/joint_states', JointState, queue_size=1)

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

		rospy.sleep(0.4)

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
					self.footholds[j].xp.append( np.asarray(map(lambda x: float(x), row[6+j*3:6+(j*3)+3])) )
					
				counter += 1
			# print self.CoB
			# print '======================'
			# print (self.footholds[0].xp)

	def visualise_motion_plan(self):

		self.Visualizer.publish_robot(self.CoB[0])
		self.Visualizer.publish_path(self.CoB)
		self.Visualizer.publish_footholds(self.footholds)

	def cycle_snapshot(self):

		# for i in range(0,len(self.CoB)):
		i = 0
		while (i != len(self.CoB) and not rospy.is_shutdown()):

			self.Robot.P6d.world_X_base = self.CoB[i].reshape((6,1))
			# self.Robot.P6d.world_X_base[5] = 0
			self.Robot.XHd.update_world_X_base(self.Robot.P6d.world_X_base)
			
			self.Visualizer.publish_robot((self.CoB[i]))

			qp = []
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

				if (err_list[j] == 0):
					for z in range(0,3):
						qp.append(qpd.item(z))
				else:
					print 'Error!'

			## Set some variables to default values
			v3ca = v3wa = np.array([0,0,0])
			self.Robot.Gait.cs = [0]*6

			p_foot = []
			for j in range(0,6):
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

			for j in range(3,6):
				if (abs(self.footholds[j].xp[i].item(1))<1.12):
					self.snorm[j*3:j*3+3] = np.array([-1,0,0]).reshape((3,1))	# chimney wall
					# self.snorm[j*3:j*3+3] = np.array([0,0,1]).reshape((3,1)) 	# wall walking ground
				else:
					self.snorm[j*3:j*3+3] = np.array([0,1,0]).reshape((3,1))
					
			# print np.round(self.footholds[1].xp[i],6), np.round(self.footholds[3].xp[i],6)
			print self.snorm.flatten()
			## Resolve forces and compute joint torque
			foot_force = self.ForceDist.resolve_force(gv, v3ca, v3wa, p_foot, 
														self.Robot.P6c.base_X_CoM[:3], 
														self.Robot.CRBI, self.Robot.Gait.cs, self.snorm )
			joint_torque = self.Robot.force_to_torque(foot_force)
			# print np.round(self.snorm,3)
			# print 'Fworld ', np.round(foot_force[15:18].flatten(),3)
			print 'Torque ', np.round(joint_torque[15:18],3)
			
			self.publish(self.CoB[i], qp)
			fforce_local = np.zeros((18,1))
			for j in range(0,6):
				self.Robot.Leg[j].XHd.update_base_X_foot(qp[j*3:j*3+3])
				R_world_X_foot = np.transpose(mX(self.Robot.XHd.world_X_base[:3,:3], self.Robot.Leg[j].XHd.base_X_foot[:3,:3]))
				fforce_local[j*3:j*3+3] = mX(R_world_X_foot, foot_force[j*3:j*3+3])
			self.Visualizer.publish_foot_force(self.Robot.Gait.cs, fforce_local)
			i += 1
			raw_input('continue')

if __name__ == "__main__":

	rviz = RvizSnapshot()
	
	rviz.load_file('chimney_medRes.csv')
	# rviz.load_file('wall_medRes.csv')

	rviz.visualise_motion_plan()
	raw_input('Start motion!')
	rviz.cycle_snapshot()