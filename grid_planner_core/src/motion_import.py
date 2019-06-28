#!/usr/bin/env python

""" Imports motion from csv or yaml file """

import rospy
from corin_control import *			# library modules to include
from corin_control.constant import *
from rviz_visual import *
from std_msgs.msg import *

from corin_msgs.msg import LoggingState
from grid_planner_msgs.srv import *

import csv
import numpy as np
import yaml

def seq(start, stop, step=1):
	n = int(round((stop - start)/float(step)))
	if n > 1:
		return([start + step*i for i in range(n+1)])
	elif n == 1:
		return([start])
	else:
		return([])

class YamlImport:
	def __init__(self):
		self.motion_plan = MotionPlan()
		self.mapname = None
		self.cell_res = 0.0

	def get_MotionPlan(self, filename):
		""" Imports motion plan from yaml file """

		## Reads from yaml file
		with open(filename, 'r') as stream:
			data = yaml.load(stream)

		## Mapname
		self.mapname  = data['map']
		self.cell_res = data['cell_resolution'] 
		rospy.set_param('GridMap/map_name', self.mapname)
		rospy.set_param('GridMap/resolution', self.cell_res)

		## Start & End
		start = data['start']
		goal  = data['goal']

		## Place data in MotionPlan() msg
		self.motion_plan = MotionPlan()
		self.motion_plan.qb.X.t = data['path_x_t']
		self.motion_plan.gait_phase = data['gphase']

		for i in range(len(data['path_x_xp'])):
			self.motion_plan.qb.X.xp = np.vstack((self.motion_plan.qb.X.xp, np.array(data['path_x_xp'][i])))
			self.motion_plan.qb.X.xv = np.vstack((self.motion_plan.qb.X.xv, np.array(data['path_x_xv'][i])))
			self.motion_plan.qb.X.xa = np.vstack((self.motion_plan.qb.X.xa, np.array(data['path_x_xa'][i])))

			self.motion_plan.qb.W.xp = np.vstack((self.motion_plan.qb.W.xp, np.array(data['path_w_xp'][i])))
			self.motion_plan.qb.W.xv = np.vstack((self.motion_plan.qb.W.xv, np.array(data['path_w_xv'][i])))
			self.motion_plan.qb.W.xa = np.vstack((self.motion_plan.qb.W.xa, np.array(data['path_w_xa'][i])))

		for i in range(len(data['cob'])):
			self.motion_plan.qbp.append(np.array(data['cob'][i]))

		for j in range(6):
			for i in range(len(data['wXf'][j])):
				self.motion_plan.f_world_X_foot[j].xp.append(np.array(data['wXf'][j][i]))
			for i in range(len(data['bXf'][j])):
				self.motion_plan.f_base_X_foot[j].xp.append(np.array(data['bXf'][j][i]))
			for i in range(len(data['bXn'][j])):
				self.motion_plan.f_world_base_X_NRP[j].xp.append(np.array(data['bXn'][j][i]))

		self.motion_plan.qb.X.xp[0][0] = start['x']
		self.motion_plan.qb.X.xp[0][1] = start['y']
		self.motion_plan.qb.X.xp[0][2] = start['z']

		# self.motion_plan = mplan

		return self.motion_plan

class CsvImport:
	def __init__(self):
		self.CoB = []
		self.footholds = [MarkerList()]*6
		self.joint_states = []
		self.motion_plan = MotionPlan()

	def get_MotionPlan(self, filename):

		## Instantiate leg transformation class & append initial footholds
		world_X_base = []
		world_X_footholds = [None]*6
		base_X_footholds  = [None]*6
		world_base_X_NRP  = [None]*6
		for j in range (0, 6):
			world_X_footholds[j] = MarkerList()
			base_X_footholds[j] = MarkerList()
			world_base_X_NRP[j] = MarkerList()

		print 'Reading file: ', filename
		
		with open(filename, 'rb') as csvfile:
			motion_data = csv.reader(csvfile, delimiter=',')
			counter = 0
			for row in motion_data:
				# Append for CoB
				self.CoB.append( np.asarray(map(lambda x: float(x), row[0:6])) )
				world_X_base.append( np.asarray(map(lambda x: float(x), row[0:6])) )
				
				xpoint = np.asarray(map(lambda x: float(x), row[0:3]))
				wpoint = np.asarray(map(lambda x: float(x), row[3:6]))
				
				wpoint[1] = 0.0
				if counter == 0:
					x_cob = xpoint
					w_cob = wpoint
				else:
					x_cob = np.vstack((x_cob, xpoint))
					w_cob = np.vstack((w_cob, wpoint))

				# Append for footholds
				for j in range(0,6):
					self.footholds[j].t.append(counter)
					mod_foothold = np.asarray(map(lambda x: float(x), row[6+j*3:6+(j*3)+3]))
					self.footholds[j].xp.append(mod_foothold)
				
					world_X_footholds[j].t.append(counter*CTR_INTV)
					world_X_footholds[j].xp.append(mod_foothold.copy())

				self.joint_states.append( np.asarray(map(lambda x: float(x), row[24:42])) )
				
				counter += 1

		# Interpolate base path
		PathGenerator = Pathgenerator.PathGenerator() 	# path generator for robot's base
		PathGenerator.V_MAX = 10;
		PathGenerator.W_MAX = 10;

		t_cob = seq(0, (len(x_cob)-1)*GAIT_TPHASE*6, GAIT_TPHASE*6.)
		base_path = PathGenerator.generate_base_path(x_cob, w_cob, CTR_INTV, t_cob) # Trajectory for robot's base
		# print len(x_cob), len(t_cob), t_cob
		# Plot.plot_2d(base_path.X.t, base_path.X.xp)
		# Plot.plot_2d(base_path.W.t, base_path.W.xp)

		self.motion_plan.set_base_path(np.zeros((6,1)), base_path, world_X_base, None)
		self.motion_plan.set_footholds(world_X_footholds, base_X_footholds, world_base_X_NRP)

		return self.motion_plan

class MotionImport:
	def __init__(self):
		rospy.init_node('csv_import') 		# Initialises node
		self.rate 	  = rospy.Rate(CTR_RATE)	# Controller rate

		self.yaml_reader = YamlImport()
		self.csv_reader  = CsvImport()
		# self.Visualizer = RvizVisualise()

		self.__initialise_services__()

	def __initialise_topics__(self):
		# self.joint_pub_ = rospy.Publisher(ROBOT_NS + '/joint_states', JointState, queue_size=1)
		self.log_pub_ = rospy.Publisher('/corin/log_states', LoggingState, queue_size=1)	# LOGGING publisher

	def __initialise_services__(self):
		""" setup service call """

		self.motion_import = rospy.Service(ROBOT_NS + '/import_motion_plan', PlanPath, self.serv_import_import)

	def serv_import_import(self, req):
		""" Plans path given start and end position """

		print "SERVICE CALL - CSVImport"

		filename = req.filename.data
		
		if filename.endswith('.yaml'):
			mplan = self.yaml_reader.get_MotionPlan(filename)
		elif filename.endswith('.csv'):
			mplan = self.csv_reader.get_MotionPlan(filename)

		qbp, qbi, gphase, wXf, bXf, bXN = motionplan_to_planpath(mplan, "world")
		
		return PlanPathResponse(base_path = qbp, CoB = qbi, 
												gait_phase = gphase, 
												f_world_X_foot = wXf,
												f_base_X_foot = bXf,
												f_world_base_X_NRP = bXN)

	# def publish(self, CoB, qp):
	# 	self.Visualizer.publish_robot(CoB)

	# 	dqp = JointState()
	# 	dqp.header.stamp = rospy.Time.now()
	# 	for n in range(0,18): 	# loop for each joint
	# 		dqp.name.append(str(JOINT_NAME[n])) # joint name
	# 		dqp.position.append(qp[n])			# joint angle
	# 	self.joint_pub_.publish(dqp)

	# def visualise_motion_plan(self):

	# 	self.Visualizer.publish_robot(self.CoB[0])
	# 	self.Visualizer.publish_path(self.CoB)
	# 	self.Visualizer.publish_footholds(self.footholds)

	# 	if self.joint_states:
	# 		for xi in range(0,3):
	# 			self.publish(self.CoB[0], self.joint_states[0])
	# 			rospy.sleep(0.5)

	# def cycle_states(self):
	# 	""" Publishes joint states directly """

	# 	i = 0
		
	# 	while (i != len(self.CoB) and not rospy.is_shutdown()):
	# 		for xi in range(0,2):
	# 			self.publish(self.CoB[i], self.joint_states[i])
	# 		rospy.sleep(0.1)
				
	# 		i += 1
	# 		# raw_input('cont')
	# 		rospy.sleep(0.5)

def call_motion_import(filename):
	print 'calling import'
	rospy.wait_for_service(ROBOT_NS + '/import_motion_plan',1.0)
	try:
		path_planner = rospy.ServiceProxy(ROBOT_NS + '/import_motion_plan', PlanPath)
		path_planned = path_planner(Pose(), Pose(), String(filename))
		print 'Service processed!'
	# 	mplan = planpath_to_motionplan(path_planned)
		
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

if __name__ == "__main__":

	MotionImport = MotionImport()
	# call_motion_import('data.yaml')
	# call_motion_import('chimney_autom.csv')
	# call_motion_import('chimney_s054_03.csv')
	# call_motion_import('chimney_sc_02.csv')
	# call_motion_import('wall_convex_02.csv')
	# call_motion_import('wall_concave_02.csv')
	
	rospy.spin()

	# csv_import.visualise_motion_plan()
	# raw_input('Start motion!')
	# for i in range(0,5):
	# 	# rviz.cycle_snapshot()
	# 	csv_import.cycle_states()