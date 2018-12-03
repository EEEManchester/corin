#!/usr/bin/env python

""" Snapshot of motion plan imported from csv """

import rospy
from corin_control import *			# library modules to include
from rviz_visual import *

from corin_msgs.msg import LoggingState
from corin_msgs.srv import *

import csv
import numpy as np

def seq(start, stop, step=1):
	n = int(round((stop - start)/float(step)))
	if n > 1:
		return([start + step*i for i in range(n+1)])
	elif n == 1:
		return([start])
	else:
		return([])

class CSVimport:
	def __init__(self):
		rospy.init_node('csv_import') 		# Initialises node
		self.rate 	  = rospy.Rate(CTR_RATE)	# Controller rate

		self.CoB = []
		self.footholds = [None]*6
		self.snorm = np.zeros((18,1))
		self.Visualizer = RvizVisualise()

		self.Robot = robot_class.RobotState()

		self.motion_plan = MotionPlan()

		self.joint_states = []

		self.__initialise_variables__()
		self.__initialise_topics__()
		self.__initialise_services__()

	def __initialise_topics__(self):
		# self.joint_pub_ = rospy.Publisher(ROBOT_NS + '/joint_states', JointState, queue_size=1)
		self.log_pub_ = rospy.Publisher('/corin/log_states', LoggingState, queue_size=1)	# LOGGING publisher

	def __initialise_services__(self):
		""" setup service call """

		self.plan_path = rospy.Service('import_csv', PlanPath, self.serv_import_csv)

	def __initialise_variables__(self):
		for j in range(0,6):
			self.footholds[j] = MarkerList()

	def serv_import_csv(self, req):
		""" Plans path given start and end position """

		print "SERVICE CALL - CSVImport"
		qbp, qbi, gphase, wXf, bXf, bXN = motionplan_to_planpath(self.motion_plan, "world")
		
		return PlanPathResponse(base_path = qbp, CoB = qbi, 
												gait_phase = gphase, 
												f_world_X_foot = wXf,
												f_base_X_foot = bXf,
												f_world_base_X_NRP = bXN)
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
				if counter == 0:
					x_cob = xpoint
					w_cob = wpoint
				else:
					x_cob = np.vstack((x_cob, xpoint))
					w_cob = np.vstack((w_cob, wpoint))

				# Append for footholds
				# self.footholds.append( map(lambda x: float(x), row[6:25]) )
				for j in range(0,6):
					self.footholds[j].t.append(counter)
					# self.footholds[j].xp.append( np.asarray(map(lambda x: float(x), row[6+j*3:6+(j*3)+3])) )
					mod_foothold = np.asarray(map(lambda x: float(x), row[6+j*3:6+(j*3)+3]))# + np.array([0.,0.,0.05])
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

		self.motion_plan.set_base_path(self.Robot.P6c.world_X_base.copy(), base_path, world_X_base, None)
		self.motion_plan.set_footholds(world_X_footholds, base_X_footholds, world_base_X_NRP)
		self.motion_plan.set_gait(self.Robot.Gait.np, self.Robot.Gait.np)
		
	def visualise_motion_plan(self):

		self.Visualizer.publish_robot(self.CoB[0])
		self.Visualizer.publish_path(self.CoB)
		self.Visualizer.publish_footholds(self.footholds)

		if self.joint_states:
			for xi in range(0,3):
				self.publish(self.CoB[0], self.joint_states[0])
				rospy.sleep(0.5)

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

def call_csv_import():
	rospy.wait_for_service('import_csv')
	try:
		start = Pose()
		goal  = Pose()
		start.position.x = 0
		start.position.y = 0
		goal.position.x = 0
		goal.position.y = 0

		path_planner = rospy.ServiceProxy('import_csv', PlanPath)
		path_planned = path_planner(start, goal)

	# 	mplan = planpath_to_motionplan(path_planned)
		
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

if __name__ == "__main__":

	csv_import = CSVimport()

	# csv_import.load_file('chimney_s054_03.csv')
	csv_import.load_file('chimney_sc_02.csv')
	# csv_import.load_file('wall_convex_02.csv')
	# csv_import.load_file('wall_concave_02.csv')

	# call_csv_import()
	rospy.spin()

	# csv_import.visualise_motion_plan()
	# raw_input('Start motion!')
	# for i in range(0,5):
	# 	# rviz.cycle_snapshot()
	# 	csv_import.cycle_states()