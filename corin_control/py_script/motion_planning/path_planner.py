#!/usr/bin/env python

""" Grid based motion planning using NetworkX
""" 

import sys; sys.dont_write_bytecode = True
sys.path.insert(0, '/home/wilson/catkin_ws/src/corin/corin_control/py_script')
from library import *			# library modules to include 
from grid_map import *

import networkx as nx
import matplotlib.pyplot as plt

import numpy as np
import math
from fractions import Fraction
from collections import OrderedDict 	# remove duplicates
# import plotgraph as Plot

import csv

## Global variables ##
POSE_TABLE = {}

## Foot position from base
p_base_X_lf_foot = ( 0.244, 0.24)
p_base_X_lm_foot = (    0., 0.29)
p_base_X_lr_foot = (-0.244, 0.24)
p_base_X_rf_foot = ( 0.244,-0.24)
p_base_X_rm_foot = (    0.,-0.29)
p_base_X_rr_foot = (-0.244,-0.24)

## Leg stances for ground walking
p_leg  = [p_base_X_lf_foot,p_base_X_lm_foot,p_base_X_lr_foot,p_base_X_rf_foot,p_base_X_rm_foot,p_base_X_rr_foot]
## Leg stances for wall & chimney walking
pw_leg = [(0.15, 0.29), (0., 0.29), (-0.15, 0.29), (0.15, -0.29), (0., -0.29), (-0.15, -0.29)]

## Robot footprint
NOM_FL = 0.30 	# nominal footprint length
MAX_FP = 0.72 	# maximum footprint width
MIN_FP = 0.20 	# minimum footprint width
MAX_WP = 0.62 	# maximum wall walking footprint
MIN_CFW = 0.60 	# minimum footprint width for chimney
MAX_CFW = 0.73 	# maximum footprint width for chimney

TEST_POINT = (38,13)

class BodyposeTable:
	def __init__(self):
		self.table = self._generate_pose_table()

	def _generate_pose_table(self):

		## Define Variables ##
		POSE_TABLE = {} # dictionary of poses

		BASE_hnom = BODY_HEIGHT # nominal base height
		BASE_hmin = 0.02 		# ground walking min. height
		BASE_hmax = 0.42 		# wall walking max. base height

		## ====================================================== ## 
		## Flat stance for ground and chimney walking
		## ====================================================== ## 
		fs = 8
		delta_fp = (MAX_FP - MAX_WP)/fs
		delta_bh = (BASE_hnom - BASE_hmin)/fs

		for i in range(0,fs+1):
			# Footprint width
			fp_w = MAX_FP - delta_fp*(i)
			fp_h = BASE_hmin + delta_bh*i

			# Leg stance width
			leg_stance = np.array([(fp_w - 2*COXA_Y)/2,	0., -fp_h])
			
			world_X_NRP = [None]*6
			for j in range(0,6):
				base_X_femur   = np.array([TRN_BASE_X_LEG[j][0], TRN_BASE_X_LEG[j][1], 0.])
				world_X_NRP[j] = np.round(mX(rot_Z(np.deg2rad(ROT_BASE_X_LEG[j]+LEG_OFFSET[j])), leg_stance) + \
											base_X_femur, 4)

			POSE_TABLE[i] = {"footprint":np.array([0.27, fp_w, fp_h]), 
								"bodypose":np.array([0.,0., fp_h, 0.,0.,0.]),
								"ground":world_X_NRP,
								"wall": [None]*6 }
		# print POSE_TABLE
		## ====================================================== ## 
		## Rolled stance for wall walking
		## ====================================================== ## 
		ns = 20
		delta_fp = (MAX_WP - MIN_FP)/ns
		delta_bh = (BASE_hmax - BASE_hnom)/ns

		for i in range(1,ns+1):

			# variables
			gnd_world_base_X_NRP  = [None]*6
			wall_world_base_X_NRP = [None]*6

			# update base parameters
			qr = i*np.pi/(2*ns) 			# base roll
			fp_h = BASE_hnom + delta_bh*i 	# base height
			fp_w = MAX_WP - delta_fp*i 		# footprint width

			world_ground_X_base_z  = fp_h
			world_base_X_femur_z   = (COXA_Y + L1)*np.sin(qr)
			world_base_X_femur_y   = (COXA_Y + L1)*np.cos(qr)

			## *********************  Ground Footholds  ********************* ##
			world_ground_X_femur_z = world_ground_X_base_z - world_base_X_femur_z

			hy = world_ground_X_femur_z - L3 	# h_femur_X_tibia
			yy = np.sqrt(L2**2 - hy**2) 		# world horizontal distance from femur to foot
			by = np.cos(qr)*(COXA_Y + L1) 		# world horizontal distance from base to femur 
			sy = by + yy						# world frame base to foot in base y-axis
			
			for j in range(0,6):
				px = 0
				py = sy*np.sin(np.deg2rad(ROT_BASE_X_LEG[j]+LEG_OFFSET[j])) 		# y_base_X_foot - world frame
				gnd_world_base_X_NRP[j] = np.array([px, py, -world_ground_X_base_z])	
				
			## *********************  Wall Footholds  ********************* ##
			bXfy = (MAX_WP - i*delta_fp)/2 		# world horizontal distance base to foot
			
			dy = bXfy - world_base_X_femur_y 	# horizontal distance from femur joint to foot
			dh = np.sqrt(L3**2 - dy**2) 		# vertical distance from femur joint to foot

			bXfz = world_base_X_femur_z + L2 + dh # vertical distance from base to foot
			
			for j in range(0,6):
				sside = 1 if (j < 3) else -1
				wall_world_base_X_NRP[j] = np.array([0., sside*bXfy, bXfz])
				# if (j==1): 
				# 	print wall_world_base_X_NRP[1]
			fp_w = gnd_world_base_X_NRP[1][1] + wall_world_base_X_NRP[1][1]
			
			POSE_TABLE[fs+i] = {"footprint":np.array([0.27, fp_w, fp_h]), 
								"bodypose":np.array([0.,0., fp_h, np.round(qr,3),0.,0.]),
								"ground":gnd_world_base_X_NRP,
								"wall": wall_world_base_X_NRP }

		return POSE_TABLE

	def search_table(self, **options):
		""" Finds desired robot pose or foothold given an input arg """
		
		## Variables
		selection = -1
		
		if options.get('width'):
			dic_arg = "footprint"
			val_arr = 1
			val_arg = options.get("width")
		elif options.get('angle'):
			dic_arg = "bodypose"
			val_arr = 3
			val_arg = options.get("angle")
		else:
			return selection

		for i in self.table:
			if options.get('width'):
				if (self.table[i][dic_arg][val_arr]<=val_arg):
					selection = i
					break
				else:
					selection = i
						
			elif options.get('angle'):
				if (self.table[i][dic_arg][val_arr]>=val_arg):
					selection = i
					break
				else:
					selection = i

		return selection

# tb = BodyposeTable()
# print tb.table
class PathPlanner:
	def __init__(self, Map):
		self.GridMap = Map
		self.Robot = robot_class.RobotState()
		self.PoseTable = BodyposeTable()

		self.rbfp_size = (0.488, 0.5) 	# robot footprint size, (m,m) - TARGET: (0.488,0.58)
		self.rbbd_size = (0.27, 0.18) 	# robot body size, (m,m) - 

		self.advance_capable = True

		self.T_GND_X_WALL = False
		self.T_GND_X_CHIM = False
		self.T_WALL_X_GND = False
		self.T_CHIM_X_GND = False
		self.W_GND 	= True
		self.W_WALL = False
		self.W_CHIM = False

		self.di_wall = 0. 	# distance from base to wall

		self.__initialise_graph__()

	def __initialise_graph__(self):

		# determine robot related size in grid cell
		self.rbfp_gd = tuple(map(lambda x: int(math.ceil(x/self.GridMap.resolution)), self.rbfp_size))					# footprint
		self.rbbd_gd = tuple(map(lambda x: int(math.ceil(x/self.GridMap.resolution)), self.rbbd_size))					# body
		self.rblg_gd = (int(math.ceil(LEG_AREA_LX/self.GridMap.resolution)),int(math.ceil(LEG_AREA_LY/self.GridMap.resolution)))	# leg

		self.base_map_sz = self.GridMap.get_map_size()			# map size in number of grid cells
		self.base_map = nx.grid_graph(dim=[self.base_map_sz[1], self.base_map_sz[0]]) 	# truncated body map

		# Graph for illustrations: 
		self.GM_walk = nx.Graph()
		self.GM_wall = nx.Graph()
		self.GM_chim = nx.Graph()

		# add moore edges and remove edges linked to obstacles
		self.GridMap.set_moore_edge(self.base_map)

		## set attribute for map_free nodes
		# cost of motion
		nx.set_node_attributes(self.base_map, {e: 0 for e in self.base_map.nodes()}, 'cost')
		# motion identifier: -1 = None, 0 = walk, 1 = chimney, 2 = wall
		nx.set_node_attributes(self.base_map, {e: -1 for e in self.base_map.nodes()}, 'motion') 	
		# footprint lateral width
		nx.set_node_attributes(self.base_map, {e: 0 for e in self.base_map.nodes()}, 'width') 	
		# robot bodypose - height, roll, pitch, yaw
		nx.set_node_attributes(self.base_map, {e: [0.]*4 for e in self.base_map.nodes()}, 'pose')	

		print 'Initialised Path Planner - '
		print '\tMap Grid  : ', self.base_map_sz
		print '\tRobot grid: ', self.rbfp_gd
		print '\tBody grid : ', self.rbbd_gd
		print '\tFoot grid : ', self.rblg_gd
		print '==============================================='

	def remove_motion_edges(self, G):
		""" remove edges with obstacles """

		# First, remove edges linked to obstacles
		# self.GridMap.remove_edges(self.base_map)

		e_init = G.number_of_edges()	# determine number of edges
		r_edge = [] 					# list for edges to remove
		
		for e in G.edges():
			# if either cell contains obstacles, remove edge
			# if (self.GridMap.Map.nodes[e[0]]['height']!= 0 or self.GridMap.Map.nodes[e[1]]['height']!=0): 
			# 	# stack into list to remove
				# r_edge.append(e)
			if (self.base_map.nodes[e[0]]['motion'] == -1 or self.base_map.nodes[e[1]]['motion'] == -1):
				r_edge.append(e)

		for i in range(0,len(r_edge)):
			G.remove_edge(r_edge[i][0],r_edge[i][1])

	def check_area_collision(self, p, body_area, yaw=0):
		""" Check area centered at p, orientated about q, for collision """
		""" Input: 	1) p -> grid cell location
					2) area -> size of area in metre^2 to check
					3) yaw -> base yaw angle about world frame
			Output: valid -> True if no collision 						"""
		
		## variables ##
		valid = False

		## skip if centre occupied - fast check
		if (self.GridMap.Map.nodes[p]['height'] != 1):
			bx = int(math.floor(body_area[0])/2) # longitudinal distance to check
			by = int(math.floor(body_area[1])/2) # lateral distance to check

			## check entire body area
			for y in range(0,by+1): 		# cycle lateral - inflated if body_area[1] is even
				for x in range(0,2*bx+1):	# cycle longitudinal
					# displacement from centre rotated about q
					nx1 = p[0] + int(np.round((-bx+x)*np.cos(yaw) - (-y)*np.sin(yaw)))
					ny1 = p[1] + int(np.round((-bx+x)*np.sin(yaw) + (-y)*np.cos(yaw)))
					nx2 = p[0] + int(np.round((-bx+x)*np.cos(yaw) - (+y)*np.sin(yaw)))
					ny2 = p[1] + int(np.round((-bx+x)*np.sin(yaw) + (+y)*np.cos(yaw)))

					try:
						EL_edge = self.GridMap.Map.nodes[(nx1,ny1)]['height']		# left edge
						ER_edge = self.GridMap.Map.nodes[(nx2,ny2)]['height']		# right edge
					except KeyError:
						valid = False
						break
					# if (p==TEST_POINT):
					# 	print p, body_area, (nx1,ny1), (nx2,ny2)
					# 	print '------------------------'
					# check if collision exists
					if (EL_edge==1 or ER_edge==1):
						valid = False
						break 			# exit immediately as area invalid
					else:
						valid = True

		return valid

	def ground_walking(self, p, q=0):
		""" Checks if foothold area valid for robot's CoB at p 	"""
		""" Input: 	1) p -> robot's CoB point (grid)
					2) q -> base yaw (rad)`
			Output: 1) motion_valid -> True if foothold area for 
									all legs
					2) m_width -> robot footprint 				"""

		## Variables ##
		m_width = 0.6 				# footprint width of motion
		bx = (self.rblg_gd[0]-1)/2  # longitudinal distance from robot centre
		by = (self.rblg_gd[1]-1)/2 	# lateral distance from robot centre
		motion_valid = True
		foothold_exist = False

		## Check if valid foothold exist in each leg region, outputs boolean
		for i in range(0,6):
			# Approximate nominal stance to nearest cell
			px 	  = int(np.round( (p_leg[i][0]*np.cos(q)-p_leg[i][1]*np.sin(q))/self.GridMap.resolution))
			py 	  = int(np.round( (p_leg[i][0]*np.sin(q)+p_leg[i][1]*np.cos(q))/self.GridMap.resolution))
			pbnom = (px,py)

			pinom = tuple(map(lambda x,y: y+x, p, pbnom))	# leg nominal stance in world frame - equal for left and right side
			fcost = 0 										# reset total cost per foot
			
			## Flag for skipping row & column
			skip_row = False
			# skip_col = False

			foothold_exist = False 	# reset flag

			## calculate sum for each area
			for cx in range(0,self.rblg_gd[0]): 		# cycle through x
				skip_row = False 						# reset flag 

				for cy in range(0,self.rblg_gd[1]):		# cycle through y
					## skip rest of row if inner cell occupied
					if (skip_row == False):
						## foothold in leg grid area - start inside to outside, top to bottom
						if (i<3):
							pwset = (pinom[0]+bx-cx,pinom[1]-(by-cy))
						else:
							pwset = (pinom[0]+bx-cx,pinom[1]+(by-cy))

						## check if cell occupied
						try:
							# if (p == TEST_POINT):
							# 	print p, pinom, pwset, self.GridMap.Map.nodes[pwset]['height']
							if (self.GridMap.Map.nodes[pwset]['height']==1):
								fcost = fcost + self.GridMap.Map.nodes[pwset]['height'] 	# calculate cost of area - not true since will be skiping through
								skip_row = True 								# set flag to skip remainder of row - computationally efficient
							elif (self.GridMap.Map.nodes[pwset]['height'] == -1):
								pass
							else:
								foothold_exist = True
						except:
							pass

			if (foothold_exist is False):
				motion_valid = False
				break

		return motion_valid, m_width

	def wall_walking(self, p, yaw=0):
		""" Check if wall walking is feasible for CoB at point p """
		""" Input: 	p -> CoB location 
					yaw -> base yaw orientation wrt world frame """

		## variables ##
		width = 0 					# footprint width (cells)
		LA = [0]*6 					# flag for each leg
		EL_edge = False				# external left edge
		ER_edge = False				# external right edge
		motion_valid = False 		# boolen flag if area is permissible
		
		bx_min = int(np.ceil((NOM_FL/self.GridMap.resolution)/2))	# min. footprint area
		by_min = int(np.ceil((MIN_FP/self.GridMap.resolution)/2))	# min. footprint area

		bx_max = bx_min 									# max. footprint area
		by_max = int(np.ceil((MAX_WP/self.GridMap.resolution)/2))	# max. footprint area
		
		# skip if centre occupied
		if (self.GridMap.Map.nodes[p]['height'] != 1):

			# Check between minimum to maximum wall walking footprint
			for d in range(0,by_max-by_min):
				# update footprint area for collision check
				fp_area = (int(NOM_FL/self.GridMap.resolution), int((MIN_FP/self.GridMap.resolution)+2*d))
					
				# continues to check boundary edges only if internal boundary valid
				if (self.check_area_collision(p, fp_area, yaw) is True):

					for i in range(0,6):
						# Foot central position - TODO: YAW DEPENDANT
						pxi = p[0] + int(np.round( (pw_leg[i][0]*np.cos(yaw)-pw_leg[i][1]*np.sin(yaw))/self.GridMap.resolution))
						# offset from central
						ix_off = int(np.floor(self.rblg_gd[0]/2))

						for x in range(0, self.rblg_gd[0]):
							# Set leg foothold - TODO: YAW DEPENDANT
							if (i<3):
								pf1 = (pxi-ix_off+x, p[1]+by_min+d)
								pf2 = (pxi-ix_off+x, p[1]+by_min+d-1)
							else:
								pf1 = (pxi-ix_off+x, p[1]-(by_min+d))
								pf2 = (pxi-ix_off+x, p[1]-(by_min+d-1))
								# if (p==TEST_POINT):
								# 	print i, p, d, pf1, pf2
							try:
								if (self.GridMap.Map.nodes[pf1]['height']==1 and self.GridMap.Map.nodes[pf2]['height']==1):
									LA[i] = -1;
									break
								else:
									LA[i] += self.GridMap.Map.nodes[pf1]['height'] 	# front leg
							except KeyError:
								LA[i] = -1;
								break
					# if (p==TEST_POINT):
					# 	print 'flag: ', fp_area, LA

					## valid if THREE legs on wall have footholds and ground are all ground
					if (np.amin(LA) < 0):
						# reset flags as statically unfeasible - no holes allowed in proximity
						EL_edge	= ER_edge = False
						motion_valid = False
					else:
						if (all(LA[:3]) and LA[3:6] == [0]*3):
							# LHS wall walking
							EL_edge = motion_valid = True
						elif (all(LA[3:6]) and LA[:3] == [0]*3):
							# RHS wall walking
							ER_edge = motion_valid = True

					# Exit loop if valid configuration exists
					if (motion_valid):
						sided = 1 if EL_edge else -1	# set signs according to rotation direction
						width = sided*(fp_area[1]+1)	# footprint width (cells)
						break
				# Exit search as internal boundary violated
				else:
					break
		# if (p==TEST_POINT):
		# 	print 'this: ', p, motion_valid, width
		return motion_valid, width

	def chimney_walking(self, p, yaw=0):
		""" Check if chimney walking is feasible for CoB at point p """
		""" Input: 	p -> CoB location 
					yaw -> base yaw orientation wrt world frame """

		## variables ##
		i_edge = 0 				# internal edge
		width = 0 				# footprint width (cells)
		LA = [0]*6 				# flag for each leg
		EL_edge = False			# external left edge
		ER_edge = False			# external right edge
		motion_valid = False 	# boolen flag if area is permissible
		
		bx_min = int(np.ceil((NOM_FL/self.GridMap.resolution)/2))	# min. footprint area
		by_min = int(np.ceil((MIN_CFW/self.GridMap.resolution)/2))	# min. footprint area

		bx_max = bx_min 									# max. footprint area
		by_max = int(np.ceil((MAX_CFW/self.GridMap.resolution)/2))	# max. footprint area
		# if (p==TEST_POINT):
		# 	print 'chim: ', (bx_min, by_min), (bx_max, by_max)
		
		# skip if centre occupied
		if (self.GridMap.Map.nodes[p]['height'] != 1):

			# Check between minimum to maximum wall walking footprint
			for d in range(0,by_max-by_min):
				LA = [0]*6
				# update footprint area for collision check
				fp_area = (int(NOM_FL/self.GridMap.resolution), int((MIN_FP/self.GridMap.resolution)+2*d))
				# if (p==TEST_POINT):
				# 	print p, fp_area, self.check_area_collision(p, fp_area, yaw), LA
				# continues to check boundary edges only if internal boundary valid
				if (self.check_area_collision(p, fp_area, yaw) is True):

					for i in range(0,6):
						# Foot central position - TODO: YAW DEPENDANT
						pxi = p[0] + int(np.round( (pw_leg[i][0]*np.cos(yaw)-pw_leg[i][1]*np.sin(yaw))/self.GridMap.resolution))
						# offset from central
						ix_off = int(np.floor(self.rblg_gd[0]/2))

						for x in range(0, self.rblg_gd[0]):
							# Set leg foothold - TODO: YAW DEPENDANT
							if (i<3):
								pf1 = (pxi-ix_off+x, p[1]+by_min+d)
								pf2 = (pxi-ix_off+x, p[1]+by_min+d-1)
							else:
								pf1 = (pxi-ix_off+x, p[1]-(by_min+d))
								pf2 = (pxi-ix_off+x, p[1]-(by_min+d-1))
							# if (p==TEST_POINT):
							# 	print p, i, pxi, pf1, pf2, LA
							# 	print 'cost: ', self.GridMap.Map.nodes[pf1]['height'], self.GridMap.Map.nodes[pf2]['height']
							try:
								if (self.GridMap.Map.nodes[pf1]['height']==1 and self.GridMap.Map.nodes[pf2]['height']==1):
									LA[i] = -1;
									break
								else:
									LA[i] += self.GridMap.Map.nodes[pf1]['height'] 	# front leg
							except KeyError:
								LA[i] = -1;
								break

					## check motion validity
					if (np.amin(LA) < 0):
						# reset flags as statically unfeasible - no holes allowed in proximity
						EL_edge	= ER_edge = False
						motion_valid = False
					else:
						""" VALID: front or rear TWO pairs of leg has wall contact footholds on both sides """
						if (all(LA[:2]) and LA[3:4] > [0]*3):
							if (LA[2] >= 0 and LA[5] >=0 ):
								motion_valid = True
								break
								# if (p==TEST_POINT):
								# 	print 'chimney exists!'
						elif (all(LA[1:3]) and LA[4:6] > [0]*3):
							if (LA[0] >= 0 and LA[3] >=0 ):
								motion_valid = True
								break
								# if (p==TEST_POINT):
								# 	print 'chimney exists!'

						## valid: THREE wall contact footholds on both sides
						# if (all(LA[:3]) and LA[3:6] == [1]*3):
						# 	EL_edge = ER_edge = motion_valid = True
				# Exit search as internal boundary violated
				else:
					break
		
		# Set footprint width if motion valid
		width = (by_min+d)*2-1 if motion_valid else 0.
		# if (p==TEST_POINT):
		# 	print p, motion_valid, width, by_min, d
		return motion_valid, width
		
	def check_motion_primitive(self, p):
		""" Checks if cell is accessible using motion primitives, 
			start with ground then chimney and finally wall walking	"""
		## TODO: YAW DEPENDANT
		## Variables ##
		m_valid = False
		m_width = -1
		primitive = -1		# motion primitive: 0 = walk,1 = chimney,2 = wall
		
		## Ground walking ##
		#  condition: valid foothold exists within foothold area
		base_valid = self.check_area_collision(p, self.rbbd_gd)
		if (base_valid):
			m_valid, m_width = self.ground_walking(p)
			if (m_valid):
				primitive = 0
				self.GM_walk.add_node(p)		# ILLUSTRATION: stack into graph
		
		## Advanced motion primitives
		if (self.advance_capable):
			
			## Chimney ##
			if (not m_valid and base_valid):
				# condition: wall on both sides
				m_valid, m_width = self.chimney_walking(p)
				if (m_valid):
					primitive = 1
					self.GM_chim.add_node(p)	# ILLUSTRATION: stack into graph
				
			## Wall ##
			if (not m_valid):# and self.check_area_collision(p,(self.rbbd_gd[0],3))):
				# condition: 1) wall within X.X m from base point, 2) ground is present
				m_valid, m_width = self.wall_walking(p)
				if (m_valid):
					primitive = 2
					self.GM_wall.add_node(p)	# ILLUSTRATION: stack into graph

		return m_valid, primitive, m_width

	def process_map_on_primitive(self):
		""" Process entire map for motion primitive feasibility """

		## Variables ##
		ax = int(np.ceil((self.rbfp_gd[0])/2)) 			# robot longitudinal distance (grid cell)
		ay = int(np.ceil((MIN_FP/self.GridMap.resolution)/2))	# robot lateral distance (grid cell) - HARDCODED
		self.WIDTH_SAFETY = 0.05
		# print 'Body Edges: ', ax, ' ', ay
		
		node_ignored = [] 	# list for nodes to ignore: either near map border or intersect obstacles/holes
		node_free 	 = [] 	# list for free nodes

		for e in self.GridMap.Map.nodes():
			# Ignore CoB that intersect edges
			## TODO: YAW DEPENDANT
			if (e[0]<=(ax-1) or e[0]>=(self.base_map_sz[0]-ax) or e[1]<=(ay-1) or e[1]>=(self.base_map_sz[1]-ay)): 	
				node_ignored.append(e)
				self.base_map.nodes[e]['cost'] = 1
			else:
				# Check through possible motion primitives
				m_valid, primitive, m_width = self.check_motion_primitive(e)
				# if (e == TEST_POINT):
				# 	print e, m_valid, primitive
				if (m_valid):
					# identify pose for wall walking
					if (primitive==2):
						idx = self.PoseTable.search_table(width=abs(m_width)*self.GridMap.resolution - self.WIDTH_SAFETY)
						bh = self.PoseTable.table[idx]['bodypose'][2]
						sb = 1 if (m_width > 0.) else -1
						qr = sb*self.PoseTable.table[idx]['bodypose'][3]
						# if (e == (13,14)):
						# 	print 'at this point: '
						# 	print 'itb: ', idx, abs(m_width)*self.GridMap.resolution
						# 	print bh, qr
						# 	print '---------------------'
					else:
						bh = BODY_HEIGHT
						qr = 0.

					node_free.append(e)
					self.base_map.nodes[e]['cost']   = primitive # set cost using primitive
					self.base_map.nodes[e]['motion'] = primitive
					self.base_map.nodes[e]['width']  = m_width
					self.base_map.nodes[e]['pose']   = [bh,qr,0.,0.]
				else:
					self.base_map.nodes[e]['cost'] = 1
				
		self.Gfree = nx.path_graph(node_free)

	def find_base_path(self,start,end):
		""" Finds a path given start and end position on map """

		def eucld_dist(a,b):
			""" heuristics used for A* planner """
			(x1, y1) = a
			(x2, y2) = b
			return (((x1 - x2)**2 + (y1 - y2)**2)**0.5)

		self.process_map_on_primitive() # finds path then use motion primitives to fit in
		self.remove_motion_edges(self.base_map) 	# remove edges linked to invalid cells
		self.GridMap.set_edge_cost(self.base_map)	# assign cost to edge
		
		## Display graph of grid map connectivity
		# b_map = dict(zip(self.base_map, self.base_map))
		# ax = nx.draw_networkx(self.base_map, b_map, with_labels=False, node_size=20, node_color='#ff6600', width=0.5, alpha=1.0, node_shape="s");
		# plt.grid('on');		 plt.grid(which='major', linestyle=':', linewidth='0.5', color='black')
		# plt.show()
		
		## Robot represented with a number of nodes
		try:
			# list_path	= nx.shortest_path(self.base_map,start,end,weight='cost')
			# list_path	= nx.dijkstra_path(self.base_map,start,end,weight='cost')
			return nx.astar_path(self.base_map, start, end, heuristic=eucld_dist, weight='cost')

		except nx.exception.NetworkXNoPath:
			print 'ERROR, no body path exist!'
			return None

	def footprint_cost(self):
		""" Identify valid cells based on the existence of valid foothold areas """
		""" based on valid cells, create body map Gfree, and find path using A* """

		ax = (self.rbfp_gd[0]-1)/2 	# longitudinal distance from robot centre
		ay = (self.rbfp_gd[1]-2)/2	# lateral distance from robot centre
		# print 'Edges: ', ax, ' ', ay
		
		node_ignored = [] 	# list for nodes to ignore: either near map border or intersect obstacles/holes
		node_free 	 = [] 	# list for free nodes

		for e in self.GridMap.Map.nodes():
			# ignore edges
			if (e[0]<=(ax-1) or e[0]>=(self.base_map_sz[0]-ax) or e[1]<=(ay-1) or e[1]>=(self.base_map_sz[1]-ay)): 	
				node_ignored.append(e)
				self.base_map.nodes[e]['cost'] = 1
			else:
				# check body collision TODO: automate this to a square dependant on resolution 
				valid = self.check_area_collision(e, self.rbbd_gd)
				
				# compute for nodes that are obstacle free
				if (valid):
					footholds_cost, valid_flag = self.ground_walking(e)

					if (valid_flag == True):
						node_free.append(e)
						self.base_map.nodes[e]['cost'] = 0
					else:
						self.base_map.nodes[e]['cost'] = 1
				else:
					self.base_map.nodes[e]['cost'] = 1

		self.Gfree = nx.path_graph(node_free)

	def find_valid_foothold(self, sp, j):
		""" Find lowest valid foothold in leg grid area """
		print 'area sp: ', np.round(sp,4)
		footholds = self.GridMap.square_spiral_search(sp, (3,3), j)
		cost = np.zeros(len(footholds))
		print footholds
		for i in range(0, len(footholds)):
			cost[i] = self.GridMap.get_index('height', footholds[i])
			if (cost[i] == 0):
				# convert to (m)
				position = np.array([self.GridMap.resolution*footholds[i][0],
										self.GridMap.resolution*footholds[i][1],
										0.])
				print 'Valid foothold at ', footholds[i], position
				return position

		print 'No Valid Foothold in Foot Area'

	def foothold_planner(self, base_path):
		""" Computes foothold based on generated path 	"""
		""" Input: 1) path -> linear and angular path 
			Output: Foothold array 						"""

		def set_motion_plan():
			## Set MotionPlan class
			motion_plan = MotionPlan()
			motion_plan.set_base_path(self.Robot.P6c.world_X_base.copy(), base_path, world_X_base, gait_phase)
			motion_plan.set_footholds(world_X_footholds, base_X_footholds, world_base_X_NRP)
			motion_plan.set_gait(self.Robot.Gait.np, self.Robot.Gait.np)

			return motion_plan

		def compute_vector_heading(v3cp, v3cp_prev, snorm):
			v3_dv = (v3cp - v3cp_prev).flatten() 			# direction vector from previous to current CoB
			v3_pv = v3_dv - (np.dot(v3_dv,snorm))*snorm 	# project direction vector onto plane
			m1_dv = np.linalg.norm(v3_pv) 					# magnitude of direction vector
			return v3_pv/m1_dv 								# unit vector direction

		def compute_ground_footholds():
			""" Computes NRP position for ground footholds """
			## TODO: this should also hold for normal ground walking
			world_ground_X_base = self.Robot.XHd.world_X_base.copy()
			world_ground_X_base[:2,3:4] = np.zeros((2,1))
			world_ground_X_femur = mX(world_ground_X_base, self.Robot.Leg[j].XHd.base_X_femur)

			hy = world_ground_X_femur[2,3] - L3 - 0. 		# h_femur_X_tibia
			yy = np.sqrt((L2**2 - hy**2)) 				# world horizontal distance from femur to foot
			by = np.cos(v3wp[0])*(COXA_Y + L1) 				# world horizontal distance from base to femur 
			sy = by + yy									# y_base_X_foot - leg frame
			py = sy*np.sin(np.deg2rad(ROT_BASE_X_LEG[j]+LEG_OFFSET[j])) 	# y_base_X_foot - world frame
			
			# Create temp array, which is the new base to foot position, wrt world frame
			# The x-component uses the base frame NRP so that it remains the same

			temp = np.array([[self.Robot.Leg[j].XHd.base_X_NRP[0,3]],
								[py],
								[-world_ground_X_base[2,3]]])
			self.Robot.Leg[j].XHd.world_base_X_NRP[:3,3:4] = mX(rot_Z(v3wp[2]), temp)
			self.Robot.Leg[j].XHd.world_X_NRP[:3,3] = np.round( (v3cp + self.Robot.Leg[j].XHd.world_base_X_NRP[:3,3:4]).flatten(),4)
			
		def compute_wall_footholds(d_wall):
			""" Compute footholds for legs in wall contact """

			qj = np.pi/2 if (j < 3) else -np.pi/2 	# generalisation rotation from base to leg frame
			world_base_X_femur_z = abs((COXA_Y + L1)*np.sin(v3wp[0]))
			world_base_X_femur_y = (COXA_Y + L1)*np.cos(v3wp[0])*np.sin(qj)

			bXfy = d_wall[1]*np.sin(qj) 			# world horizontal distance base to foot
			dy = bXfy - world_base_X_femur_y 		# horizontal distance from femur joint to foot
			dh = np.sqrt(abs(L3**2 - dy**2)) 		# vertical distance from femur joint to foot
			bXfz = world_base_X_femur_z + L2/2 + dh # vertical distance from base to foot
			
			temp = np.array([[self.Robot.Leg[j].XHd.base_X_NRP[0,3]],[bXfy],[bXfz]])
			self.Robot.Leg[j].XHd.world_base_X_NRP[:3,3:4] = mX(rot_Z(v3wp[2]), temp)
			self.Robot.Leg[j].XHd.world_X_NRP[:3,3] = np.round( (v3cp + self.Robot.Leg[j].XHd.world_base_X_NRP[:3,3:4]).flatten(),4)
			
		def set_leg_stance():
			""" Sets leg offset and update robot stance """

			if (self.T_GND_X_WALL or self.T_GND_X_CHIM or self.T_CHIM_X_GND):
				print 'Setting to narrow stance'
				leg_offset = (40., -40.)
				
				stance = self.Robot.set_leg_stance(STANCE_WIDTH, BODY_HEIGHT, leg_offset, "flat")
				self.Robot.Gait.set_step_stroke(stance, LEG_CLEAR, STEP_STROKE)

				for j in [0,2,3,5]:
					self.Robot.Leg[j].XHd.update_base_X_NRP(self.Robot.KDL.leg_IK(stance[j]))
					self.Robot.Leg[j].XHd.update_world_base_X_NRP(self.Robot.P6c.world_X_base)

				return stance

		def set_selected_leg_stance(leg_offset):
			stance = self.Robot.set_leg_stance(STANCE_WIDTH, BODY_HEIGHT, leg_offset, "flat")
			self.Robot.Leg[j].XHd.update_base_X_NRP(self.Robot.KDL.leg_IK(stance[j]))
			self.Robot.Leg[j].XHd.update_world_base_X_NRP(self.Robot.P6c.world_X_base)
		
		i = ig = 0
		if (self.T_GND_X_WALL):
			do_wall = np.array([0., 0.32, 0.]) 		# Initial distance from robot's base to wall (base frame)
		elif (self.T_WALL_X_GND):
			do_wall = self.di_wall
		elif (self.T_GND_X_CHIM):
			do_wall = self.di_wall

		if (self.W_GND is False):
			leg_stance = set_leg_stance()
		
		gphase_intv  = [] 						# intervals in which gait phase changes
		gait_phase	 = []
		world_X_base = []
		world_X_footholds = [None]*6
		base_X_footholds  = [None]*6
		world_base_X_NRP  = [None]*6
		break_n = 0
		break_count = 0

		v3cp_prev = self.Robot.P6c.world_X_base[0:3].copy()	# previous CoB position
		v3wp_prev = self.Robot.P6c.world_X_base[3:6].copy()	# previous CoB orientation

		## Instantiate leg transformation class & append initial footholds
		for j in range (0, 6):
			world_X_footholds[j] = MarkerList()
			base_X_footholds[j] = MarkerList()
			world_base_X_NRP[j] = MarkerList()

		## Returns as footholds remain fixed for full support mode
		if (self.Robot.support_mode is True):
			return set_motion_plan()

		## Cycle through trajectory
		while (i != len(base_path.X.t)):
			# print i, ' Gait phase ', self.Robot.Gait.cs 
			leg_exceed = None
			bound_exceed = False 	# reset suspension flag
			ig = i 					# last count which gait changes

			# Cycles through one gait phase for support legs
			for m in range(0,int(GAIT_TPHASE*CTR_RATE)+1):
				## Variable mapping to R^(3x1) for base linear and angular time, position, velocity, acceleration
				try:
					v3cp = base_path.X.xp[i].reshape(3,1) #+ Robot.P6c.world_X_base[0:3]
					v3wp = base_path.W.xp[i].reshape(3,1) #+ self.Robot.P6c.world_X_base[3:6]
					
					P6d_world_X_base = np.vstack((v3cp,v3wp))
					
					## update robot's pose
					self.Robot.XHd.update_world_X_base(P6d_world_X_base)
					world_X_base_rot = self.Robot.XHd.world_X_base.copy()
					world_X_base_rot[:3,3:4] = np.zeros((3,1))
					
					for j in range (0, self.Robot.active_legs):
						## Legs in support phase:
						if (self.Robot.Gait.cs[j] == 0):
							self.Robot.Leg[j].XHd.base_X_foot = mX(self.Robot.XHd.base_X_world, 
																	self.Robot.Leg[j].XHd.world_X_foot)
							self.Robot.Leg[j].XHd.world_base_X_foot = mX(world_X_base_rot, 
																			self.Robot.Leg[j].XHd.base_X_foot)
							bound_exceed = self.Robot.Leg[j].check_boundary_limit(self.Robot.Leg[j].XHd.world_base_X_foot,
																					self.Robot.Leg[j].XHd.world_base_X_NRP,
																					self.Robot.Gait.step_stroke)
							# if (j==3):
							# 	print 'wbXf: ', np.round(self.Robot.Leg[j].XHd.world_base_X_foot[:3,3],3)
							# 	print 'wbXn: ', np.round(self.Robot.Leg[j].XHd.world_base_X_NRP[:3,3],3)
							# 	print self.Robot.Gait.step_stroke
							base_X_q2 = mX(self.Robot.Leg[j].XHd.base_X_coxa, v3_X_m(np.array([L1,0.,0.])))
							world_base_X_q2 = mX(world_X_base_rot, base_X_q2)
							world_q2_X_foot = self.Robot.Leg[j].XHd.world_base_X_foot[:3,3] - world_base_X_q2[:3,3]
							mag = np.sqrt(world_q2_X_foot[1]**2 + world_q2_X_foot[2]**2)
							
							if (mag > (L2+L3-0.005)):
								print 'leg length exceeded on ', j, ' at n=', i, mag
								bound_exceed = True
								leg_exceed = j
								break

							if (bound_exceed == True):
								print 'bound exceed on ', j, ' at n=', i
								leg_exceed = j
								break

					## compute stability margin
					## Define Variables ##
					stack_base_X_world = []
					for j in range(6):
						stack_base_X_world.append(self.Robot.Leg[j].XHd.world_base_X_foot[:3,3])
					# compute Longitudinal Stability Margin
					sm = self.Robot.SM.LSM(stack_base_X_world, self.Robot.Gait.cs)
					if (sm[0] <= SM_MIN or sm[1] <= SM_MIN):
						print m, ' current sm: ', sm, self.Robot.Gait.cs
						# print stack_base_X_world

					if (bound_exceed is True):
						if (i == break_n):
							break_count += 1
							# print 'break count ', break_count
							if (break_count == 10):
								print 'Inifinite loop'
								raw_input('Local minima - invalid solution')
								return None
						else:
							break_n = i
							break_count = 0
						## To break through two layers of loops
						break
					else:	
						i += 1

				except IndexError:
					## Trajectory finished, skip support
					print 'Finishing foothold planning at ', i
					break

			## Stack to list next CoB location
			world_X_base.append(P6d_world_X_base.flatten())
			gphase_intv.append(i)
			gait_phase.append(self.Robot.Gait.cs)
			print self.Robot.Gait.cs
			# print 'qbp:: ', np.round(P6d_world_X_base.reshape(1,6),3), i

			## Set foothold for legs in transfer phase
			for j in range (0, 6):
				if (self.Robot.Gait.cs[j] == 1 and i <= len(base_path.X.t)):
					
					## 1) Update NRP
					if (self.W_WALL):
						## Identify sides for ground or wall contact based on body roll, 1 = wall left, -1 = wall right
						if (self.T_GND_X_WALL):
							delta_w = 1 if (base_path.W.xp[-1][0] >= 0.) else -1
						elif (self.T_WALL_X_GND):
							delta_w = 1 if (base_path.W.xp[0][0] > 0.) else -1
						else:
							delta_w = 1 if (P6d_world_X_base[3] > 0.) else -1

						# Update Robot's base distance travelled from origin for transition
						if (self.T_GND_X_WALL or self.T_WALL_X_GND):
							dt_base = (mX(rot_Z(-v3wp[2]), (v3cp.reshape(3) - base_path.X.xp[0]).reshape(3,1) )).reshape(3)
							self.di_wall = do_wall - delta_w*np.round(dt_base ,3)

						# Set sides for ground or wall contact
						if (delta_w == 1):
							# LHS going up
							if (j >= 3):
								compute_ground_footholds()
							else:
								if (self.T_WALL_X_GND and self.di_wall[1] == 0.32 and abs(dt_base[1])<0.005):
									# compute ground contact at end of wall to ground transition
									set_selected_leg_stance((40.,-40.))
								else:
									compute_wall_footholds(self.di_wall)

						else:
							# RHS going up
							if (j >= 3):
								if (self.T_WALL_X_GND and self.di_wall[1] == 0.32 and abs(dt_base[1])<0.005):
									# compute ground contact at end of wall to ground transition
									set_selected_leg_stance((40.,-40.))
								else:
									compute_wall_footholds(self.di_wall)
							else:
								compute_ground_footholds()
						
					elif (self.W_CHIM):
						if (self.T_GND_X_CHIM):
							# Set to wall contacts	
							qj = np.pi/2 if (j < 3) else -np.pi/2 	# generalisation rotation from base to leg frame
							bXfy = self.di_wall[1]*np.sin(qj)
							temp = np.array([[self.Robot.Leg[j].XHd.base_X_NRP[0,3]],[bXfy],[0.]])
							self.Robot.Leg[j].XHd.world_base_X_NRP[:3,3:4] = mX(rot_Z(v3wp[2]), temp)
							self.Robot.Leg[j].XHd.world_X_NRP[:3,3] = np.round( (v3cp + self.Robot.Leg[j].XHd.world_base_X_NRP[:3,3:4]).flatten(),4)
						
						elif (self.T_CHIM_X_GND):
							# set to ground contacts
							KDL = kdl.KDL()
							self.Robot.Leg[j].XHd.update_base_X_NRP(KDL.leg_IK(leg_stance[j])) 
							self.Robot.Leg[j].XHd.update_world_base_X_NRP(P6d_world_X_base)
							self.Robot.Leg[j].XHd.world_X_NRP[:3,3] = np.round( (v3cp + self.Robot.Leg[j].XHd.world_base_X_NRP[:3,3:4]).flatten(),4)
					
					else:
						self.Robot.Leg[j].XHd.update_world_base_X_NRP(P6d_world_X_base)

					## 2) Compute magnitude & vector direction
					if (i == len(base_path.X.t)):
						v3_uv = np.zeros(3)
					else:
						snorm = self.GridMap.get_cell('norm', self.Robot.Leg[j].XHd.world_X_NRP[:3,3], j) 	# Get surface normal
						v3_uv = compute_vector_heading(v3cp, v3cp_prev, snorm)

						if (np.isnan(v3_uv).any() or np.isinf(v3_uv).any()):
							try:
								v3cp_nex = base_path.X.xp[i+1].reshape(3,1)
							except:
								v3cp_nex = np.zeros((3,1))
							v3_uv = compute_vector_heading(v3cp_nex, v3cp_prev, snorm)
					
					if (self.T_GND_X_WALL):
						v3_uv = np.zeros(3)

					## 3) Compute AEP wrt base and world frame
					
					self.Robot.Leg[j].XHd.world_base_X_AEP[:3,3] = self.Robot.Leg[j].XHd.world_base_X_NRP[:3,3] + \
																		np.nan_to_num(v3_uv*self.Robot.Gait.step_stroke/2.)
					self.Robot.Leg[j].XHd.base_X_AEP[:3,3:4] = mX(self.Robot.XHd.base_X_world[:3,:3], 
																	self.Robot.Leg[j].XHd.world_base_X_AEP[:3,3:4])
					self.Robot.Leg[j].XHd.base_X_NRP[:3,3:4] = mX(self.Robot.XHd.base_X_world[:3,:3], 
																	self.Robot.Leg[j].XHd.world_base_X_NRP[:3,3:4])
					self.Robot.Leg[j].XHd.world_X_foot = mX(self.Robot.XHd.world_X_base, 
																self.Robot.Leg[j].XHd.base_X_AEP)
					# if (j==5):
					# 	print 'wbXn: ', np.round(self.Robot.Leg[j].XHd.world_base_X_NRP[:3,3],3)
					# 	print 'wbXa: ', np.round(self.Robot.Leg[j].XHd.world_base_X_AEP[:3,3],3)
					# 	print 'bXa:  ', np.round(self.Robot.Leg[j].XHd.base_X_AEP[:3,3],3)
					# 	print 'bXn:  ', np.round(self.Robot.Leg[j].XHd.base_X_NRP[:3,3],3)
					# 	print 'wXb:  ', np.round(self.Robot.XHd.world_X_base[:3,3],3)
					# 	# print mX(self.Robot.XHd.world_X_base, self.Robot.Leg[j].XHd.base_X_AEP)
					# 	print 'wXf:  ', np.round(self.Robot.Leg[j].XHd.world_X_foot[:3,3],3)

					## Get cell height in (x,y) location of world_X_foot
					cell_h = np.array([0., 0., self.GridMap.get_cell('height', self.Robot.Leg[j].XHd.world_X_foot[:3,3], j)])
					
					## Cell height above threshold gets ignored as this requires advanced motions
					if (abs(cell_h.item(2)) < 0.1):# and chim_trans is False and self.W_CHIM is False):
						self.Robot.Leg[j].XHd.world_X_foot[2,3] = cell_h.item(2) 	# set z-component to cell height

					elif (abs(cell_h.item(2)) > 0.1 and self.W_GND is True):
						print 'Search for next valid foothold for ', j, cell_h.item(2), np.round(self.Robot.Leg[j].XHd.world_base_X_NRP[:3,3],3)
						print np.round(self.Robot.XHd.world_X_base[:3,3],3)
						# self.Robot.Leg[j].XHd.world_X_foot[:3,3] = self.find_valid_foothold(self.Robot.Leg[j].XHd.world_X_foot[:3,3], j)
						self.Robot.Leg[j].XHd.world_X_foot[:3,3] = self.find_valid_foothold(self.Robot.Leg[j].XHd.world_base_X_NRP[:3,3] + 
																							self.Robot.XHd.world_X_base[:3,3], j)

					## Check if foothold valid for chimney transition
					elif (self.T_GND_X_CHIM is True):
						# height difference between desired foothold location and cell height
						# valid contact is when dh is (-ve)
						dh = self.Robot.Leg[j].XHd.world_X_foot[2,3] - cell_h.item(2)

						# recompute to ground
						if (dh > 0.001):
							KDL = kdl.KDL()
							self.Robot.Leg[j].XHd.update_base_X_NRP(KDL.leg_IK(leg_stance[j])) 
							self.Robot.Leg[j].XHd.update_world_base_X_NRP(P6d_world_X_base)
							self.Robot.Leg[j].XHd.world_X_NRP[:3,3] = np.round( (v3cp + self.Robot.Leg[j].XHd.world_base_X_NRP[:3,3:4]).flatten(),4)

							## Recompute AEP wrt base and world frame					
							self.Robot.Leg[j].XHd.world_base_X_AEP[:3,3] = self.Robot.Leg[j].XHd.world_base_X_NRP[:3,3] + \
																			np.nan_to_num(v3_uv*self.Robot.Gait.step_stroke/2.)
							self.Robot.Leg[j].XHd.base_X_AEP[:3,3:4] = mX(self.Robot.XHd.base_X_world[:3,:3], 
																			self.Robot.Leg[j].XHd.world_base_X_AEP[:3,3:4])
							self.Robot.Leg[j].XHd.base_X_NRP[:3,3:4] = mX(self.Robot.XHd.base_X_world[:3,:3], 
																			self.Robot.Leg[j].XHd.world_base_X_NRP[:3,3:4])
							self.Robot.Leg[j].XHd.world_X_foot = mX(self.Robot.world_X_base, self.Robot.Leg[j].XHd.base_X_AEP)

					elif (self.T_CHIM_X_GND):
						# height difference between desired foothold location and cell height
						# valid contact is when dh is (-ve)
						dh = self.Robot.Leg[j].XHd.world_X_foot[2,3] - cell_h.item(2)
						# recompute to wall
						if (dh > 0.001):
							# Set to wall contacts	
							qj = np.pi/2 if (j < 3) else -np.pi/2 	# generalisation rotation from base to leg frame
							bXfy = self.di_wall[1]*np.sin(qj)
							temp = np.array([[self.Robot.Leg[j].XHd.base_X_NRP[0,3]],[bXfy],[0.]])
							self.Robot.Leg[j].XHd.world_base_X_NRP[:3,3:4] = mX(rot_Z(v3wp[2]), temp)
							self.Robot.Leg[j].XHd.world_X_NRP[:3,3] = np.round( (v3cp + self.Robot.Leg[j].XHd.world_base_X_NRP[:3,3:4]).flatten(),4)
							
							## Recompute AEP wrt base and world frame					
							self.Robot.Leg[j].XHd.world_base_X_AEP[:3,3] = self.Robot.Leg[j].XHd.world_base_X_NRP[:3,3] + \
																			np.nan_to_num(v3_uv*self.Robot.Gait.step_stroke/2.)
							self.Robot.Leg[j].XHd.base_X_AEP[:3,3:4] = mX(self.Robot.XHd.base_X_world[:3,:3], 
																			self.Robot.Leg[j].XHd.world_base_X_AEP[:3,3:4])
							self.Robot.Leg[j].XHd.base_X_NRP[:3,3:4] = mX(self.Robot.XHd.base_X_world[:3,:3], 
																			self.Robot.Leg[j].XHd.world_base_X_NRP[:3,3:4])
							self.Robot.Leg[j].XHd.world_X_foot = mX(self.Robot.world_X_base, self.Robot.Leg[j].XHd.base_X_AEP)

					## Recompute base_X_AEP based on cell height
					self.Robot.Leg[j].XHd.base_X_AEP = mX(self.Robot.XHd.base_X_world, 
															v3_X_m(self.Robot.Leg[j].XHd.world_X_foot[:3,3]))

					## Stack to array
					world_X_footholds[j].t.append(i*CTR_INTV)
					world_X_footholds[j].xp.append(self.Robot.Leg[j].XHd.world_X_foot[:3,3:4].copy())
					
					world_base_X_NRP[j].t.append(i*CTR_INTV)
					world_base_X_NRP[j].xp.append(self.Robot.Leg[j].XHd.world_base_X_NRP[:3,3:4].copy())

					base_X_footholds[j].t.append(i*CTR_INTV)
					base_X_footholds[j].xp.append(self.Robot.Leg[j].XHd.base_X_AEP[:3,3:4].copy())
					
			## Alternate gait phase
			self.Robot.Gait.change_phase()
			# self.Robot.Gait.change_exceeded_phase(leg_exceed)
			v3cp_prev = v3cp.copy()
			v3wp_prev = v3wp.copy()
			
		print '============================================='

		# Update robot's current state
		self.Robot.P6c.world_X_base = P6d_world_X_base.copy()
		for j in range(0,6):
			self.Robot.Leg[j].XHc.world_X_foot[:3,3] = self.Robot.Leg[j].XHd.world_X_foot[:3,3].copy()
			self.Robot.Leg[j].XHc.base_X_foot[:3,3]  = self.Robot.Leg[j].XHd.base_X_foot[:3,3].copy()
			self.Robot.Leg[j].XHc.world_base_X_NRP[:3,3] = self.Robot.Leg[j].XHd.world_base_X_NRP[:3,3].copy()

		return set_motion_plan()

	def transition_routine(self, ttype, p1, p2, tn=0.1):

		Q_BASE = 20.
		H_BASE = 0.15

		## define variables ##
		xi = np.array([p1[0]*self.GridMap.resolution, p1[1]*self.GridMap.resolution, self.base_map.nodes[p1]['pose'][0]])
		xf = np.array([p2[0]*self.GridMap.resolution, p2[1]*self.GridMap.resolution, self.base_map.nodes[p2]['pose'][0]])
		qi = self.base_map.nodes[p1]['pose'][1]
		qf = self.base_map.nodes[p2]['pose'][1]
		
		x_cob = xi.copy()
		w_cob = np.array([qi, 0., 0.])

		PathGenerator = path_generator.PathGenerator()

		print 'xi: ', xi, xf, qi, qf

		if (ttype=='Gnd_X_Wall'):
			if (self.T_GND_X_WALL):
				print 'TR: g2w'
				qi = np.deg2rad(np.sign(qf)*Q_BASE)
				qf = self.base_map.nodes[p2]['pose'][1]

				xi = np.array([p1[0]*self.GridMap.resolution, p1[1]*self.GridMap.resolution, H_BASE])
				xf = np.array([p2[0]*self.GridMap.resolution, p2[1]*self.GridMap.resolution, self.base_map.nodes[p2]['pose'][0]])
				
				x_cob = xi.copy()
				w_cob = np.array([qi, 0., 0.])

			# Generate ellipsoidal base trajectory
			qrange = range(0,91,10)
			delta_q = (qf-qi)/(len(qrange)-1)
			delta_x = xf - xi
			
			for i in range(1, len(qrange)):
				qr = np.deg2rad(qrange[i])
				xd = xi + np.array([(1.-np.cos(qr))*delta_x[0],
									(1.-np.cos(qr))*delta_x[1], 
									(np.sin(qr))*delta_x[2]])

				x_cob = np.vstack(( x_cob, xd ))
				w_cob = np.vstack(( w_cob, np.array([qi+i*delta_q, 0., 0.]) ))
				
			base_path = PathGenerator.generate_base_path(x_cob, w_cob, tn)
			
		elif (ttype=='Wall_X_Gnd'):
			print 'TR: w2g ', p1, p2
			qi = self.base_map.nodes[p1]['pose'][1]
			qf = np.deg2rad(np.sign(qi)*Q_BASE)

			xi = np.array([p2[0]*self.GridMap.resolution, p2[1]*self.GridMap.resolution, H_BASE])
			xf = np.array([p1[0]*self.GridMap.resolution, p1[1]*self.GridMap.resolution, self.base_map.nodes[p1]['pose'][0]])

			x_cob = xi.copy()
			w_cob = np.array([qi, 0., 0.])

			# Generate ellipsoidal base trajectory
			qrange = range(0,91,10)
			delta_q = (qf-qi)/(len(qrange)-1)
			delta_x = xf - xi
			
			for i in range(1, len(qrange)):
				qr = np.deg2rad(qrange[i])
				xd = xi + np.array([(1.-np.cos(qr))*delta_x[0],
									(1.-np.cos(qr))*delta_x[1], 
									(np.sin(qr))*delta_x[2]])

				x_cob = np.vstack(( x_cob, xd ))
				w_cob = np.vstack(( w_cob, np.array([qi+i*delta_q, 0., 0.]) ))
				
			if (self.T_WALL_X_GND):
				x_cob = x_cob[::-1]

			base_path = PathGenerator.generate_base_path(x_cob, w_cob, tn)

		elif (ttype=='Gnd_X_Chim' or 'Chim_X_Gnd'):
			""" change from ground to chimney wall support footholds """
			
			print 'TR: g2c'
			x_cob = np.vstack(( x_cob, xf ))
			w_cob = np.vstack(( w_cob, np.array([qf, 0., 0.]) ))
			base_path = PathGenerator.generate_base_path(x_cob, w_cob, tn)
			# Plot.plot_2d(base_path.X.t, base_path.X.xp)
			# Plot.plot_2d(base_path.X.t, base_path.W.xp)
			
		return base_path

	def routine_motion(self, p1, p2, routine=None, tn=0.1):
		""" Routine for moving pose and leg to desired stances """

		Q_BASE = 20. 	# base angle to rotate for wall transition
		H_BASE = 0.15	# base height to move to
		
		PathGenerator = path_generator.PathGenerator()

		if (routine is not None):
			if (routine == 'reset_stance'):
				self.Robot.Gait.set_gait_type(5)
				self.Robot.Gait.cs = self.Robot.Gait.phases[0]

				xi = np.array([p1[0]*self.GridMap.resolution, p1[1]*self.GridMap.resolution, self.base_map.nodes[p1]['pose'][0]])
				qi = self.base_map.nodes[p1]['pose'][1]

				x_cob = xi.copy()
				w_cob = np.array([qi, 0., 0.])
				t_cob = np.zeros(1)

				n_phases = len(self.Robot.Gait.phases)
				for i in range(1, n_phases+1):
					x_cob = np.vstack(( x_cob, xi ))
					w_cob = np.vstack(( w_cob, np.array([qi, 0., 0.]) ))
					t_cob = np.hstack(( t_cob, i*self.Robot.Gait.tphase))

				# print 'tcob ', t_cob
				# print 'xcob ', x_cob
				PathGenerator.V_MAX = PathGenerator.W_MAX = 100
				base_path = PathGenerator.generate_base_path(x_cob, w_cob, CTR_INTV, t_cob)

				if (self.W_GND is False):
					leg_offset = (40., -40.)
				elif (self.W_GND is True):
					leg_offset = (0., 0.)

				stance = self.Robot.set_leg_stance(STANCE_WIDTH, BODY_HEIGHT, leg_offset, "flat")
				self.Robot.Gait.step_stroke = 0.

				for j in range(0,6):
					self.Robot.Leg[j].XHd.update_base_X_NRP(self.Robot.KDL.leg_IK(stance[j]))
					self.Robot.Leg[j].XHd.update_world_base_X_NRP(self.Robot.P6c.world_X_base)

		elif (self.T_GND_X_WALL):
			delta_w = 1 if (self.base_map.nodes[p2]['pose'][1] >= 0.) else -1

			# set gait parameters to wave gait
			self.Robot.Gait.set_gait_type(1)
			if (delta_w == 1):
				self.Robot.Gait.np = 0
				self.Robot.Gait.cs = self.Robot.Gait.phases[0]
			elif (delta_w == -1):
				self.Robot.Gait.np = 3
				self.Robot.Gait.cs = self.Robot.Gait.phases[3]
			# set stance parameters

			xi = np.array([p1[0]*self.GridMap.resolution, p1[1]*self.GridMap.resolution, self.base_map.nodes[p1]['pose'][0]])
			xf = np.array([p1[0]*self.GridMap.resolution, p1[1]*self.GridMap.resolution, H_BASE])
			
			qi = self.base_map.nodes[p1]['pose'][1]
			qf = delta_w*np.deg2rad(Q_BASE)
			
			qrange = range(0,int(Q_BASE),4)

			delta_q = (qf-qi)/(len(qrange)-1)
			delta_x = (xf-xi)/(len(qrange)-1)

			x_cob = xi.copy()
			w_cob = np.array([qi, 0., 0.])
			t_cob = np.zeros(1)
			
			total_time = self.Robot.Gait.tphase*6

			for i in range(1, len(qrange)):
				qr = np.deg2rad(qrange[i])
				xd = xi + delta_x*i
				wd = qi+i*delta_q

				x_cob = np.vstack(( x_cob, xd ))
				w_cob = np.vstack(( w_cob, np.array([wd, 0., 0.]) ))
				t_cob = np.hstack(( t_cob, i*total_time/(2*(len(qrange)-1)) ))
				
			for i in range(len(qrange),2*len(qrange)-1):
				x_cob = np.vstack(( x_cob, xd ))
				w_cob = np.vstack(( w_cob, np.array([wd, 0., 0.]) ))
				t_cob = np.hstack(( t_cob, i*total_time/(2*(len(qrange)-1)) ))
			# print x_cob
			# print t_cob
			PathGenerator.V_MAX = PathGenerator.W_MAX = 100
			base_path = PathGenerator.generate_base_path(x_cob, w_cob, CTR_INTV, t_cob)
			# Plot.plot_2d(base_path.X.t, base_path.X.xp)
			# Plot.plot_2d(base_path.W.t, base_path.W.xp)

		elif (self.T_WALL_X_GND):
			delta_w = 1 if (self.base_map.nodes[p1]['pose'][1] >= 0.) else -1

			# set gait parameters to wave gait
			self.Robot.Gait.set_gait_type(1)
			self.Robot.Gait.step_stroke = 0.
			if (delta_w == 1):
				self.Robot.Gait.np = 3
				self.Robot.Gait.cs = self.Robot.Gait.phases[3]
			elif (delta_w == -1):
				self.Robot.Gait.np = 0
				self.Robot.Gait.cs = self.Robot.Gait.phases[0]

			# set stance parameters
			xi = np.array([p2[0]*self.GridMap.resolution, p2[1]*self.GridMap.resolution, H_BASE])
			xf = np.array([p2[0]*self.GridMap.resolution, p2[1]*self.GridMap.resolution, self.base_map.nodes[p2]['pose'][0]])
			
			qi = delta_w*np.deg2rad(Q_BASE)
			qf = self.base_map.nodes[p2]['pose'][1]
			
			qrange = range(0,int(Q_BASE),4)

			delta_q = (qf-qi)/(len(qrange)-1)
			delta_x = (xf-xi)/(len(qrange)-1)

			x_cob = xi.copy()
			w_cob = np.array([qi, 0., 0.])
			t_cob = np.zeros(1)

			total_time = self.Robot.Gait.tphase*6
			nside = int(np.ceil(len(self.Robot.Gait.phases)/2))
			for i in range(1,nside+1):
				x_cob = np.vstack(( x_cob, xi ))
				w_cob = np.vstack(( w_cob, np.array([qi, 0., 0.]) ))
				t_cob = np.hstack(( t_cob, i*self.Robot.Gait.tphase ))
				# print 'in 1: ', xi, i*self.Robot.Gait.tphase, nside
			for i in range(1, len(qrange)):
				qr = np.deg2rad(qrange[i])
				xd = xi + delta_x*i
				wd = qi+i*delta_q

				x_cob = np.vstack(( x_cob, xd ))
				w_cob = np.vstack(( w_cob, np.array([wd, 0., 0.]) ))
				t_cob = np.hstack(( t_cob, nside*self.Robot.Gait.tphase + i*total_time/(2*(len(qrange)-1)) ))
			
			PathGenerator.V_MAX = PathGenerator.W_MAX = 100
			base_path = PathGenerator.generate_base_path(x_cob, w_cob, CTR_INTV, t_cob)
			# Plot.plot_2d(base_path.X.t, base_path.X.xp)
			# Plot.plot_2d(base_path.W.t, base_path.W.xp)

		elif (self.T_CHIM_X_GND):
			self.Robot.Gait.set_gait_type(1)
			self.Robot.Gait.cs = self.Robot.Gait.phases[0]

			xi = np.array([p1[0]*self.GridMap.resolution, p1[1]*self.GridMap.resolution, self.base_map.nodes[p1]['pose'][0]])
			qi = self.base_map.nodes[p1]['pose'][1]
			xf = np.array([p2[0]*self.GridMap.resolution, p2[1]*self.GridMap.resolution, self.base_map.nodes[p2]['pose'][0]])
			qf = self.base_map.nodes[p2]['pose'][1]

			x_cob = xi.copy()
			w_cob = np.array([qi, 0., 0.])
			t_cob = np.zeros(1)

			n_phases = len(self.Robot.Gait.phases)
			delta_q = (qf-qi)/n_phases
			delta_x = (xf-xi)/n_phases

			for i in range(1, n_phases+1):
				x_cob = np.vstack(( x_cob, xi + delta_x*i ))
				w_cob = np.vstack(( w_cob, np.array([delta_q*i, 0., 0.]) ))
				t_cob = np.hstack(( t_cob, i*self.Robot.Gait.tphase))

			PathGenerator.V_MAX = PathGenerator.W_MAX = 100
			base_path = PathGenerator.generate_base_path(x_cob, w_cob, CTR_INTV, t_cob)

		return base_path

	def post_process_path(self,path, qi):
		""" Identifies motion primitive from path, introduce transition 
			routines, and call foothold for the respective sections 	"""
		""" Input: 	1) path -> path in grid format
					2) Robot -> Robot class
			Output: MotionPlan() type 									"""
		
		def ts_gw_segmentisation(temp_path):
			""" segmentise the path to be linear """

			if (path[i][0] - sp[0] > 0):
				ep = (sp[0],path[i-1][1])	# set new end point
				# modify path to start from end of segmentised transition
				temp_path = map(lambda x: (x[0],path[i][1]) , temp_path)
				temp_path.insert(0,ep)
				# remove duplicates
				temp_path = list(OrderedDict.fromkeys(temp_path))
				print 'segmentised forward linear path: ', temp_path
			else:
				ep = path[i]
				temp_path = []
			return temp_path, ep

		def ts_gnd_X_wall():
			""" Full routine for  ground to wall transition """

			print 'TRANSITION: gnd_X_wall - ', sp, ep
			print 'Transition pt. 1: reset stance'
			self.T_GND_X_WALL = self.W_WALL = False
			path_01 = self.routine_motion(sp,ep,'reset_stance')
			motion_plan.append(self.foothold_planner(path_01))
			self.T_GND_X_WALL = self.W_WALL = True

			print 'Transition pt. 2: change foothold surface', self.W_GND
			path_01 = self.routine_motion(sp,ep)
			motion_plan.append(self.foothold_planner(path_01))
			
			print 'Transition pt. 3: moving to setpoint'
			path_01 = self.transition_routine('Gnd_X_Wall', sp, ep)
			motion_plan.append(self.foothold_planner(path_01))
			
			self.T_GND_X_WALL = self.W_GND = False

		def ts_wall_X_gnd():
			""" Full routine for wall to ground transition """

			print 'TRANSITION: wall_X_gnd - ', sp, ep
			print 'Transition pt. 1: moving to setpoint'
			path_01 = self.transition_routine('Wall_X_Gnd', sp, ep) #Wall_X_Gnd
			motion_plan.append(self.foothold_planner(path_01))
			
			print 'Transition pt. 2: change foothold surface', self.W_GND
			path_01 = self.routine_motion(sp,ep)
			motion_plan.append(self.foothold_planner(path_01))
			self.W_WALL = self.T_WALL_X_GND = False
			self.W_GND  = True
			
			print 'Transition pt. 3: reset stance'
			path_01 = self.routine_motion(ep,ep,'reset_stance')
			motion_plan.append(self.foothold_planner(path_01))
			self.Robot.Gait.set_gait_type(4)

			self.W_WALL = self.T_WALL_X_GND = False
			self.W_GND  = True

		def ts_gnd_X_chim():
			""" Full routine for ground to chimney transition """

			print 'G2C Transition at ', sp
			print 'Transition: Reset Leg Stances'
			path_01 = self.routine_motion(sp, sp,'reset_stance')
			motion_plan.append(self.foothold_planner(path_01))
			self.T_GND_X_CHIM = self.W_CHIM = True
			self.Robot.Gait.set_gait_type(4)

		def ts_chim_X_gnd():
			""" Full routine for chimney to ground transition """

			print 'C2G Transition at ', sp, path[i+1]
			print 'Transition: Change foothold surface'
			path_01 = self.routine_motion(sp,ep)
			motion_plan.append(self.foothold_planner(path_01))
			self.T_CHIM_X_GND = self.W_CHIM = False
			self.W_GND = True
			
			print 'Transition: Reset Leg Stances'
			path_01 = self.routine_motion(ep, ep,'reset_stance')
			motion_plan.append(self.foothold_planner(path_01))
			self.Robot.Gait.set_gait_type(4)

		def generate_plan(path_int):
			""" convenience for generating and interpolating path """

			return self.foothold_planner(self.path_interpolation(path_int))

		print 'Post-processing path......'
		
		## Define Variables ##
		start_pos = path[0]
		temp_path = []
		base_path = Trajectory6D()
		motion_plan = MotionPlan()
		m = np.zeros(3) 	# motion primitive for next 3 steps

		## cycle through path
		for i in range(0,len(path)):
			print 'now: ', path[i], self.base_map.nodes[path[i]]['motion']
			temp_path.append(path[i])
			## check transition
			m0 = self.base_map.nodes[path[i]]['motion']
			try:
				m1 = self.base_map.nodes[path[i+1]]['motion']
				m2 = self.base_map.nodes[path[i+2]]['motion']
			except:
				pass

			if (self.T_GND_X_WALL or self.T_WALL_X_GND):
				qn1 = self.base_map.nodes[path[i]]['pose'][1]
				qn0 = self.base_map.nodes[path[i-1]]['pose'][1]
				print path[i], path[i-1], qn1, qn0
				# Body roll stabilises, so interpolate
				if (abs(qn1-qn0) < 0.05):
					temp_path, ep = ts_gw_segmentisation(temp_path)
					
					if (self.T_GND_X_WALL):
						ts_gnd_X_wall()
					elif (self.T_WALL_X_GND):
						ts_wall_X_gnd()
						
			## ============================================================================ ##

			## Check transition instances
			if (m0==0 and m1==2):
				## Walk to Wall ##
				# First, plan path and foothold for ground walking
				if (len(temp_path) > 1):
					print 'Interpolating Wg prior to Ww...', temp_path[0], temp_path[-1]
					motion_plan.append(generate_plan(temp_path))
					temp_path = []

				sp = path[i]
				# Next, set the respective state machine flags
				self.T_GND_X_WALL = self.W_WALL = True
				self.W_GND = False

			elif (m0==2 and m1==0):
				## Wall to Walk ##
				if (m2==2):
					# force motion to walling for gap of one
					self.base_map.nodes[path[i+1]]['motion'] = 1
					self.GM_chim.add_node(path[i+1])
					temp_path.append(path[i])
				else:
					# First, plan path and foothold for ground walking
					if (len(temp_path) > 1):
						print 'Interpolating Ww prior to Wg... ', temp_path
						# Append the transitioned point
						temp_path.append(path[i+1])
						# Segmentise temp_path
						for n in temp_path:
							temp_path = map(lambda x: (x[0],temp_path[0][1]) , temp_path)
						temp_path = list(OrderedDict.fromkeys(temp_path))
						print 'new temp path ', temp_path
						motion_plan.append(generate_plan(temp_path))

						sp = temp_path[-1]
						temp_path = []

						# Next, set the respective state machine flags
						self.T_WALL_X_GND = True

			elif (m0==0 and m1==1):
				## Walk to Chimney ##
				# First, plan path and foothold for ground walking
				if (len(temp_path) > 1):
					print 'Interpolating Wg prior to Wc...', temp_path
					motion_plan.append(generate_plan(temp_path))
					temp_path = []

				sp = path[i]
				self.di_wall = np.array([0., self.base_map.nodes[path[i+1]]['width']*self.GridMap.resolution/2, 0.])
				# Next, set the respective state machine flags
				self.T_GND_X_CHIM = self.W_CHIM = self.W_GND = False

				ts_gnd_X_chim()

			elif (m0==1 and m1==0):
				## Chimney to Walk ##
				if (m2==1):
					# force motion to chimney for gap of one
					self.base_map.nodes[path[i+1]]['motion'] = 1
					self.GM_chim.add_node(path[i+1])
					temp_path.append(path[i])
				else:
					# First, plan path and foothold for ground walking
					if (len(temp_path) > 1):

						print 'Interpolating Wc prior to Wg... ',
						# Segmentise temp_path
						for n in temp_path:
							temp_path = map(lambda x: (x[0],temp_path[0][1]) , temp_path)
						temp_path = list(OrderedDict.fromkeys(temp_path))
						motion_plan.append(generate_plan(temp_path))
						
						# set respective flags
						self.T_GND_X_CHIM = False
						self.T_CHIM_X_GND = self.W_CHIM = True

						sp = temp_path[-1]
						ep = path[i+1]
						temp_path = []
						
						ts_chim_X_gnd()

		# Interpolate final sub-division if not empty
		print 'Cycle exited: ', temp_path
		
		# Check if transition required
		if (self.T_GND_X_WALL):
			print 'Final interpolation, Wg-w: ', sp, 'to ', path[-1]
			temp_path, ep = ts_gw_segmentisation(temp_path)
			ts_gnd_X_wall()

		elif (self.T_WALL_X_GND):
		 	print 'Final interpolation, Ww-g: ', sp, 'to ', path[-1]
		 	temp_path, ep = ts_gw_segmentisation(temp_path)
			ts_wall_X_gnd()

		elif (self.T_GND_X_CHIM):
			print 'Final G2C Transition from ', sp, 'to ', temp_path[0]
			ts_gnd_X_chim()

		elif (self.T_CHIM_X_GND):
			print 'Final G2C Transition from ', sp, 'to ', temp_path[-1]
			ts_chim_X_gnd()

		# reset all transition flags
		self.T_GND_X_WALL = self.T_WALL_X_GND = False
		self.T_GND_X_CHIM = self.T_CHIM_X_GND = False

		if (len(temp_path) > 1):
			print 'Final walking interpolation: ', temp_path
			motion_plan.append(generate_plan(temp_path))
				
		print 'Path completed'

		## Update spline time interval
		tspan = len(motion_plan.qb.X.t)
		
		for i in range(0,tspan):
			motion_plan.qb.X.t[i] = np.round(i*CTR_INTV,4)
			motion_plan.qb.W.t[i] = np.round(i*CTR_INTV,4)
		# Plot.plot_2d_multiple(1,motion_plan.qb.X.t,motion_plan.qb.X.xp)
		
		## Transform path from global to local frame
		for i in range(1,tspan):
			motion_plan.qb.X.xp[i] = motion_plan.qb.X.xp[i] - motion_plan.qb.X.xp[0]	
		motion_plan.qb.X.xp[0] = motion_plan.qb.X.xp[0] - motion_plan.qb.X.xp[0]
		
		# print motion_plan.qbp
		# write to csv file
		# with open('trajectory.csv', 'wb') as csvfile:
		# 	csvwriter = csv.writer(csvfile, delimiter=',')#, quotechar='|', quoting=csv.QUOTE_MINIMAL)
		# 	for i in range(0,len(x_cob.t)):
		# 		data = np.hstack((x_cob.t[i],x_cob.xp[i],x_cob.xv[i],x_cob.xa[i],w_cob.t[i],w_cob.xp[i],w_cob.xv[i],w_cob.xa[i]))
		# 		csvwriter.writerow(data)
		
		## Regenerate base path
		x_cob = np.zeros(3)
		w_cob = qi[3:6].flatten()

		t_cob = np.zeros(1)
		for i in range(0,len(motion_plan.qbp)):
			x_cob_local = motion_plan.qbp[i][:3] - qi[:3]#motion_plan.qbp[0][:3]
			w_cob_local = motion_plan.qbp[i][3:6]

			x_cob = np.vstack((x_cob, x_cob_local))
			w_cob = np.vstack((w_cob, w_cob_local))
			t_cob = np.hstack((t_cob, (i+1)*GAIT_TPHASE))
		
		path_generator = Pathgenerator.PathGenerator()
		path_generator.V_MAX = path_generator.W_MAX = 10
		path = path_generator.generate_base_path(x_cob, w_cob, CTR_INTV, t_cob)
		
		sum_footholds = 0
		for j in range(0,6):
			sum_footholds += len(motion_plan.f_world_X_foot[j].xp)
		print 'no. footholds: ', sum_footholds
		print 'time: ', motion_plan.qb.X.t[-1]
		
		# print len(path.X.t)
		# Plot.plot_2d(path.X.t,path.X.xp)
		# Plot.plot_2d(path.W.t,path.W.xp)
		# fig, ax = plt.subplots()
		# ax.plot(motion_plan.qb.X.t, motion_plan.qb.X.xp[:,0], 'ro');
		# ax.plot(path.X.xp, label='x');
		# ax.plot(path.W.xp, label='w');
		# ax.plot(path.X.t, path.X.xp[:,0], label='acc');
		# plt.grid('on');
		# plt.show()
		
		motion_plan.qb = path
		return motion_plan

	def generate_motion_plan(self, Robot, **options):
		""" Generate motion plan for the robot """

		# Duplicate robot class instant
		self.Robot.duplicate_self(Robot)
		# print 'before ', Robot.P6c.world_X_base.flatten()
		# Generate path given start and end location on map
		if (options.get("start")):
			start = options.get("start")
			end   = options.get("end")

			list_gpath = self.find_base_path(start, end)

			if (list_gpath is not None):
				print 'Path Found: ', list_gpath
				# raw_input('continue')
				
				## High level preview
				# gpath = nx.path_graph(list_gpath)
				# self.GridMap.graph_representation(gpath)
				
				## Convert path from cell index to Re^3 format 
				x_cob, w_cob = self.GridMap.get_path_to_nparray(list_gpath, start)

			else:
				# No path exist, exit
				return None

		# Generate path given set of via points in Re^3
		elif (options.get("path")):
			x_cob = options.get("path")[0]
			w_cob = options.get("path")[1]

		else:
			print 'Invalid motion options selected!'
			return None

		# Interpolate spline
		PathGenerator = Pathgenerator.PathGenerator() 	# path generator for robot's base
		
		# Set all legs to support mode for bodyposing, prevent AEP from being set
		if (Robot.support_mode == True):
			Robot.Gait.support_mode()
			self.Robot.Gait.support_mode()
			PathGenerator.V_MAX = PathGenerator.W_MAX = 0.4
		else:
			Robot.Gait.walk_mode()
			self.Robot.Gait.walk_mode()
		## Rough Trajectory for robot's base
		# base_path = PathGenerator.generate_base_path(x_cob, w_cob, CTR_INTV)
		# Plot.plot_2d(base_path.X.t, base_path.X.xp)
		# Plot.plot_2d(base_path.W.t, base_path.W.xp)

		## Split path according to motion primitive
		return self.post_process_path(list_gpath, Robot.P6c.world_X_base.flatten())

	def path_interpolation(self, path, tn=0.1):
		""" interpolate cells using cubic spline """

		## Define Variables ##
		x_cob = np.array([.0,.0,.0])
		w_cob = np.array([.0,.0,.0])
		
		## populate array
		for e in path:
			x_cob = np.vstack((x_cob,np.array([e[0]*self.GridMap.resolution, e[1]*self.GridMap.resolution, self.base_map.nodes[e]['pose'][0]])))
			w_cob = np.vstack((w_cob,np.array([self.base_map.nodes[e]['pose'][1:4]])))
			# print e, (np.round(e[0]*self.GridMap.resolution,3), np.round(e[1]*self.GridMap.resolution,3))
		
		x_cob = np.delete(x_cob,0,0)
		w_cob = np.delete(w_cob,0,0)
		# print x_cob
		# print w_cob
		PathGenerator = path_generator.PathGenerator()
		base_path = PathGenerator.generate_base_path(x_cob, w_cob, tn)

		# Plot.plot_2d_multiple(1,wn_com.t,wn_com.xp*180/np.pi)
		# Plot.plot_2d_multiple(1,base_path.X.t,base_path.X.xp)#, base_path.W.xv)
		
		return base_path


## ================================================================================================ ##
## 												TESTING 											##
## ================================================================================================ ##

## Create map and plan path
# grid_map = GridMap('hole_demo')
# planner = PathPlanner(grid_map)

