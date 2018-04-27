#!/usr/bin/env python

""" Grid based motion planning using NetworkX
""" 

import sys; sys.dont_write_bytecode = True
sys.path.insert(0, '/home/wilson/catkin_ws/src/corin/corin_control/py_script')
from library import *			# library modules to include 

import networkx as nx
import matplotlib.pyplot as plt

import numpy as np
import math
from fractions import Fraction
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

# XH = transforms.HomogeneousTransform()

p_leg = [p_base_X_lf_foot,p_base_X_lm_foot,p_base_X_lr_foot,p_base_X_rf_foot,p_base_X_rm_foot,p_base_X_rr_foot]

def cw_spiral_left(sp,grid_area):
	""" spiral grid: rotates in clockwise direction starting with left, up, right then down """
	end = 8
	point_list = []
	end = grid_area[0]+grid_area[0]-2

	for i in range(1,end):
		if (i%2):
			# move left
			for x in range(0,i):
	            # skip last cell - ends in square shape
				if (end-i==1 and i-x==1):
					pass
				else:
					sp = (sp[0]-1,sp[1])
					point_list.append(sp)
			# move up
			if (i!=end-1):
				for x in range(0,i):
					sp = (sp[0],sp[1]+1)
					point_list.append(sp)
		else:
			# move right
			for x in range(0,i):
	            # skip last cell - ends in square shape
				if (end-i==1 and i-x==1):
					pass
				else:
					sp = (sp[0]+1,sp[1])
					point_list.append(sp)
			# move down
			if (i!=end-1):
				for x in range(0,i):
					sp = (sp[0],sp[1]-1)
					point_list.append(sp)
	return point_list

class BodyposeTable:
	def __init__(self):
		self.wall = self.compute_wall_table()
		# self.chim = self.compute_chim_table()
		self.table = self._generate_pose_table()

	def _generate_pose_table(self):

		## Define Variables ##
		POSE_TABLE = {} # dictionary of poses

		MAX_FP = 0.72 # maximum footprint width
		MIN_FP = 0.20 # minimum footprint width
		MAX_WP = 0.62 # maximum wall walking footprint

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
				# print j, np.round(base_stance,4)#, np.round(base_X_femur,4)

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

			# print qr, fp_h
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
				px = 0#TRN_BASE_X_LEG[j][0]
				py = sy*np.sin(np.deg2rad(ROT_BASE_X_LEG[j]+LEG_OFFSET[j])) 		# y_base_X_foot - world frame
				gnd_world_base_X_NRP[j] = np.array([px, py, -world_ground_X_base_z])	
				# if (j == 0):
				# 	print np.round(qr,3), np.round(gnd_world_base_X_NRP[j],4)
			
			# Create temp array, which is the new base to foot position, wrt world frame
			# The x-component uses the base frame NRP so that it remains the same
			# temp = np.array([[Leg[j].base_X_NRP[0,3]],[py],[Leg[j].world_base_X_NRP[2,3]]])
			# Leg[j].world_base_X_NRP[:3,3:4] = mX(rot_Z(v3wp[2]), temp)

			## *********************  Wall Footholds  ********************* ##
			bXfy = (MAX_WP - i*delta_fp)/2 		# world horizontal distance base to foot
			
			dy = bXfy - world_base_X_femur_y 	# horizontal distance from femur joint to foot
			dh = np.sqrt(L3**2 - dy**2) 		# vertical distance from femur joint to foot

			bXfz = world_base_X_femur_z + L2 + dh # vertical distance from base to foot
			# bXf  = np.array([0., bXfy, bXfz])
			
			for j in range(0,6):
				# py = bXfy*np.sin(np.deg2rad(ROT_BASE_X_LEG[j]+LEG_OFFSET[j])) 	# y_base_X_foot - world frame
				sside = 1 if (j < 3) else -1
				wall_world_base_X_NRP[j] = np.array([0., sside*bXfy, bXfz])
				# if (j==1): 
				# 	print wall_world_base_X_NRP[1]

			POSE_TABLE[fs+i] = {"footprint":np.array([0.27, fp_w, fp_h]), 
								"bodypose":np.array([0.,0., fp_h, np.round(qr,3),0.,0.]),
								"ground":gnd_world_base_X_NRP,
								"wall": wall_world_base_X_NRP }
		# for i in POSE_TABLE:
		# 	print POSE_TABLE[i]["bodypose"][2], '\t', POSE_TABLE[i]["bodypose"][3], POSE_TABLE[i]["footprint"][1]
		return POSE_TABLE

	def compute_wall_table(self):
		""" computes bodypose lookup table for footprint sizing """

		## Define Variables ##
		dhi = 0.12	# initial wall contact height (from coxa wrt world frame)
		dhf = 0.18 	# final wall contact height (from coxa wrt world frame)
		bh  = 0.1	# base height (m)
		bhm = 0.42	# max. base height
		qr  = 0.0	# base roll (rad)
		ns  = 20 	# samping size of table - above this resolution no further observable change 
		POSE_TABLE = {}

		for i in range(0,ns+1):
			
			qr = i*np.pi/(2*ns) 		# update base roll
			bn = bh + i*(bhm-bh)/ns 	# update base height
			# Transform from world to leg frame (both located at leg frame)
			world_X_base = np.array([ [ np.cos(qr),  -np.sin(qr)],[ np.sin(qr),  np.cos(qr)] ])

			world_X_leg = rot_X(-qr) #np.array([ [np.cos(-qr), -np.sin(-qr)],[np.sin(-qr), np.cos(-qr)] ])
			leg_X_world = rot_X(qr) # np.array([ [ np.cos(qr),  -np.sin(qr)],[ np.sin(qr),  np.cos(qr)] ])

			# leg on ground contact - distances in world frame
			gch = bn - (COXA_Y+L1)*np.sin(qr)			# height of femur joint from ground
			gcx = np.sqrt(abs((L2**2 - (gch - L3)**2)))	# horizontal distance of femur joint from foot
			gpW = np.array([0.,gcx,gch])				# foot position from coxa wrt world frame

			gpL = mX(leg_X_world,gpW)				# foot position wrt leg frame
			
			# leg on wall contact - distances in world frame
			wch = i*(dhf-dhi)/ns + dhi
			wcx = (ns-i)*(STANCE_WIDTH-0.0)/ns + 0.1 	# horizontal distance of femur joint from foot
			wpW = np.array([0., wcx, wch])				# foot position from coxa wrt world frame
			
			wpL = mX(leg_X_world,wpW)				# foot position wrt leg frame
			
			# foot position wrt world frame
			world_X_base = rot_X(qr)
			wall_wXfoot  = np.array([0.,0.,bn]) + mX(world_X_base, np.array([0.,COXA_Y,0.])) + wpW
			
			## robot footprint
			wR = ((COXA_Y+L1)*np.cos(qr)+gcx) + (COXA_Y*np.cos(qr)+wcx)
			# print'table: ',  qr, wR
			## write to dictionary
			POSE_TABLE[i] = {"footprint":np.array([0.,wR,0.]), 
								"bodypose":np.array([0.,0.,bn, qr,0.,0.]),
								"ground":np.array([0.,gcx,-gch]),
								"wall": wall_wXfoot }
			
		return POSE_TABLE

	def search_wall_table(self, **options):
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
		
		for i in self.wall:
			if options.get('width'):
				if (self.wall[i][dic_arg][val_arr]<=val_arg):
					selection = i
					break
				else:
					selection = i
						
			elif options.get('angle'):
				if (self.wall[i][dic_arg][val_arr]>=val_arg):
					selection = i
					break
				else:
					selection = i

		return selection

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
# i1, i2 = tb.search_wall_table(width=0.4)

class GridPlanner:
	def __init__(self,smap):
		self.resolution	= 0.03			# size of cell, (m) - TARGET: 0.03
		# self.leg_size  	= 0.18		# area of foothold per leg, (m^2) 	- TAKEN FROM CONSTANT*
		self.map_size  = (0,0) 			# select or set map size
		self.rbfp_size = (0.488, 0.5) 	# robot footprint size, (m,m) - TARGET: (0.488,0.58)
		self.rbbd_size = (0.27, 0.18) 	# robot body size, (m,m) - 
		self.PoseTable = BodyposeTable()
		self.advance_capable = True

		self.__initialise_graph__(smap)

		## TEMP
		self.T_GND_X_WALL = False
		self.T_GND_X_CHIM = False
		self.T_WALL_X_GND = False
		self.T_CHIM_X_GND = False
		self.W_GND 	= False
		self.W_WALL = False
		self.W_CHIM = False

	def __map_select__(self,smap):
		""" Set to pre-defined map """
		
		if (isinstance(smap,str)):
			if (smap=='simple'):
				return (1.02,1.52)
			elif (smap == 'iros'):
				return (1.27,5.48)
			elif (smap == 'rolling'):
				return (0.3,0.3)
			elif (smap == 'wall_demo'):
				return (0.75,0.9)
		elif (isinstance(smap,tuple) and len(smap) == 2):
			return smap
		else:
			print 'Invalid Map Selected/Defined....'

	def __initialise_graph__(self, smap):

		# Set map size
		self.map_size = self.__map_select__(smap)
		
		# determine robot related size in grid cell
		self.rbfp_gd = tuple(map(lambda x: int(math.ceil(x/self.resolution)), self.rbfp_size))					# footprint
		self.rbbd_gd = tuple(map(lambda x: int(math.ceil(x/self.resolution)), self.rbbd_size))					# body
		self.rblg_gd = (int(math.ceil(LEG_AREA_LX/self.resolution)),int(math.ceil(LEG_AREA_LY/self.resolution)))	# leg

		# determine number of grids
		gridx = int(math.ceil(self.map_size[0]/self.resolution))
		gridy = int(math.ceil(self.map_size[1]/self.resolution))

		# initiate graph instances
		self.G  	= nx.grid_graph(dim=[gridy, gridx]) 	# world grid map
		self.Gbody  = nx.grid_graph(dim=[gridy, gridx]) 	# truncated body map
		self.Gpath 	= nx.Graph() 							# final path
		self.Gfree  = nx.Graph() 							# illustration - body map with zero cost
		self.G_wall = nx.Graph()							# illustration - cell with wall
		self.G_hole = nx.Graph()							# illustration - cell with holes
		self.mp_sz 	= (gridx,gridy)							# map size in number of grid cells

		# footholds graph
		self.G_LF = nx.Graph()
		self.G_LM = nx.Graph()
		self.G_LR = nx.Graph()
		self.G_RF = nx.Graph()
		self.G_RM = nx.Graph()
		self.G_RR = nx.Graph()

		# graph for motion primitives used
		self.GM_walk = nx.Graph()
		self.GM_wall = nx.Graph()
		self.GM_chim = nx.Graph()

		# add moore edges
		self._moore_edge(self.G)
		self._moore_edge(self.Gbody)

		# set attribute for nodes
		nx.set_node_attributes(self.G, {e: 0 for e in self.G.nodes()}, 'cost') 	# obstacle identifier: 0 = ground, 1 = wall, -1 = ground
		nx.set_node_attributes(self.G, {e: np.array([0.,0.,1.]) for e in self.G.nodes()}, 'norm') 	# surface normal

		nx.set_node_attributes(self.Gbody, {e: 0 for e in self.Gbody.nodes()}, 'cost')		#
		nx.set_node_attributes(self.Gbody, {e: 0 for e in self.Gbody.nodes()}, 'motion') 	# motion identifier: 0 = walk,1 = chimney,2 = wall
		nx.set_node_attributes(self.Gbody, {e: 0 for e in self.Gbody.nodes()}, 'width') 	# footprint lateral width
		nx.set_node_attributes(self.Gbody, {e: [0.,0.,0.,0.] for e in self.Gbody.nodes()}, 'pose')	# robot bodypose - height, roll, pitch, yaw

		self.__set_obstacle__(smap)
		self.__set_surface_normal__()
		self.edge_removal(self.G)

		print 'Initialised - '
		print 'Map Grid  : ', gridx, ' by ', gridy
		print 'Robot grid: ', self.rbfp_gd
		print 'Body grid : ', self.rbbd_gd
		print 'Foot grid : ', self.rblg_gd
		print 'Resolution: ', self.resolution, 'm^2'
		print '==============================================='

	def __set_obstacle__(self, smap):

		if (isinstance(smap,str)):
			if (smap=='empty'):
				pass
			elif (smap=='simple'):
				self._set_map02()
			elif (smap == 'iros'):
				self._set_map01()
			elif (smap == 'rolling'):
				pass
			elif (smap == 'wall_demo'):
				self._set_wall_demo()

	def _moore_edge(self, G):
		""" generates moore edges for graph """

		aedge = []
		# cycle through all nodes
		for e in G.nodes():
			# diagonal right up
			try:
				du_node = (e[0]+1,e[1]+1)
				G.nodes[du_node] 			# test if node exist
				aedge.append([e,du_node]) 	# append to list if exists
			except:
				pass

			# diagonal right down
			try:
				dd_node = (e[0]+1,e[1]-1)
				G.nodes[dd_node] 			# test if node exist
				aedge.append([e,dd_node])	# append to list if exist
			except:
				pass

		G.add_edges_from(aedge) 	# add edges to map

	def _obstacle_area(self, box, obstacle):
		""" sets the area to be obstacles (wall or hole) """

		dx = box[1][0] - box[0][0]
		dy = box[1][1] - box[0][1]

		if (obstacle=='wall'): 
			cost = 1;
			norm = np.array([0.,0.,1.])

		elif (obstacle=='hole'):
			cost = -1
			norm = np.array([0.,0.,1.])

		for x in range(0,dx+1):
			for y in range(0,dy+1):
				p = ( (box[0][0]+x), (box[0][1]+y))
				self.G.nodes[p]['cost'] = cost

				if (obstacle == 'wall'):
					if (x == 0):
						snorm = np.array([0.,1.,0.])
					elif (x == dx):
						snorm = np.array([0.,-1.,0.])
					if (y == 0):
						snorm = np.array([-1.,0.,0.])
					elif (y == dy):
						snorm = np.array([1.,0.,0.])

				self.G.nodes[p]['norm'] = norm

	def __set_surface_normal__(self):
		""" Sets the surface normal for cells """

		for e in self.G.nodes():
			# Normals for wall
			if (self.G.nodes[e]['cost'] > 0.):
				""" check in a circle starting from front if its wall, 
					side without wall is surface normal direction 		"""
				try:
					if (self.G.nodes[tuple(map(sum, zip(e, (1,0))))]['cost'] <= 0.):
						self.G.nodes[e]['norm'] = np.array([1.,0.,0.])
				except:
					pass
				try:
					if (self.G.nodes[tuple(map(sum, zip(e, (0,1))))]['cost'] <= 0.):
						self.G.nodes[e]['norm'] = np.array([0.,1.,0.])
				except:
					pass
				try:	
					if (self.G.nodes[tuple(map(sum, zip(e, (-1,0))))]['cost'] <= 0.):
						self.G.nodes[e]['norm'] = np.array([-1.,0.,0.])
				except:
					pass	
				try:	
					if (self.G.nodes[tuple(map(sum, zip(e, (0,-1))))]['cost'] <= 0.):
						self.G.nodes[e]['norm'] = np.array([0.,-1.,0.])
				except:
					pass	
				
			# Normals for ground
			elif (self.G.nodes[e]['cost'] == 0.):
				self.G.nodes[e]['norm'] = np.array([0.,0.,1.])
				
			# Normal for holes
			elif (self.G.nodes[e]['cost'] < 0.):
				self.G.nodes[e]['norm'] = np.array([0.,0.,1.])


	def wall_length(self, p):
		""" determine longest wall on either side given a point p """

		lw_dist = 0		# distance to left wall
		rw_dist = 0		# distance to right wall
		lw_len	= 0		# length of left wall
		rw_len	= 0		# length of right wall

		n_left 	= (0,0)	# left node where wall occurs
		n_right	= (0,0)	# right node where wall occurs
		scan = False
		# search left
		while (not scan):
			try:
				n_left = (p[0]-lw_dist,p[1])
				if (self.G.nodes[n_left]['cost'] == 1):
					# wall found, set distance to wall & exit
					scan 	= True
				else:
					# continues search to the left
					lw_dist += 1
			except:
				# none found, break
				lw_dist = -1
				scan 	= True

		# search first left wall instance
		scan = False
		while (not scan and lw_dist>0):
			try:
				map_item = self.G.nodes[(n_left[0],n_left[1]-lw_len)]['cost']
				if ( map_item == 0 or map_item == -1):
					# break found
					scan 	= True
				else:
					# continues search downwards
					lw_len += 1
			except:
				scan = True

		scan = False
		# search right
		while (not scan):
			try:
				n_right = (p[0]+rw_dist,p[1])
				if (self.G.nodes[n_right]['cost'] == 1):
					# wall found, set distance to wall & exit
					scan = True
				else:
					# continues search to the left
					rw_dist += 1
			except:
				# none found, break
				rw_dist = -1
				scan 	= True

		# search first left wall instance
		scan = False
		while (not scan and rw_dist>0):
			try:
				map_item = self.G.nodes[(n_right[0],n_right[1]-rw_len)]['cost']
				if ( map_item == 0 or map_item == -1):
					# break found
					scan 	= True
				else:
					# continues search downwards
					rw_len += 1
			except:
				scan = True

		return (lw_dist,lw_len), (rw_dist, rw_len)
 
	def _set_map01(self):
		""" Set obstacle area and cost - map for IROS submission """

		## Obstacle boxes - bottom left and upper right [TODO:set start point, size in (m)]
		#  Chimney demonstration
		wall_01 = [ (0,0),(1,140)]
		wall_02 = [(23,20),(25,160)] 	# walling: [(37,25),(43,50)] 	chimney: [(40,25),(43,50)]

		wall_03 = [(2,60),(16,80)]
		wall_04 = [(2,100),(16,120)]

		wall_05 = [(17,155),(25,164)]

		hole_01 = [(2,25),(22,42)] 	# [(40,80),(60,110)] 	#
		hole_02 = [(11,85),(29,105)]

		#  Wall walking demonstration
		wall_bar = [(21,30),(36,30)]

		self._obstacle_area(wall_01, 'wall')
		self._obstacle_area(wall_02, 'wall')
		self._obstacle_area(wall_03, 'wall')
		self._obstacle_area(wall_04, 'wall')
		self._obstacle_area(wall_05, 'wall')

		self._obstacle_area(hole_01, 'hole')

		## Create obstacle path based on above - for illustration
		i_wall = []
		i_hole = []
		for e in self.G.nodes():
			if (self.G.nodes[e]['cost'] == 1):
				i_wall.append(e)
			elif (self.G.nodes[e]['cost'] == -1):
				i_hole.append(e)

		self.G_wall = nx.path_graph(i_wall) 	
		self.G_hole = nx.path_graph(i_hole)

	def _set_map02(self):
		""" Simple map with walls on both sides and hole in front
			Wall distance from world centre: 0.32 m
			Hole distance from world centre: 1.00 m  """

		self._obstacle_area([ (4,0),( 6,33)], 'wall')
		self._obstacle_area([(28,0),(30,33)], 'wall')
		self._obstacle_area([(7,25),(27,33)], 'hole')
		
	def _set_wall_demo(self):
		print 'setting walls'
		self._obstacle_area([ (0,0),( 1,30)], 'wall')
		self._obstacle_area([(23,0),(24,30)], 'wall')

	def edge_removal(self, G):
		""" remove edges with obstacles """

		e_init = G.number_of_edges()	# determine number of edges
		r_edge = [] 					# list for edges to remove
		
		for e in G.edges():
			# if either cell contains obstacles, remove edge
			if (G.nodes[e[0]]['cost']!= 0 or G.nodes[e[1]]['cost']!=0): 
				# stack into list to remove
				r_edge.append(e)
				
		for i in range(0,len(r_edge)):
			G.remove_edge(r_edge[i][0],r_edge[i][1])

		# print 'Init, final: ', e_init, '  ', G.number_of_edges()

	def edge_cost(self, G):
		""" assign cost to body graph edges 			"""
		""" cost 1: diagonal > vertical/horizontal edges"""
		""" cost 2: change in motion 					"""
		""" cost 3: change in bodypose 					"""

		## variables ##	
		
		# search for all edges
		for e in G.edges():
			# reset cost variables
			cost_dist 	= 0 	# von-neumann or moore edge
			cost_motion = 0		# change in motion primitive
			cost_d_bh	= 0		# change in body height			
			cost_d_qr	= 0		# change in body roll
			cost_d_qp	= 0		# change in body pitch
			cost_d_qy	= 0		# change in body yaw

			## cost 1: edge type
			dist = np.sqrt((e[1][0]-e[0][0])**2 + (e[1][1]-e[0][1])**2)
			if (dist<=1):		# von neumann
				cost_dist = 1
			else:				# moore edge
				cost_dist = 1.6

			## cost 2: change in motion primitive
			m1 = G.nodes[e[0]]['motion']
			m2 = G.nodes[e[1]]['motion']
			if ((m1==0 and m2==2) or (m1==2 and m2==0)):
				# walking to walling
				cost_motion = 5
			elif ((m1==0 and m2==1) or (m1==1 and m2==0)):
				# walking to chimney
				cost_motion = 3

			## Cost 3: Change in bodypose (height, roll, pitch, yaw)
			cost_d_bh = abs(G.nodes[e[0]]['pose'][0] - G.nodes[e[1]]['pose'][0])
			cost_d_qr = abs(G.nodes[e[0]]['pose'][1] - G.nodes[e[1]]['pose'][1])
			cost_d_qp = abs(G.nodes[e[0]]['pose'][2] - G.nodes[e[1]]['pose'][2])
			cost_d_qy = abs(G.nodes[e[0]]['pose'][3] - G.nodes[e[1]]['pose'][3])

			## Cost 4: Wall selection - Wall Walking ONLY
			cost_d_lgw = 0
			if (m1==2 or m2==2):
				ldist, rdist = self.wall_length(e[0])
				# consider only if robot is between walls
				if (ldist[0] >=0 and rdist[0] >=0):
					if (ldist[1] > rdist[1]):	# left wall longer
						cost_d_lgw = ldist[0] 	
					else:						# right wall longer
						cost_d_lgw = rdist[0]
			## Sum of cost
			G.edges[e]['cost'] = cost_dist + cost_motion + cost_d_bh + cost_d_qr + cost_d_qp + cost_d_qy + cost_d_lgw*0.01

		nx.set_edge_attributes(self.G, 1, 'cost') 	# set cost based on vertical distance to next cell

	def check_body_area(self, p, body_area):
		""" check body area for collision - returns True or False """
		
		## variables ##
		valid = False
		
		## skip if centre occupied - fast check
		if (self.G.nodes[p]['cost'] != 1):
			bx = int(math.floor(body_area[0])/2) # longitudinal distance to check
			by = int(math.floor(body_area[1])/2) # lateral distance to check
			
			## check entire body area
			for y in range(0,by+1): 				# cycle lateral - inflated if body_area[1] is even
				for x in range(0,body_area[0]+1):		# cycle longitudinal
					EL_edge = self.G.nodes[(p[0]-y,p[1]-bx+x)]['cost']		# left edge
					ER_edge = self.G.nodes[(p[0]+y,p[1]-bx+x)]['cost']		# right edge
					# if (p==(12,85)):
					# 	print bx, by,(p[0]-y,p[1]-bx+x), EL_edge, (p[0]+y,p[1]-bx+x), ER_edge
					# check if collision exists
					if (EL_edge==1 or ER_edge==1):
						valid = False
						break 			# exit immediately as area invalid
					else:
						valid = True
		# if (p==(13,20)):
		# 	print p, valid
		return valid

	def ground_walking(self, p):
		
		## Variables ##
		m_width = 0.6 				# footprint width of motion
		bx = (self.rblg_gd[0]-1)/2  # longitudinal distance from robot centre
		by = (self.rblg_gd[1]-1)/2 	# lateral distance from robot centre
		motion_valid = True
		foothold_exist = False

		## Check if valid foothold exist in each leg region, outputs boolean
		for i in range(0,6):
			# Approximate nominal stance to nearest cell
			px 	  = int(np.round( (np.cos(np.pi/2)*(p_leg[i][0])-np.sin(np.pi/2)*(p_leg[i][1]))/self.resolution))
			py 	  = int(np.round( (np.sin(np.pi/2)*(p_leg[i][0])+np.cos(np.pi/2)*(p_leg[i][1]))/self.resolution))
			pbnom = (px,py)

			pwnom = tuple(map(lambda x,y: y+x, p, pbnom))	# leg nominal stance in world frame - equal for left and right side
			fcost = 0 										# reset total cost per foot
			
			## Flag for skipping row & column
			skip_row = False
			# skip_col = False

			foothold_exist = False 	# reset flag
			# if (p==(15,17) and i==1):
			# 	print '----------------'
			# 	print pbnom, pwnom
			## calculate sum for each area
			for cx in range(0,self.rblg_gd[0]): 		# cycle through x
				skip_row = False 						# reset flag 

				for cy in range(0,self.rblg_gd[1]):		# cycle through y
					## skip rest of row if inner cell occupied
					if (skip_row == False):
						## foothold in leg grid area - start top to bottom, inside to outside
						if (i<3):
							## LHS
							pwset = (pwnom[0]+by-cy,pwnom[1]+bx-cx) 
							# if (p==(15,17) and i==1):
							# 	print pwset
						else:
							## RHS
							pwset = (pwnom[0]-by+cy,pwnom[1]+bx-cx) 

						## check if cell occupied
						try:
							if (self.G.nodes[pwset]['cost']==1):
								fcost = fcost + self.G.nodes[pwset]['cost'] 	# calculate cost of area - not true since will be skiping through
								skip_row = True 								# set flag to skip remainder of row - computationally efficient
							elif (self.G.nodes[pwset]['cost']==-1):
								# skip_row = True
								pass
							else:
								foothold_exist = True
								# assign cost to this foothold
							# if (p==(15,17) and i==1):
							# 	print skip_row
						except:
							pass

			## Calculate average of area and add to total
			# area  = self.rblg_gd[0]*self.rblg_gd[1]
			# ncost = math.floor( fcost/float(area) )
			# scost = scost + ncost 
			# if (p==(12,55)):
			# 	print 'walking ', p, foothold_exist
			## Set flag for foothold area
			# if (foothold_exist == True):
			# 	leg_valid[i] = 1
			# else:
			# 	leg_valid[i] = 0
			# 	valid_flag = False
			if (foothold_exist is False):
				motion_valid = False
				break
		
		return motion_valid, m_width

	def wall_walking(self, p):
		""" Check if wall walking is feasible for CoB at p """
		""" Input: p -> CoB location """

		## variables ##
		# scost = 0 						# total cost
		width = 0 						# footprint width (cells)
		i_edge = 0 						# internal edge
		EL_edge = False					# external left edge
		ER_edge = False					# external right edge
		body_area  = self.rbbd_gd 		# body area (grid cells)
		# leg_valid  = np.zeros(6)		# flag array if foothold area valid
		motion_valid = False 				# boolen flag if area is permissible
		MIN_WALLING_AREA = (0.3,0.18)	# minimum footprint area for wall walking - based on kinematic and torque study
		MAX_WALLING_AREA = (0.3,0.7) 	# maximum footprint area for wall walking - based on kinematic and torque study

		bx_min = int((MIN_WALLING_AREA[0]/self.resolution-1)/2)
		by_min = int((MIN_WALLING_AREA[1]/self.resolution)/2)
		
		bx_max = int((MAX_WALLING_AREA[0]/self.resolution-1)/2)
		by_max = int((MAX_WALLING_AREA[1]/self.resolution-1)/2)
		
		test_point = (15,17)
		# if (p==test_point):
		# 	print 'wall: ', bx_min, by_min, bx_max, by_max
		# skip if centre occupied
		if (self.G.nodes[p]['cost'] != 1):
			i_edge = 0 	# internal edge

			# Check between by_min to by_max
			for d in range(0,by_max-by_min):
				# check internal space - outer edge excluded (MAY NEED TO MOVE ELSEWHERE TB more efficient)
				for y in range(0,by_min):
					for x in range(0,bx_min*2):
						try:
							# left edge and right edge
							if (self.G.nodes[(p[0]-y,p[1]-bx_min+x)]['cost']==1 or self.G.nodes[(p[0]+y,p[1]-bx_min+x)]['cost']==1):
								i_edge += 1
						except:
							pass

				# continues to check boundary edges only if internal boundary valid
				if (i_edge==0):
					LA_1 = 0; LA_4 = 0	
					LA_2 = 0; LA_5 = 0
					LA_3 = 0; LA_6 = 0
					for x in range(0,3):
						try:
							## Position of legs
							pf_1  = (p[0]-by_min-d,p[1]+bx_min-x)
							pf_1n = (pf_1[0]+1,pf_1[1])
							pf_2  = (p[0]-by_min-d,p[1]+bx_min-x-3)
							pf_2n = (pf_2[0]+1,pf_2[1])
							pf_3  = (p[0]-by_min-d,p[1]+bx_min-x-6) 
							pf_3n = (pf_3[0]+1,pf_3[1])

							pf_4  = (p[0]+by_min+d,p[1]+bx_min-x)
							pf_4n = (pf_4[0]-1,pf_4[1])
							pf_5  = (p[0]+by_min+d,p[1]+bx_min-x-3)
							pf_5n = (pf_5[0]-1,pf_5[1])
							pf_6  = (p[0]+by_min+d,p[1]+bx_min-x-6) 
							pf_6n = (pf_6[0]-1,pf_6[1])

							# if (p==test_point):
							# 	print p, d, pf_1, pf_2, pf_3, pf_4, pf_5, pf_6

							# LEG 1
							if (self.G.nodes[pf_1]['cost']==1 and self.G.nodes[pf_1n]['cost']==1):
								LA_1 = -1;
								break
							else:
								LA_1  += self.G.nodes[pf_1]['cost'] 	# front leg
							
							# LEG 2
							if (self.G.nodes[pf_2]['cost']==1 and self.G.nodes[pf_2n]['cost']==1):
								LA_2 = -1;
								break
							else:
								LA_2 += self.G.nodes[pf_2]['cost']
							
							# LEG 3
							if (self.G.nodes[pf_3]['cost']==1 and self.G.nodes[pf_3n]['cost']==1):
								LA_3 = -1;
								break
							else:
								LA_3 += self.G.nodes[pf_3]['cost']

							# LEG 4
							if (self.G.nodes[pf_4]['cost']==1 and self.G.nodes[pf_4n]['cost']==1):
								LA_4 = -1;
								break
							else:			
								LA_4 += self.G.nodes[pf_4]['cost']

							# LEG 5							
							if (self.G.nodes[pf_5]['cost']==1 and self.G.nodes[pf_5n]['cost']==1):
								LA_5 = -1;
								break
							else:
								LA_5 += self.G.nodes[pf_5]['cost']

							# LEG 6
							if (self.G.nodes[pf_6]['cost']==1 and self.G.nodes[pf_6n]['cost']==1):
								LA_6 = -1;
								break
							else:
								LA_6 += self.G.nodes[pf_6]['cost']
						except:
							pass

					# if (p==test_point):
					# 	print 'flag: ', LA_1, LA_2, LA_3, LA_4, LA_5, LA_6
					## check validity
					if (LA_1<0 or LA_2<0 or LA_3<0 or LA_4<0 or LA_5<0 or LA_6<0):
						# reset flags as statically unfeasible - no holes allowed in proximity
						EL_edge		= False
						ER_edge 	= False
						motion_valid= False
					else:
						## valid if TWO legs on wall have footholds and ground are all ground
						# if ( (LA_1 > 0 and LA_2 > 0) or  (LA_3 > 0 and LA_2 > 0) ):
						# 	if (LA_4==0 and LA_5==0 and LA_6==0):
						# 		# LHS wall walking
						# 		EL_edge    = True
						# 		valid_flag = True
						# 		if (p==test_point):
						# 			print 'LHS exists!'
						# elif( (LA_4 > 0 and LA_5 > 0) or  (LA_5 > 0 and LA_6 > 0) ):
						# 	if (LA_1==0 and LA_2==0 and LA_3==0):
						# 		# RHS wall walking
						# 		ER_edge    = True
						# 		valid_flag = True
						# 		if (p==test_point):
						# 			print 'RHS exists!'
						## valid if THREE legs on wall have footholds and ground are all ground
						if ( LA_1 > 0 and LA_2 > 0 and LA_3 > 0 ):
							if (LA_4==0 and LA_5==0 and LA_6==0):
								# LHS wall walking
								EL_edge    = True
								motion_valid = True
								# if (p==test_point):
								# 	print 'LHS exists!'
						elif( LA_4 > 0 and LA_5 > 0 and LA_6 > 0 ):
							if (LA_1==0 and LA_2==0 and LA_3==0):
								# RHS wall walking
								ER_edge    = True
								motion_valid = True
								# if (p==test_point):
								# 	print 'RHS exists!'
					
					# Exit loop if valid configuration exists
					if (motion_valid):
						sided = 1 if EL_edge else -1
						width = sided*((p[0]+by_min+d) - (p[0]-by_min-d)) 	# footprint width (cells)
						break
				# Exit search as internal boundary violated
				else:
					break
		# else: 
		# 	scost = 1
		# if (p==(27,21)):
		# 	print 'this: ', p, valid_flag, i_edge, EL_edge, ER_edge
		return motion_valid, width

	def chimney_walking(self, p):

		## variables ##
		m_width = 0						# footprint width (cells)
		i_edge = 0 						# internal edge
		EL_edge = 0 					# external left edge
		ER_edge = 0 					# external right edge
		body_area  = self.rbbd_gd 		# body area (grid cells)
		
		motion_valid = False 			# boolen flag if area is permissible
		MIN_CHIMNEY_AREA = (0.3,0.6)	# minimum footprint area for chimney - based on kinematic and torque study
		MAX_CHIMNEY_AREA = (0.3,0.72) 	# maximum footprint area for chimney - based on kinematic and torque study

		bx_min = int((MIN_CHIMNEY_AREA[0]/self.resolution-1)/2)
		by_min = int((MIN_CHIMNEY_AREA[1]/self.resolution-1)/2)
		
		bx_max = int((MAX_CHIMNEY_AREA[0]/self.resolution-1)/2)
		by_max = int((MAX_CHIMNEY_AREA[1]/self.resolution-1)/2)

		# print 'chim: ', bx_min*2, by_min*2, bx_max*2, by_max*2
		test_point = (12,85)
		
		# skip if centre occupied
		if (self.G.nodes[p]['cost'] != 1):
			i_edge = 0 	# internal edge
			## cycle through all four edges TODO: remove overlap on points
			for x in range(0,bx_min*2):
				# check left and right edge
				try:
					p1 = (p[0]-by_min,p[1]-bx_min+x)
					p2 = (p[0]+by_min,p[1]-bx_min+x)
					if (self.G.nodes[p1]['cost']==1 or self.G.nodes[p2]['cost']==1):
						i_edge += 1
				except:
					pass # ignores cells outside of map
				
			for y in range(0,by_min*2):
				# check top and bottom edge
				try:
					p1 = (p[0]-by_min+y,p[1]-bx_min)
					p2 = (p[0]-by_min+y,p[1]+bx_min)
					if (self.G.nodes[p1]['cost']== 1 or self.G.nodes[p2]['cost']==1):
						i_edge += 1
					# if (p==(12,55)):
					# 	print p1, p2
				except:
					pass # ignores cells outside of map

			# continues to check the WHOLE middle space of boundary only if internal boundary valid
			if (i_edge == 0):
				## cycle through all left & right columns				
				for d in range(0,by_max-by_min+1): 	# lateral width to cycle through
					LA_1 = 0; LA_4 = 0	
					LA_2 = 0; LA_5 = 0
					LA_3 = 0; LA_6 = 0
					for x in range(0,3):
						try:
							## Position of legs
							pf_1  = (p[0]-by_min-d,p[1]+bx_min-x)
							pf_1n = (pf_1[0]+1,pf_1[1])
							pf_2  = (p[0]-by_min-d,p[1]+bx_min-x-3)
							pf_2n = (pf_2[0]+1,pf_2[1])
							pf_3  = (p[0]-by_min-d,p[1]+bx_min-x-6) 
							pf_3n = (pf_3[0]+1,pf_3[1])

							pf_4  = (p[0]+by_min+d,p[1]+bx_min-x)
							pf_4n = (pf_4[0]-1,pf_4[1])
							pf_5  = (p[0]+by_min+d,p[1]+bx_min-x-3)
							pf_5n = (pf_5[0]-1,pf_5[1])
							pf_6  = (p[0]+by_min+d,p[1]+bx_min-x-6) 
							pf_6n = (pf_6[0]-1,pf_6[1])

							# if (p==test_point):
							# 	print p, pf_1, pf_2, pf_3, pf_4, pf_5, pf_6

							# LEG 1
							if (self.G.nodes[pf_1]['cost']==1 and self.G.nodes[pf_1n]['cost']==1):
								LA_1 = -1;
								break
							else:
								LA_1  += self.G.nodes[pf_1]['cost'] 	# front leg
							
							# LEG 2
							if (self.G.nodes[pf_2]['cost']==1 and self.G.nodes[pf_2n]['cost']==1):
								LA_2 = -1;
								break
							else:
								LA_2 += self.G.nodes[pf_2]['cost'] 	# middle leg
							
							# LEG 3
							if (self.G.nodes[pf_3]['cost']==1 and self.G.nodes[pf_3n]['cost']==1):
								LA_3 = -1;
								break
							else:
								LA_3 += self.G.nodes[pf_3]['cost'] 	# middle leg

							# LEG 4
							if (self.G.nodes[pf_4]['cost']==1 and self.G.nodes[pf_4n]['cost']==1):
								LA_4 = -1;
								break
							else:			
								LA_4 += self.G.nodes[pf_4]['cost']

							# LEG 5							
							if (self.G.nodes[pf_5]['cost']==1 and self.G.nodes[pf_5n]['cost']==1):
								LA_5 = -1;
								break
							else:
								LA_5 += self.G.nodes[pf_5]['cost']

							# LEG 6
							if (self.G.nodes[pf_6]['cost']==1 and self.G.nodes[pf_6n]['cost']==1):
								LA_6 = -1;
								break
							else:
								LA_6 += self.G.nodes[pf_6]['cost']
						except:
							pass

					# if (p==test_point):
					# 	print 'flag: ', LA_1, LA_2, LA_3, LA_4, LA_5, LA_6
					
					## check validity
					if (LA_1<0 or LA_2<0 or LA_3<0 or LA_4<0 or LA_5<0 or LA_6<0):
						# reset flags as statically unfeasible - no holes allowed in proximity
						EL_edge		= False
						ER_edge 	= False
						motion_valid= False
					else:
						## valid: front TWO pair wall contact footholds on both sides and rear pair ground or wall contact
						if ( (LA_1>0 and LA_2>0) and (LA_4>0 and LA_5>0) ):
							if ((LA_3==0 or LA_3>0) and (LA_6==0 or LA_6>0)): 
								motion_valid = True
								# if (p==test_point):
								# 	print 'chimney exists!'
						elif ( (LA_3>0 and LA_2>0) and (LA_6>0 and LA_5>0) ):
							if ((LA_1==0 or LA_1>0) and (LA_4==0 or LA_4>0)): 
								motion_valid = True
								# if (p==test_point):
								# 	print 'chimney exists!'

						## valid: THREE wall contact footholds on both sides
						# if ( LA_1 > 0 and LA_2 > 0 and LA_3 > 0 ):
						# 	if( LA_4 > 0 and LA_5 > 0 and LA_6 > 0 ):
						# 		valid_flag = True
						# 		if (p==test_point):
						# 			print 'chimney exists!'
			else:
				motion_valid = False
		else: 
			scost = 1
		
		return motion_valid, m_width

	def check_motion_primitive(self, p):
		""" Checks if cell is accessible using motion primitives, 
			start with ground then chimney and finally wall walking	"""

		## Variables ##
		m_valid = False
		m_width = -1
		primitive = -1		# motion primitive: 0 = walk,1 = chimney,2 = wall
		
		## Ground walking ##
		#  condition: valid foothold exists within foothold area
		base_valid = self.check_body_area(p,self.rbbd_gd)
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
			if (not m_valid and self.check_body_area(p,(self.rbbd_gd[0],3))):
				# condition: 1) wall within X.X m from base point, 2) ground is present
				m_valid, m_width = self.wall_walking(p)
				if (m_valid):
					primitive = 2
					self.GM_wall.add_node(p)	# ILLUSTRATION: stack into graph

		return m_valid, primitive, m_width

	def process_map_on_primitive(self):
		""" Process entire map for motion primitive feasibility """

		## Variables ##
		ax = (self.rbfp_gd[0]-1)/2 		# robot longitudinal distance (grid cell)
		ay = (self.rbfp_gd[1]-1)/2-5	# robot lateral distance (grid cell) - HARDCODED
		# POSE_TABLE  = bodypose_table()	# lookup table for walling bodypose
		print 'Body Edges: ', ax, ' ', ay
		
		node_ignored = [] 	# list for nodes to ignore: either near map border or intersect obstacles/holes
		node_free 	 = [] 	# list for free nodes

		for e in self.G.nodes():
			# Ignore CoB that intersect edges
			if (e[0]<=(ay-1) or e[0]>=(self.mp_sz[0]-ay) or e[1]<=(ax-1) or e[1]>=(self.mp_sz[1]-ax)): 	
				node_ignored.append(e)
				self.Gbody.nodes[e]['cost'] = 1
			else:
				# Check through possible motion primitives
				m_valid, primitive, m_width = self.check_motion_primitive(e)
				
				if (m_valid):
					if (primitive==2):
						# itb = self.PoseTable.search_wall_table(width=abs(m_width)*self.resolution)
						# bh 	= self.PoseTable.wall[itb]['bodypose'][2]
						# qr 	= self.PoseTable.wall[itb]['bodypose'][3]
						# print 'itb: ', itb, m_width*self.resolution
						idx = self.PoseTable.search_table(width=abs(m_width)*self.resolution)
						bh	= self.PoseTable.table[idx]['bodypose'][2]
						sb = 1 if (m_width > 0.) else -1
						qr	= sb*self.PoseTable.table[idx]['bodypose'][3]
						# if (e == (14,17)):
						# 	print 'at this point: '
						# 	print abs(m_width)*self.resolution
						# 	print bh
						# 	print qr
						# 	print '---------------------'
					else:
						bh = BODY_HEIGHT
						qr = 0.

					node_free.append(e)
					self.Gbody.nodes[e]['cost']   = 0
					self.Gbody.nodes[e]['motion'] = primitive
					self.Gbody.nodes[e]['width']  = m_width
					self.Gbody.nodes[e]['pose']   = [bh,qr,0.,0.]
				else:
					self.Gbody.nodes[e]['cost'] = 1
				
			# if (e==(30,31)):
			# 	print e, valid_flag 
		self.Gfree = nx.path_graph(node_free)

	def eucld_dist(self,a,b):
		""" heuristics used for A* planner """
		(x1, y1) = a
		(x2, y2) = b
		return (((x1 - x2)**2 + (y1 - y2)**2)**0.5)

	def find_base_path(self,start,end):
		""" Finds a path given start and end position on map """

		self.process_map_on_primitive() # finds path then use motion primitives to fit in
		self.edge_removal(self.Gbody) 	# remove edges linked to invalid cells
		self.edge_cost(self.Gbody)		# assign cost to edge

		## Robot represented with a number of nodes
		try:
			# list_path	= nx.shortest_path(self.Gbody,start,end,weight='cost')
			# list_path	= nx.dijkstra_path(self.Gbody,start,end,weight='cost')
			list_path 	= nx.astar_path(self.Gbody,start,end,heuristic=self.eucld_dist,weight='cost')
			# graph_path 	= nx.path_graph(list_path)
			# self.spline = self.path_interpolation(list_path)
			# return self.path_interpolation(list_path)
			# self.G_LF, self.G_LM, self.G_LR, self.G_RF, self.G_RM, self.G_RR = self.find_foothold(list_path)
			print 'Path exist!' #, len(list_path)
			
			return list_path

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

		for e in self.G.nodes():
			# ignore edges
			if (e[0]<=(ax-1) or e[0]>=(self.mp_sz[0]-ax) or e[1]<=(ay-1) or e[1]>=(self.mp_sz[1]-ay)): 	
				node_ignored.append(e)
				self.Gbody.nodes[e]['cost'] = 1
			else:
				# check body collision TODO: automate this to a square dependant on resolution 
				valid = self.check_body_area(e, self.rbbd_gd)
				
				# compute for nodes that are obstacle free
				if (valid):
					footholds_cost, valid_flag = self.ground_walking(e)

					# set node cost based on foothold average cost					
					# self.Gbody.nodes[e]['cost'] = footholds_cost
					# if (e == (13,17) ):
					# 	print e, valid_flag

					if (valid_flag == True):
						node_free.append(e)
						self.Gbody.nodes[e]['cost'] = 0
					else:
						self.Gbody.nodes[e]['cost'] = 1
				else:
					self.Gbody.nodes[e]['cost'] = 1

		self.Gfree = nx.path_graph(node_free)

	def foothold_planner(self, base_path, Robot):
		""" Computes foothold based on generated path 	"""
		""" Input: 1) path -> linear and angular path 
			Output: Foothold array 						"""

		def set_motion_plan():
			## Set MotionPlan class
			motion_plan = MotionPlan()
			motion_plan.set_base_path(Robot.P6c.world_X_base.copy(), base_path, world_X_base)
			motion_plan.set_footholds(world_X_footholds, base_X_footholds, world_base_X_NRP)
			motion_plan.set_gait(Robot.Gait.np, Gait.np)

			return motion_plan

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
			
			# Create temp array, which is the new base to foot position, wrt world frame
			# The x-component uses the base frame NRP so that it remains the same
			temp = np.array([[Leg[j].base_X_NRP[0,3]],[py],[Leg[j].world_base_X_NRP[2,3]]])
			Leg[j].world_base_X_NRP[:3,3:4] = mX(rot_Z(v3wp[2]), temp)
			
		def compute_wall_footholds(d_wall):
			""" Compute footholds for legs in wall contact """

			world_base_X_femur_z = (COXA_Y + L1)*np.sin(v3wp[0])
			world_base_X_femur_y = (COXA_Y + L1)*np.cos(v3wp[0])

			qoffset = np.pi/2 if (j < 3) else -np.pi/2
			bXfy = d_wall[1]*np.sin(qoffset) 		# world horizontal distance base to foot
			dy = bXfy - world_base_X_femur_y 		# horizontal distance from femur joint to foot
			dh = np.sqrt(abs(L3**2 - dy**2)) 		# vertical distance from femur joint to foot
			bXfz = world_base_X_femur_z + L2/2 + dh # vertical distance from base to foot
			
			temp = np.array([[Leg[j].base_X_NRP[0,3]],[bXfy],[bXfz]])
			Leg[j].world_base_X_NRP[:3,3:4] = mX(rot_Z(v3wp[2]), temp)
			Leg[j].world_X_NRP[:3,3] = np.round( (v3cp + Leg[j].world_base_X_NRP[:3,3:4]).flatten(),4)
			# print '>>>> ', world_base_X_femur_z, dy, dh

		## Define Variables ##
		i = 0
		XHd  = robot_transforms.HomogeneousTransform()
		Leg  = [None]*6
		Gait = gait_class.GaitClass(GAIT_TYPE)	# gait class
		do_wall = np.array([0., 0.31, 0.]) 		# Initial distance from robot's base to wall (base frame)

		gphase_intv  = [] 						# intervals in which gait phase changes
		world_X_base = []
		world_X_footholds = [None]*6
		base_X_footholds  = [None]*6
		world_base_X_NRP  = [None]*6

		Gait.cs = list(Robot.Gait.cs) 	# update local gait current state 
		Gait.np = Robot.Gait.np
		v3cp_prev = Robot.P6c.world_X_base[0:3].copy()	# previous CoB position
		v3wp_prev = Robot.P6c.world_X_base[3:6].copy()	# previous CoB orientation
		world_X_base.append(Robot.P6c.world_X_base.flatten())

		## Instantiate leg transformation class & append initial footholds
		for j in range (0, 6):
			Leg[j] = robot_transforms.ArrayHomogeneousTransform(j)
			Leg[j].duplicate(Robot.Leg[j].XHd)

			world_X_footholds[j] = MarkerList()
			world_X_footholds[j].t.append(0.)
			world_X_footholds[j].xp.append(Robot.Leg[j].XHc.world_X_foot[:3,3:4])

			base_X_footholds[j] = MarkerList()
			base_X_footholds[j].t.append(0.)
			base_X_footholds[j].xp.append(Robot.Leg[j].XHc.base_X_foot[:3,3:4])

			world_base_X_NRP[j] = MarkerList()
			world_base_X_NRP[j].t.append(0.)
			world_base_X_NRP[j].xp.append(Robot.Leg[j].XHc.world_base_X_NRP[:3,3:4])

		## Returns as footholds remain fixed for full support mode
		if (Robot.support_mode is True):
			return set_motion_plan()

		## Cycle through trajectory
		while (i != len(base_path.X.t)):
			print i, ' Gait phase ', Gait.cs 
			bound_exceed = False

			# Cycles through one gait phase for support legs
			for m in range(0,int(GAIT_TPHASE*CTR_RATE)+1):
				## Variable mapping to R^(3x1) for base linear and angular time, position, velocity, acceleration
				try:
					v3cp = base_path.X.xp[i].reshape(3,1) #+ Robot.P6c.world_X_base[0:3]
					v3wp = base_path.W.xp[i].reshape(3,1) + Robot.P6c.world_X_base[3:6]
					
					# print np.round(v3cp.flatten(),4)
					P6d_world_X_base = np.vstack((v3cp,v3wp))

					## update robot's pose
					XHd.update_world_X_base(P6d_world_X_base)
					world_X_base_rot = XHd.world_X_base.copy()
					world_X_base_rot[:3,3:4] = np.zeros((3,1))

					for j in range (0, Robot.active_legs):
						## Legs in support phase:
						if (Gait.cs[j] == 0):
							Leg[j].base_X_foot = mX(XHd.base_X_world, Leg[j].world_X_foot)

							Leg[j].world_base_X_foot = mX(world_X_base_rot, Leg[j].base_X_foot)
							bound_exceed = Robot.Leg[j].check_boundary_limit(Leg[j].world_base_X_foot,
																					Leg[j].world_base_X_NRP)

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
					print 'Finishing foothold planning at ', i
					break

			## Stack to list next CoB location
			world_X_base.append(P6d_world_X_base.reshape(1,6))
			gphase_intv.append(i)
			print 'qbp: ', np.round(P6d_world_X_base.reshape(1,6),4)
			## Set foothold for legs in transfer phase
			for j in range (0, 6):
				if (Gait.cs[j] == 1 and i <= len(base_path.X.t)):
					
					## 1) Update NRP
					if (self.T_GND_X_WALL is True):
						
						# Distance robot's base travelled from origin
						dt_base = (mX(rot_Z(-v3wp[2]), (v3cp.reshape(3) - base_path.X.xp[0]).reshape(3,1) )).reshape(3)

						## Identify sides for ground or wall contact based on body roll
						delta_w = base_path.W.xp[-1]# - base_path.W.xp[0]
						# idx = self.PoseTable.search_table(angle=delta_w[0])
						
						if (delta_w[0] > 0.):
							## Right side ground contact, Left side wall contact
							if (j >= 3):
								compute_ground_footholds()
							else:
								di_wall = do_wall - np.round(dt_base ,3)
								compute_wall_footholds(di_wall)
								# print 'do wall ', np.round(dt_base,3), np.round(di_wall,3)
						else:
							## Left side in ground contact, Right side wall contact
							if (j >= 3):
								di_wall = do_wall + np.round(dt_base ,3)
								compute_wall_footholds(di_wall)
							else:
								compute_ground_footholds()
					
					elif (self.T_WALL_X_GND is True):
						if (j >= 3):
							compute_ground_footholds()
						else:
							compute_wall_footholds(di_wall)

					elif (self.W_WALL is True):
						# SE(3) with linear components and only yaw rotation
						XHy_world_X_base = mX(v3_X_m(P6d_world_X_base[:3]), r3_X_m(np.array([P6d_world_X_base[3],
																					0.,P6d_world_X_base[5]])))

						Leg[j].world_X_NRP = np.dot(XHy_world_X_base, Leg[j].base_X_NRP)
						Leg[j].world_base_X_NRP[:3,3:4] = mX((XHy_world_X_base[:3,:3]), Leg[j].base_X_NRP[:3,3:4])
						
					else:
						Leg[j].update_world_base_X_NRP(P6d_world_X_base)
					
					## 2) Compute magnitude & vector direction
					if (j>=3):
						print j, ' dwa : ', di_wall, np.round(v3cp.flatten(),3), np.round(dt_base ,3)
						print j, ' wXn : ', np.round(Leg[j].world_X_NRP[:3,3],3)
						print j, ' grp : ', (Leg[j].world_X_NRP[0,3]/self.resolution), int(Leg[j].world_X_NRP[1,3]*self.resolution)
						print j, ' wXbn: ', np.round(Leg[j].world_base_X_NRP[:3,3],3)

					# Get surface normal
					snorm = self.get_cell('norm', Leg[j].world_X_NRP[:3,3], j)

					v3_dv = (v3cp - v3cp_prev).flatten() 			# direction vector from previous to current CoB
					v3_pv = v3_dv - (np.dot(v3_dv,snorm))*snorm 	# project direction vector onto plane
					m1_dv = np.linalg.norm(v3_pv) 					# magnitude of direction vector
					v3_uv = np.nan_to_num(v3_pv/m1_dv) 				# unit vector direction

					## TEMP: overwrite last transfer phase on base spline
					if (i == len(base_path.X.t)):
						v3_uv = np.zeros(3)
					
					## 3) Compute AEP wrt base and world frame					
					Leg[j].world_base_X_AEP[:3,3] = Leg[j].world_base_X_NRP[:3,3] + (v3_uv*STEP_STROKE/2.)
					Leg[j].base_X_AEP[:3,3:4] = mX(XHd.base_X_world[:3,:3], Leg[j].world_base_X_AEP[:3,3:4])
					Leg[j].base_X_NRP[:3,3:4] = mX(XHd.base_X_world[:3,:3], Leg[j].world_base_X_NRP[:3,3:4])
					Leg[j].world_X_foot = mX(XHd.world_X_base, Leg[j].base_X_AEP)
					
					## Get cell height in (x,y) location of world_X_foot
					cell_h = np.array([0., 0., self.get_cell('cost', Leg[j].world_X_foot[:3,3], j)]) 	# cost=height
					
					if (j>=3):
						# print j, (Leg[j].world_X_NRP[0,3]/self.resolution), int(Leg[j].world_X_NRP[1,3]*self.resolution)
						# print j, ' wXn : ', np.round(Leg[j].world_X_NRP[:3,3],3)
						# print j, ' wXbn: ', np.round(Leg[j].world_base_X_NRP[:3,3],3)
						print j, ' wXba: ', np.round(Leg[j].world_base_X_AEP[:3,3],3)
						print j, ' snorm ', snorm, cell_h, np.round(v3cp.flatten(),3)
						print j, ' wXf:  ', np.round(Leg[j].world_X_foot[:3,3], 4)

					## Cell height above threshold gets ignored as this requires advanced motions
					if (cell_h.item(2) < 0.1):# and chim_trans is False and self.W_CHIM is False):
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
					
					# if (j==1):
					# 	print 'snorm ', snorm, cell_h, self.W_WALL
					# 	# print 'v3pv: ', v3_pv.flatten()
					# 	# print 'v3uv: ', v3_uv.flatten()
					# 	# print 'bXN:  ', np.round(Leg[j].base_X_NRP[:3,3],4)
					# 	print 'wXf:  ', np.round(Leg[j].world_X_foot[:3,3], 4)
					# 	print 'wbXN: ', np.round(Leg[j].world_base_X_NRP[:3,3],4)
					# 	print 'wbXA: ', np.round(Leg[j].world_base_X_AEP[:3,3],4)
						# print 'wXN:  ', XHd.world_X_base[:3,3] + Leg[j].world_base_X_NRP[:3,3]
						# print 'wXA:  ', XHd.world_X_base[:3,3] + Leg[j].world_base_X_AEP[:3,3]
						# print 'wXb:  ', np.round(XHd.base_X_world,4)
						# 	print '--------------------------------------------'

			## Alternate gait phase
			Gait.change_phase()
			v3cp_prev = v3cp.copy()
			v3wp_prev = v3wp.copy()
		# print 'Held up in foothold planning'
		# Replace last footholds with default ground NRP
		if (self.T_WALL_X_GND is True or self.T_CHIM_X_GND is True):
			KDL = kdl.KDL()
			for j in range(0,6):
				Leg[j].update_base_X_NRP(KDL.leg_IK(LEG_STANCE[j]))
				base_X_footholds[j].xp[-1] = Leg[j].base_X_NRP[:3,3:4].copy()
				world_base_X_NRP[j].xp[-1] = Leg[j].base_X_NRP[:3,3:4].copy()
				world_X_footholds[j].xp[-1] = mX(XHd.world_X_base, Leg[j].base_X_NRP)[:3,3:4]
		print '============================================='
		## Smoothen base path
		# print gphase_intv
		glength = int(GAIT_TPHASE*CTR_RATE)+1 
		PathGenerator = Pathgenerator.PathGenerator()

		# for i in range(1,len(gphase_intv)-1):
		# 	# Identify phases to extend
		# 	if (gphase_intv[i] - gphase_intv[i-1] - glength < 0):
		# 		# print 'Extend', gphase_intv[i]*CTR_INTV, gphase_intv[i-1]*CTR_INTV
		# 		## Set initial and final position/orientation
		# 		## TODO: INCLUDE VELOCITY N ACCELERATION IN INTERPOLATION
		# 		xn_cob = base_path.X.xp[gphase_intv[i-1]]
		# 		xn_cob = np.vstack((xn_cob, base_path.X.xp[gphase_intv[i]]))

		# 		wn_cob = base_path.W.xp[gphase_intv[i-1]]
		# 		wn_cob = np.vstack((wn_cob, base_path.W.xp[gphase_intv[i]]))
				
		# 		## Generate new segment
		# 		new_segment = PathGenerator.generate_base_path(xn_cob, wn_cob, CTR_INTV, np.array([0.,GAIT_TPHASE]))
				
				## Append into existing trajectory
				# base_path.insert(gphase_intv[i-1],gphase_intv[i],new_segment)

		# Plot.plot_2d(base_path.X.t, base_path.X.xp)
		# print np.round(world_X_footholds[0].xp,3)
		# print 'in planner: ', np.round(world_X_footholds[0].xp,4)
		return set_motion_plan()

	def generate_motion_plan(self, Robot, **options):
		""" Generate motion plan for the robot """

		# Generate path given start and end location on map
		if (options.get("start")):
			start = options.get("start")
			end   = options.get("end")

			list_gpath = self.find_base_path(start, end)

			if (list_gpath is not None):
				print 'Path: ', list_gpath
				# High level preview
				# gpath = nx.path_graph(list_gpath)
				# self.graph_representation(False, gpath)
				# plt.show()
				# Convert path from cell index to Re^3 format 
				x_cob, w_cob = self.path_to_nparray(list_gpath, start)

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
		wXbase_offset = Robot.P6c.world_X_base.copy()

		# Set all legs to support mode for bodyposing, prevent AEP from being set
		if (Robot.support_mode == True):
			Robot.Gait.support_mode()
			PathGenerator.V_MAX = PathGenerator.W_MAX = 0.4
		else:
			Robot.Gait.walk_mode()

		## Rough Trajectory for robot's base
		# base_path = PathGenerator.generate_base_path(x_cob, w_cob, CTR_INTV)
		# Plot.plot_2d(base_path.X.t, base_path.X.xp)
		# Plot.plot_2d(base_path.W.t, base_path.W.xp)

		## Split path according to motion primitive
		return self.post_process_path(list_gpath, Robot)

	def transition_routine(self, ttype, p1, p2, tn=0.1):

		## define variables ##
		xi = np.array([p1[0]*self.resolution, p1[1]*self.resolution, self.Gbody.nodes[p1]['pose'][0]])
		xf = np.array([p2[0]*self.resolution, p2[1]*self.resolution, self.Gbody.nodes[p2]['pose'][0]])
		qi = self.Gbody.nodes[p1]['pose'][1]
		qf = self.Gbody.nodes[p2]['pose'][1]
		
		x_cob = xi.copy()
		w_cob = np.array([qi, 0., 0.])

		PathGenerator = path_generator.PathGenerator()

		print 'xi: ', xi, xf, qi, qf

		if (ttype=='Gnd_X_Wall'):
			print 'TR: g2w'
			# Generate ellipsoidal base trajectory
			qrange = range(0,91,10)
			delta_q = qf/(len(qrange)-1)
			delta_x = xf - xi

			for i in range(1, len(qrange)):
				qr = np.deg2rad(qrange[i])
				xd = xi + np.array([(1.-np.cos(qr))*delta_x[0],
									(1.-np.cos(qr))*delta_x[1], 
									(np.sin(qr))*delta_x[2]])

				x_cob = np.vstack(( x_cob, xd ))
				w_cob = np.vstack(( w_cob, np.array([i*delta_q, 0., 0.]) ))
			
			base_path = PathGenerator.generate_base_path(x_cob, w_cob, tn)
			# Plot.plot_2d(base_path.X.t, base_path.W.xp)

		elif (ttype=='Wall_X_Gnd'):
			print 'TR: w2g'
			# n_cycle = self.wall_transition(p1, p2)
			# x_inst, w_inst = self.path_interpolation([p1,p2],np.array([0.,tspan]))
			base_path = self.path_interpolation([p1,p2])

		elif (ttype=='Gnd_X_Chim'):
			""" change from ground to chimney wall support footholds """
			""" one gait cycle of stationary x_cob and w_cob """
			print 'TR: g2c'
			# inst_path = self.path_interpolation([p1,p2],np.array([0.,tspan]))
			base_path = self.path_interpolation([p1,p2])

		elif (ttype=='Chim_X_Gnd'):
			""" change from chimney wall to ground support footholds """
			""" one gait cycle of stationary x_cob and w_cob """
			print 'TR: c2g'
			# inst_path = self.path_interpolation([p1,p2],np.array([0.,tspan]))
			base_path = self.path_interpolation([p1,p2])
			
		return base_path

	def path_interpolation(self, path, tn=0.1):
		""" interpolate cells using cubic spline """

		## Define Variables ##
		x_cob = np.array([.0,.0,.0])
		w_cob = np.array([.0,.0,.0])
		
		## populate array
		for e in path:
			x_cob = np.vstack((x_cob,np.array([e[0]*self.resolution, e[1]*self.resolution, self.Gbody.nodes[e]['pose'][0]])))
			w_cob = np.vstack((w_cob,np.array([self.Gbody.nodes[e]['pose'][1:4]])))
			# print e, (np.round(e[0]*self.resolution,3), np.round(e[1]*self.resolution,3))
		
		x_cob = np.delete(x_cob,0,0)
		w_cob = np.delete(w_cob,0,0)
		# print x_cob, w_cob
		PathGenerator = path_generator.PathGenerator()
		base_path = PathGenerator.generate_base_path(x_cob, w_cob, tn)

		# Plot.plot_2d_multiple(1,wn_com.t,wn_com.xp*180/np.pi)
		# Plot.plot_2d_multiple(1,base_path.X.t,base_path.X.xp)
		
		return base_path

	def post_process_path(self,path, Robot):
		""" Identifies motion primitive from path, introduce transition 
			routines, and call foothold for the respective sections 	"""
		""" Input: 	1) path -> path in grid format
					2) Robot -> Robot class
			Output: MotionPlan() type 									"""
		print 'Post-processing path......'
		
		## Define Variables ##
		start_pos = path[0]
		temp_path = []
		base_path = Trajectory6D()
		motion_plan = MotionPlan()
		m = np.zeros(3) 	# motion primitive for next 3 steps

		## cycle through path
		for i in range(0,len(path)):
			print path[i]
			temp_path.append(path[i])
			## check transition
			m[0] = self.Gbody.nodes[path[i]]['motion']
			try:
				m[1] = self.Gbody.nodes[path[i+1]]['motion']
				m[2] = self.Gbody.nodes[path[i+2]]['motion']
			except:
				pass

			if (self.T_GND_X_WALL):
				qn1 = self.Gbody.nodes[path[i]]['pose'][1]
				qn0 = self.Gbody.nodes[path[i-1]]['pose'][1]
				# TODO: check if this condition feasible
				if (abs(qn1-qn0) < 0.05):
					print 'Interpolating transition ...'
					print sp, path[i]
					path_01 = self.transition_routine('Gnd_X_Wall', sp, path[i])
					plan_01 = self.foothold_planner(path_01, Robot)
					motion_plan.append(plan_01)
					self.T_GND_X_WALL = False

			## Check transition instances
			if (m[0]==0 and m[1]==2):
				## Walk to Wall ##
				print path[i], path[i+1], 'Ground to Wall'
				# First, plan path and foothold for ground walking
				if (len(temp_path) > 1):
					print 'Interpolating Wg prior to Ww...'
					path_01 = self.path_interpolation(temp_path)
					plan_01 = self.foothold_planner(path_01, Robot)
					motion_plan.append(plan_01)
					temp_path = []

				sp = path[i]

				# Next, generate CoB path and plan foothold for wall walking
				self.T_GND_X_WALL = True
				self.W_WALL = True
				self.W_GND  = False
				# path_02 = self.transition_routine('Gnd_X_Wall',path[i],path[i+1])
				# plan_02 = self.foothold_planner(path_02, Robot)
				# motion_plan.append(plan_02)
				# temp_path = []

			elif (m[0]==2 and m[1]==0):
				## Wall to Walk ##
				print path[i-1], path[i], 'wall to walk'
				if (m[2]==2):
					# force motion to walling for gap of one
					self.Gbody.nodes[path[i+1]]['motion'] = 1
					self.GM_chim.add_node(path[i+1])
					temp_path.append(path[i])
				else:
					inst_path = self.path_interpolation(temp_path)
					tran_path = self.transition_routine('Wall_X_Gnd',path[i],path[i+1])
					
					base_path.append(inst_path)
					base_path.append(tran_path)
					temp_path = []

			elif (m[0]==0 and m[1]==1):
				## Walk to Chimney ##
				print path[i], path[i+1], 'walk to chimney'
				inst_path = self.path_interpolation(temp_path)
				# TODO: plan foothold for above
				tran_path = self.transition_routine('Gnd_X_Chim',path[i],path[i+1])
				# TODO: plan foothold for above

				base_path.append(inst_path)
				base_path.append(tran_path)
				# Append footholds:
				# append(temp_path, shuffle, transition)
				temp_path = []

			elif (m[0]==1 and m[1]==0):
				## Chimney to Walk ##
				print path[i-1], path[i], 'chimney to walk'
				if (m[2]==1):
					# force motion to chimney for gap of one
					self.Gbody.nodes[path[i+1]]['motion'] = 1
					self.GM_chim.add_node(path[i+1])
					temp_path.append(path[i])
				else:
					inst_path = self.path_interpolation(temp_path)
					# TODO: plan foothold for above
					tran_path = self.transition_routine('Chim_X_Gnd',path[i],path[i+1])
					# TODO: plan foothold for above

					base_path.append(inst_path)
					base_path.append(tran_path)
					temp_path = []

		# Interpolate final sub-division if not empty
		if (len(temp_path) > 1):
			# Check type of interpolation
			if (self.W_GND):
				print 'Final interpolation, Wg: ', temp_path
				path_01 = self.path_interpolation(temp_path)

			elif (self.W_WALL):
				print 'Final interpolation, Ww: ', 
				print sp, path[i], self.T_GND_X_WALL
				path_01 = self.transition_routine('Gnd_X_Wall', sp, path[i])
				
				# plan_01 = self.foothold_planner(path_01, Robot)
				# motion_plan.append(plan_01)

			plan_01 = self.foothold_planner(path_01, Robot)
			motion_plan.append(plan_01)
		
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
		return motion_plan

	def graph_representation(self,initialized, gpath):
		""" Plots graph functions """

		# dictionary of node names->positions
		pos    = dict(zip(self.G, self.G)) 			
		p_fin  = dict(zip(gpath, gpath))
		# p_bas  = dict(zip(self.Gbasic, self.Gbasic))

		p_wall = dict(zip(self.G_wall, self.G_wall))
		p_hole = dict(zip(self.G_hole, self.G_hole))
		p_body = dict(zip(self.Gfree, self.Gfree))

		m_walk = dict(zip(self.GM_walk, self.GM_walk))
		m_wall = dict(zip(self.GM_wall, self.GM_wall))
		m_chim = dict(zip(self.GM_chim, self.GM_chim))

		# p_LF = dict(zip(self.G_LF, self.G_LF))
		# p_LM = dict(zip(self.G_LM, self.G_LM))
		# p_LR = dict(zip(self.G_LR, self.G_LR))
		# p_RF = dict(zip(self.G_RF, self.G_RF))
		# p_RM = dict(zip(self.G_RM, self.G_RM))
		# p_RR = dict(zip(self.G_RR, self.G_RR))

		# Grid map & obstacles
		plt.style.use('presentation')
		nsize  = 20	# size of nodes
		labels = False 	# label on nodes

		if (initialized == False):
			print 'First time plot'
			# Grid map
			ax = nx.draw_networkx(self.Gbody,pos,with_labels=labels,node_size=nsize,node_color='0.9',width=0.0,alpha=1.0,node_shape="s");
			# Body path
			# nx.draw_networkx(self.Gfree,p_body,with_labels=labels,node_size=25,node_color='y',edge_color='r',width=0.0,node_shape="s");
			# Wall
			nx.draw_networkx(self.G_wall,p_wall,with_labels=labels,node_size=nsize,node_color='#ff0000',width=0.0,node_shape="s");
			# Holes 
			nx.draw_networkx(self.G_hole,p_hole,with_labels=labels,node_size=nsize,node_color='#663300',width=0.0,node_shape="s");

			# Motion primitives
			nx.draw_networkx(self.GM_walk,m_walk,with_labels=labels,node_size=nsize, node_color='#ff6600',	width=0.0);
			nx.draw_networkx(self.GM_wall,m_wall,with_labels=labels,node_size=nsize, node_color='#00ccff',	width=0.0);
			nx.draw_networkx(self.GM_chim,m_chim,with_labels=labels,node_size=nsize, node_color='#66ff00',	width=0.0);

			# Feasible path - single point
			if (type(gpath) == list):
				gpath = nx.path_graph(gpath)
			
			nx.draw_networkx(gpath,p_fin, with_labels=labels,node_size=10,node_color='#ff33ff',edge_color='#ff33ff',alpha=1.0,width=1.0);

		else:
			# Feasible path - single point
			# nx.draw_networkx(gpath,p_fin, with_labels=labels,node_size=20,node_color='#ff6600',edge_color='#ff6600',alpha=1.0,width=5.0);
			pass

		
		# nx.draw_networkx(self.Gbasic,p_bas,with_labels=labels,node_size=5,node_color='b',edge_color='b',alpha=1.0,width=5.0);

		# Footholds
		# nx.draw_networkx(self.G_LF,p_LF,with_labels=labels,node_size=4,node_color='b',edge_color='b',width=0.5);
		# nx.draw_networkx(self.G_LM,p_LM,with_labels=labels,node_size=4,node_color='b',edge_color='b',width=0.0);
		# nx.draw_networkx(self.G_LR,p_LR,with_labels=labels,node_size=4,node_color='b',edge_color='b',width=0.0);
		# nx.draw_networkx(self.G_RF,p_RF,with_labels=labels,node_size=4,node_color='r',edge_color='r',width=0.5);
		# nx.draw_networkx(self.G_RM,p_RM,with_labels=labels,node_size=4,node_color='b',edge_color='b',width=0.0);
		# nx.draw_networkx(self.G_RR,p_RR,with_labels=labels,node_size=4,node_color='b',edge_color='b',width=0.0);

		## enable major and minor grid
		plt.grid('on');		 plt.grid(which='major', linestyle=':', linewidth='0.5', color='black')
		# plt.minorticks_on(); plt.grid(which='minor', linestyle=':', linewidth='0.25', color='black')

		# minor_ticks = np.arange(0, 9+1, 0.5)
		# plt.xticks(minor_ticks)

		plt.xlabel('y, [1]')
		plt.ylabel('x, [1]')

		# maximise plot window
		# print plt.get_backend()
		# mng = plt.get_current_fig_manager()
		# mng.resize(*mng.window.maxsize())
		plt.tight_layout()  
		# fig_manager.resize(*fig_manager.window.maxsize())
		
	def graph_to_nparray(self):
		""" converts NetworkX graph to array """

		## Define Variables ##
		sz = self.G.number_of_nodes() # size of grid map
		cx = np.zeros(sz)
		cy = np.zeros(sz)
		cz = np.zeros(sz)

		i = 0
		for p in self.G.nodes():
			if (self.G.nodes[p]['cost'] >= 0): 		# TODO: change to minimum permissible depth
				# Append to array walls and ground
				cx[i] = p[0]*self.resolution
				cy[i] = p[1]*self.resolution
				cz[i] = self.G.nodes[p]['cost']*0.9 	# cost corresponds to height
				# stack cells vertically until height of cell
				for n in range(0,int(cz[i]/self.resolution)):
					cx = np.append(cx,cx[i])
					cy = np.append(cy,cy[i])
					cz = np.append(cz,self.resolution*n)
			else:
				# Ignore holes, leave them blank
				pass
			i += 1

		return cx, cy, cz

	def path_to_nparray(self, path, p_start):
		""" converts grid based path to cartesian path """

		## Define Variables ##
		x_cob = np.zeros((len(path),3))
		w_cob = np.zeros((len(path),3))

		i = 0
		for p in path:
			# Position
			x_cob[i][0] = (p[0] - p_start[0])*self.resolution
			x_cob[i][1] = (p[1] - p_start[1])*self.resolution
			x_cob[i][2] = 0.
			# Orientation
			w_cob[i][0] = self.Gbody.nodes[p]['pose'][1]
			w_cob[i][1] = self.Gbody.nodes[p]['pose'][2]
			w_cob[i][2] = self.Gbody.nodes[p]['pose'][3]

			i += 1

		return x_cob, w_cob

	def list_to_nparray(self, qlist):
		""" converts from list to 2D array """

		sz = len(qlist)

	def get_cell(self, info, p, j=0):
		""" Returns cell characteristic at point p (in m) """
		
		# Convert cell location to grid format
		if (j < 3):
			grid_p = (int((p[0]/self.resolution)), int((p[1]/self.resolution)))
		else:
			grid_p = (int(np.ceil(p[0]/self.resolution)), int(np.ceil(p[1]/self.resolution)))
			# print 'snorm ', grid_p
		# if (info == 'cost'):
		# 	print 'Location: ', p, grid_p
		# Find surface normal from map
		try:
			return self.G.nodes[grid_p][info]
		except:
			print 'Cell Invalid at ', grid_p, info, j
			return None


## ================================================================================================ ##
## 												TESTING 											##
## ================================================================================================ ##

# ## Create map and plan path
# pp = GridPlanner('wall_demo')
# pp = GridPlanner('simple') 		# select pre-defined map or set map size, [x,y] (m,m)
# gpath = pp.find_base_path((17, 17),(17,27))	# define start and end points
# npath = pp.post_process_path(gpath)		# post-processed path
# pp.graph_representation(False, gpath)

# pp = GridPlanner((1.27,5.48))
# pp.advance_capable = False
# # bpath = pp.find_base_path((15, 10),(10,150))			# define start and end points (20,125) 	(20,60)
# # pp.graph_representation(True,bpath)

# plt.show()
# gmap = GridPlanner((0.1,0.1))
# snorm_1 = gmap.get_cell_snorm(np.array([0.,1.,0.]))
# snorm_2 = gmap.get_cell_snorm(np.array([0.,2.,0.]))
# avg_norm= (snorm_1 + snorm_2)/2
