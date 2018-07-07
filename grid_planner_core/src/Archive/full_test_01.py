#!/usr/bin/env python

## Grid based motion planning
## High resolution grid cells - 0.03m^2

import os
import sys
sys.path.insert(0, '/home/wilson/catkin_ws/src/corin/mcorin_control/py_script/library')
sys.dont_write_bytecode = True

import networkx as nx
import matplotlib.pyplot as plt

import numpy as np
import math
from constant import *
import transforms
import gait_class

from TrajectoryPoints import TrajectoryPoints
import path_generator
from fractions import Fraction
import plotgraph as pltg

import csv

## Global variables ##
POSE_TABLE = {}

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

def compute_wall_bodypose(POSE_TABLE,fwidth):
	""" calculates robot bodypose based on footprint required """
	selection = -1
	for i in POSE_TABLE:
		if (POSE_TABLE[i]['footprint']<=fwidth):
			selection = i
			break
		else:
			selection = i
	return selection

def bodypose_table():
	""" computes bodypose lookup table for footprint sizing """

	## define variables
	bh  = 0.1	# base height (m)
	bhm = 0.42	# max. base height
	qr  = 0.0	# base roll (rad)
	ns  = 50 	# samping size of table - above this resolution no further observable change 
	POSE_TABLE = {}

	for i in range(0,ns+1):
		
		qr = i*np.pi/(2*ns) 		# update base roll
		bn = bh + i*(bhm-bh)/ns 	# update base height
		# Transform from world to leg frame (both located at leg frame)
		world_X_leg = np.array([ [np.cos(-qr), -np.sin(-qr)],[np.sin(-qr), np.cos(-qr)] ])
		leg_X_world = np.array([ [ np.cos(qr),  -np.sin(qr)],[ np.sin(qr),  np.cos(qr)] ])

		# leg on ground contact - distances in world frame
		gch = bn - (COXA_Y+L1)*np.sin(qr)			# height from coxa joint towards ground
		gcx = np.sqrt(abs((L2**2 - (gch - L3)**2)))	# distance from coxa joint towards ground
		gpW = np.array([[gcx],[gch]])				# foot position in located at coxa joint

		gpL = np.dot(leg_X_world,gpW)				# foot position in joint frame located at coxa joint
		
		# leg on wall contact - distances in world frame
		wch = 0
		wcx = (ns-i)*(STANCE_WIDTH-0.1)/ns + 0.1
		wpW = np.array([[wcx],[wch]])				# foot position in located at coxa joint
		
		wpL = np.dot(leg_X_world,wpW)				# foot position in joint frame located at coxa joint
		
		## robot footprint
		wR = ((COXA_Y+L1)*np.cos(qr)+gcx) + (COXA_Y*np.cos(qr)+wcx)
		## write to dictionary
		POSE_TABLE[i] = {'footprint':wR, "bodypose":np.array([0.,0.,bn,qr,0.,0.]),"ground":np.array([gcx,0.,-gch]),"wall":np.array([wcx,0.,-wch])}
		# print np.round([np.round(wR/0.03),wR,bn,qr*180/np.pi],3)
	return POSE_TABLE

# def append_to_spline()
class grid_planner:
	def __init__(self,size):
		self.cell_size 	= 0.03			# size of cell, (m) - TARGET: 0.03
		# self.leg_size  	= 0.18			# area of foothold per leg, (m^2) 	- TAKEN FROM CONSTANT*
		self.map_size  	= size			# map size, (m^2)
		self.rbfp_size  = (0.488, 0.5) 	# robot footprint size, (m,m) - TARGET: (0.488,0.58)
		self.rbbd_size  = (0.27, 0.18) 	# robot body size, (m,m) - 

		self.initialise_graph()

	def initialise_graph(self):
		# determine robot related size in grid cell
		self.rbfp_gd = tuple(map(lambda x: int(math.ceil(x/self.cell_size)), self.rbfp_size))					# footprint
		self.rbbd_gd = tuple(map(lambda x: int(math.ceil(x/self.cell_size)), self.rbbd_size))					# body
		self.rblg_gd = (int(math.ceil(LEG_AREA_LX/self.cell_size)),int(math.ceil(LEG_AREA_LY/self.cell_size)))	# leg

		# determine number of grids
		gridx = int(math.ceil(self.map_size[0]/self.cell_size))
		gridy = int(math.ceil(self.map_size[1]/self.cell_size))

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
		self.moore_edge(self.G)
		self.moore_edge(self.Gbody)

		# set attribute for nodes
		nx.set_node_attributes(self.G, {e: 0 for e in self.G.nodes()}, 'cost') 			# obstacle identifier: 0 = ground, 1 = wall, -1 = ground
		nx.set_node_attributes(self.Gbody, {e: 0 for e in self.Gbody.nodes()}, 'cost')		#
		nx.set_node_attributes(self.Gbody, {e: 0 for e in self.Gbody.nodes()}, 'motion') 	# motion identifier: 0 = walk,1 = chimney,2 = wall
		nx.set_node_attributes(self.Gbody, {e: 0 for e in self.Gbody.nodes()}, 'width') 	# footprint lateral width
		nx.set_node_attributes(self.Gbody, {e: [0.,0.,0.,0.] for e in self.Gbody.nodes()}, 'pose')	# robot bodypose - height, roll, pitch, yaw

		print 'Initialised - '
		print 'Map Grid  : ', gridx, ' by ', gridy
		print 'Robot grid: ', self.rbfp_gd
		print 'Body grid : ', self.rbbd_gd
		print 'Foot grid : ', self.rblg_gd
		print 'Resolution: ', self.cell_size, 'm^2'
		print '==============================================='

	def moore_edge(self, G):
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

	def obstacle_area(self, box, obstacle):
		""" sets the area to be obstacles (wall or hole) """

		dx = box[1][0] - box[0][0]
		dy = box[1][1] - box[0][1]

		if (obstacle=='wall'): 
			cost = 1;
		elif (obstacle=='hole'):
			cost = -1

		for x in range(0,dx+1):
			for y in range(0,dy+1):
				p = ( (box[0][0]+x), (box[0][1]+y))
				self.G.nodes[p]['cost'] = cost

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

	def map_cell_cost(self):
		""" Set obstacle area and cost """

		## Obstacle boxes - bottom left and upper right [TODO:set start point, size in (m)]
		#  Chimney demonstration
		wall_01 = [ (0,0),(20,60)]
		wall_02 = [(25,20),(43,45)] 	# walling: [(37,25),(43,50)] 	chimney: [(40,25),(43,50)]

		wall_03 = [(0,80),(10,110)]
		wall_04 = [(30,80),(39,110)]

		hole_01 = [(40,80),(60,110)] 	#[(21,30),(39,40)]
		hole_02 = [(11,85),(29,105)]

		#  Wall walking demonstration
		# wall_01 = [ (0,25),(25,50)]
		# wall_02 = [(40,25),(60,50)]
		wall_bar = [(21,30),(24,30)]
		self.obstacle_area(wall_bar, 'wall')

		self.obstacle_area(wall_01, 'wall')
		self.obstacle_area(wall_02, 'wall')
		# self.obstacle_area(wall_03, 'wall')
		# self.obstacle_area(wall_04, 'wall')

		# self.obstacle_area(hole_01, 'hole')
		# self.obstacle_area(hole_02, 'hole')
		# self.G.nodes[(32,19)]['cost'] = 1
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

		print 'Init, final: ', e_init, '  ', G.number_of_edges()

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
			G.edges[e]['cost'] = cost_dist + cost_motion + cost_d_bh + cost_d_qr + cost_d_qp + cost_d_qy #+ cost_d_lgw*0.01

		nx.set_edge_attributes(self.G, 1, 'cost') 	# set cost based on vertical distance to next cell

	def check_body_area(self, p, body_area):
		""" check body area for collision - returns True or False """
		## variables ##
		valid = False
		body_area = (8,3)
		# skip if centre occupied
		if (self.G.nodes[p]['cost'] != 1):
			bx = int(math.floor(body_area[0])/2) # longitudinal distance to check
			by = int(math.floor(body_area[1])/2) # lateral distance to ccheck

			## check entire body area
			for y in range(0,by+1): 				# cycle lateral - inflated if body_area[1] is even
				for x in range(0,body_area[0]):		# cycle longitudinal
					# if (p==(13,21)):
					# 	print (p[0]-y,p[1]-bx+x), (p[0]+y,p[1]-bx+x) 
					EL_edge = self.G.nodes[(p[0]-y,p[1]-bx+x)]['cost']		# left edge
					ER_edge = self.G.nodes[(p[0]+y,p[1]-bx+x)]['cost']		# right edge
					# check if collision exists
					if (EL_edge==1 or ER_edge==1):
						valid = False
						break 			# exit immediately as area invalid
					else:
						valid = True
		# if (p==(13,20)):
		# 	print p, valid
		return valid

	def check_walking_leg_area(self, p):
		scost = 0 	# total cost
		
		bx = (self.rblg_gd[0]-1)/2  	# longitudinal distance from robot centre
		by = (self.rblg_gd[1]-1)/2 		# lateral distance from robot centre
		
		leg_valid  = np.zeros(6)		# flag array if foothold area valid
		valid_flag = True
		foothold_exist = False

		## Check if valid foothold exist in each leg region, outputs boolean
		for i in range(0,6):
			# set foot and transform (x,y) (m) - CHANGE TO USE XH
			px 	  = int(np.round( (np.cos(np.pi/2)*(p_leg[i][0])-np.sin(np.pi/2)*(p_leg[i][1]))/self.cell_size))
			py 	  = int(np.round( (np.sin(np.pi/2)*(p_leg[i][0])+np.cos(np.pi/2)*(p_leg[i][1]))/self.cell_size))
			pbnom = (px,py)

			pwnom = tuple(map(lambda x,y: y+x, p, pbnom))	# leg nominal stance in world frame - equal for left and right side
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
						## foothold in leg grid area - start top to bottom, inside to outside
						if (i<3):
							## LHS
							pwset = (pwnom[0]+by-cy,pwnom[1]+bx-cx) 
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
						except:
							pass

					# if (p == (13,17) and i==0):
					# 	print i, pbnom, pwnom, pwset, fcost

			## calculate average of area and add to total
			area  = self.rblg_gd[0]*self.rblg_gd[1]
			ncost = math.floor( fcost/float(area) )
			scost = scost + ncost 

			## set flag for foothold area
			if (foothold_exist == True):
				leg_valid[i] = 1
			else:
				leg_valid[i] = 0
				valid_flag = False
		
		return int(scost), valid_flag, 0.6 # last item is width - HARDCODED

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
					footholds_cost, valid_flag = self.check_walking_leg_area(e)

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

	def find_foothold(self, p):
		## variables
		gait = gait_class.GaitClass(1) 	# gait class - wave
		BASE_X_FOOT = [] 	# array of gait phased foot position for each leg
		LF_foothold = [] 	# list for foothold nodes
		LM_foothold = [] 	# list for foothold nodes
		LR_foothold = [] 	# list for foothold nodes
		RF_foothold = [] 	# list for foothold nodes
		RM_foothold = [] 	# list for foothold nodes
		RR_foothold = [] 	# list for foothold nodes

		## edge distance for leg area
		bx = (self.rblg_gd[0]-1)/2	# heading longitudinal distance
		by = (self.rblg_gd[1]-1)/2	# heading lateral distance

		## gait parameters
		STEP_STROKE = 0.08
		d_cycle = STEP_STROKE/gait.gdic['beta'] 			# distance travelled per gait cycle, (m)
		d_cell  = int(np.round(d_cycle/self.cell_size)) 	# distance travelled per gait cycle, (number of cells)
		# d_phase = d_cycle/gait.gdic['beta'].denominator 	# distance travelled per gait phase
		
		## Aim: set robot stance to start of gait
		div = np.round(STEP_STROKE/gait.gdic['beta'].numerator,3)
		EP  = STEP_STROKE/2

		# cycle through each gait phase
		for i in range(0,gait.gdic['beta'].denominator):
			# cycle through each leg
			for j in range(0,6):
				# set position based on gait phase
				if (gait.cs[j]==1):
					# offset foot to starting position
					p_leg[j] = (np.round(p_leg[j][0]-EP+i*div,3), p_leg[j][1])

					# compute gait phased stance based on grid cells
					px = int(np.round( (np.cos(np.pi/2)*(p_leg[i][0])-np.sin(np.pi/2)*(p_leg[i][1]))/self.cell_size))
					py = int(np.round( (np.sin(np.pi/2)*(p_leg[i][0])+np.cos(np.pi/2)*(p_leg[i][1]))/self.cell_size))
					BASE_X_FOOT.append((px,py))

			gait.change_phase() 	# change to next gait phase
		
		## Aim: find footholds for generated base path
		ncount = 0 			# counter position for path
		while (ncount<len(p)):

			# cycle through all legs at each stance
			for leg in range(0,6):
				# new foothold after displaced by d in world frame - e[ncount is base position]
				new_foothold = tuple(map(lambda x,y: y+x, p[ncount], BASE_X_FOOT[leg]))

				# search neighbouring cells within area
				try:
					if (self.G.nodes[new_foothold]['cost']!=0):
						# generate list of footholds in area based on foothold grid area
						foothold_list = cw_spiral_left(new_foothold,self.rblg_gd)
						
						# cycle through list of footholds
						for i in range(0,len(foothold_list)):
							if (self.G.nodes[foothold_list[i]]['cost']==0):
								new_foothold = foothold_list[i] 		# set new foothold
								break 									# exit from loop when foothold found
							
						if (i == len(foothold_list)-1):
							print 'Foothold Planner: Failed - No valid foothold exist!'
				except KeyError:
					pass
				# stack to path
				if   (leg==0):	LF_foothold.append(new_foothold)
				elif (leg==1):	LM_foothold.append(new_foothold)
				elif (leg==2):	LR_foothold.append(new_foothold)
				elif (leg==3):	RF_foothold.append(new_foothold)
				elif (leg==4):	RM_foothold.append(new_foothold)
				elif (leg==5):	RR_foothold.append(new_foothold)

			# move forward by d (in number of cells) 
			ncount += d_cell
		
		return (nx.path_graph(LF_foothold), nx.path_graph(LM_foothold), nx.path_graph(LR_foothold), nx.path_graph(RF_foothold),
			nx.path_graph(RM_foothold),nx.path_graph(RR_foothold))

	def check_walling_leg_area(self, p):
		## variables ##
		scost = 0 						# total cost
		width = 0 						# footprint width (cells)
		i_edge = 0 						# internal edge
		EL_edge = False					# external left edge
		ER_edge = False					# external right edge
		body_area  = self.rbbd_gd 		# body area (grid cells)
		# leg_valid  = np.zeros(6)		# flag array if foothold area valid
		valid_flag = False 				# boolen flag if area is permissible
		MIN_WALLING_AREA = (0.3,0.20)	# minimum footprint area for wall walking - based on kinematic and torque study
		MAX_WALLING_AREA = (0.3,0.57) 	# maximum footprint area for wall walking - based on kinematic and torque study

		bx_min = int((MIN_WALLING_AREA[0]/self.cell_size-1)/2)
		by_min = int((MIN_WALLING_AREA[1]/self.cell_size-1)/2)
		
		bx_max = int((MAX_WALLING_AREA[0]/self.cell_size-1)/2)
		by_max = int((MAX_WALLING_AREA[1]/self.cell_size-1)/2)

		# print 'wall: ', bx_min*2, by_min*2, bx_max*2, by_max*2

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
					# checks all the space between the inner and outer boundaries
					for x in range(0,bx_min*2):
						try:
							# edges move from inside to outside
							LHS_edge_1 = self.G.nodes[(p[0]-by_min-d,p[1]-bx_min+x)]['cost']
							RHS_edge_1 = self.G.nodes[(p[0]+by_min+d,p[1]-bx_min+x)]['cost']
							LHS_edge_2 = self.G.nodes[(p[0]-by_min-d+1,p[1]+bx_min-x)]['cost']	# LHS inner neighbour cell
							RHS_edge_2 = self.G.nodes[(p[0]+by_min+d-1,p[1]+bx_min-x)]['cost']	# RHS inner neighbour cell
							
							if (LHS_edge_1== -1 or RHS_edge_1==-1):
								# reset flags and break as statically unfeasible
								EL_edge		= False
								ER_edge 	= False
								valid_flag 	= False
								break
							elif (LHS_edge_1 == 1 and LHS_edge_2 != 1 and RHS_edge_1 == 0 ):
								# LHS wall walking
								EL_edge    = True
								valid_flag = True
							elif (LHS_edge_1 == 0 and RHS_edge_1 == 1 and RHS_edge_2 != 1):
								# RHS wall walking
								ER_edge    = True
								valid_flag = True

							# if (p==(22,37)):
							# 	print p,(p[0]-by_min-d,p[1]+bx_min-x),(p[0]+by_min+d,p[1]+bx_min-x), LHS_edge_1, RHS_edge_1, EL_edge, ER_edge
						except:
							pass

					# Exit loop if valid configuration exists
					if (valid_flag==True):
						width = (p[0]+by_min+d) - (p[0]-by_min-d) 	# footprint width (cells)
						break
				# Exit search as internal boundary violated
				else:
					break
		else: 
			scost = 1
		# if (p==(27,21)):
		# 	print 'this: ', p, valid_flag, i_edge, EL_edge, ER_edge
		return int(scost), valid_flag, width

	def check_chimney_leg_area(self, p):

		## variables ##
		scost = 0 						# total cost
		width = 0						# footprint width (cells)
		i_edge = 0 						# internal edge
		EL_edge = 0 					# external left edge
		ER_edge = 0 					# external right edge
		body_area  = self.rbbd_gd 		# body area (grid cells)
		# leg_valid  = np.zeros(6)		# flag array if foothold area valid
		valid_flag = False 				# boolen flag if area is permissible
		MIN_CHIMNEY_AREA = (0.3,0.6)	# minimum footprint area for chimney - based on kinematic and torque study
		MAX_CHIMNEY_AREA = (0.3,0.72) 	# maximum footprint area for chimney - based on kinematic and torque study

		bx_min = int((MIN_CHIMNEY_AREA[0]/self.cell_size-1)/2)
		by_min = int((MIN_CHIMNEY_AREA[1]/self.cell_size-1)/2)
		
		bx_max = int((MAX_CHIMNEY_AREA[0]/self.cell_size-1)/2)
		by_max = int((MAX_CHIMNEY_AREA[1]/self.cell_size-1)/2)

		# print 'chim: ', bx_min*2, by_min*2, bx_max*2, by_max*2

		# skip if centre occupied
		if (self.G.nodes[p]['cost'] != 1):
			i_edge = 0 	# internal edge
			## cycle through all four edges TODO: remove overlap on points
			for x in range(0,bx_min*2):
				# check left and right edge
				try:
					if (self.G.nodes[(p[0]-by_min,p[1]-bx_min+x)]['cost']==1 or self.G.nodes[(p[0]+by_min,p[1]-bx_min+x)]['cost']==1):
						i_edge += 1
				except:
					pass # ignores cells outside of map
				
			for y in range(0,by_min*2):
				try:
					# check top and bottom edge
					if (self.G.nodes[(p[0]-by_min+y,p[1]-bx_min)]['cost']== 1 or self.G.nodes[(p[0]-by_min+y,p[1]+bx_min)]['cost']==1):
						i_edge += 1
				except:
					pass # ignores cells outside of map

			# continues to check the WHOLE middle space of boundary only if internal boundary valid
			if (i_edge == 0):
				## cycle through all left & right columns				
				for y in range(0,by_max-by_min+1): 	# lateral width to cycle through
					EL_edge = 0
					ER_edge = 0
					for x in range(0,bx_max*2): 	# cycle along lateral edge
						try:
							if (self.G.nodes[(p[0]-by_min-y,p[1]-bx_max+x)]['cost']==1): 	# left edge
								EL_edge += 1
							if (self.G.nodes[(p[0]+by_min+y,p[1]-bx_max+x)]['cost']==1): 	# right edge
								ER_edge += 1
							# if (p==(30,30)):
							# 	print (p[0]-by_max+y,p[1]-bx_max+x), (p[0]+by_max+y,p[1]-bx_max+x), EL_edge, ER_edge
						except:
							pass 	# ignores cells outside of map
						# if (p==(30,24)):
						# 	print p, (p[0]-by_min-y,p[1]-bx_max+x), (p[0]+by_min+y,p[1]-bx_max+x), EL_edge, ER_edge
					if (EL_edge > 1 and ER_edge > 1):
						valid_flag = True
					if (valid_flag):
						width = (p[0]+by_min+y) - (p[0]-by_min-y)
						# print width
						break
				## Add break condition
			else:
				valid_flag = False
		else: 
			scost = 1
		
		return int(scost), valid_flag, width

	def check_motion(self, p):
		""" checks if cell is accessible using motion primitives, 
			start with walking then walling then chimney 			"""

		## variables ##
		valid_flag  = False
		motion_cost = 0		# motion cost: 0 = walk,1 = chimney,2 = wall
		
		## Walking ##
		#  condition: valid foothold exists within foothold area 
		footholds_cost, valid_flag, width = self.check_walking_leg_area(p)
		if (valid_flag):
			motion_cost = 0
			self.GM_walk.add_node(p)		# ILLUSTRATION: stack into graph

		## Chimney ##
		if (not valid_flag):
			footholds_cost, valid_flag, width = self.check_chimney_leg_area(p)
			if (valid_flag):
				motion_cost = 1
				self.GM_chim.add_node(p)	# ILLUSTRATION: stack into graph

		## Walling ##
		if (not valid_flag):
			# condition: 1) wall within X.X m from base point, 2) ground is present
			footholds_cost, valid_flag, width = self.check_walling_leg_area(p)
			if (valid_flag):
				motion_cost = 2
				self.GM_wall.add_node(p)	# ILLUSTRATION: stack into graph

		return motion_cost, valid_flag, width

	def primitive_search_path(self):
		""" First find the shortest path using minimum footprint, then fit motion primitives """

		## Variables ##
		ax = (self.rbfp_gd[0]-1)/2 		# robot longitudinal distance (grid cell)
		ay = (self.rbfp_gd[1]-1)/2-5	# robot lateral distance (grid cell) - HARDCODED
		POSE_TABLE  = bodypose_table()	# lookup table for walling bodypose
		print 'Body Edges: ', ax, ' ', ay
		
		node_ignored = [] 	# list for nodes to ignore: either near map border or intersect obstacles/holes
		node_free 	 = [] 	# list for free nodes

		for e in self.G.nodes():
			# ignore edges
			if (e[0]<=(ay-1) or e[0]>=(self.mp_sz[0]-ay) or e[1]<=(ax-1) or e[1]>=(self.mp_sz[1]-ax)): 	
				node_ignored.append(e)
				self.Gbody.nodes[e]['cost'] = 1
			else:
				# check body collision TODO: automate this to a square dependant on resolution 
				valid = self.check_body_area(e,(ax,ay))
				# if (e==(39,21)):
				# 	print e, body_cost
				# compute for nodes that are obstacle free
				if (valid):
					motion_cost, valid_flag, width = self.check_motion(e)
					
					if (valid_flag == True):
						if (motion_cost==2):
							itb = compute_wall_bodypose(POSE_TABLE,width*self.cell_size) 	# returns lookup table index
							bh 	= POSE_TABLE[itb]['bodypose'][2]
							qr 	= POSE_TABLE[itb]['bodypose'][3]
						else:
							bh = BODY_HEIGHT
							qr = 0.

						node_free.append(e)
						self.Gbody.nodes[e]['cost']   = 0
						self.Gbody.nodes[e]['motion'] = motion_cost
						self.Gbody.nodes[e]['width']  = width
						self.Gbody.nodes[e]['pose']   = [bh,qr,0.,0.]
					else:
						self.Gbody.nodes[e]['cost'] = 1
				else:
					self.Gbody.nodes[e]['cost'] = 1
			# if (e==(30,31)):
			# 	print e, valid_flag 
		self.Gfree = nx.path_graph(node_free)

	def wall_transition(self,p1,p2):
		""" motion routine for wall transition """

		## define variables ##
		qr_per_gait = 15.*np.pi/180 	# change in roll angle per gait cycle
		delta_q 	= 0.				# change in roll between p1 and p2
		delta_h 	= 0.				# change in roll between p1 and p2
		n_cycle 	= 0.				# number of gait cycles required

		## calculate changes in roll & body height and number of gait cycles to achieve it
		delta_q	= self.Gbody.nodes[p2]['pose'][1] - self.Gbody.nodes[p1]['pose'][1]
		delta_h	= self.Gbody.nodes[p2]['pose'][0] - self.Gbody.nodes[p1]['pose'][0]
		n_cycle	= int(np.ceil(abs(delta_q/qr_per_gait)))
		print 'delta q: ', delta_q, delta_h, n_cycle

		## interpolate
		return n_cycle

	def transition_routine(self, ttype, p1, p2):

		## define variables ##
		x_inst 	= TrajectoryPoints()
		w_inst 	= TrajectoryPoints()
		pgen 	= path_generator.PathGenerator()
		tspan 	= TRAC_PERIOD*pgen.gait.gdic['beta'].denominator 	# time for one gait cycle

		x_i = np.array([p1[0]*self.cell_size, p1[1]*self.cell_size, self.Gbody.nodes[p1]['pose'][0]])
		# print 'xi: ', x_i
		if (ttype=='Gnd_X_Wall'):
			print 'ground to wall'
			x_inst, w_inst = self.spline_interpolation([p1,p2])
			
		elif (ttype=='Wall_X_Gnd'):
			print 'wall to ground'
			# n_cycle = self.wall_transition(p1, p2)
			# x_inst, w_inst = self.spline_interpolation([p1,p2],np.array([0.,tspan]))
			x_inst, w_inst = self.spline_interpolation([p1,p2])

		elif (ttype=='Gnd_X_Chim'):
			""" change from ground to chimney wall support footholds """
			""" one gait cycle of stationary x_cob and w_cob """
			print 'ground to chimney'
			x_inst, w_inst = self.spline_interpolation([p1,p2],np.array([0.,tspan]))

		elif (ttype=='Chim_X_Gnd'):
			""" change from chimney wall to ground support footholds """
			""" one gait cycle of stationary x_cob and w_cob """
			print 'chimney to ground'
			x_inst, w_inst = self.spline_interpolation([p1,p2],np.array([0.,tspan]))
			
		return x_inst, w_inst

	def spline_interpolation(self,path, tint=None):
		""" interpolate cells using cubic spline """

		## define variables ##
		x_com = np.array([.0,.0,.0])
		w_com = np.array([.0,.0,.0])

		## populate array
		for e in path:
			x_com = np.vstack((x_com,np.array([e[0]*self.cell_size, e[1]*self.cell_size, self.Gbody.nodes[e]['pose'][0]])))
			w_com = np.vstack((w_com,np.array([self.Gbody.nodes[e]['pose'][1:4]])))
			# print e, (np.round(e[0]*self.cell_size,3), np.round(e[1]*self.cell_size,3))
		
		x_com = np.delete(x_com,0,0)
		w_com = np.delete(w_com,0,0)
		# print w_com
		pgen = path_generator.PathGenerator()
		pgen.gait = {	'name': "wave",
						'beta': Fraction(5, 6),
						'dphase': Fraction(1, 5),
						'phase':np.matrix([ Fraction(6, 6), Fraction(5, 6), Fraction(4, 6), Fraction(3, 6), Fraction(2, 6), Fraction(1, 6) ])}
		
		if (tint is not None):
		    xn_com, wn_com = pgen.velocity_method(x_com, w_com, tint)
		else:
		    xn_com, wn_com = pgen.velocity_method(x_com, w_com)

		# pltg.plot_2d_multiple(1,wn_com.t,wn_com.xp*180/np.pi)
		# pltg.plot_2d_multiple(1,xn_com.t,xn_com.xp)
		return xn_com, wn_com
	
	def eucld_dist(self,a,b):
		""" heuristics used for A* planner """
		(x1, y1) = a
		(x2, y2) = b
		return (((x1 - x2)**2 + (y1 - y2)**2)**0.5)

	def find_path(self,start,end):
		self.map_cell_cost()				# assign cost based on free=0 or obstacle=1
		# self.edge_removal(self.G)		# remove edges linked to obstacles
		# self.footprint_cost()			# assign cost based on footprint of robot
		self.primitive_search_path() 	# finds path then use motion primitives to fit in
		self.edge_removal(self.Gbody) 	# remove edges linked to invalid cells
		self.edge_cost(self.Gbody)		# assign cost to edge

		## Robot represented with a number of nodes
		try:
			# list_path	= nx.shortest_path(self.Gbody,start,end,weight='cost')
			# list_path	= nx.dijkstra_path(self.Gbody,start,end,weight='cost')
			list_path 	= nx.astar_path(self.Gbody,start,end,heuristic=self.eucld_dist,weight='cost')
			self.Gpath 	= nx.path_graph(list_path)
			# self.spline = self.spline_interpolation(list_path)
			print list_path
			self.G_LF, self.G_LM, self.G_LR, self.G_RF, self.G_RM, self.G_RR = self.find_foothold(list_path)
			print 'Path exist!'
			# print type(list_path)
			# print len(list_path)
			# self.post_process_path(list_path)

		except nx.exception.NetworkXNoPath:
			print 'ERROR, no body path exist!'

	def post_process_path(self,path):
		""" Introduce transition routines into body spline """

		## declare variables ##
		cpath = []
		x_cob = TrajectoryPoints()
		w_cob = TrajectoryPoints()
		m = np.zeros(3)
		## cycle through path
		for i in range(0,len(path)-1):
			# print i, path[i]
			cpath.append(path[i])
			## check transition
			m[0] = self.Gbody.nodes[path[i]]['motion']
			m[1] = self.Gbody.nodes[path[i+1]]['motion']
			try:
				m[2] = self.Gbody.nodes[path[i+2]]['motion']
			except:
				pass

			if (m[0]==0 and m[1]==2):
				## Walk to Wall ##
				# print path[i-1], path[i], 'walk to wall'
				x_inst, w_inst = self.spline_interpolation(cpath)
				x_tran, w_tran = self.transition_routine('Gnd_X_Wall',path[i],path[i+1])
				
				x_cob.append(x_inst); x_cob.append(x_tran)
				w_cob.append(w_inst); w_cob.append(w_tran)
				cpath = []

			elif (m[0]==2 and m[1]==0):
				## Wall to Walk ##
				# print path[i-1], path[i], 'wall to walk'
				if (m[2]==2):
					# force motion to walling for gap of one
					self.Gbody.nodes[path[i+1]]['motion'] = 1
					self.GM_chim.add_node(path[i+1])
					cpath.append(path[i])
				else:
					x_inst, w_inst = self.spline_interpolation(cpath)
					x_tran, w_tran = self.transition_routine('Wall_X_Gnd',path[i],path[i+1])
					
					x_cob.append(x_inst); x_cob.append(x_tran)
					w_cob.append(w_inst); w_cob.append(w_tran)
					cpath = []

			elif (m[0]==0 and m[1]==1):
				## Walk to Chimney ##
				# print path[i], path[i+1], 'walk to chimney'
				x_inst, w_inst = self.spline_interpolation(cpath)
				# TODO: plan foothold for above
				x_tran, w_tran = self.transition_routine('Gnd_X_Chim',path[i],path[i+1])
				# TODO: plan foothold for above

				x_cob.append(x_inst); x_cob.append(x_tran)
				w_cob.append(w_inst); w_cob.append(w_tran)
				# Append footholds:
				# append(cpath, shuffle, transition)
				cpath = []

			elif (m[0]==1 and m[1]==0):
				## Chimney to Walk ##
				# print path[i-1], path[i], 'chimney to walk'
				if (m[2]==1):
					# force motion to chimney for gap of one
					self.Gbody.nodes[path[i+1]]['motion'] = 1
					self.GM_chim.add_node(path[i+1])
					cpath.append(path[i])
				else:
					x_inst, w_inst = self.spline_interpolation(cpath)
					# TODO: plan foothold for above
					x_tran, w_tran = self.transition_routine('Chim_X_Gnd',path[i],path[i+1])
					# TODO: plan foothold for above

					x_cob.append(x_inst); x_cob.append(x_tran)
					w_cob.append(w_inst); w_cob.append(w_tran)
					# Append footholds:
					# append(cpath, shuffle, transition)
					cpath = []

		# interpolate final sub-division
		x_inst, w_inst = self.spline_interpolation(cpath)
		x_cob.append(x_inst);
		w_cob.append(w_inst);

		## generate path without segmenting them
		# x_inst, w_inst = self.spline_interpolation(path)
		# x_cob = x_inst 
		# w_cob = w_inst 

		## update spline time interval
		tspan = len(x_cob.t)
		for i in range(0,tspan):
			x_cob.t[i] = i*TRAC_INTERVAL
			w_cob.t[i] = i*TRAC_INTERVAL

		# pltg.plot_2d_multiple(1,w_cob.t,w_cob.xv*180/np.pi)
		# pltg.plot_2d_multiple(1,x_cob.t,x_cob.xp)

	def graph_representation(self):
		""" Plots graph functions """
		# dictionary of node names->positions
		pos    = dict(zip(self.G, self.G)) 			
		p_fin  = dict(zip(self.Gpath, self.Gpath))
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

		# Grid map
		ax = nx.draw_networkx(self.Gbody,pos,with_labels=labels,node_size=nsize,node_color='0.8',width=1.0,node_shape="s");
		# Body path
		nx.draw_networkx(self.Gfree,p_body,with_labels=labels,node_size=25,node_color='y',edge_color='r',width=0.0,node_shape="s");
		# Wall
		nx.draw_networkx(self.G_wall,p_wall,with_labels=labels,node_size=10,node_color='r',width=0.0,node_shape="s");
		# Holes 
		nx.draw_networkx(self.G_hole,p_hole,with_labels=labels,node_size=10,node_color='tab:brown',width=0.0,node_shape="s");
		# Motion primitives
		nx.draw_networkx(self.GM_walk,m_walk,with_labels=labels,node_size=15,node_color='tab:orange',width=0.0,node_shape="d");
		nx.draw_networkx(self.GM_wall,m_wall,with_labels=labels,node_size=6,node_color='tab:cyan',width=0.0,node_shape="d");
		nx.draw_networkx(self.GM_chim,m_chim,with_labels=labels,node_size=4,node_color='red',width=0.0,node_shape="d"); 	# dark grey

		# Feasible path - single point
		nx.draw_networkx(self.Gpath,p_fin,with_labels=labels,node_size=0,node_color='g',edge_color='b',width=5.0);

		# Footholds
		# nx.draw_networkx(self.G_LF,p_LF,with_labels=labels,node_size=4,node_color='b',edge_color='b',width=0.5);
		# nx.draw_networkx(self.G_LM,p_LM,with_labels=labels,node_size=4,node_color='b',edge_color='b',width=0.0);
		# nx.draw_networkx(self.G_LR,p_LR,with_labels=labels,node_size=4,node_color='b',edge_color='b',width=0.0);
		# nx.draw_networkx(self.G_RF,p_RF,with_labels=labels,node_size=4,node_color='r',edge_color='r',width=0.5);
		# nx.draw_networkx(self.G_RM,p_RM,with_labels=labels,node_size=4,node_color='b',edge_color='b',width=0.0);
		# nx.draw_networkx(self.G_RR,p_RR,with_labels=labels,node_size=4,node_color='b',edge_color='b',width=0.0);

		## enable major and minor grid
		plt.grid('on');		 plt.grid(which='major', linestyle=':', linewidth='0.5', color='black')
		plt.minorticks_on(); plt.grid(which='minor', linestyle=':', linewidth='0.25', color='black')

		# minor_ticks = np.arange(0, 9+1, 0.5)
		# plt.xticks(minor_ticks)

		plt.xlabel('x, [1]')
		plt.ylabel('y, [1]')

		# maximise plot window
		fig_manager = plt.get_current_fig_manager()
		fig_manager.resize(*fig_manager.window.maxsize())
		
		plt.show()

## ================================================================================================ ##
## 												TESTING 											##
## ================================================================================================ ##
## Foot position from base
p_base_X_lf_foot = ( 0.244, 0.24)
p_base_X_lm_foot = (    0., 0.29)
p_base_X_lr_foot = (-0.244, 0.24)
p_base_X_rf_foot = ( 0.244,-0.24)
p_base_X_rm_foot = (    0.,-0.29)
p_base_X_rr_foot = (-0.244,-0.24)

XH = transforms.HomogeneousTransform()

p_leg = [p_base_X_lf_foot,p_base_X_lm_foot,p_base_X_lr_foot,p_base_X_rf_foot,p_base_X_rm_foot,p_base_X_rr_foot]

## Create map and plan path
pp = grid_planner((1.8,2.2)) 			# map size, [x,y] (m,m) - (1.8,4.2)
pp.find_path((40, 10),(30,60))			# define start and end points (20,125) 	(20,60)
pp.graph_representation()


# print POSE_TABLE[table_index]
