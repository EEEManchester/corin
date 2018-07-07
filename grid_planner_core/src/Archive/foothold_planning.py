#!/usr/bin/env python

## Grid based motion planning
## High resolution grid cells - 0.03m^2

import os
import sys
sys.path.insert(0, '/home/wilson/catkin_ws/src/mcorin/mcorin_control/py_script/library')
sys.dont_write_bytecode = True

import networkx as nx
import matplotlib.pyplot as plt

import numpy as np
import math
from constant import *
import transforms
import gait_class

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

		# add moore edges
		self.moore_edge(self.G)
		self.moore_edge(self.Gbody)

		# set attribute for nodes
		nx.set_node_attributes(self.G, {e: 0 for e in self.G.nodes()}, 'cost')
		nx.set_node_attributes(self.Gbody, {e: 0 for e in self.Gbody.nodes()}, 'cost')

		print 'Initialised - '
		print 'Map Grid  : ', gridx, ' by ', gridy
		print 'Robot grid: ', self.rbfp_gd
		print 'Body grid : ', self.rbbd_gd
		print 'Foot grid : ', self.rblg_gd
		print 'Resolution: ', self.cell_size, 'm^2'
		print '==============================================='

	def obstacle_area(self, box, obstacle):
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

	def node_cost(self):

		## Obstacle boxes - bottom left and upper right [TODO:set start point, size in (m)]
		wall_01 = [ (6,22),(17,44)] 		# cell_size = 0.06m
		wall_02 = [(32,22),(40,44)]		
		hole_01 = [(15,25),(31,40)]

		self.obstacle_area(wall_01, 'wall')
		self.obstacle_area(wall_02, 'wall')
		# self.obstacle_area(hole_01, 'hole')
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

	def moore_edge(self, G):
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

	def edge_removal(self, Gve):
		## remove edges with obstacles
		e_init = Gve.number_of_edges()	# determine number of edges
		r_edge = [] 					# list for edges to remove
		
		for e in Gve.edges():
			# if either cell contains obstacles, remove edge
			if (Gve.nodes[e[0]]['cost']!= 0 or Gve.nodes[e[1]]['cost']!=0): 
				# stack into list to remove
				r_edge.append(e)
				
		for i in range(0,len(r_edge)):
			Gve.remove_edge(r_edge[i][0],r_edge[i][1])

		print 'Init, final: ', e_init, '  ', Gve.number_of_edges()

	def edge_cost(self):
		nx.set_edge_attributes(self.G, 1, 'cost') 	# set cost based on vertical distance to next cell

	def check_body_area(self, p):
		# check body area for collision
		scost = 0 	# total cost

		# skip if centre occupied
		if (self.G.nodes[p]['cost'] != 1):
			bx = int(math.ceil((self.rbbd_gd[0])/2.)) 		# longitudinal distance from robot centre
			by = int(math.ceil((self.rbbd_gd[1])/2.))		# lateral distance from robot centre

			## cycle through all four edges TODO: remove overlap on points
			for x in range(0,bx*2):
				scost = scost + self.G.nodes[(p[0]-by,p[1]-bx+x)]['cost']		# left edge
				scost = scost + self.G.nodes[(p[0]+by,p[1]-bx+x)]['cost']		# right edge

				# if (p== (13,17)):
				# 	print (p[0]-by,p[1]-bx+x), (p[0]+by,p[1]-bx+x)
			for y in range(0,by*2):
				scost = scost + self.G.nodes[(p[0]-by+y,p[1]-bx)]['cost']		# top edge
				scost = scost + self.G.nodes[(p[0]-by+y,p[1]+bx)]['cost']		# bottom edge

				# if (p== (13,17)):
				# 	print (p[0]-by+y,p[1]-bx), (p[0]-by+y,p[1]+bx)
		else: 
			scost = 1
		return scost

	def check_leg_area(self, p):
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
		
		return int(scost), valid_flag

	def footprint_cost(self):

		body_area  = [3,3]			# body area of robot

		ax = (self.rbfp_gd[0]-1)/2 	# longitudinal distance from robot centre
		ay = (self.rbfp_gd[1]-2)/2	# lateral distance from robot centre
		
		print 'Edges: ', ax, ' ', ay
		
		node_ignored = []
		node_free 	 = []

		for e in self.G.nodes():
			# ignore edges
			if (e[0]<=(ax-1) or e[0]>=(self.mp_sz[0]-ax) or e[1]<=(ay-1) or e[1]>=(self.mp_sz[1]-ay)): 	
				node_ignored.append(e)
				self.Gbody.nodes[e]['cost'] = 1
			else:
				# check body collision TODO: automate this to a square dependant on resolution 
				body_cost = self.check_body_area(e)
				
				# compute for nodes that are obstacle free
				if (body_cost < 1):
					# print '===================='
					footholds_cost, valid_flag = self.check_leg_area(e)

					# set node cost based on foothold average cost					
					# self.Gbody.nodes[e]['cost'] = footholds_cost
					
					if (e == (13,17) ):
						print e, valid_flag

					if (valid_flag == True):
						node_free.append(e)
						self.Gbody.nodes[e]['cost'] = 0
					else:
						self.Gbody.nodes[e]['cost'] = 1
				else:
					self.Gbody.nodes[e]['cost'] = 1

		self.Gfree = nx.path_graph(node_free)

	def find_foothold(self, e):
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
		while (ncount<len(e)):

			# cycle through all legs at each stance
			for leg in range(0,6):
				# new foothold after displaced by d in world frame - e[ncount is base position]
				new_foothold = tuple(map(lambda x,y: y+x, e[ncount], BASE_X_FOOT[leg]))

				# search neighbouring cells within area
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

	def smooth_edge(self,init_path):
		new_path = init_path
		return new_path
	def eucld_dist(self,a,b):
		## heuristics used for A* planner
		(x1, y1) = a
		(x2, y2) = b
		return (((x1 - x2)**2 + (y1 - y2)**2)**0.5)

	def find_path(self,start,end):
		self.node_cost()			# assign cost based on free=0 or obstacle=1
		self.edge_removal(self.G)	# remove edges linked to obstacles
		self.edge_cost()			# assign cost to edge
		self.footprint_cost()		# assign cost based on footprint of robot
		self.edge_removal(self.Gbody)

		## Robot represented with a number of nodes
		try:
			list_path  	= (nx.astar_path(self.Gbody,start,end,heuristic=self.eucld_dist,weight='cost'))
			smooth_path	= self.smooth_edge(list_path)
			self.Gpath 	= nx.path_graph(smooth_path)
			self.G_LF, self.G_LM, self.G_LR, self.G_RF, self.G_RM, self.G_RR = self.find_foothold(smooth_path)
			print 'Path exist!'
		except nx.exception.NetworkXNoPath:
			print 'No path found'

	def graph_representation(self):
		
		# dictionary of node names->positions
		pos    = dict(zip(self.G, self.G)) 			
		p_fin  = dict(zip(self.Gpath, self.Gpath))
		p_wall = dict(zip(self.G_wall, self.G_wall))
		p_hole = dict(zip(self.G_hole, self.G_hole))
		p_body = dict(zip(self.Gfree, self.Gfree))

		p_LF = dict(zip(self.G_LF, self.G_LF))
		p_LM = dict(zip(self.G_LM, self.G_LM))
		p_LR = dict(zip(self.G_LR, self.G_LR))
		p_RF = dict(zip(self.G_RF, self.G_RF))
		p_RM = dict(zip(self.G_RM, self.G_RM))
		p_RR = dict(zip(self.G_RR, self.G_RR))

		# Grid map & obstacles
		plt.style.use('presentation')
		nsize  = 20	# size of nodes
		labels = False 	# label on nodes

		ax = nx.draw_networkx(self.Gbody,pos,with_labels=labels,node_size=nsize,node_color='0.8',width=1.5,node_shape="s");
		# Body path
		nx.draw_networkx(self.Gfree,p_body,with_labels=labels,node_size=nsize,node_color='y',edge_color='r',width=0.0,node_shape="s");
		# Wall
		nx.draw_networkx(self.G_wall,p_wall,with_labels=labels,node_size=nsize,node_color='r',width=0.0,node_shape="s");
		# Holes 
		nx.draw_networkx(self.G_hole,p_hole,with_labels=labels,node_size=nsize,node_color='tab:brown',width=0.0,node_shape="s");
		# Feasible path - single point
		nx.draw_networkx(self.Gpath,p_fin,with_labels=labels,node_size=10,node_color='g',edge_color='b',width=3.0);
		# Footholds
		nx.draw_networkx(self.G_LF,p_LF,with_labels=labels,node_size=4,node_color='b',edge_color='b',width=0.5);
		# nx.draw_networkx(self.G_LM,p_LM,with_labels=labels,node_size=4,node_color='b',edge_color='b',width=0.0);
		# nx.draw_networkx(self.G_LR,p_LR,with_labels=labels,node_size=4,node_color='b',edge_color='b',width=0.0);
		nx.draw_networkx(self.G_RF,p_RF,with_labels=labels,node_size=4,node_color='r',edge_color='r',width=0.5);
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
pp = grid_planner((1.8,2.0)) 			# map size, [x,y] (m,m)
pp.find_path((24, 10),(24, 56))			# define start and end points
pp.graph_representation()

