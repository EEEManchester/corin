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


def gc(p):
	"""converts x-y-z to z-y-x convention for networkx"""
	return tuple(reversed(p))

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
		self.map_size  	= size			# map size, (m^2, layers)
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
		self.G  	= nx.grid_graph(dim=[gridx, gridy]) 	# world grid map
		self.Gbody  = nx.grid_graph(dim=[gridx, gridy]) 	# truncated body map
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

		# multi-layer motion graph
		self.Gm = nx.grid_graph(dim=[gridx, gridy, self.map_size[2]]) 	# one layer for each motion
		# layers = 3
		# self.Gm = nx.grid_graph(dim=[4, 3, layers]) 	# one layer for each motion

		# add moore edges
		self.moore_2D_edge(self.G)
		self.moore_2D_edge(self.Gbody)
		self.moore_3D_edge(self.Gm,self.map_size[2])
		print self.Gm.edges((0,1,1))
		# set attribute for nodes
		nx.set_node_attributes(self.G, {e: 0 for e in self.G.nodes()}, 'cost')
		nx.set_node_attributes(self.Gbody, {e: 0 for e in self.Gbody.nodes()}, 'cost')
		nx.set_node_attributes(self.Gm, {e: 0 for e in self.Gm.nodes()}, 'cost')

		print 'Initialised - '
		print 'Map Grid  : ', gridx, ' by ', gridy
		print 'Robot grid: ', self.rbfp_gd
		print 'Body grid : ', self.rbbd_gd
		print 'Foot grid : ', self.rblg_gd
		print 'Resolution: ', self.cell_size, 'm^2'
		print '==============================================='

	def moore_2D_edge(self, G):
		""" add moore edges for 2D graph """
		aedge = []
		# cycle through all nodes
		for e in G.nodes():
			# diagonal left up
			try:
				du_node = (e[0]+1,e[1]+1)
				G.nodes[du_node] 			# test if node exist
				aedge.append([e,du_node]) 	# append to list if exists
			except:
				pass

			# diagonal left down
			try:
				dd_node = (e[0]+1,e[1]-1)
				G.nodes[dd_node] 			# test if node exist
				aedge.append([e,dd_node])	# append to list if exist
			except:
				pass
			
		G.add_edges_from(aedge) 	# add edges to map
	def moore_3D_edge(self,G,layers):
		""" add edges between layers """
		## TODO: moore edges between layers
		aedge = []
		for e in G.nodes():
			ni = layers - e[0] 		# reduce 'try' by limiting layer increments 
			for i in range(1,ni):
				# vertical up
				try:
					du_node = (e[0]+i,e[1],e[2])
					G.nodes[(du_node)] 			# test if node exist
					aedge.append([e,du_node]) 	# append to list if exists
				except:
					pass
				# vertical up front
				try:
					du_node = (e[0]+i,e[1],e[2]+1)
					G.nodes[(du_node)] 			# test if node exist
					aedge.append([e,du_node]) 	# append to list if exists
				except:
					pass
				# vertical up back
				try:
					du_node = (e[0]+i,e[1],e[2]-1)
					G.nodes[(du_node)] 			# test if node exist
					aedge.append([e,du_node]) 	# append to list if exists
				except:
					pass
				# vertical up left
				try:
					du_node = (e[0]+i,e[1]+1,e[2])
					G.nodes[(du_node)] 			# test if node exist
					aedge.append([e,du_node]) 	# append to list if exists
				except:
					pass
				# vertical up right
				try:
					du_node = (e[0]+i,e[1]-1,e[2])
					G.nodes[(du_node)] 			# test if node exist
					aedge.append([e,du_node]) 	# append to list if exists
				except:
					pass
			
		G.add_edges_from(aedge) 	# add edges to map

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
		""" Set obstacle area and cost """
		## Obstacle boxes - bottom left and upper right [TODO:set start point, size in (m)]
		#  Chimney demonstration
		wall_01 = [ (0,0),(20,50)]
		wall_02 = [(35,25),(50,50)]

		wall_03 = [(0,80),(10,110)]
		wall_04 = [(30,80),(39,110)]

		hole_01 = [(40,80),(60,110)]
		hole_02 = [(11,85),(29,105)]

		#  Wall walking demonstration
		# wall_01 = [ (0,25),(25,50)]
		# wall_02 = [(40,25),(60,50)]
		
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

	def edge_removal(self, Gve):
		""" remove edges with obstacles """
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

	
	def primitive_search_path(self):
		""" First find the shortest path using minimum footprint, then fit motion primitives """

		## Variables ##
		ax = (self.rbfp_gd[0]-1)/2 		# robot longitudinal distance (grid cell)
		ay = (self.rbfp_gd[1]-1)/2-5	# robot lateral distance (grid cell) - HARDCODED
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
				
				# compute for nodes that are obstacle free
				if (valid):
					node_free.append(e)
					self.Gbody.nodes[e]['cost'] = 0
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

	def find_path(self,start,end):
		self.node_cost()				# assign cost based on free=0 or obstacle=1
		self.edge_removal(self.G)		# remove edges linked to obstacles
		# self.edge_cost()				# assign cost to edge - NOT USED
		# self.footprint_cost()			# assign cost based on footprint of robot
		self.primitive_search_path() 	# finds path then use motion primitives to fit in
		self.edge_removal(self.Gbody) 	# remove edges linked to invalid cells

		## Robot represented with a number of nodes
		# print (nx.astar_path(self.Gbody,start,end,heuristic=self.eucld_dist,weight='cost'))
		try:
			self.Gpath = nx.path_graph(nx.astar_path(self.Gbody,start,end,heuristic=self.eucld_dist,weight='cost'))
			# self.Gpath = nx.path_graph(nx.shortest_path(self.Gbody,start,end))
		# list_path  = nx.astar_path(self.Gbody,start,end,heuristic=self.eucld_dist,weight='cost')
		# self.G_LF, self.G_LM, self.G_LR, self.G_RF, self.G_RM, self.G_RR = self.find_foothold(list_path)
			print 'Path exist!'
		
		except:
			print
			print 'ERROR, no body path exist!'

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
		nx.draw_networkx(self.Gpath,p_fin,with_labels=labels,node_size=1,node_color='g',edge_color='b',width=5.0);

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
pp = grid_planner((2.0,1.0,3)) 			# map size, [x,y] (m,m) - (1.8,4.2)
pp.find_path((30, 10),(30, 65))			# define start and end points (20,125)
pp.graph_representation()

# G = nx.grid_graph(dim=[5,5]) 	# map size & layer [x, y, z(layers)]
# print G.nodes()
# print G.neighbors[(1,1)]
# print G.nodes[gc((4,0))] 		# accessing: z-y-x convention