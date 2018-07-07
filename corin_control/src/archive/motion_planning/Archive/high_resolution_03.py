#!/usr/bin/env python

## Grid based motion planning
## High resolution grid cells - 0.03m^2

import networkx as nx
import matplotlib.pyplot as plt

import numpy as np
import math

class grid_planner:
	def __init__(self,size):
		self.cell_size 	= 0.03			# size of cell, (m) - TARGET: 0.03
		self.leg_size  	= 0.18			# area of foothold per leg, (m^2)
		self.map_size  	= size			# map size, (m^2)
		self.rbfp_size  = (0.488, 0.5) 	# robot footprint size, (m,m) - TARGET: (0.488,0.58)
		self.rbbd_size  = (0.27, 0.18) 	# robot body size, (m,m) - 

		self.initialise_graph()

	def initialise_graph(self):
		# determine robot related size in grid cell
		self.rbfp_gd = tuple(map(lambda x: int(math.ceil(x/self.cell_size)), self.rbfp_size))	# footprint
		self.rbbd_gd = tuple(map(lambda x: int(math.ceil(x/self.cell_size)), self.rbbd_size))	# body
		self.rblg_gd = (int(math.ceil(self.leg_size/self.cell_size)),int(math.ceil(self.leg_size/self.cell_size)))	# leg

		# determine number of grids
		gridx = int(math.ceil(self.map_size[0]/self.cell_size))
		gridy = int(math.ceil(self.map_size[1]/self.cell_size))

		# initiate graph instances
		self.G  	= nx.grid_graph(dim=[gridy, gridx]) 	# initial map size
		self.Gbody  = nx.grid_graph(dim=[gridy, gridx]) 	# body map size
		self.Gfree  = nx.Graph() 							# illustration - body map with zero cost
		self.Gpath 	= nx.Graph() 							# final path 
		self.G_obs 	= nx.Graph()							# illustration - cell with obstacles
		self.mp_sz 	= (gridx,gridy)							# map size in number of grid cells

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

	def obstacle_area(self, box):
		dx = box[1][0] - box[0][0]
		dy = box[1][1] - box[0][1]

		for x in range(0,dx+1):
			for y in range(0,dy+1):
				p = ( (box[0][0]+x), (box[0][1]+y))
				self.G.nodes[p]['cost'] = 1

	def node_cost(self):

		## Obstacle boxes - bottom left and upper right [TODO:set start point, size in (m)]
		box1 = [ (8,22),(17,36)] 		# cell_size = 0.06m
		box2 = [(32,22),(37,44)]
		# box1 = [(2,5),(4,9)] 		# cell_size = 0.12m
		# box2 = [(8,5),(9,11)]

		self.obstacle_area(box1)
		self.obstacle_area(box2)
		
		# self.G.nodes[(10,9)]['cost'] = 1

		# Create obstacle path based on above - for illustration
		nobs = []
		for e in self.G.nodes():
			if (self.G.nodes[e]['cost'] == 1):
				nobs.append(e)
		
		self.G_obs = nx.path_graph(nobs) 	

	def edge_removal(self, Gve):
		## remove edges with obstacles
		e_init = Gve.number_of_edges()	# determine number of edges
		r_edge = [] 					# list for edges to remove
		
		for e in Gve.edges():
			# if either cell contains obstacles, remove edge
			if (Gve.nodes[e[0]]['cost']>= 1 or Gve.nodes[e[1]]['cost']>=1): 
				# stack into list to remove
				r_edge.append(e)
				
		for i in range(0,len(r_edge)):
			Gve.remove_edge(r_edge[i][0],r_edge[i][1])

		print 'Init, final: ', e_init, '  ', Gve.number_of_edges()

	def edge_cost(self):
		nx.set_edge_attributes(self.G, 1, 'cost') 	# set cost based on vertical distance to next cell

	def check_area(self, p, ctype):

		scost = 0 	# total cost

		# check body area
		if (ctype == 'body'):
			# skip if centre occupied
			if (self.G.nodes[p]['cost'] != 1):
				bx = int(math.ceil((self.rbbd_gd[0])/2.)) 		# longitudinal distance from robot centre
				by = int(math.ceil((self.rbbd_gd[1])/2.))		# lateral distance from robot centre

				## cycle through all four edges TODO: remove overlap on points
				for x in range(0,bx*2):
					scost = scost + self.G.nodes[(p[0]-by,p[1]-bx+x)]['cost']		# left edge
					scost = scost + self.G.nodes[(p[0]+by,p[1]-bx+x)]['cost']		# right edge
				for y in range(0,by*2):
					scost = scost + self.G.nodes[(p[0]-by+y,p[1]-bx)]['cost']		# top edge
					scost = scost + self.G.nodes[(p[0]-by+y,p[1]+bx)]['cost']		# bottom edge
			else: 
				scost = 1

		elif (ctype == 'leg'):
			## TODO: Change to same from inside out
			bx = (self.rblg_gd[0]-1)/2  	# longitudinal distance from robot centre
			by = (self.rblg_gd[1]-1)/2 		# lateral distance from robot centre
			# print 'bx ', bx, by, self.rblg_gd

			## cycle all six legs
			for i in range(0,6):
				# set foot and transform (x,y) (m)
				px 	  = int(np.round( (np.cos(np.pi/2)*(p_leg[i][0])-np.sin(np.pi/2)*(p_leg[i][1]))/self.cell_size))
				py 	  = int(np.round( (np.sin(np.pi/2)*(p_leg[i][0])+np.cos(np.pi/2)*(p_leg[i][1]))/self.cell_size))
				pbnom = (px,py)

				pwnom = tuple(map(lambda x,y: y+x, p, pbnom))	# leg nominal stance in world frame
				fcost = 0 										# total cost per foot
				# print i, pbnom

				# calculate sum for each area
				for cx in range(0,self.rblg_gd[0]):
					for cy in range(0,self.rblg_gd[1]):
						# foothold in leg grid area
						pwset = (pwnom[0]-by+cy,pwnom[1]-bx+cx) 

						try:
							fcost = fcost + self.G.nodes[pwset]['cost']

							# if at least one cell is available
							# if (self.G.nodes[pwset]['cost']==0):
							# 	fcost = 0
							# else:
							# 	fcost = 1
						except:
							pass #fcost = fcost + 1

						if (p == (24,10) and i==4):
							print i, pbnom, pwnom, pwset, fcost

				# calculate average of area and add to total
				area  = self.rblg_gd[0]*self.rblg_gd[1]
				ncost = math.floor( fcost/float(area) )
				
				# if (p == (25,24)):
				# 	print i, ' ', fcost, ' ', ncost
				scost = scost + ncost #math.floor(fcost/((2.*bx+1)*(2.*by+1)))

		return int(scost)

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
				body_cost = self.check_area(e, 'body')
				
				# compute for nodes that are obstacle free
				if (body_cost < 1):
					# print '===================='
					footholds_cost = self.check_area(e, 'leg')

					# set node cost based on foothold average cost					
					self.Gbody.nodes[e]['cost'] = footholds_cost
					
					# if (e == (24,23)):
					# 	print 'cost ', self.Gbody.nodes[e]['cost']
					# if (e == (24,22)):
					# 	print 'cost ', self.Gbody.nodes[e]['cost']
					if (self.Gbody.nodes[e]['cost'] < 1):
						node_free.append(e)
				else:
					self.Gbody.nodes[e]['cost'] = 1

		self.Gfree = nx.path_graph(node_free)

	def eucld_dist(self,a,b):
		(x1, y1) = a
		(x2, y2) = b
		return (((x1 - x2)**2 + (y1 - y2)**2)**0.5)

	def find_path(self,start,end):
		self.node_cost()			# assign cost based on free=0 or obstacle=1
		self.edge_removal(self.G)		# remove edges linked to obstacles
		self.edge_cost()			# assign cost to edge
		self.footprint_cost()		# assign cost based on footprint of robot
		self.edge_removal(self.Gbody)

		## Robot represented with a number of nodes
		try:
			self.Gpath = nx.path_graph(nx.astar_path(self.Gbody,start,end,heuristic=self.eucld_dist,weight='cost'))
			print 'Path exist!'
		except:
			print 'ERROR, no body path exist!'


	def graph_representation(self):
		# dictionary of node names->positions
		pos    = dict(zip(self.G, self.G)) 			
		p_fin  = dict(zip(self.Gpath, self.Gpath))
		p_obs  = dict(zip(self.G_obs, self.G_obs))
		p_body = dict(zip(self.Gfree, self.Gfree))
		
		# Grid map & obstacles
		plt.style.use('presentation')
		nsize  = 20	# size of nodes
		labels = False 	# label on nodes

		ax = nx.draw_networkx(self.Gbody,pos,with_labels=labels,node_size=nsize,node_color='0.8',width=1.5,node_shape="s");
		# Body path
		nx.draw_networkx(self.Gfree,p_body,with_labels=labels,node_size=nsize,node_color='y',edge_color='r',width=0.0,node_shape="s");
		# # Obstacles
		nx.draw_networkx(self.G_obs,p_obs,with_labels=labels,node_size=nsize,node_color='r',width=0.0,node_shape="s");
		# # Feasible path - single point
		nx.draw_networkx(self.Gpath,p_fin,with_labels=labels,node_size=10,node_color='g',edge_color='b',width=3.0);
		
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

p_leg = [p_base_X_lf_foot,p_base_X_lm_foot,p_base_X_lr_foot,p_base_X_rf_foot,p_base_X_rm_foot,p_base_X_rr_foot]

## Create map and plan path
pp = grid_planner((1.8,2.0)) 			# map size, [x,y] (m,m)
pp.find_path((24, 10),(24, 56))			# define start and end points
pp.graph_representation()

# print int(np.round(10/float(6)))
# a = -9.0
