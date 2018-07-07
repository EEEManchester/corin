
import networkx as nx
import matplotlib.pyplot as plt

import numpy as np
import math

class grid_planner:
	def __init__(self,size):
		self.G  = nx.grid_graph(dim=[size[1], size[0]]) 	# initial map size
		self.Gf = nx.Graph()
		self.G_obs = nx.Graph()
		self.sz = size

		self.Gbody  = nx.grid_graph(dim=[size[1], size[0]]) 	# body map size
		self.Gfree  = nx.Graph()

		self.initialise_graph()

	def initialise_graph(self):
		print 'Map Size: ', self.sz[0], ' by ', self.sz[1]
		# set attribute for nodes
		nx.set_node_attributes(self.G, {e: 0 for e in self.G.nodes()}, 'cost')
		nx.set_node_attributes(self.Gbody, {e: 0 for e in self.Gbody.nodes()}, 'cost')

	def eucld_dist(self,a,b):
		(x1, y1) = a
		(x2, y2) = b
		return (((x1 - x2)**2 + (y1 - y2)**2)**0.5)

	def occlude_obstacle(self, box):
		dx = box[1][0] - box[0][0]
		dy = box[1][1] - box[0][1]

		for x in range(0,dx+1):
			for y in range(0,dy+1):
				p = ( (box[0][0]+x), (box[0][1]+y))
				self.G.nodes[p]['cost'] = 1

	def node_cost(self):

		# Obstacle boxes - bottom left and upper right
		box1 = [(2,5),(4,9)]
		box2 = [(8,5),(9,11)]

		self.occlude_obstacle(box1)
		self.occlude_obstacle(box2)
		
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
			if (Gve.nodes[e[0]]['cost']== 1 or Gve.nodes[e[1]]['cost']==1): 
				# stack into list to remove
				r_edge.append(e)
				
		for i in range(0,len(r_edge)):
			Gve.remove_edge(r_edge[i][0],r_edge[i][1])

		print 'Init, final: ', e_init, '  ', Gve.number_of_edges()

	def edge_cost(self):
		nx.set_edge_attributes(self.G, 1, 'cost') 	# set cost based on vertical distance to next cell

	def footprint_cost(self):

		robot_area = [5,5]			# footprint area of robot
		body_area  = [3,3]			# body area of robot

		ax = (robot_area[0]-1)/2 	# longitudinal distance from robot centre
		ay = (robot_area[1]-1)/2	# lateral distance from robot centre

		bx = (body_area[0])/2 		# longitudinal distance from robot centre
		by = (body_area[1])/2		# lateral distance from robot centre
		print 'Edges: ', ax, ' ', ay
		print 'Body : ', bx, ' ', by
		node_ignored = []
		node_free 	 = []

		for e in self.G.nodes():
			# ignore edges
			if (e[0]<=(ax-1) or e[0]>=(self.sz[0]-ax) or e[1]<=(ay-1) or e[1]>=(self.sz[1]-ay)): 	
				node_ignored.append(e)
				self.Gbody.nodes[e]['cost'] = 1
			else:
				# check body collision TODO: automate this to a square dependant on resolution 
				fcost = self.G.nodes[(e[0],e[1]+bx)]['cost'] 	# front end
				bcost = self.G.nodes[(e[0],e[1]-bx)]['cost']	# rear end
				lcost = self.G.nodes[(e[0]+by,e[1])]['cost'] 	# left end
				rcost = self.G.nodes[(e[0]-by,e[1])]['cost']	# right end

				body_cost = self.G.nodes[e]['cost'] + fcost + bcost + lcost + rcost
				
				if (e==(3,4)):
					print body_cost, self.G.nodes[e]['cost'],  fcost
				# compute for nodes that are obstacle free
				if (body_cost < 1):
					# print e, body_cost, self.G.nodes[e]['cost']
					# calculate footprint cost for each node
					p1 = self.G.nodes[(e[0]+ax,e[1]+ay)]['cost'] # top right
					p2 = self.G.nodes[(e[0]-ax,e[1]+ay)]['cost'] # top left
					p3 = self.G.nodes[(e[0]+ax,e[1]-ay)]['cost'] # bottom right
					p4 = self.G.nodes[(e[0]-ax,e[1]-ay)]['cost'] # bottom left
					
					self.Gbody.nodes[e]['cost'] = int(math.ceil(float(p1 + p2 + p3 + p4)/4)) 	# average of all foothold area
					
					if (self.Gbody.nodes[e]['cost'] == 0):
						node_free.append(e)
				else:
					self.Gbody.nodes[e]['cost'] = 1

		self.Gfree = nx.path_graph(node_free)

	def find_path(self,start,end):
		self.node_cost()			# assign cost based on free=0 or obstacle=1
		self.edge_removal(self.G)		# remove edges linked to obstacles
		self.edge_cost()			# assign cost to edge
		self.footprint_cost()		# assign cost based on footprint of robot
		self.edge_removal(self.Gbody)

		## Robot represented as single node
		# try:
		# 	self.Gf = nx.path_graph(nx.astar_path(self.G,start,end,heuristic=self.eucld_dist,weight='cost'))
		# except:
		# 	print 'ERROR, no path exist!'

		## Robot represented with a number of nodes
		try:
			self.Gf = nx.path_graph(nx.astar_path(self.Gbody,start,end,heuristic=self.eucld_dist,weight='cost'))
		except:
			print 'ERROR, no body path exist!'

		# print type(self.G)
		# print type(self.Gf)
		

	def graph_representation(self):
		# dictionary of node names->positions
		pos    = dict(zip(self.G, self.G)) 			
		p_fin  = dict(zip(self.Gf, self.Gf))
		p_obs  = dict(zip(self.G_obs, self.G_obs))
		p_body = dict(zip(self.Gfree, self.Gfree))
		
		# Grid map & obstacles
		plt.style.use('presentation')
		
		ax = nx.draw_networkx(self.Gbody,pos,with_labels=True,node_size=500,node_color='0.8',width=1.5);
		# Body path
		nx.draw_networkx(self.Gfree,p_body,node_size=500,node_color='y',edge_color='r',width=0.0);
		# Obstacles
		nx.draw_networkx(self.G_obs,p_obs,with_labels=False,node_size=500,node_color='r',width=0.0);
		# Feasible path - single point
		nx.draw_networkx(self.Gf,p_fin,node_size=1000,node_color='g',edge_color='b',width=3.0);
		
		
		plt.grid('on');		 plt.grid(which='major', linestyle=':', linewidth='0.5', color='black')
		plt.minorticks_on(); plt.grid(which='minor', linestyle=':', linewidth='0.25', color='black')

		# minor_ticks = np.arange(0, 9+1, 0.5)
		# plt.xticks(minor_ticks)

		plt.xlabel('x, [1]')
		plt.ylabel('y, [1]')

		# maximise plot window
		fig_manager = plt.get_current_fig_manager()
		fig_manager.resize(*fig_manager.window.maxsize())
		# fig_manager.window.state('normal')
		
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

# robot footprint, (m)
robot_size = ( abs(p_base_X_lf_foot[0]) + abs(p_base_X_rr_foot[0]), abs(p_base_X_lm_foot[1]) + abs(p_base_X_rm_foot[1]) )

## Desired resolution
cell_size = 0.03 	# size of grid cells, (m^2)
leg_size  = 0.12	# area of foothold per leg, (m^2)
map_size  = (5,6)	# map size, (m^2)

## Simplified resolutions
cell_size  = 0.12 			# size of grid cells, (m^2)
leg_size   = 0.12			# area of foothold per leg, (m^2)
map_size   = (1.74,2.0)		# map size, (m^2)

robot_gsz  = map(lambda x: int(math.ceil(x/cell_size)), robot_size) 	# robot footprint in grid cell

gridx = int(math.ceil(map_size[0]/cell_size))
gridy = int(math.ceil(map_size[1]/cell_size))

pp = grid_planner((gridx,gridy)) 	# map size (x,y)
pp.find_path((6, 2),(6, 14))			# define start and end points
pp.graph_representation()
