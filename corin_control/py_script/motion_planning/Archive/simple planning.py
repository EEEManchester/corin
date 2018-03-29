
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
		box1 = [(1,3),(1,6)]
		box2 = [(4,3),(5,5)]

		self.occlude_obstacle(box1)
		self.occlude_obstacle(box2)
		
		# set obstacles (x,y): 
		# self.G.nodes[(1,3)]['cost'] = 1
		# self.G.nodes[(1,4)]['cost'] = 1
		# self.G.nodes[(1,5)]['cost'] = 1
		# self.G.nodes[(2,3)]['cost'] = 1
		# self.G.nodes[(2,4)]['cost'] = 1
		# self.G.nodes[(2,5)]['cost'] = 1

		# self.G.nodes[(4,3)]['cost'] = 1
		# self.G.nodes[(4,4)]['cost'] = 1
		# self.G.nodes[(4,5)]['cost'] = 1
		# self.G.nodes[(5,3)]['cost'] = 1
		# self.G.nodes[(5,4)]['cost'] = 1
		# self.G.nodes[(5,5)]['cost'] = 1

		self.G.nodes[(0,4)]['cost'] = 0
		self.G.nodes[(3,4)]['cost'] = 0
		self.G.nodes[(6,4)]['cost'] = 0

		# Create obstacle path based on above
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

		robot_area = [3,3]			# footprint area of robot

		ax = (robot_area[0]-1)/2 	# horizontal distance from robot centre
		ay = (robot_area[1]-1)/2	# vertical distance from robot centre

		node_ignored = []
		node_free 	 = []

		for e in self.G.nodes():
			# ignore edges
			if (e[0]<=(ax-1) or e[0]>=(self.sz[0]-ax) or e[1]<=(ay-1) or e[1]>=(self.sz[1]-ay)): 	
				node_ignored.append(e)
				self.Gbody.nodes[e]['cost'] = 1
			else:
				# compute for nodes that are obstacle free
				if (self.G.nodes[e]['cost'] != 1):
					# calculate footprint cost for each node
					p1 = self.G.nodes[(e[0]+1,e[1]+1)]['cost'] # top right
					p2 = self.G.nodes[(e[0]-1,e[1]+1)]['cost'] # top left
					p3 = self.G.nodes[(e[0]+1,e[1]-1)]['cost'] # bottom right
					p4 = self.G.nodes[(e[0]-1,e[1]-1)]['cost'] # bottom left
					
					self.Gbody.nodes[e]['cost'] = int(math.ceil(float(p1 + p2 + p3 + p4)/4))
					
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

		try:
			self.Gf = nx.path_graph(nx.astar_path(self.G,start,end,heuristic=self.eucld_dist,weight='cost'))
		except:
			print 'ERROR, no path exist!'

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
		
		ax = nx.draw_networkx(self.Gbody,pos,with_labels=True,node_size=1000,node_color='0.8',width=1.5);
		# Body path
		nx.draw_networkx(self.Gfree,p_body,node_size=1500,node_color='y',edge_color='r',width=0.0);
		# Obstacles
		nx.draw_networkx(self.G_obs,p_obs,with_labels=False,node_size=1000,node_color='r',width=0.0);
		# Feasible path - single point
		nx.draw_networkx(self.Gf,p_fin,node_size=1000,node_color='g',edge_color='b',width=3.0);
		
		
		plt.grid('on');		 plt.grid(which='major', linestyle=':', linewidth='0.5', color='black')
		plt.minorticks_on(); plt.grid(which='minor', linestyle=':', linewidth='0.25', color='black')

		minor_ticks = np.arange(0, 9+1, 0.5)
		plt.xticks(minor_ticks)

		
		plt.xlabel('x, m')
		plt.ylabel('y, m')
		# mng = plt.get_current_fig_manager()
		# mng.resize(*mng.window.maxsize())
		plt.tight_layout(pad=0.0, w_pad=0.0, h_pad=0)  
		plt.show()


# pp = grid_planner((10,10)) 	# map size (x,y)
# pp.find_path((3, 1),(3, 8))	# define start and end points
# pp.graph_representation()
