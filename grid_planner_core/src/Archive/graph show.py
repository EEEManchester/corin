
import networkx as nx
import matplotlib.pyplot as plt

import numpy as np

G = nx.Graph() 							# create Graph class
## add node
G.add_node('A')
G.add_node('B')
G.add_node('C')
G.add_node('D', weight=6)

## add weighted edges - this creates the nodes if not created earlier 
G.add_edge('A', 'B', weight=4) 			
G.add_edge('B', 'D', weight=2)
G.add_edge('A', 'C', weight=3)
G.add_edge('C', 'D', weight=4)
G.add_edge('D', 'E', weight=1)
# print nx.shortest_path(G, 'A', 'D', weight='weight')
# print list(G.neighbors('D'))
# G.remove_node('B')
# print nx.shortest_path(G, 'A', 'D', weight='weight')



## create 2D binary map
map_2d = np.zeros((10,10))
map_sz = np.shape(map_2d)

## set obstacles
map_2d[1:4,1:4] = 1
map_2d[3:8,5:7] = 1
map_2d[7:8,8:10] = 1

# print map_2d

## stack node into graph
G = nx.Graph()

count = 0
for i in range(0,map_sz[0]):
	for j in range (0,map_sz[1]):
		G.add_node(count, weight=map_2d.item(count))
		count += 1

## create edge
count = 0
for i in range(0,map_sz[0]): 		# rows
	for j in range (0,map_sz[1]):	# columns
		# check if node is empty
		if (map_2d[i,j]==0):
			# check if adjacent cell is empty
			if (j < (map_sz[1]-1) and map_2d[i,j+1]==0): 	# check cells on the right
				G.add_edge(i,j)
			pass
		else:
			pass
			# print i, ' ', j,' occupied'

		# if (G.nodes[count]['weight']==0):
		# 	print 'empty'
		# 	
		# 	if (G.nodes[count]['weight']==0):
		# 	G.add_edge(i,j)
			
		count += 1

class grid_planner:
	def __init__(self,size):
		self.G  = nx.grid_graph(dim=[size[0], size[1]]) 	# initial map size
		self.Gf = nx.Graph()
		self.G_obs = nx.Graph()
		self.sz = size

	def eucld_dist(self,a,b):
		(x1, y1) = a
		(x2, y2) = b
		return (((x1 - x2)**2 + (y1 - y2)**2)**0.5)

	def node_cost(self):
		nx.set_node_attributes(self.G, {e: 0 for e in self.G.nodes()}, 'cost') 			# set cost for node
		# set obstacles (x,y):
		self.G.nodes[(0,2)]['cost'] = 1
		self.G.nodes[(1,2)]['cost'] = 1
		self.G.nodes[(2,1)]['cost'] = 1
		self.G.nodes[(3,2)]['cost'] = 1
		self.G.nodes[(3,3)]['cost'] = 1

		self.G_obs = nx.path_graph([(0,2),(1,2),(2,1),(3,2),(3,3)])

	def edge_removal(self):
		## remove edges with obstacles
		e_init = self.G.number_of_edges()
		r_edge = [] 	# list for edges to remove
		
		for e in self.G.edges():
			# if either cell contains obstacles, remove edge
			if (self.G.nodes[e[0]]['cost']== 1 or self.G.nodes[e[1]]['cost']): 
				# stack into list to remove
				r_edge.append(e)
				
		for i in range(0,len(r_edge)):
			self.G.remove_edge(r_edge[i][0],r_edge[i][1])

		print 'Init, final: ', e_init, '  ', self.G.number_of_edges()

	def edge_cost(self):
		nx.set_edge_attributes(self.G, 1, 'cost') 	# set cost based on vertical distance to next cell

	def find_path(self,start,end):
		self.node_cost()
		self.edge_removal()
		self.edge_cost()

		self.Gf = nx.path_graph(nx.astar_path(self.G,start,end,heuristic=self.eucld_dist,weight='cost'))
		
		# print type(self.G)
		# print type(self.Gf)
		return self.Gf

	def graph_representation(self):
		pos  = dict(zip(self.G, self.G)) # dictionary of node names->positions
		p_fin = dict(zip(self.Gf, self.Gf))
		p_obs = dict(zip(self.G_obs, self.G_obs))
		# Grid map & obstacles
		nx.draw_networkx(self.G,pos,with_labels=True,node_size=1000,node_color='w');
		plt.hold(True)
		nx.draw_networkx(self.G_obs,p_obs,with_labels=False,node_size=1000,node_color='r',width=0.0);
		# Feasible path
		nx.draw_networkx(self.Gf,p_fin,node_size=1000,node_color='g',edge_color='r',width=3.0);
		plt.grid('on');
		plt.show()

		# nx.draw(self.G, pos)

		row_labels = range(self.sz[0])
		col_labels = range(self.sz[1])

		# plt.matshow(self.G)
		# plt.xticks(range(self.sz[1]), col_labels)
		# plt.yticks(range(self.sz[0]), row_labels)
		# plt.show()

pp = grid_planner((5,7))
print pp.find_path((3, 0),(3, 4))
pp.graph_representation()
# compute euclidean distance
# def dist(a, b):
# 	(x1, y1) = a
# 	(x2, y2) = b
# 	return (((x1 - x2)**2 + (y1 - y2)**2)**0.5)

# ## create grid graph
# G = nx.grid_graph(dim=[3, 3]) 	# nodes are two-tuples (x,y)

# ## add cost for node
# nx.set_node_attributes(G, {e: 0 for e in G.nodes()}, 'cost') 			# set cost for node


# ## remove edges with obstacles
# remove_edge = [] 	# list for edges to remove
# for e in G.edges():
# 	# if either cell contains obstacles, remove edge
# 	if (G.nodes[e[0]]['cost']== 1 or G.nodes[e[1]]['cost']): 
# 		# stack into list to remove
# 		remove_edge.append(e)
		
# for i in range(0,len(remove_edge)):
# 	G.remove_edge(remove_edge[i][0],remove_edge[i][1])

# ## set cost for edges
# nx.set_edge_attributes(G, 1, 'cost') 	# set cost based on vertical distance to next cell

# ## Find path using A*
# G_path = (nx.astar_path(G,(0, 0),(2, 2),heuristic=dist,weight='cost'))

## Plot graph

# G = nx.petersen_graph()
# plt.subplot(121)
# # <matplotlib.axes._subplots.AxesSubplot object at ...>
# nx.draw(G, with_labels=True, font_weight='bold')
# plt.show()
# plt.subplot(122)
# # <matplotlib.axes._subplots.AxesSubplot object at ...>
# nx.draw_shell(G, nlist=[range(5, 10), range(5)], with_labels=True, font_weight='bold')
# plt.show()