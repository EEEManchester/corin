#!/usr/bin/env python

""" Grid Map Builder
""" 

import sys; sys.dont_write_bytecode = True
import os.path

import networkx as nx
import matplotlib.pyplot as plt

import yaml
import numpy as np
import math
from fractions import Fraction
from collections import OrderedDict 	# remove duplicates
import random
import spiral_search

MAX_FLAT_H = 0.05
MIN_FLAT_H = -0.05

class GridMap:
	def __init__(self,map_name=None):
		self.map_name = map_name

		self.resolution	= 0.03	# size of cell, (m) - TARGET: 0.03
		self.map_size_m = (0,0) # set map size
		self.map_size_g = (0,0) # set map size
		# self.map_trunc  = (2,2) # cells to truncate
		
		self.Spiral = spiral_search.SpiralSearch()

		## Illustrations
		self.G_free = nx.Graph()
		self.G_wall = nx.Graph()
		self.G_hole = nx.Graph()
		self.viz_offset = np.zeros(3)

		if (map_name is not None):
			self.__initialise_graph__(map_name)

	def __load_map__(self, map_name):
		""" Loads from a list of pre-defined maps """

		map_size = (0,0)
		map_wall = None
		map_hole = None

		file_loc = os.path.join(os.path.dirname(__file__)) + "/Map.yaml"
		with open(file_loc, 'r') as stream:
			try:
				# Load list of maps
				map_list = yaml.load(stream)

				# Check if map exist in list
				if map_list['Map'][map_name]:
					map_size = map_list['Map'][map_name]['size']

					# Check for map obstacles
					try:
						map_wall = map_list['Map'][map_name]['wall']
					except:
						pass
					try:
						map_hole = map_list['Map'][map_name]['hole']
					except:
						pass
					try:
						voff = map_list['Map'][map_name]['offset']
						self.viz_offset = np.array([voff['x'],voff['y'],voff['z']])
					except:
						pass
					
					map_sz_tuple = (map_size['sx'],map_size['sy'])
					return map_sz_tuple, map_wall, map_hole
				
				else:
					print 'Error: Map ', map_name, ' has not been found!'
					return None
			except yaml.YAMLError as exc:
				print(exc)
				return None

	def __initialise_graph__(self, map_name):

		# Get map details
		self.map_size_m, map_wall, map_hole = self.__load_map__(map_name)
		
		# determine number of grids
		gridx = int(math.ceil(self.map_size_m[0]/self.resolution))
		gridy = int(math.ceil(self.map_size_m[1]/self.resolution))

		# initiate graph instances
		self.Map = nx.grid_graph(dim=[gridy, gridx]) 	# world grid map
		self.map_size_g = (gridx, gridy)
		
		# set default attribute for nodes
		nx.set_node_attributes(self.Map, {e: 0 for e in self.Map.nodes()}, 'height') # cell height
		nx.set_node_attributes(self.Map, {e: np.array([0.,0.,1.]) for e in self.Map.nodes()}, 'norm') 	# surface normal

		# set attributes based on map
		if (map_wall is not None):
			self.__set_obstacle__(map_wall, 'wall')
		if (map_hole is not None):
			self.__set_obstacle__(map_hole, 'hole')
		
		self.__set_surface_normal__()
		# self.__set_irregular_terrain__()

		self.set_illustrations()
		# add moore edges
		# self.Map = self.set_moore_edge(self.Map)		
		# self.Map = self.remove_edges(self.Map)

		print 'Initialised Map - ', self.map_name
		print '  - Grid  \t: ', gridx, 'x', gridy
		print '  - Resolution\t: ', self.resolution, 'm^2'
		print '==============================================='

	def __set_obstacle__(self, obs_list, obstacle):
		""" sets the area to be obstacles (wall or hole) """

		for w in obs_list:
			# remap box edges
			c1 = obs_list[w]['s1']
			c2 = obs_list[w]['s2']

			dx = c2[0] - c1[0]
			dy = c2[1] - c1[1]
			
			if (obstacle == 'wall'): 
				height = obs_list[w]['sh']
				snorm  = np.array([0.,0.,1.])
			
			elif (obstacle == 'hole'):
				height = obs_list[w]['sh']
				snorm  = np.array([0.,0.,1.])

			for x in range(0,dx+1):
				for y in range(0,dy+1):
					p = ( (c1[0]+x), (c1[1]+y))
					self.Map.nodes[p]['height'] = height

					if (obstacle == 'wall'):
						if (x == 0):
							snorm = np.array([0.,1.,0.])
						elif (x == dx):
							snorm = np.array([0.,-1.,0.])
						if (y == 0):
							snorm = np.array([-1.,0.,0.])
						elif (y == dy):
							snorm = np.array([1.,0.,0.])

					self.Map.nodes[p]['norm'] = snorm

	def __set_surface_normal__(self):
		""" Sets the surface normal for cells """

		for e in self.Map.nodes():
			# Normals for wall
			if (self.Map.nodes[e]['height'] > 0.):
				""" check in a circle starting from front if its wall, 
					side without wall is surface normal direction 		"""
				try:
					if (self.Map.nodes[tuple(map(sum, zip(e, (1,0))))]['height'] <= 0.):
						self.Map.nodes[e]['norm'] = np.array([1.,0.,0.])
				except:
					pass
				try:
					if (self.Map.nodes[tuple(map(sum, zip(e, (0,1))))]['height'] <= 0.):
						self.Map.nodes[e]['norm'] = np.array([0.,1.,0.])
				except:
					pass
				try:	
					if (self.Map.nodes[tuple(map(sum, zip(e, (-1,0))))]['height'] <= 0.):
						self.Map.nodes[e]['norm'] = np.array([-1.,0.,0.])
				except:
					pass	
				try:	
					if (self.Map.nodes[tuple(map(sum, zip(e, (0,-1))))]['height'] <= 0.):
						self.Map.nodes[e]['norm'] = np.array([0.,-1.,0.])
				except:
					pass	
				
			# Normals for ground
			elif (self.Map.nodes[e]['height'] == 0.):
				self.Map.nodes[e]['norm'] = np.array([0.,0.,1.])
				
			# Normal for holes
			elif (self.Map.nodes[e]['height'] < 0.):
				self.Map.nodes[e]['norm'] = np.array([0.,0.,1.])

	def __set_irregular_terrain__(self):
		""" Sets the surface normal for cells """

		for e in self.Map.nodes():
			# Normals for wall
			if (self.Map.nodes[e]['height'] >= 0. and self.Map.nodes[e]['height'] <= 0.5):
				self.Map.nodes[e]['height'] = random.uniform(MIN_FLAT_H, MAX_FLAT_H)

	def getIndex(self, p, j):
		""" Returns cell index at position (m) """
		
		def point_to_cell(x):
			idx = int(x/self.resolution)
			rem = x % self.resolution - self.resolution/2
			# print x, idx, x % self.resolution
			idx += 1 if rem > 0. else 0
			return idx

		grid_p = (point_to_cell(p[0]), point_to_cell(p[1]))

		# if (j < 3):
		# 	grid_p = (int(np.floor(position[0]/self.resolution)), int(np.ceil(position[1]/self.resolution)))
		# else:
		# 	grid_p = (int(np.floor(position[0]/self.resolution)), int(np.floor(position[1]/self.resolution)))

		return grid_p

	def get_cell(self, info, p, j=0):
		""" Returns cell characteristic at point p (in m) """
		
		# Convert cell location to grid format
		grid_p = self.getIndex(p,0)
		try:
			# grid_p = (point_to_cell(p[0]), point_to_cell(p[1]))
			# if (j < 3):
			# 	grid_p = (int(np.floor(p[0]/self.resolution)), int(np.ceil(p[1]/self.resolution)))
			# else:
			# 	grid_p = (int(np.floor(p[0]/self.resolution)), int(np.floor(p[1]/self.resolution)))
			# Check if cell within bound
			
			if (self.get_index_exists(grid_p)):
				return self.get_index(info, grid_p)
			else:
				print 'Index error: ', info, p, j
				return None
		except ValueError:	
			print 'get_cell ValueError: ', p, j
	
	def get_index_exists(self, p):
		""" Checks if cell index exists or out of bounds """
		
		# Check if type is array or tuple - array will be expressed in meters, convert to tuple
		if isinstance(p, np.ndarray):
			p = self.getIndex(tuple(map(float, p[0:2])),0)
		elif isinstance(p, tuple):
			pass

		try:
			self.Map.nodes[p]
			return True
		except KeyError, e:
			print 'Index %s out of bounds'%e
			return False

	def get_index(self, info, p):
		""" Returns cell characteristic at index p """
		
		# Return cell characteristic
		try:
			return self.Map.nodes[p][info]
		except:
			print 'Cell Invalid at ', p, info
			return None

	def get_map_size(self):
		return self.map_size_g

	def get_median_width(self, p):
		""" get the width clearance ahead of p """

		grid_p = (int(np.floor(p[0]/self.resolution)), int(np.ceil(p[1]/self.resolution)))
		
		SCAN_WIDTH = 12
		SCAN_DEPTH = 3
		range_scan = [10]*3
		
		for row in range(0,SCAN_DEPTH):
			lwall = (grid_p[0]+row, grid_p[1]+SCAN_WIDTH/2)
			rwall = (grid_p[0]+row, grid_p[1]-SCAN_WIDTH/2)
			
			for column in range(SCAN_WIDTH/2,0,-1):
				pl = (grid_p[0]+row, grid_p[1]+column)
				pr = (grid_p[0]+row, grid_p[1]-column)
				
				try:
					cell_h = self.Map.nodes[pl]['height']
					if (cell_h > 0.5):
						lwall = pl
				except:
					pass
				try:
					cell_h = self.Map.nodes[pr]['height']
					if (cell_h > 0.5):
						rwall = pr
				except:
					pass
			# 	print pl, pr, lwall, rwall
			# print '---------------------------'
			# calculate width (m)
			range_scan[row] = (lwall[1] - rwall[1]-1)*self.resolution

		return np.median(range_scan)

	def get_wall_length(self, p):
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
				if (self.Map.nodes[n_left]['cost'] == 1):
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
				map_item = self.Map.nodes[(n_left[0],n_left[1]-lw_len)]['cost']
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
				if (self.Map.nodes[n_right]['cost'] == 1):
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
				map_item = self.Map.nodes[(n_right[0],n_right[1]-rw_len)]['cost']
				if ( map_item == 0 or map_item == -1):
					# break found
					scan 	= True
				else:
					# continues search downwards
					rw_len += 1
			except:
				scan = True

		return (lw_dist,lw_len), (rw_dist, rw_len)

	def get_path_to_nparray(self, path, p_start):
		""" converts grid path to cartesian path, 
			relative to starting point 				"""
		""" Input: 	path -> graph path
					p_start -> starting position
			Output: x_cob -> CoB position 			"""

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
			w_cob[i][0] = 0.#self.base_map.nodes[p]['pose'][1]
			w_cob[i][1] = 0.#self.base_map.nodes[p]['pose'][2]
			w_cob[i][2] = 0.#self.base_map.nodes[p]['pose'][3]

			i += 1

		return x_cob, w_cob

	def set_illustrations(self):
		""" set items for nx_graph illustrations """

		i_wall = []
		i_hole = []
		for e in self.Map.nodes():
			if (self.Map.nodes[e]['height'] > 0):
				i_wall.append(e)
			elif (self.Map.nodes[e]['height'] < 0):
				i_hole.append(e)

		self.G_wall = nx.path_graph(i_wall) 	
		self.G_hole = nx.path_graph(i_hole)

	def set_map(self, map_name):
		""" Initialise map """
		self.map_name = map_name
		self.__initialise_graph__(map_name)

	def set_moore_edge(self, G):
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
		
	def set_edge_cost(self, G):
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
			elif (m1==0 and m2==0):
				# ground walking
				cost_motion = 0
			elif (m1==1 and m2==1):
				# wall walking
				cost_motion = 3
			elif (m1==2 and m2==2):
				# chimney walking
				cost_motion = 3

			## Cost 3: Change in bodypose (height, roll, pitch, yaw)
			cost_d_bh = abs(G.nodes[e[0]]['pose'][0] - G.nodes[e[1]]['pose'][0])
			cost_d_qr = abs(G.nodes[e[0]]['pose'][1] - G.nodes[e[1]]['pose'][1])
			cost_d_qp = abs(G.nodes[e[0]]['pose'][2] - G.nodes[e[1]]['pose'][2])
			cost_d_qy = abs(G.nodes[e[0]]['pose'][3] - G.nodes[e[1]]['pose'][3])

			## Cost 4: Wall selection - Wall Walking ONLY
			cost_d_lgw = 0
			if (m1==2 or m2==2):
				ldist, rdist = self.get_wall_length(e[0])
				# consider only if robot is between walls
				if (ldist[0] >=0 and rdist[0] >=0):
					if (ldist[1] > rdist[1]):	# left wall longer
						cost_d_lgw = ldist[0] 	
					else:						# right wall longer
						cost_d_lgw = rdist[0]
			## Sum of cost
			G.edges[e]['cost'] = cost_dist + cost_motion + (cost_d_bh + cost_d_qr + cost_d_qp + cost_d_qy) + cost_d_lgw*0.01

		# nx.set_edge_attributes(G, 1, 'cost') 	# set cost based on vertical distance to next cell
		
	def remove_edges(self, G):
		""" remove edges with obstacles """

		e_init = G.number_of_edges()	# determine number of edges
		r_edge = [] 					# list for edges to remove
		
		for e in G.edges():
			# if either cell contains obstacles, remove edge
			if (self.Map.nodes[e[0]]['height']!= 0 or self.Map.nodes[e[1]]['height']!=0): 
				# stack into list to remove
				r_edge.append(e)
				
		for i in range(0,len(r_edge)):
			G.remove_edge(r_edge[i][0],r_edge[i][1])

	def graph_to_nparray(self):
		""" converts NetworkX graph to array """

		## Define Variables ##
		sz = self.Map.number_of_nodes() # size of grid map
		cx = np.zeros(sz)
		cy = np.zeros(sz)
		cz = np.zeros(sz)

		i = 0
		for p in self.Map.nodes():
			cell_h = self.Map.nodes[p]['height']
			if (cell_h >= 0.5):
				# Append to array walls
				cx[i] = p[0]*self.resolution
				cy[i] = p[1]*self.resolution
				cz[i] = self.Map.nodes[p]['height']*0.9
				# stack cells vertically until height of cell
				for n in range(0,int(cz[i]/self.resolution)):
					cx = np.append(cx,cx[i])
					cy = np.append(cy,cy[i])
					cz = np.append(cz,self.resolution*n)
			elif (cell_h >= MIN_FLAT_H and cell_h <= MAX_FLAT_H):
				# Append to array ground
				cx[i] = p[0]*self.resolution
				cy[i] = p[1]*self.resolution
				cz[i] = self.Map.nodes[p]['height']
			else:
				# Ignore holes, leave them blank
				cx[i] = -1000
				cy[i] = -1000
				
				
			i += 1

		return cx, cy, cz

	def graph_attributes_to_nparray(self, attr):
		""" returns map attributes as 1D array """

		## Define Variables ##
		sz = self.Map.number_of_nodes() # size of grid map
		idx = 0

		try:
			depth = len(self.Map.nodes[(0,0)][attr])
		except TypeError:
			depth = 1
		
		cx = np.zeros(sz*depth)
		for x in range(0, self.map_size_g[0]):
			for y in range(0, self.map_size_g[1]):
				cx[idx:idx+depth] = self.Map.nodes[(x,y)][attr]
				# print (x,y), idx, cx[idx:idx+depth], sz, self.Map.nodes[(x,y)][attr]
				idx += depth
		return cx, depth

	def search_area(self, p, grid_area, j):

		# sets the appropriate foothold in index form
		if (j == 0):
			sp = (int(np.floor(p[0]/self.resolution)), int(np.ceil(p[1]/self.resolution)))
		elif (j == 1):
			sp = (int(np.round(p[0]/self.resolution)), int(np.ceil(p[1]/self.resolution)))
		elif (j == 2):
			sp = (int(np.ceil(p[0]/self.resolution)), int(np.ceil(p[1]/self.resolution)))
		elif (j == 3):
			sp = (int(np.floor(p[0]/self.resolution)), int(np.floor(p[1]/self.resolution)))
		elif (j == 4):
			sp = (int(np.round(p[0]/self.resolution)), int(np.floor(p[1]/self.resolution)))
		elif (j == 5):
			sp = (int(np.ceil(p[0]/self.resolution)), int(np.floor(p[1]/self.resolution)))
		
		print 'Search centered at ', np.round(p,3), sp
		result, grid_points = self.Spiral.get_grid(36)
		# print sp, grid_points
		for i in range(0, len(grid_points)):
			grid_points[i] = (grid_points[i][0] + sp[0], grid_points[i][1] + sp[1])
		# print sp, grid_points
		return grid_points

	def graph_representation(self,**options):
		""" Plots graph functions """


		# dictionary of node names->positions
		p_map  = dict(zip(self.Map, self.Map))
		p_wall = dict(zip(self.G_wall, self.G_wall))
		p_hole = dict(zip(self.G_hole, self.G_hole))
		# p_path = dict(zip(self.Gfree, self.Gfree))

		# m_walk = dict(zip(self.GM_walk, self.GM_walk))
		# m_wall = dict(zip(self.GM_wall, self.GM_wall))
		# m_chim = dict(zip(self.GM_chim, self.GM_chim))

		# Grid map & obstacles
		# plt.style.use('presentation')
		nsize  = 20	# size of nodes
		labels = False 	# label on nodes

		# Map
		ax = nx.draw_networkx(self.Map, p_map, with_labels=labels, node_size=nsize, node_color='0.9', width=0.0, alpha=1.0, node_shape="s");
		# Wall
		nx.draw_networkx(self.G_wall, p_wall, with_labels=labels, node_size=nsize, node_color='#ff0000', width=0.0, node_shape="s");
		# Holes 
		nx.draw_networkx(self.G_hole, p_hole, with_labels=labels, node_size=nsize, node_color='#663300', width=0.0, node_shape="s");
		
		# Motion primitives
		if (options.get('gprim') is not None):
			print 'in prim'
			gprim = options.get('gprim')
			m_walk = dict(zip(gprim[0], gprim[0]))
			m_wall = dict(zip(gprim[1], gprim[1]))
			m_chim = dict(zip(gprim[2], gprim[2]))
		
			# nx.draw_networkx(gprim[0],m_walk,with_labels=labels,node_size=nsize, node_color='#ff6600',	width=0.0);
			# nx.draw_networkx(gprim[1],m_wall,with_labels=labels,node_size=nsize, node_color='#00ccff',	width=0.0);
			nx.draw_networkx(gprim[2],m_chim,with_labels=labels,node_size=nsize, node_color='#66ff00',	width=0.0);
		# nx.draw_networkx(self.GM_walk,m_walk,with_labels=labels,node_size=nsize, node_color='#ff6600',	width=0.0);
		# nx.draw_networkx(self.GM_wall,m_wall,with_labels=labels,node_size=nsize, node_color='#00ccff',	width=0.0);
		# nx.draw_networkx(self.GM_chim,m_chim,with_labels=labels,node_size=nsize, node_color='#66ff00',	width=0.0);

		# Feasible path - single point
		if options.get("gpath") is not None:
			p_path = dict(zip(gpath, gpath))
			if (type(gpath) == list):
				gpath = nx.path_graph(gpath)
		
			nx.draw_networkx(gpath, p_path, with_labels=labels, node_size=10, node_color='#ff33ff', edge_color='#ff33ff', alpha=1.0, width=1.0);

		# else:
		# 	# Feasible path - single point
		# 	# nx.draw_networkx(gpath,p_fin, with_labels=labels,node_size=20,node_color='#ff6600',edge_color='#ff6600',alpha=1.0,width=5.0);
		# 	pass

		## enable major and minor grid
		plt.grid('on');		 plt.grid(which='major', linestyle=':', linewidth='0.5', color='black')
		# plt.minorticks_on(); plt.grid(which='minor', linestyle=':', linewidth='0.25', color='black')

		# minor_ticks = np.arange(0, 9+1, 0.5)
		# plt.xticks(minor_ticks)

		plt.xlabel('x, [1]')
		plt.ylabel('y, [1]')

		# maximise plot window
		# print plt.get_backend()
		# mng = plt.get_current_fig_manager()
		# mng.resize(*mng.window.maxsize())
		plt.tight_layout()
		plt.show() 
		# fig_manager.resize(*fig_manager.window.maxsize())

## ================================================================================================ ##
## 												TESTING 											##
## ================================================================================================ ##
map_offset = (0.33, 0.39)
ps = np.array([map_offset[0], map_offset[1], 0.1, 0., 0., 0.]).reshape(6,1)
p = [0.374, 0.745,      0.5       ]
p = [0.915, 0.745,      0.5       ]
gmap = GridMap('chimney_corner_053')
print gmap.get_cell('norm', p, 3)
# print gmap.getIndex(p, 0)
 
# print (26.%7.)