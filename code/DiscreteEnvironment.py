import numpy as np

class DiscreteEnvironment(object):

	def __init__(self, resolution, lower_limits, upper_limits):

		# Store the resolution
		self.resolution = resolution

		# Store the bounds
		self.lower_limits = lower_limits
		self.upper_limits = upper_limits

		# Calculate the dimension
		self.dimension = len(self.lower_limits)

		# Figure out the number of grid cells that are in each dimension
		self.num_cells = self.dimension*[0]
		for idx in range(self.dimension):
			self.num_cells[idx] = np.ceil((upper_limits[idx] - lower_limits[idx])/resolution)

		self.node_convrsn = self.dimension*[1]
		for idx in range(self.node_convrsn):
			self.node_convrsn[idx] = self.num_cells[idx]**idx




	def ConfigurationToNodeId(self, config):
		
		# TODO: DONE
		# This function maps a node configuration in full configuration
		# space to a node in discrete space
		#
		node_id = 0
		node_id = self.GridCoordToNodeId(self.ConfigurationToGridCoord(config))
		return node_id

	def NodeIdToConfiguration(self, nid):
		
		# TODO: DONE
		# This function maps a node in discrete space to a configuraiton
		# in the full configuration space
		#
		config = [0] * self.dimension
		config = self.GridCoordToConfiguration(self.NodeIdToGridCoord(nid))
		return config
		
	def ConfigurationToGridCoord(self, config):
		
		# TODO: DONE
		# This function maps a configuration in the full configuration space
		# to a grid coordinate in discrete space
		#
		coord = [0] * self.dimension
		coord = round(np.dot((config - np.asarray(self.lower_limits))/(np.asarray(self.upper_limits) - np.asarray(self.lower_limits))), (np.asarray(self.num_cells)))
		coord.tolist()
		return coord

	def GridCoordToConfiguration(self, coord):
		
		# TODO: 
		# This function smaps a grid coordinate in discrete space
		# to a configuration in the full configuration space
		#
		config = [0] * self.dimension
		config =  (np.asarray(coord + .5))*((np.asarray(self.upper_limits) - np.asarray(self.lower_limits))/np.asaray(self.num_cells))
		config.tolist()
		return config

	def GridCoordToNodeId(self,coord):
		
		# TODO: DONE
		# This function maps a grid coordinate to the associated
		# node id 
		node_id = 0
		node_id = np.dot(np.transpose(np.asarray(coord)), np.asarray(self.node_convrsn))
		return node_id

	def NodeIdToGridCoord(self, node_id):
		
		# TODO:
		# This function maps a node id to the associated
		# grid coordinate
		#coord = [0] * self.dimension
		rem = node_id
		coord = []
		for idx in xrange(self.dimension, 0):
			dim_id  = (self.num_cells[idx])**idx)
			coord.append(rem//dim_id)
			rem = rem%dim_id
		np.reverse(coord)
		return coord
		
		
		
