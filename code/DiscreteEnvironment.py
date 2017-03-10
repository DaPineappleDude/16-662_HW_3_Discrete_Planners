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
			self.num_cells[idx] = np.ceil((upper_limits[idx] - lower_limits[idx])/resolution)+1




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
		for idx in range(self.dimension):
			coord[idx] = (config[idx] - self.lower_limits[idx])
			coord[idx] = round(coord[idx]/self.resolution)
			#coord[idx] = round(coord[idx]*self.num_cells[idx])
		return coord

	def GridCoordToConfiguration(self, coord):
		
		# TODO: 
		# This function smaps a grid coordinate in discrete space
		# to a configuration in the full configuration space
		#
		config = [0] * self.dimension
		for idx in range(self.dimension):
			config[idx] = self.lower_limits[idx] + coord[idx] * self.resolution
		return config

	def GridCoordToNodeId(self,coord):
		
		# TODO: DONE
		# This function maps a grid coordinate to the associated
		# node id 
		node_id = 0
		num_cells_factor = 1
		
		for idx in range(self.dimension):
			node_id += coord[idx]*num_cells_factor
			num_cells_factor = num_cells_factor*self.num_cells[idx] 
		return node_id

	def NodeIdToGridCoord(self, node_id):
		
		# TODO:
		# This function maps a node id to the associated
		# grid coordinate
		coord = [0] * self.dimension
		rem = node_id
		product = np.prod(np.array(self.num_cells))
		for idx in xrange(self.dimension, 0, -1):
			idx = idx - 1 
			product  = (product/self.num_cells[idx])
			coord[idx] = int(rem/product)
			rem = rem%product
		return coord



		
