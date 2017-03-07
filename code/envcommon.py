import numpy as np
import numpy.linalg as la
import time

def cost(p1, p2):
    
    return sum(abs(np.array(p1)-np.array(p2)))

def neighbors(config, env, check):
    valid_neighbors = []
    for dim in range(env.dimension):
       neighbor = config[:]
       neighbor[dim] += env.resolution
       neighbor[dim] = round(neighbor[dim], 10)
       if(neighbor[dim] < env.upper_limits[dim] and check(neighbor) is False):
           valid_neighbors.append(neighbor)

       neighbor = config[:]
       neighbor[dim] -= env.resolution
       neighbor[dim] = round(neighbor[dim], 10)
       if(neighbor[dim] > env.lower_limits[dim] and check(neighbor) is False):
           valid_neighbors.append(neighbor)
      
    #print valid_neighbors 
    return valid_neighbors
