import numpy as np
import pylab as pl
import envcommon as ec
from DiscreteEnvironment import DiscreteEnvironment

import time
import random
import openravepy

import collections
import pdb

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.robot = herb.robot
        self.lower_limits = [-5., -5.]
        self.upper_limits = [5., 5.]
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = np.array([[ 0, 0, -1, 1.5], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

    def GetCollision(self, config):

        pose = np.array([[ 1., 0,  0, 0], 
                         [ 0, 1.,  0, 0], 
                         [ 0, 0,  1., 0], 
                         [ 0, 0,  0, 1.]])
        pose[:2,3] = config
        self.robot.SetTransform(pose)
        collision = self.robot.GetEnv().CheckCollision(self.robot)

        return collision

    def GetSuccessors(self, node_id):

        successors = []

        node_config = self.discrete_env.NodeIdToConfiguration(node_id)

        neighbors_config = ec.neighbors(node_config, self.discrete_env, self.GetCollision)
        for config in neighbors_config:
            successors.append(self.discrete_env.ConfigurationToNodeId(config))
        
        return successors

    def ComputeDistance(self, start_id, end_id):

        start_coord = self.discrete_env.NodeIdToGridCoord(start_id)
        end_coord = self.discrete_env.NodeIdToGridCoord(end_id)

        dist = ec.cost(start_coord, end_coord)

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        start_coord = self.discrete_env.NodeIdToGridCoord(start_id)
        goal_coord = self.discrete_env.NodeIdToGridCoord(goal_id)

        cost = ec.cost(start_coord, goal_coord)

        return cost

    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        pl.xlim([self.lower_limits[0], self.upper_limits[0]])
        pl.ylim([self.lower_limits[1], self.upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')
                    
                     
        pl.ion()
        pl.show()
        
    def PlotEdge(self, sconfig, econfig, color):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                color+'.-', linewidth=2.5)
        pl.draw()



#
#
# H-RRT METHODS
#
#
###################################################

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p
        
    def GenerateRandomConfiguration(self):
        config = [0] * 2;
        lower_limits, upper_limits = self.boundary_limits
        #
        # TODO: Generate and return a random configuration
        #
        s = self.robot.GetTransform()
        while True:
            config = np.multiply(np.subtract(upper_limits, lower_limits), np.random.random_sample((len(config),))) +  np.array(lower_limits)
            snew = self.robot.GetTransform()
            snew[0][3] = config[0]
            snew[1][3] = config[1]
            self.robot.SetTransform(snew)
            #Check if 
            if(not(self.robot.GetEnv().CheckCollision(self.robot.GetEnv().GetBodies()[0], self.robot.GetEnv().GetBodies()[1]))):
                self.robot.SetTransform(s)
                return np.array(config)

#    DIFFERENT DISTANCE
#    def ComputeDistance(self, start_config, end_config):
#        #
#        # TODO: Implement a function which computes the distance between
#        # two configurations
#        #
#        start_config_temp = np.array(start_config)
#        end_config_temp   = np.array(end_config)
#        return np.linalg.norm(start_config_temp - end_config_temp)
#        pass

    def Extend(self, start_config, end_config):
        
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #
        s = self.robot.GetTransform()
        num_steps = 100
        epsilon = 0.001
        dist = self.ComputeDistance(start_config, end_config)
        step_size = dist/num_steps
        
        direction = (end_config - start_config)/dist
        
        config = start_config + step_size*direction
        lower_limits, upper_limits = np.array(self.boundary_limits)

        lower_limits = lower_limits.tolist()
        upper_limits = upper_limits.tolist()
        
        steps = 1

#        print self.robot.GetEnv().GetBodies()
        while True:
            snew = self.robot.GetTransform()
            snew[0][3] = config[0]
            snew[1][3] = config[1]
            
            self.robot.SetTransform(snew)
            #Check if the first step is out of limits, return None
            if((steps==1) and ((config.tolist() < lower_limits) or (config.tolist() > upper_limits))):
                self.robot.SetTransform(s)
            #    print "limits crossed"
                return None
            #Check if the first step collides then return None
            elif((steps==1) and (self.robot.GetEnv().CheckCollision(self.robot.GetEnv().GetBodies()[0], self.robot.GetEnv().GetBodies()[1]))):
                self.robot.SetTransform(s)
            #    print "collision"
                return None
            #If config is out of limits, return previous step
            elif((config.tolist() < lower_limits) or (config.tolist() > upper_limits)):
                self.robot.SetTransform(s)
             #   print "previous step"
                return config - step_size*direction
            #If collision occured later, return previous step
            elif((self.robot.GetEnv().CheckCollision(self.robot.GetEnv().GetBodies()[0], self.robot.GetEnv().GetBodies()[1]))):
                self.robot.SetTransform(s)
              #  print "previous step"
                return config - step_size*direction

            if np.all((self.ComputeDistance(config, end_config) < epsilon)):
                self.robot.SetTransform(s)
               # print "jai mata di"
                return config
            config += step_size*direction
            steps += 1

    def ShortenPath(self, path, timeout=5.0):
        
        # 
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the 
        #  given timout (in seconds).
        #
        delta = 1
        current_time = time.clock()
        
        
        distance_initial = 0;
        for i in range(len(path)-1):
            distance_initial += self.ComputeDistance(path[i],path[i+1])
        
        print distance_initial, "initial distance"
        
        while(not(delta > 5)):
            
            leng = len(path)
            g = random.randint(1, leng)
            h = random.randint(1, leng-2)

            while(g >= h):
                g = random.randint(1, leng)
                h = random.randint(1, leng-2)

            ##### two points ### 
            first_vertex =  ((np.array(path[g]) +  np.array(path[g-1]))/2).tolist()
            second_vertex = ((np.array(path[h]) +  np.array(path[h+1]))/2).tolist()
            
            config = self.Extend(np.asarray(first_vertex), np.asarray(second_vertex))
            
            if(config != None):
                #print vertex, vertex.dtype, local_path[ul], local_path[ul].dtype
                #print config, np.asarray(second_vertex), "hey"

                if (self.ComputeDistance(config, np.asarray(second_vertex)) < 0.00001):
                #if all(map(lambda v: v in config, np.asarray(second_vertex))):
            
                    path[g] = first_vertex
                    path[h] = second_vertex
                    for remove_ind in range(g+1, h):
                        path.pop(g+1)
            delta = time.clock()  - current_time
        distance_final = 0;
        print "Final vertices:", len(path) 
        for i in range(len(path)-1):
            distance_final += self.ComputeDistance(path[i],path[i+1])
        for points in xrange(len(path)-1):
            self.PlotEdge(path[points], path[points+1], 'g')           

        print "shortened distance:", distance_final
        return path
