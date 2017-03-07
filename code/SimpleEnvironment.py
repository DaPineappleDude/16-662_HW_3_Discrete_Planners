import numpy as np
import pylab as pl
import envcommon as ec
from DiscreteEnvironment import DiscreteEnvironment

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

        
