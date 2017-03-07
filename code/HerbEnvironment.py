import numpy
import envcommon as ec
from DiscreteEnvironment import DiscreteEnvironment

class HerbEnvironment(object):
    
    def __init__(self, herb, resolution):
        
        self.robot = herb.robot
        self.lower_limits, self.upper_limits = self.robot.GetActiveDOFLimits()
        print self.lower_limits
        print self.upper_limits
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # account for the fact that snapping to the middle of the grid cell may put us over our
        #  upper limit
        upper_coord = [x - 1 for x in self.discrete_env.num_cells]
        upper_config = self.discrete_env.GridCoordToConfiguration(upper_coord)
        for idx in range(len(upper_config)):
            self.discrete_env.num_cells[idx] -= 1

        # add a table and move the robot into place
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 0.7], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)
        
        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
    def GetCollision(self, config):

        self.robot.SetActiveDOFValues(config)
        rigidBody = self.robot.GetEnv().GetBodies()
        bodyNum = len(rigidBody)

        if bodyNum == 2: 
            check = self.robot.GetEnv().CheckCollision(rigidBody[0], rigidBody[1])
    
        elif bodyNum == 3: 
            check = self.robot.GetEnv().CheckCollision(rigidBody[0],rigidBody[1],rigidBody[2])
    
        if check is not False and self.robot.CheckSelfCollision() is not False:
            collision = False
        else:
            collision = True

        return collision 

    def GetSuccessors(self, node_id):

        successors = []

        node_config = self.discrete_env.NodeIdToConfiguration(node_id)
        #print "node_id in GetSuccessors: %d" % node_id
        #print node_config
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

