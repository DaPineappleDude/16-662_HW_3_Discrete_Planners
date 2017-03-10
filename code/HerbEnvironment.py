import numpy
import envcommon as ec
from DiscreteEnvironment import DiscreteEnvironment

class HerbEnvironment(object):
    
    def __init__(self, herb, resolution):
        
        self.robot = herb.robot
        self.lower_limits, self.upper_limits = self.robot.GetActiveDOFLimits()
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
        self.p = 0.0

    def GetCollision(self, config):

        self.robot.SetActiveDOFValues(config)
        rigidBody = self.robot.GetEnv().GetBodies()
        bodyNum = len(rigidBody)
        
        if bodyNum == 2: 
            check = self.robot.GetEnv().CheckCollision(rigidBody[0], rigidBody[1])
    
        elif bodyNum == 3: 
            check = self.robot.GetEnv().CheckCollision(rigidBody[0],rigidBody[1],rigidBody[2])
    
        if check is False and self.robot.CheckSelfCollision() is False:
            collision = False
        else:
            collision = True
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

#
#HERB ENVIRONMENT HRRT METHODS
#
#
#
    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p
 
    def HRRTComputeDistance(self, start_config, end_config):
        
        start_config_temp = numpy.array(start_config)
        end_config_temp   = numpy.array(end_config)
        
        return numpy.linalg.norm(start_config_temp - end_config_temp)
        pass

       

    def GenerateRandomConfiguration(self):
        config = [0] * len(self.robot.GetActiveDOFIndices())
            
        #
        # TODO: Generate and return a random configuration
        #
        s = self.robot.GetActiveDOFValues()
        lower_limits, upper_limits = self.robot.GetActiveDOFLimits()
        while True:
            config = numpy.multiply(numpy.subtract(upper_limits, lower_limits), numpy.random.random_sample((len(config),))) +  numpy.array(lower_limits)
            self.robot.SetActiveDOFValues(config)
            if(not((self.robot.GetEnv().CheckCollision(self.robot, self.robot.GetEnv().GetKinBody('conference_table'))))):
                self.robot.SetActiveDOFValues(s)
                return numpy.array(config)

    def Extend(self, start_config, end_config):
        
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #
        s = self.robot.GetActiveDOFValues()
        num_steps = 10
        epsilon = 0.01
        dist = self.HRRTComputeDistance(start_config, end_config)
        step_size = dist/num_steps
        
        direction = (end_config - start_config)/dist
        
        config = start_config + step_size*direction
        lower_limits, upper_limits = numpy.array(self.robot.GetActiveDOFLimits())

        lower_limits = lower_limits.tolist()
        upper_limits = upper_limits.tolist()
        
        steps = 1
        while True:
            self.robot.SetActiveDOFValues(config)
            #Check if the first step is out of limits, return None
            if((steps==1) and ((config.tolist() < lower_limits) or (config.tolist() > upper_limits))):
                self.robot.SetActiveDOFValues(s)
               # print "none"
                return None
            #Check if the first step collides then return None
            elif((steps==1) and (self.robot.GetEnv().CheckCollision(self.robot.GetEnv().GetBodies()[0], self.robot.GetEnv().GetBodies()[1]) or self.robot.CheckSelfCollision())):
                self.robot.SetActiveDOFValues(s)
              #  print "none"
                return None
            #If config is out of limits, return previous step
            elif((config.tolist() < lower_limits) or (config.tolist() > upper_limits)):
                self.robot.SetActiveDOFValues(s)
              #  print  'out of limits'
                return config - step_size*direction
            #If collision occured later, return previous step
            elif((self.robot.GetEnv().CheckCollision(self.robot.GetEnv().GetBodies()[0], self.robot.GetEnv().GetBodies()[1]) or self.robot.CheckSelfCollision())):
                self.robot.SetActiveDOFValues(s)
              #  print  'return previous step'
                return config - step_size*direction

            if numpy.all((numpy.subtract(config, end_config) < epsilon)):
                self.robot.SetActiveDOFValues(s)
               # print 'end config'
                return end_config
            config += step_size*direction
            steps += 1
        pass
        

    def ShortenPath(self, path, timeout=5.0):
        
        # 
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the 
        #  given timout (in seconds).
        #
        delta = 1
        current_time = time.clock()

        while(not(delta > 5)):
            
            leng = len(path)
            g = random.randint(1, leng)
            h = random.randint(1, leng-2)

            while(g >= h):
                g = random.randint(1, leng)
                h = random.randint(1, leng-2)
            
            ##### two points ######

            first_vertex =  ((numpy.array(path[g]) +  numpy.array(path[g-1]))/2).tolist()
            second_vertex = ((numpy.array(path[h]) +  numpy.array(path[h+1]))/2).tolist()
            
            config = self.Extend(numpy.asarray(first_vertex), numpy.asarray(second_vertex))
            
            if(config != None):
                #print vertex, vertex.dtype, local_path[ul], local_path[ul].dtype
                if all(map(lambda v: v in config, numpy.asarray(second_vertex))):
            
                    path[g] = first_vertex
                    path[h] = second_vertex
                    for remove_ind in range(g+1, h):
                        path.pop(g+1)
            delta = time.clock()  - current_time

        distance_final = 0;
        print "final vertices", len(path)
        for i in range(len(path)-1):
            distance_final += self.HRRTComputeDistance(path[i],path[i+1])
        print  "shortened distance", distance_final
        return path


