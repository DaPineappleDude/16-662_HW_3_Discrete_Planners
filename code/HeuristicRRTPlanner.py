import numpy
from RRTTree import RRTTree
import pdb


class HeuristicRRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize

    def computeVertexCost(self, vid, vertexCosts, tree): 
#        pdb.set_trace()
        if vid != 0:        
            vertexCost = 0.0
            prevId = tree.edges[vid]
            vertexCost = vertexCosts[vid]# + vertexCosts[prevId]
        else:
            vertexCost = vertexCosts[vid]
        
        return vertexCost
            
    def Plan(self, start_config, goal_config, heuristic, epsilon):
        
        #Default to regular RRT

        tree = RRTTree(self.planning_env, start_config)
        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        plan_temp = []
        plan_temp.append(start_config)
#        tree.AddVertex(start_config)  
        plan_parent = []
        plan_parent.append(start_config);

#        epsilon = .01 
        print "start: ", start_config, " goal: ", goal_config
       
        count = 0;    
        self.env = self.planning_env.robot.GetEnv()
        robot_name = self.env.GetBodies()[0].GetName()
       
        #heuristic stuff
        # initialize first vertex cost (start id)


        # get node ids for goal, start
        startId = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goalId  = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        startHeuristic = self.planning_env.ComputeHeuristicCost(startId, goalId)
        vertexCosts = {0 : startHeuristic}       # compute optimal heuristic cost constant 
        optCost = self.planning_env.ComputeHeuristicCost(startId, goalId)
        # initialize maximum cost
        maxCost= 0.0
        # initialize mFloor
        mFloor = 0.6
          
#        with self.env:
#            with self.planning_env.robot.CreateRobotStateSaver():
        while(1):
#                    import pdb; pdb.set_trace()
            if heuristic:
                r = 1
                mQuality = 0
                vid1 = 1
                while r > mQuality:
                    if count%10 == 0:
                        sample_config = goal_config
                    else:
                        sample_config = self.planning_env.GenerateRandomConfiguration()
                    vid1, nearest_vertex = tree.GetNearestVertex(sample_config) # Finding the nearest vertex
                    vertexNodeId = self.planning_env.discrete_env.ConfigurationToNodeId(nearest_vertex)
                    vertexCost = self.computeVertexCost(vid1, vertexCosts, tree)# + self.planning_env.ComputeHeuristicCost(vertexNodeId, goalId)

#                            vertexCosts[vid1] = vertexCost

                    mQuality = 1.0 - numpy.divide((vertexCost-optCost), (maxCost-optCost))
                    mQuality = min(mQuality, mFloor)
                    r  = numpy.random.random(1)[0]

#                            import pdb; pdb.set_trace()
                    if vid1 == 0:
                        break
#                            import pdb; pdb.set_trace()
            else:
                if count%10 == 0:
                    sample_config = goal_config
                else:
                    sample_config = self.planning_env.GenerateRandomConfiguration()
                
                vid1, nearest_vertex = tree.GetNearestVertex(sample_config) # Finding the nearest vertex
            sample_extend_config = self.planning_env.Extend(nearest_vertex,sample_config, epsilon) # Checking for collision
            if sample_extend_config == None:
                count = count + 1
                continue
    
            plan_parent.append(nearest_vertex)
            plan_temp.append(sample_extend_config)
            vid2 = tree.AddVertex(sample_extend_config)  
            tree.AddEdge(vid1,vid2)
            
            if heuristic:
#                        import pdb; pdb.set_trace()
                prevId = tree.edges[vid2]
                vertexNodeId2 = self.planning_env.discrete_env.ConfigurationToNodeId(sample_extend_config)
                newVertexCost = vertexCosts[prevId] + self.planning_env.ComputeDistance(prevId, vertexNodeId2) + 3*self.planning_env.ComputeHeuristicCost(vertexNodeId2, goalId)
                vertexCosts[vid2] = newVertexCost

                if newVertexCost > maxCost:
                    maxCost = newVertexCost

            if (robot_name != 'Herb2'): # Visualize only for PR2
                self.planning_env.PlotEdge(nearest_vertex,plan_temp[-1])

            if self.planning_env.HRRTComputeDistance(plan_temp[-1],goal_config) < epsilon: # Break Condition
                break
    
            count = count + 1

        plan = []
        plan.append(goal_config)
        plan.append(tree.vertices[-1])
        lastid = vid2
        
#                import pdb; pdb.set_trace()

        while(1):
            nextid = tree.edges[lastid]
                               
#                    nextid = tree.edges.keys()[tree.edges.values().index(lastid)]
            plan.append(tree.vertices[nextid])

            lastid = nextid
            
            if numpy.array_equal(plan[-1],start_config):
                break

        plan = plan[::-1]
        dist = 0.0
        for i in xrange(0,len(plan) - 1):
            dist = dist + numpy.linalg.norm(plan[i + 1] - plan[i]) 
            if (robot_name != 'Herb2'): # Visualize only for PR2
                self.planning_env.PlotEdge(plan[i], plan[i+1], 'r')
        
        print "Path Length: ", dist
        print "Number of vertices: ", len(plan)
        return plan       
