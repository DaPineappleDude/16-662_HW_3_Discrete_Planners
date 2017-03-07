from collections import deque

class BreadthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize

        self.nodes = dict()
        # Key =  node_id (visited)
        # Val =  parent node_id
        
    def Plan(self, start_config, goal_config):
        
        plan = []

        # TODO: Here you will implement the breadth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_id  = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        queue = deque([start_id])
        
        while len(queue) != 0:
        # queue not empty
            curr_id = queue.popleft() # dequeue
                
            neighbors = planning_env.GetSuccessors(curr_id)
            for elem in range(len(neighbors)):
                if elem == goal_id:
                    self.nodes[goal_id] = elem
                    break
                # check if the (collision free) neighbor is visited
                # collision check should be done in GetSuccessors function 
                if elem not in self.nodes:
                    self.nodes[elem] = curr_id
                    queue.append(elem)
        
        # initialize plot (only for Simple Environment visualization)
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        
        # generate execution plan
        plan.append(goal_config)
        node_id = goal_id 
        last_id = 0
        while node_id != start_id:
            last_id = node_id
            node_id = self.nodes.get(node_id)
            node_config = self.planning_env.discrete_env.NodeIdToConfiguration(node_id)
            last_config = self.planning_env.discrete_env.NodeIdToConfiguration(last_id)
            plan.append(node_config)
            # only for Simple Environment visualization 
            self.planning_env.PlotEdge(node_config, last_config)

        # plan.append(start_config) 
        # generate plan from start_config to end_config   
        plan = plan.reverse()   

        return plan
