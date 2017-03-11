from collections import deque
import numpy as np

class BreadthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()
        
    def Plan(self, start_config, goal_config):
        plan = []

        # initialize plot (only for Simple Environment visualization)
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)

        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        queue = deque([start_id])
        success = False
        while success is not True and queue:
            # queue not empty
            curr_id = queue.popleft()
                
            neighbors = self.planning_env.GetSuccessors(curr_id)
            for elem in neighbors:
                if elem == goal_id:
                    self.nodes[goal_id] = curr_id
                    success = True
                    break
                # check if the (collision free) neighbor is visited
                # collision check should be done in GetSuccessors function 
                elif elem not in self.nodes:
                    self.nodes[elem] = curr_id
                    queue.append(elem)
                    if self.visualize:
                        self.planning_env.PlotEdge(self.planning_env.discrete_env.NodeIdToConfiguration(curr_id), \
                                                   self.planning_env.discrete_env.NodeIdToConfiguration(elem), 'k')
        print "nodes expanded: %d" % len(self.nodes)
        plan.append(goal_config)
        node_id = goal_id 
        while self.nodes[node_id] != start_id:
            node_id = self.nodes[node_id]
            node_config = self.planning_env.discrete_env.NodeIdToConfiguration(node_id)
            plan.append(np.array(node_config))

        plan.append(start_config)

        plan.reverse()

        return plan

