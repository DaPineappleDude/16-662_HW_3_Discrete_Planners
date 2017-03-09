import sets
import collections

class AStarPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()

    def heuristic (self, start_id, end_id):
        cost = self.planning_env.ComputeHeuristicCost(start_id, end_id)
        return cost

    def distance (self, start_id, goal_id):
        dist = self.planning_env.ComputeDistance(start_id, goal_id)
        return dist
    
    def getConfig (self, iid):
        config = self.planning_env.discrete_env.NodeIdToConfiguration(iid)
        return config

    def getLowestId (self, iid, costs):
        val = 0.0
        for key in costs:
            tempval = costs[key]
            if tempval < val:
              val = tempval
              curr_id = key
        return curr_id      

    def Plan(self, start_config, goal_config):

        plan = []
        
        # TODO: Here you will implement the AStar planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
            goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)

            # Open /closed list of nodes / ids
            openList = collections.deque([start_id])
            closedList = collections.deque()
            # Dictionary of edges
            edges = {}
            # evaluation cost
            ecosts = {start_id = 0}
            # operating cost
            ocosts = {start_id = 0}

            curr_id = start_id
            while openList:
                # Save previous id to compoute operating cost
                prev_id = curr_id
                # Get lowest evaluation cost ID
                curr_id = self.getLowestId (curr_id, ecosts)
                # Remove the best curr_id from open, add to closed
                openList.remove(curr_id)
                closedList.append(curr_id)
                # Get neighbors
                neighbors = self.planning_env.GetSuccessors(curr_id)
#                # Initialize the lowest neighbor heuristic cost
#                bestCost = None

                # Compute operating cost
#                ocosts[curr_id] = self.distance(prev_id, curr_id) + ocosts[prev_id]

                # early exit
                if curr_id == goal_id:
                  break

                for neighbor in neighbors:
                    # If neighbor is not visited yet; otherwise, move to next neighbor
                    if neighbor not in closedList:
                        ocosts[neighbor] = self.distance(curr_id, neighbor)

                        # If neighbor isn't in openlist, add it, precompute evaluation function
                        if neighbor not in openList:
                            openList.append(neighbor)
                            ecosts[neighbor] = self.heuristic(neighbor, goal_id) + ocosts[neighbor]

                        else:
                            print curr_id
                            print neighbor
                            


                            if bestCost == None or fcost < bestCost:
                                bestCost = fcost
                                edges[neighbor] = curr_id

                                if self.visualize:
                                    self.planning_env.PlotEdge(self.planning_env.discrete_env.NodeIdToConfiguration(curr_id),\
                                                               self.planning_env.discrete_env.NodeIdToConfiguration(neighbor), 'k')
                          
        plan_id = goal_id

        while plan_id != start_id:
            config = self.getConfig(plan_id)
            plan.append(config)
            import pdb; pdb.set_trace()
            plan_id = edges[plan_id]
            
        plan = plan[::-1]
        return plan
