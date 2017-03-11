import operator

class RRTTree(object):
    
    def __init__(self, planning_env, start_config):
        
        self.planning_env = planning_env
        self.vertices = []
        self.vertices.append(start_config)
        self.edges = dict()

    def GetRootId(self):
        return 0

    def GetNearestVertex(self, config):
        
        nodeId1 = self.planning_env.discrete_env.ConfigurationToNodeId(config)

        dists = []
        for v in self.vertices:
            nodeId2 = self.planning_env.discrete_env.ConfigurationToNodeId(v)
            dists.append(self.planning_env.ComputeDistance(nodeId1, nodeId2))

        vid, vdist = min(enumerate(dists), key=operator.itemgetter(1))

        return vid, self.vertices[vid]
            

    def AddVertex(self, config):
        vid = len(self.vertices)
        self.vertices.append(config)
        return vid

    def AddEdge(self, sid, eid):
        self.edges[eid] = sid

