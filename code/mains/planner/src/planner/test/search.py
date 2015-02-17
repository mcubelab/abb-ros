import copy
from random import choice

class GraphSearch:

    def __init__(self, graph, a):
        self.graph = graph
        self.states = self.graph.keys()
        self.actions = a

    def dijkstra(self, start, goal):
        vertices = self.graph.keys()
        dist = {}
        pred = {}
        for v in vertices:
            dist[v] = 0
            pred[v] = None
        
        q = copy.deepcopy(vertices) 
        dist[start] = 1

        while q:
            u = max(q, key=dist.get)
            q.remove(u)
            
            for neighbor in self.graph[u].keys():
                if dist[u] == 0:
                    alt = self.graph[u][neighbor]
                else:
                    alt = dist[u] * self.actions[self.graph[u][neighbor]]
                if alt > dist[neighbor]:
                    dist[neighbor] = alt
                    pred[neighbor] = u

        return (dist, pred)

    def dijkstraShortestPath(self, start, goal):
        end = copy.deepcopy(goal)
        (distances, predecessors) = self.dijkstra(start, end)
        
        path = []
        while True:
            path.append(end)
            if end == start:
                break
            #print "start: "+str(start) + " pred: "+str(predecessors[end])
            end = predecessors[end]
        path.reverse()
        return path

    def getEpisode(self, policy, start, goal):
        end = copy.deepcopy(goal)

        path = []
        while True:
            path.append((end, policy[end]))
            if end == start:
                break
            end = policy[end]
        path.reverse()
        return path

    def MCEstimationES(self, start end):
        # initialize, for all s in S, a in A
        # Q(s,a) < arbitrary
        # pi(s) < arbitrary
        # Returns(s,a) < empty list
        #
        # Repeat forever:
        # (a) generate an episode using exploring starts and pi
        # (b) for each pair s,a appearing in the episode
        #     R < return following the first occurrence of s,a
        #     append R to Returns(s,a)
        #     Q(s,a) < average(Returns(s,a))
        # (c) for each s in the episode: pi(s) < arg max_a Q(s,a)

        q = {}
        Returns = {}
#        (dist, pred) = self.dijkstra(start, end)
        policy[state] = choice(self.actions)
#        p = pred
        for s in self.states:
            for a in self.actions:
                q[(s,a)] = 0
                Returns[(s,a)] = []

        while True:
            randoStart = choice(self.states)
            ep = getEpisode(policy, randoStart, end)

            for (s,a) in ep
            if len(Returns[(s,a)]) == 0:
                r = 
            Returns[(s,a)].append(r)
            q[(s,a)] = sum(Returns[(s,a)])/len(Returns([s,a]))
            for s in ep:
                policy[s] = max(Q.keys(), key=Q.get)[1]
            
if __name__ == '__main__':

    # TEST CASE
    # v = ['a','b','c','d','e']
    # g = {}
    # temp = {}
    # for v1 in v:
    #     temp[v1] = 0
    # for v2 in v:
    #     g[v2] = copy.deepcopy(temp) 
    # g['a']['b'] = 1
    # g['b']['a'] = 1
    # g['b']['d'] = 1
    # g['d']['b'] = 1
    # g['c']['d'] = 2
    # g['d']['c'] = 2
    # g['c']['e'] = 1
    # g['e']['c'] = 1
    # g['a']['e'] = 2
    # g['e']['a'] = 2
    # g['b']['e'] = 3
    # g['e']['b'] = 3

    #print "graph: "+str(g)

    #gs = GraphSearch(g)
    #p = gs.dijkstraShortestPath('a','c')
    # print "path: "+str(p)

    # TRIANGLE WORLD
    flatWorld = "flat world"
    longWorld = "long world"
    flatFingertip = "flat fingertip"
    longFingertip = "long fingertip"
    flatPalm = "flat palm"
    longPalm = "long palm"
    actions = {'pick': 0.9, 'place': 1.0, 'rollToGround': 0.9, 'throwToPalm': 0.8, 
               'throwToFingertip': 0.7, 'throwAndFlip': 0.7, 'rollToPalm': 0.95, 
               'end': 1}
 
    g = {}
    g[flatWorld] = {flatFingertip: pick}
    g[flatFingertip] = {flatWorld: place, flatPalm: throwToPalm, 
                     longWorld: rollToGround, longPalm: rollToPalm}
    g[flatPalm] = {longPalm: throwAndFlip, flatFingertip: throwToFingertip}
    g[longPalm] = {longFingertip: throwToFingertip}
    g[longFingertip] = {longPalm: throwToPalm, flatFingertip: place*pick, 
                        longWorld: place}
    g[longWorld] = {longFingertip: pick}


    
    #print "graph: "+str(g)
    gs = GraphSearch(g, [pick,place,throwToPalm,rollToPalm,throwToFingertip,
                         throwAndFlip,rollToGround, end])
    p = gs.dijkstraShortestPath(longWorld, flatPalm)
    print "path: "+str(p)
