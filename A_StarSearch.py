from myutils import *
inf = float('inf')
class Node:
    def __init__(s, state, par=None, action=None, pathcost=0):
        s.state = state
        s.par = par
        s.action = action
        s.pathcost = pathcost
        s.depth = 0
        if par:
            s.depth = par.depth + 1
    def __repr__(s):
        return "<Node {}>".format(s.state)
    def expand(s, prob):
        return [s.child_node(prob, action)
                for action in prob.actions(s.state)]
    def child_node(s, prob, action):
        nextstate = prob.result(s.state, action)
        newcost = prob.pathcost(s.pathcost, s.state,action, nextstate)
        nextnode = Node(nextstate, s, action,newcost )      
        return nextnode
    def solution(s):
        return [node.state for node in s.path()]
    def path(s):
        node, pathback = s, []
        while node:
            pathback.append(node)
            node = node.par
        return list(reversed(pathback)) 
class Graph: 
    def __init__(s, graphdict=None, directed=True):
        s.graphdict = graphdict or {}
        s.directed = directed
    def get(s, a, b=None):
        links = s.graphdict.setdefault(a, {})
        if b is None:
            return links
        else:
            return links.get(b)
class prob(object):
    def __init__(s, initial, goal=None):
       s.initial = initial
       s.goal = goal
    def actions(s, state):
         raise NotImplementedError
    def result(s, state, action):
        raise NotImplementedError
    def goal_test(s, state):
        if isinstance(s.goal, list):
            return is_in(state, s.goal)
        else:            
            return state == s.goal
    def pathcost(s, c, state1, action, state2):
        return c + 1
class Graphprob(prob):
    def __init__(s, initial, goal, graph):
        prob.__init__(s, initial, goal)
        s.graph = graph
    def actions(s, A):
        return list(s.graph.get(A).keys())
    def result(s, state, action):
        return action
    def pathcost(s, cost_so_far, A, action, B):
        return cost_so_far + (s.graph.get(A, B) or inf)
    def h(s, node):
        locs = getattr(s.graph, 'locations', None)
        if locs:
            return int(distance(locs[node.state], locs[s.goal]))             
        else:
            return inf
def astar_search(prob): 
    node = Node(prob.initial)
    if prob.goal_test(node.state):
        return node
    gval = node.pathcost 
    hval = prob.h(node) 
    nodelist = [{gval+hval:node}]    
    while nodelist:      
        entrynum = closenode_entrynum(nodelist)        
        mindist = list(nodelist[entrynum].keys())[0] 
        closenode = nodelist[entrynum][mindist] 
        print("Current Nodes : ", nodelist)
        print("min dist = ", mindist, ", closestnode = ", closenode)
        input("Press enter to continue...")
        if prob.goal_test(closenode.state): 
            return closenode        
        nodelist.pop(entrynum)        
        for child in closenode.expand(prob):
            gval = child.pathcost
            hval = prob.h(child)
            nodelist.append( {gval+hval : child})    
def closenode_entrynum(nodelist): 
    minindex=0
    mindist = list(nodelist[0].keys())[0]
    for n in range(1,len(nodelist)):
        dist = list(nodelist[n].keys())[0]
        if dist < mindist :
            minindex=n
            mindist = dist
    return minindex
def UndirectedGraph(graphdict=None):
    return Graph(graphdict = graphdict, directed=False)

Tnmap = UndirectedGraph(
    {'Chennai': {'puducherry': 75, 'salem': 140, 'kanyakumari': 118},     
     'salem': {'vellore': 85, 'chennai': 101, 'ooty': 90},
     'vellore': {'coimbatore': 120, 'puducherry': 146, 'salem': 138},
     'coimbatore': {'trichy': 75, 'vellore': 120},
     'trichy': {'ooty': 86},
     'ooty': {'coimabtore': 98, 'trichy': 86},
     'puducherry': {'chennai': 80, 'vellore': 146, 'kanyakumari': 97},
     'kanyakumari': {'Chennai': 118, 'puducherry': 111}})
Tnmap.locations = dict( Chennai=(91, 492), salem=(400, 327), vellore=(253, 288),
    coimbatore=(165, 299), trichy=(562, 293), ooty=(534, 350), puducherry=(233, 410),
    kanyakumari=(94, 410))
print("\nSolving for Chennai to kanyakumari...")
TnMapProb = Graphprob('Chennai','kanyakumari', Tnmap)
result = astar_search(TnMapProb)
print("Path taken :" , result.path())
print("Path Cost :" , result.pathcost)

