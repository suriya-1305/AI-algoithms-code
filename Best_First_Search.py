from myutils import *
inf = float('inf')
class Node:
    def __init__(s, state, par=None, action=None, pathcost=0):
        s.state = state
        s.par = par
        s.action = action
        s.pathcost = pathcost
        s.f=0
        s.depth = 0
        if par:
            s.depth = par.depth + 1
    def __repr__(s):
        return "{}".format(s.state)
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
    def value(s, state):
        raise NotImplementedError
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
            if type(node) is str:
                return int(distance(locs[node],locs[s.goal]))
            return int(distance(locs[node.state],locs[s.goal]))
        else:
            return inf
  
def UndirectedGraph(graphdict=None):
    return Graph(graphdict = graphdict, directed=False)
def mymax(childf,nodef, child,node):
    if childf>=nodef:
        return childf
    else:      
        return nodef
def Recur_Best_First_Search(prob):
    startnode = Node(prob.initial)
    startnode.f = prob.h(prob.initial)
    return Best_First_Search(prob,startnode,inf)

def Best_First_Search(prob, node,f_limit):
    if prob.goal_test(node.state):
        return [node, None]  
    successors=[]
    for child in node.expand(prob):
        gval=child.pathcost
        hval=prob.h(child)
        child.f=mymax(gval+hval , node.f,child, node)
        successors.append(child)    
    if len(successors) == 0 :
        return [None,inf]
    while True:        
        best=lowfvaluenode(successors)
        if best.f > f_limit :
            return [None,best.f]
        alter=secondlowfvaluenode(successors, best.f)        
        x = Best_First_Search(prob, best, min(f_limit, alter))
        result = x[0]               
        best.f = x[1]                    
        if result != None :
            return [result, None]     
def lowfvaluenode(nodelist):
    minfval = nodelist[0].f
    minFvalIndex=0    
    for n in range(1,len(nodelist)):        
        if nodelist[n].f < minfval :
            minFvalIndex = n
            minfval=nodelist[n].f
    return nodelist[minFvalIndex]
def secondlowfvaluenode(nodelist,lowest_f): 
    secondminfval = inf
    for n in range(0,len(nodelist)):        
        if nodelist[n].f > lowest_f and nodelist[n].f < secondminfval :            
            secondminfval = nodelist[n].f
    return secondminfval
       
Tnmap = UndirectedGraph(
    {'Chennai': {'puducherry': 75, 'salem': 140, 'kanyakumari': 118},     
     'salem': {'vellore': 85, 'Chennai': 101, 'ooty': 90},
     'vellore': {'coimbatore': 120, 'puducherry': 146, 'salem': 138},
     'coimbatore': {'trichy': 75, 'vellore': 120},
     'trichy': {'ooty': 86},
     'ooty': {'coimbatore': 98, 'trichy': 86},
     'puducherry': {'Chennai': 80, 'vellore': 146, 'kanyakumari': 97},
     'kanyakumari': {'Chennai': 118, 'puducherry': 111}})
Tnmap.locations = dict( Chennai=(91, 492), salem=(400, 327), vellore=(253, 288),
    coimbatore=(165, 299), trichy=(562, 293), ooty=(534, 350), puducherry=(233, 410),
    kanyakumari=(94, 410))
print("\nSolving for ooty to kanyakumari...")
TnMapProb=Graphprob('ooty','kanyakumari', Tnmap)
result= Recur_Best_First_Search(TnMapProb)
if(result[0] != None ):
    print("Path taken :" , result[0].path())
    print("Path Cost :" , result[0].pathcost)

