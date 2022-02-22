class Graph:
    def __init__(S,graphdict=None,direct=True):
        S.graphdict=graphdict or {}
        S.direct=direct    
    def get(S,a,b=None):
        link=S.graphdict.setdefault(a,{})
        if b is None:
            return link
        else:
            return link.get(b)

class Prob(object):
    def __init__(S,init,goal=None):
       S.init=init
       S.goal=goal
    def acts(S,state):
         raise NotImplementedError
    def result(S,state,act):
        raise NotImplementedError
    def goaltest(S,state):
            return state==S.goal
    def pathcost(S,c,state1,act,state2):
        return c+1
    def value(S,state):
        raise NotImplementedError
infini= float('inf')

class GraphProb(Prob):
    def __init__(S,init,goal,graph):
        Prob.__init__(S,init,goal)
        S.graph=graph
    def acts(S,A):
        return S.graph.get(A)    
    def result(S,state,act):
        return act
    def pathcost(S,cost_so_far, A,act,B):
        return cost_so_far+(S.graph.get(A,B)or infini)

class Node:
    def __init__(S, state,parent=None,act=None,pathcost=0):
        S.state=state
        S.parent=parent
        S.act=act
        S.pathcost=pathcost
        S.depth=0
        if parent:
            S.depth=parent.depth+1
    def __repr__(S):
        return "Node {}".format(S.state)
    def expand(S,prob):
        return [S.childnode(prob,act)
                for act in prob.acts(S.state)]
    def childnode(S,prob,act):
        nextstate=prob.result(S.state,act)
        newcost=prob.pathcost(S.pathcost,S.state,act,nextstate)        
        nextnode=Node(nextstate,S,act,newcost)   
        return nextnode
    def path(S):
        node,pathback=S,[]
        while node:
            pathback.append(node)
            node=node.parent
        return list(reversed(pathback))     
    def solution(S):        
        return [node.state for node in S.path()]


def recurDLS(node,prob,lim):
    if prob.goaltest(node.state):
        return node
    elif lim == 0:
        return 'pass'
    else:
        passoccur = False
        for child in node.expand(prob):
            result=recurDLS(child,prob,lim-1)
            if result=='pass':
                passoccur=True
            elif result is not None:
                return result
        return 'pass' if passoccur else 'Not found'

def DLS(prob,lim=50):
    return recurDLS(Node(prob.init),prob,lim)
    
def iterative_deepening_search(prob,lim):
    for depth in range(0,lim):
        print("checking with depth :",depth)
        result=DLS(prob,depth)
        print("result :",result)

chennaigraph=Graph({
    'Ambattur':{'padi':7,'korattur':9},
    'mogappair':{'mogappair east park':8, 'annanagar':2},
    'thirumullaivoyal':{'avadi':10,'pattabiram':3},    
    },False)
print("----searching from Ambattur to avadi with level 2")
placeProb = GraphProb('ambattur','avadi',chennaigraph)
iterative_deepening_search(placeProb,2)



















