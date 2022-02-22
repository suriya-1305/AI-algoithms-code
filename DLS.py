from _collections import defaultdict
class Graph:
    def __init__(self,v):
        self.ver=v
        self.graphlist=defaultdict(list)
        
    def AddEdge(self,u,v):
        self.graphlist[u].append(v)
        
    def DLS(self,s,t,d):
        if s==t: return True

        if d<= 0: return False

        for i in self.graphlist[s]:
            if(self.DLS(i,t,d-1)):
                return True
        return False

    def IDDFS(self,s,t,d):
        for i in range(d):
            if(self.DLS(s,t,i)):
                return True
        return False

verti=int(input("enter no of vertices"))
g=Graph(verti)
for i in range(verti):
    x=int(input("enter 1st value"))
    y=int(input("enter 2nd value"))
    g.AddEdge(x,y)    
s=int(input("Enter source"))
t=int(input("enter target"))
d=int(input("enter depth value"))

if g.IDDFS(s,t,d)== False:
    for i in range(6):
        d=int(input("enter depth limit"))
        if g.IDDFS(s,t,d)== True:
            print("Target is reachable")
            break
        else:
            print("Target is not reachable")
else:
    print("Target is reachable within limit")
