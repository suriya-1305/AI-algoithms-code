vertex=['0','1','2','3','4','5','6']
edge=[(0,1),(0,2),(1,0),(1,3),(2,0),(2,4),(2,5),(3,1),(4,2),(4,6),(5,2),(6,4)]
graph=(vertex,edge)

def dfs(g,s):
    vertex,edge=g
    Visi=[]
    stack=[s]
    adj=[[] for v in vertex]
    for edge in edge:
        adj[edge[0]].append(edge[1])
    while stack:
        curr=stack.pop()
        for neigh in adj[curr]:
            if not neigh in Visi:
                stack.append(neigh)
        Visi.append(curr)
    return Visi
print(dfs(graph, 0))
