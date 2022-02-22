def BFS(g,s,t,par):
    visi=[False]*len(g)
    q=[]
    q.append(s)
    visi[s]=True
    while q:
        u=q.pop(0)
        for ind in range(len(g[u])):
            if visi[ind] is False and g[u][ind] > 0:
                q.append(ind)
                visi[ind]=True
                par[ind]=u
    return True if visi[t] else False
 
def FordFulkerson(g,source,sink):
    par=[-1]*(len(g))
    maxflow = 0
    while BFS(g,source,sink,par):
        pathflow=float("Inf")
        s=sink
        while s!=source:
            pathflow=min(pathflow,g[par[s]][s])
            s=par[s]
        maxflow +=pathflow
        v=sink
        while v!=source:
            u=par[v]
            g[u][v] -=pathflow
            g[v][u] +=pathflow
            v=par[v]
    return maxflow
 
g = [
    [0, 21, 7, 18, 5],
    [0, 0, 3, 11, 0],
    [0, 6, 0, 1, 9],
    [0, 0, 9, 0, 12],
    [0, 14, 0, 7, 0],
    [0, 0, 5, 0, 2],
]
source,sink=0,4
print(FordFulkerson(g,source,sink))
