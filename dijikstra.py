def dijikstra(maze,source):
    
    infinity=float('infinity')
    n=len(maze)
    dist=[infinity]*n
    prev=[infinity]*n
    dist[source]=0
    q=list(range(n))
    while q:
        u=min(q,key=lambda n: dist[n])
        q.remove(u)
        if dist[u]==infinity:
            break
        for v in range(n):
            if maze[u][v] and (v in q):
                alt=dist[u]+maze[u][v]
                if alt<dist[v]:
                    dist[v]=alt
                    prev[v]=u
    return dist,prev

def disp(pred):
    cell=len(pred)-1
    while cell:
        print(cell , end='<')
        cell=pred[cell]
    print(0)


maze=((0,0,0,0,1,0),
      (0,0,0,0,1,0),
      (0,0,0,0,1,0),
      (0,0,0,0,1,0),
      (0,0,0,0,1,0),
      (0,0,0,0,0,0))
graph=(
    (0,1,0,0,0,0,),
    (1,0,1,0,1,0,),
    (0,1,0,0,0,1,),
    (0,0,0,0,1,0,),
    (0,1,0,1,0,0,),
    (0,0,1,0,0,0,),
    )

values=dijikstra(graph,0)
print(values)
disp(values[1])
values=dijikstra(maze,0)
print(values)
disp(values[1])
