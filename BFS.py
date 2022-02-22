def BFS():  
	adjmat= [[0,1,1,0],
                  [0,0,1,0],
                  [1,0,0,1],
                  [0,0,0,1]];
	visi = [0,0,0,0]
	queue = [0]
	visi[0] = 1
	n = queue.pop(0);
	print (n)
	while True:
		for x in range (0, len(visi)):
			if adjmat[n][x] == 1 and visi[x] == 0:
				visi[x] = 1;
				queue.append(x)
		if len(queue) == 0:
			break;
		else:
			n = queue.pop(0)
			print (n)
	
BFS()
