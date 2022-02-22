def dfs():
    adjmat = [[0,1,1,0],
                  [0,0,1,0],
                  [1,0,0,1],
                  [0,0,0,1]];
    visi = [0,0,0,0]
    stack = [0]
    visi[0] = 1
    n = stack.pop(len(stack) - 1);
    print(n)
    while True:
        for x in range (0, len(visi)):

            if adjmat[n][x] == 1 and visi[x] == 0:
                visi[x] = 1;                              
                stack.append(x)
        if len(stack) == 0:
            break;
        else:
            n = stack.pop(len(stack) - 1)
            print(n)
     
dfs()
 
 
 
 
 
 
 
 
 
 
 
 
 
    
