# Basic searching algorithms
import numpy as np
from numpy import inf 
import math

def init(grid, start, goal):
    path = []
    steps = 0
    found = False
    rq=[]
    cq=[]
    rq.append(start[0])
    cq.append(start[1])
    Row=len(grid)                                       # Initialise Row
    Col=len(grid[0])                                    # Initialise column 
    if start[0]==goal[0] and start[1]==goal[1]:         # Condition if start and goal are at same point 
        found=True
        steps=-1
    
    grid[goal[0]][goal[1]]="B"                          # Assigning B as goal name on the grid
    
    visited=[[False for j in range(Col)] for i in range(Row)]       # Creating bool false array of RxC to check if node has been visited
    visited[start[0]][start[1]]=True
    steps=steps+1
    prev = [[0 for i in range(Col)] for j in range(Row)]            # Array for storing parent node of the current node visted

    return path, steps, found, Row, Col, visited, prev,rq,cq,grid

def backtrack(goal,prev,path):
    count=0
    at = [goal[0],goal[1]]                              # backtracking algorithm
    while at!=0:
        count=count+1
        path.append(at)
        at = prev[at[0]][at[1]]
    path = path[::-1]

    return path,count

def cost1(point,start,prev):

    cost=0
    at=point
    while at!=start:
        cost=cost+1
        at = prev[at[0]][at[1]]
    
    return cost

def bfs(grid, start, goal):
    '''Return a path found by BFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> bfs_path, bfs_steps = bfs(grid, start, goal)
    It takes 10 steps to find a path using BFS
    >>> bfs_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path, steps, found, Row, Col, visited, prev,rq,cq,grid= init(grid, start, goal)
    cost=[[inf for i in range(Col)] for j in range(Row)]
    cost[start[0]][start[1]]=(abs(start[0]-goal[0])+abs(start[1]-goal[1]))
    distance=0
    while (len(rq))>0:                                  # Initialising while loop to iterate over the queue till it is not empty

        if found==True:                                 # To terminate loop if start=goal
            break
        r=rq[0] 
        c=cq[0]
        rq.pop(0)
        cq.pop(0)
        dr=[0,+1,0,-1]                                  # direction right down left up 
        dc=[+1,0,-1,0]
        for i in range(4):                              # loop for finding neighbor nodes 
            rr = r + dr[i]
            cc = c + dc[i]
            if rr<0 or cc<0:                            # out of grid condition
                continue
            if rr>=Row or cc>=Col:                      # out of grid condition     
                continue
            if visited[rr][cc]==True:                   # if already visited
                continue
            if grid[rr][cc]==1:                         # if obstacle is hit 
                continue
            rq.append(rr)                               # add new nodes to the queue so that they can be explored FIFO
            cq.append(cc)
            visited[rr][cc]=True
            prev[rr][cc] = [r,c]
            steps=steps+1
            if grid[rr][cc]== "B":                      # if goal is reached 
                found=True
                break
            

    path,count=backtrack(goal,prev,path)
    
    if found==True:
        print(f"It takes {steps} steps to find a path using BFS")
        print(f"The shortest path using BFS is: ",count)
    else:
        print("No path found")
    return path, steps


def dfs(grid, start, goal):
    '''Return a path found by DFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')'''
    ### YOUR CODE HERE ###
    path, steps, found, Row, Col, visited, prev,rq,cq,grid= init(grid, start, goal)
    while (len(rq))>0:
        if found==True:
            break
        r=rq[0]
        c=cq[0]
        rq.pop(0)
        cq.pop(0)
        dr=[0,+1,0,-1]
        dc=[+1,0,-1,0]
        temprq=[]                       #temporary stack to store neighbors
        tempcq=[]
        visited[r][c]=True
        steps=steps+1
    
        for i in range(4):
            rr = r + dr[i]
            cc = c + dc[i]
            if rr<0 or cc<0:
                continue
            if rr>=Row or cc>=Col:
                continue
            if visited[rr][cc]==True:
                continue
            if grid[rr][cc]==1:
                continue
            temprq.append(rr)           # add new neighbors to temp stack
            tempcq.append(cc)
            prev[rr][cc] = [r,c]
            rq=temprq+rq                # add new neighbors stack to original queue LIFO
            cq=tempcq+cq
            if grid[rr][cc]== "B":
                found=True
                break
            
    path,count=backtrack(goal,prev,path)

           
    if found:
        print(f"It takes {steps} steps to find a path using DFS")
        print(f"The shortest path using DFS is: ",count)

    else:
        print("No path found")
    return path, steps


def dijkstra(grid, start, goal):
    '''Return a path found by Dijkstra alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')'''
    ### YOUR CODE HERE ###
    path, steps, found, Row, Col, visited, prev,rq,cq,grid= init(grid, start, goal)
    cost=[[inf for i in range(Col)] for j in range(Row)]                #initialise cost of all nodes as infinity

    queue=[]
    queue.append([0,start[0],start[1]])                                 # add cost and the corresponding node to queue
    distance=0

    while (len(queue))>0:

        if found==True:
            break
        temp=queue.pop(0)                   #create temp queue after popping element already visited
        r=temp[1]                           #take x and y coordinate of node stored in temp
        c=temp[2]
        dr=[0,+1,0,-1]
        dc=[+1,0,-1,0]
        for i in range(4):
            rr = r + dr[i]
            cc = c + dc[i]
            distance=abs(rr-start[0])+abs(cc-start[1])          # calculate cost to node as manhattan distance from start
            if rr<0 or cc<0:
                continue
            if rr>=Row or cc>=Col:
                continue
            if visited[rr][cc]==True:
                continue
            if grid[rr][cc]==1:
                continue

            prev[rr][cc] = [r,c]
            distance=cost1([rr,cc],start,prev)
            cost[rr][cc]=distance
            queue.append([cost[rr][cc], rr,cc])                 #add cost and corresponding neighbor node to queue
            visited[rr][cc]=True
            steps=steps+1

            if grid[rr][cc]== "B":
                found=True
                break
        
        queue.sort()                                            # sort queue based on cost 
    
    path,count=backtrack(goal,prev,path)

    if found==True:
        print(f"It takes {steps} steps to find a path using Dijkstra")
        print(f"The shortest path using Dijkstra is: ",count)

    else:
        print("No path found")
    return path, steps


def astar(grid, start, goal):
    '''Return a path found by A* alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> astar_path, astar_steps = astar(grid, start, goal)
    It takes 7 steps to find a path using A*
    >>> astar_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path, steps, found, Row, Col, visited, prev,rq,cq,grid= init(grid, start, goal)
    cost=[[inf for i in range(Col)] for j in range(Row)]
    cost[start[0]][start[1]]=(abs(start[0]-goal[0])+abs(start[1]-goal[1]))
    queue=[]
    queue.append([0,start[0],start[1]])
    distance=0

    while (len(queue))>0:

        if found==True:
            break

        temp=queue.pop(0)
        r=temp[1]
        c=temp[2]
        dr=[0,+1,0,-1]
        dc=[+1,0,-1,0]
        steps=steps+1
        for i in range(4):
            rr = r + dr[i]
            cc = c + dc[i]
            if rr<0 or cc<0:
                continue
            if rr>=Row or cc>=Col:
                continue
            if visited[rr][cc]==True:
                if cost[rr][cc]>cost1([rr,cc],start,prev) + (abs(rr-goal[0])+abs(cc-goal[1])):
                    cost[rr][cc]=cost1([rr,cc],start,prev) + (abs(rr-goal[0])+abs(cc-goal[1]))
                    prev[rr][cc]=[r,c]
                continue
            if grid[rr][cc]==1:
                continue
            prev[rr][cc] = [r,c]
            distance=cost1([rr,cc],start,prev)                  # cost to node g(x)
            heuristic=abs(rr-goal[0])+abs(cc-goal[1])                   # h(x) as manhattan distance of node from goal 
            cost[rr][cc]=distance+heuristic                             # calculate total cost 
            queue.append([cost[rr][cc], rr,cc])
            visited[rr][cc]=True
            
            if grid[rr][cc]== "B":
                found=True
                break
        
        queue.sort()                                                    # sort queue based on total cost 

    path,count=backtrack(goal,prev,path)

    if found:
        print(f"It takes {steps} steps to find a path using A*")
        print(f"The shortest path using A* is: ",count)

    else:
        print("No path found")
    return path, steps


# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples
    # Test all the functions
    testmod()
