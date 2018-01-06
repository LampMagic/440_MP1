
import numpy as np
import time
import math
import itertools

# self defined class of maze node
class node:
    def __init__(self,x,y,chrct):
        self.x = x
        self.y = y
        self.char = chrct
        self.visited = 0
        self.parent = None
        self.g = 0
        self.h = 0
        self.root = self
        self.rank = 0

    def isDestination(self):
        return (self.char == '.')       
        
# Create class of set of Minimum Search Tree, and each set contains two goal nodes and edge between them.
class edge:
    def __init__(self, node1, node2):
        self.node1 = node1
        self.node2 = node2
        self.length = 0
    
print("Modules Imported!")


# helper fucntion calculating current heuristic
# ret_val:
# heuristic to node2 based on node1
def heuristic(node1, node2):
    return np.abs(node1.x-node2.x)+np.abs(node1.y-node2.y)


def updateCost(front_list):
    for i in front_list:
        i.h = MST(i)

# helper function checking available movement
# ret val:
# 0 for non-viable movement,
# 1 for viable movement
def movCheck(curr, heading):
    if(getNeighbor(curr, heading).char == '%'):
        return 0
    else:
        return 1


# helper function to find neighbor of current node
def getNeighbor(curr, heading):
    # return up child
    if(heading == 0):
        return MazeNode[curr.y-1][curr.x]
    # return right child
    elif(heading == 1):
        return MazeNode[curr.y][curr.x+1]
    # return down child
    elif(heading == 2):
        return MazeNode[curr.y+1][curr.x]
    # return left child
    else:
        return MazeNode[curr.y][curr.x-1]


# helper function implementing Breadth First Search
def BFS(start, goal):
    
    node_exp = 0
    front_list = [start]
    visit_list = []
    
    while(len(front_list)):
        frontier = front_list.pop(0)
        # check for goal
        if(frontier == goal):
            print("BFS expanded:",node_exp)
            return frontier
        
        if(not frontier.visited):
            visit_list.append(frontier)
            frontier.visited = 1
            node_exp = node_exp + 1
            # check for all neighbors
            for i in range(0,4):
                if(movCheck(frontier, i)):
                    # receive the child
                    child = getNeighbor(frontier, i)
                    # check if child is on lists
                    if((child not in front_list) & (child not in visit_list)):
                        front_list.append(child)
                        # set child specs
                        child.parent = frontier
               


# helper function implementing Depth First Search
def DFS(start, goal):
    
    node_exp = 0
    front_list = [start]
    visit_list = []
    
    while(len(front_list)):
        frontier = front_list.pop(0)
        # check for goal
        if(frontier == goal):
            print("DFS expanded:",node_exp)
            return frontier
        
        if(not frontier.visited):
            visit_list.append(frontier)
            frontier.visited = 1
            node_exp = node_exp + 1
            # check for all neighbors
            for i in range(0,4):
                if(movCheck(frontier, i)):
                    # receive the child
                    child = getNeighbor(frontier, i)
                    # check if child is on lists
                    if((child not in front_list) & (child not in visit_list)):
                        front_list.insert(0,child)
                        # set child specs
                        child.parent = frontier

# helper function implementing A* search
def A_star(start, goal):
    # current cost
    #print("\nV1",start.x,start.y,"V2",goal[0].x,goal[0].y)
    found = 0
    node_exp = 0
    front_list = [start]
    visit_list = []
    path_list = []

    while(len(front_list)):
        # sort the frontier list w.r.t. total path cost
        front_list = sorted(front_list, key=lambda node: (node.g+node.h))                  
                       
        # pop out the node with least cost
        frontier = front_list.pop(0)
        if(frontier in goal):
            goal.remove(frontier)
            updateCost(front_list)
            found = found + 1
            #visit_list = []
            path_list.append(findPath(start, frontier))
            last_exp = frontier

            if(found < 10):
                frontier.char = str(found)
            else:
                frontier.char = chr(87+found)

            if(len(goal) == 0):
                #print("\nA* expanded:",node_exp)
                return path_list
        
        # mark current fronier as visited
        visit_list.append(frontier)
        frontier.visited = 1
        node_exp = node_exp + 1
        
        # find its child
        for i in range(0,4):
            if(movCheck(frontier, i)):
                # receive the child
                child = getNeighbor(frontier, i)
                # check if child is on lists
                # if((child in visit_list) & (eatFlag == 1)):
                #     visit_list.remove(child)

                if((child not in front_list) & (child not in visit_list)):
                    # add child to frontier list
                    front_list.append(child)
                    # set child specs
                    child.parent = frontier
                    child.g = len(findPath(start,child))
                    child.h = MST(child)

                elif((child in front_list) & (child.g > frontier.g+1)):
                    child.parent = frontier
                    child.g = frontier.g+1
                    child.h = MST(child)               


# helper function implementing A* search
def A_star_single(start, goal):
    # current cost
    node_exp = 0
    front_list = [start]
    visit_list = []

    while(len(front_list)):
        # sort the frontier list w.r.t. total path cost
        front_list = sorted(front_list, key=lambda node: (node.g+node.h))                  
                       
        # pop out the node with least cost
        frontier = front_list.pop(0)
        if(frontier in goal):
            # print("A* expanded:",node_exp)
            return frontier,node_exp
        
        # mark current fronier as visited
        visit_list.append(frontier)
        frontier.visited = 1
        node_exp = node_exp + 1
        
        # find its child
        for i in range(0,4):
            if(movCheck(frontier, i)):
                # receive the child
                child = getNeighbor(frontier, i)
                # check if child is on lists
                if((child not in front_list) & (child not in visit_list)):
                    # add child to frontier list
                    front_list.append(child)
                    # set child specs
                    child.parent = frontier
                    child.g = len(findPath(start,child))
                    child.h = MST(child)
                elif((child in front_list) & (child.g > frontier.g+1)):
                    child.parent = frontier
                    child.g = frontier.g+1
                    child.h = MST(child)  


# helper function implementing A* search
def A_star_sub(start, goal):
    # current cost
    node_exp = 0
    front_list = [start]
    visit_list = []

    while(len(front_list)):
        # sort the frontier list w.r.t. total path cost
        front_list = sorted(front_list, key=lambda node: (node.g+node.h))                  
                       
        # pop out the node with least cost
        frontier = front_list.pop(0)
        if(frontier == goal):
            #print("A* expanded:",node_exp)
            return frontier
        
        # mark current fronier as visited
        visit_list.append(frontier)
        frontier.visited = 1
        node_exp = node_exp + 1
        
        # find its child
        for i in range(0,4):
            if(movCheck(frontier, i)):
                # receive the child
                child = getNeighbor(frontier, i)
                # check if child is on lists
                if((child not in front_list) & (child not in visit_list)):
                    # add child to frontier list
                    front_list.append(child)
                    # set child specs
                    child.parent = frontier
                    child.g = len(findPath(start,child))
                    child.h = heuristic(child,goal)
                elif((child in front_list) & (child.g > frontier.g+1)):
                    child.parent = frontier
                    child.g = frontier.g+1
                    child.h = heuristic(child,goal)  


# Helper function to do Greedy Best-First Search.
def Greedy(start, goal):
    # current cost
    node_exp = 0
    front_list = [start]
    visit_list = []

    while len(front_list):
        # sort the frontier list w.r.t. total path cost
        front_list = sorted(front_list, key=lambda node: node.h)

        # pop out the node with least cost
        frontier = front_list.pop(0)
        if frontier == goal:
            print("Greedy expanded:",node_exp)
            return frontier

        # mark current fronier as visited
        visit_list.append(frontier)
        frontier.visited = 1
        node_exp = node_exp + 1

        # find its child
        for i in range(0, 4):
            if movCheck(frontier, i):
                # receive the child
                child = getNeighbor(frontier, i)
                # check if child is on lists
                if (child not in front_list) & (child not in visit_list):
                    # add child to frontier list
                    front_list.append(child)
                    child.parent = frontier
                    child.g = node_exp
                    child.h = heuristic(child, goal)


# helper function to compute path cost
def findPath(node1, node2):
    path = []
    while(node2 != node1):
        node2 = node2.parent
        path.append(node2)
    return list(reversed(path))

def findPath_plus(node1, node2):
    return A_star_single(node1,node2)


# helper function to compute solution cost
def drawSolution(path):
    for k in path:
        temp = MazeNode[k.y][k.x].char
        if(MazeNode[k.y][k.x].char == ' '):
            MazeNode[k.y][k.x].char = '.'
    

def drawMaze():
    for i in range(0,height):
        temp = []
        for j in range(0, width):
            temp.append(MazeNode[i][j].char)
        temp = ''.join(temp)
        print(temp)


# helper functions for Union by Rank
# wiki reference
def find(x):
    if(x.root != x):
        x.root = find(x.root)
    return x.root

def union(x,y):
    xRoot = find(x)
    yRoot = find(y)  
    # if two nodes are in same set
    if(xRoot == yRoot):
        return 0
    # comparing ranks and linking them
    if(xRoot.rank < yRoot.rank):
        xRoot.root = yRoot
    elif(xRoot.rank > yRoot.rank):
        yRoot.root = xRoot
    else:
        yRoot.root = xRoot
        xRoot.rank += 1
    return 1

def resetUnion(goal):
    for i in goal:
        i.root = i
        i.rank = 0


def setDis(goal):
    dists = []
    for i in goal:
        for j in goal:
            curr_path = findPath(A_star_sub(i,j))
            dists.append(len(curr_path))
    print(dists)


def setGraph(curr):
    goal.append(curr)
    goalGraph = []
    resetUnion(goal)

    for i in goal:
        for j in goal:
            if(i != j):
                if((edge(i,j) not in goalGraph) & (edge(j,i) not in goalGraph)):
                    new = edge(i,j)
                    new.length = heuristic(i,j)
                    goalGraph.append(new)
                
    goal.remove(curr)       
    return sorted(goalGraph, key=lambda edge: edge.length)

def MST(curr):
    goalGraph = setGraph(curr)
    goalTree = []
    while len(goalGraph):
        top = goalGraph.pop(0)
        if(union(top.node1,top.node2)):
            goalTree.append(top)

    goalLength = 0
    for i in goalTree:
        goalLength += i.length
    return goalLength


def resetMaze(MazeNode):
    for i in range(0,len(MazeNode)):
        for j in range(0,len(MazeNode[0])):
            MazeNode[i][j].visited = 0
            MazeNode[i][j].parent = None
            MazeNode[i][j].g = 0
            MazeNode[i][j].h = 0
            MazeNode[i][j].root = MazeNode[i][j]
            MazeNode[i][j].rank = 0




#############################
#       Main Function       #
#############################

# load maze file and compute specs
print("Enter a file index:")
idx = input()

start_time = time.time()
txt_list = ["mediumMaze.txt", "bigMaze.txt", "openMaze.txt",
            "tinySearch.txt", "smallSearch.txt", "mediumSearch.txt"]
txt = open(txt_list[int(idx)],"r")
maze = txt.read().splitlines()
MazeNode = []
height = len(maze)
width = len(maze[0])
txt.close()

# find start pt and goal pt
goal = []
for i in range(0,height):
    temp = []
    for j in range(0,width):
        new = node(j,i,maze[i][j])
        temp.append(new)
        if(maze[i][j] == 'P'):
            start = new
        if(maze[i][j] == '.'):
            goal.append(new)    
    MazeNode.append(temp) 
    
start.g = 0
start.h = heuristic(start,goal[0])

# print the maze
print("\nOriginal:")
for i in range(0,height):
    print(maze[i])



###########################
#   Searching Algorithm   #
###########################

# BFS
reach = BFS(start,goal[0])
path = findPath(start,reach)
print("BFS cost:",len(path))
print("Time Useds:",(time.time() - start_time))
drawSolution(path)
drawMaze()


# DFS
# reach = BFS(start,goal[0])
# path = findPath(start,reach)
# print("DFS cost:",len(path))
# print("Time Useds:",(time.time() - start_time))
# drawSolution(path)
# drawMaze()


# Greedy Search
# reach = Greedy(start,goal[0])
# path = findPath(start,reach)
# print("Greedy cost:",len(path))
# print("Time Useds:",(time.time() - start_time))
# drawSolution(path)
# drawMaze()


# A star Search
# curr = start
# path_list = []
# path_cost = 0
# found = 0
# total_exp = 0
# while(len(goal)):
#     reach,expanded = A_star_single(curr,goal)
#     total_exp += expanded
#     curr_path = findPath(curr,reach)
#     path_list.append(curr_path)
#     path_cost += len(curr_path)
#     found += 1

#     if(found < 10):
#         reach.char = str(found)
#     else:
#         reach.char = chr(87+found)

#     curr = reach
#     goal.remove(reach)
#     resetMaze(MazeNode)
#     drawSolution(curr_path)
# print("A* cost:",path_cost)
# print("A* node expanded:",total_exp)
# print("Time Useds:",(time.time() - start_time))
# drawMaze()
