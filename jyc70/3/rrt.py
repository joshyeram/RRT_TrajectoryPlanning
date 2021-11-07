import numpy as np
import matplotlib.pyplot as plt
from matplotlib.pyplot import show
import matplotlib.path as mplPath

from file_parse import *
from sampler import *
from tree import *

def getPath(tree, start, goal):
    endNode = tree.getNode(goal)
    path = []
    while (endNode != None):
        path.append(endNode.point)
        endNode = endNode.parent
    if (len(path) == 0):
        return None
    temp = list(reversed(path))
    return temp

def rrt(robot, obstacles, start, goal, iter_n):
    tree = Tree(robot, obstacles, start, goal)
    while (iter_n > 0):
        sampled = sample()
        near = tree.nearest(sampled)
        if (iter_n % 8 == 0):
            near = tree.nearest(goal)

        actual = tree.extend(near, 15, 20, .1)

        temp = tree.nodes[0]

        for i in tree.nodes:
            if (tree.distanceEuc(i.point, goal) < tree.distanceEuc(temp.point, goal)):
                temp = i

        if (tree.distanceEuc(temp.point, goal) <= .25):
            attempt = tree.extend1(temp.point, goal)
            if (attempt == goal):
                path = getPath(tree, start, goal)
                for i in range(len(path)):
                    while(path[i][2] > np.pi):
                        path[i] = (path[i][0], path[i][1], path[i][2] - 2 * np.pi)
                    while (path[i][2] < -np.pi):
                        path[i] = (path[i][0], path[i][1], path[i][2]+ 2 * np.pi)
                return path
        iter_n -= 1
    return None

def rrtWithTree(robot, obstacles, start, goal):
    tree = Tree(robot, obstacles, start, goal)
    iter_n = 0
    while (True):
        print(iter_n)
        sampled = sample()

        near = tree.nearest(sampled)
        if(iter_n % 2 ==0):
            near = tree.nearest(goal)
            print(near)

        actual = tree.extend(near, 15, 20, 1)

        temp = tree.nodes[0]
        for i in tree.nodes:
            if(tree.distanceEuc(i.point, goal)< tree.distanceEuc(temp.point, goal)):
                temp = i
        if(tree.distanceEuc(temp.point, goal)<=.5):
            attempt = tree.extend1(temp.point, goal)
            if(attempt == goal):
                path = getPath(tree, start, goal)
                return (path, tree)

        iter_n += 1
    """print(len(tree.nodes))
    lastNode = lastResort(tree, robot, obstacles, goal)
    if (lastNode == -1):
        return (None, tree)
    attempt = tree.extend1(lastNode.point, goal)
    path = getPath(tree, start, goal)
    return (path,tree)"""

def lastResort(tree, robot, obstacles, goal):
    all = tree.nodes
    distances = []
    far = None
    added = False
    initTheta = robot.rotation
    for i in all:
        thetas = tree.getThetaList(i.point, goal)
        for k in thetas:
            if (isCollisionFree(robot, (i.point[0],i.point[1],k), obstacles)==False):
                robot.rotation = initTheta
                distances.append(-1)
                added = True
                break
        if(added==True):
            added = False
            continue
        robot.rotation = goal[2]
        pointList = tree.getList(i.point, goal)
        for j in pointList:
            if(isCollisionFree(robot, (j[0],j[1],goal[2]), obstacles) == False):
                distances.append(-1)
                added = True
                break
        if(added == False):
            distances.append(tree.distance(i.point, goal))
        added = False
    for i in distances:
        if i >= 0:
            added = True
            break
    if(added == True):
        index = distances.index(min([i for i in distances if i > 0]))
        return all[index]
    return -1

