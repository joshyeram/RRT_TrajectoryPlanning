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
    while(endNode!=None):
        path.append(endNode.point)
        endNode = endNode.parent
    temp = list(reversed(path))
    #print(temp)
    if(len(path)==0):
        path.append(False)
        path.append(start)
        path.append(goal)
    return temp

def rrt_starWithTree(robot, obstacles, start, goal, iter_n,):
    tree = Tree(robot, obstacles, start, goal)
    while (iter_n >= 0):
        print(iter_n)
        sampled = sample()
        if (iter_n % 10 == 0):
            sampled = goal
        if (tree.getNode(sampled) != False):
            iter_n -= 1
            continue
        #gets nearest node in consideration of the distance from the start
        near = tree.nearest_star(sampled, 1)
        #connects to nearest node
        actual = tree.extend(near, sampled)
        #rewire actual node around r
        tree.rewire(actual, r = 1)

        if (tree.getNode(goal)==False and tree.distanceEuc(actual, goal) <= .25):
            attempt = tree.extend(actual, goal)
        iter_n -= 1

    if(tree.getNode(goal)!=False):
        return (getPath(tree, start, goal), tree)

    lastNode = lastResort(tree, robot, obstacles, goal)
    if (lastNode == -1):
        return (None, start, goal, tree)
    attempt = tree.extend(lastNode.point, goal)
    path = getPath(tree, start, goal)
    return (path,tree)

def rrt_star(robot, obstacles, start, goal, iter_n,):
    tree = Tree(robot, obstacles, start, goal)
    while (iter_n >= 0):
        print(iter_n)
        sampled = sample()
        if (iter_n % 10 == 0):
            sampled = goal
        if (tree.getNode(sampled) != False):
            iter_n -= 1
            continue
        #gets nearest node in consideration of the distance from the start
        near = tree.nearest_star(sampled, 1)
        #connects to nearest node
        actual = tree.extend(near, sampled)
        #rewire actual node around r
        tree.rewire(actual, r = 1)

        if (tree.getNode(goal)==False and tree.distanceEuc(actual, goal) <= .25):
            attempt = tree.extend(actual, goal)
        iter_n -= 1

    if(tree.getNode(goal)!=False):
        return getPath(tree, start, goal)

    lastNode = lastResort(tree, robot, obstacles, goal)
    if (lastNode == -1):
        return []
    attempt = tree.extend(lastNode.point, goal)
    path = getPath(tree, start, goal)
    return path


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