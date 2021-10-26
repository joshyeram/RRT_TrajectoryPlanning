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
    while(endNode!=None and endNode!=False):
        path.append(endNode.point)
        endNode = endNode.parent
    temp = list(reversed(path))
    #print(temp)
    if(len(path)==0):
        path.append(False)
        path.append(start)
        path.append(goal)
    return temp

def rrt(robot, obstacles, start, goal, iter_n):
    tree = Tree(robot, obstacles, start, goal)
    while(iter_n >=0):
        #print(iter_n)
        sampled = sample()
        if(iter_n % 10 ==0):
            sampled = goal
        if(tree.getNode(sampled)!=False):
            continue
        near = tree.nearest(sampled)
        actual = tree.extend(near, sampled)

        if(tree.distance(actual, goal)<=.25):
            attempt = tree.extend(actual, goal)
            if(attempt == goal):
                path = getPath(tree, start, goal)
                return path
        iter_n-=1
    lastNode = lastResort(tree, robot, obstacles, goal)
    if(lastNode == -1):
        return (False, start, goal)
    attempt = tree.extend(lastNode.point, goal)
    path = getPath(tree, start, goal)

    return path

def rrtWithTree(robot, obstacles, start, goal, iter_n):
    tree = Tree(robot, obstacles, start, goal)
    while(iter_n >=0):
        #print(iter_n)
        sampled = sample()
        if(iter_n % 10 ==0):
            sampled = goal
        if(tree.getNode(sampled)!=False):
            continue
        near = tree.nearest(sampled)
        actual = tree.extend(near, sampled)

        if(tree.distance(actual, goal)<=.25):
            attempt = tree.extend(actual, goal)
            if(attempt == goal):
                path = getPath(tree, start, goal)
                return (path,tree)
        iter_n-=1
    lastNode = lastResort(tree, robot, obstacles, goal)
    if(lastNode == -1):
        return (False, start, goal, tree)
    attempt = tree.extend(lastNode.point, goal)
    path = getPath(tree, start, goal)
    return (path,tree)

def lastResort(tree, robot, obstacles, goal):
    all = tree.nodes
    distances = []
    far = None
    added = False
    for i in all:
        pointList = tree.getList(i.point, goal)
        for j in pointList:
            if(isCollisionFree(robot, j, obstacles) == False):
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
