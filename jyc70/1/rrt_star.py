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
    if(len(path)==0):
        return None
    temp = list(reversed(path))
    return temp

def rrt_star_vis(robot, obstacles, start, goal, iter_n,):
    tree = Tree(robot, obstacles, start, goal)
    while (iter_n >= 0):
        sampled = sample()
        if (iter_n % 10 == 0):
            sampled = goal
        if (tree.getNode(sampled) != False):
            iter_n -= 1
            continue
        r = 1
        #gets nearest node in consideration of the distance from the start
        near = tree.nearest_star(sampled, r)
        #connects to nearest node
        actual = tree.extend(near, sampled)
        #rewire actual node around r
        tree.rewire(actual, r)

        if (tree.getNode(goal)==False and tree.distance(actual, goal) <= .25):
            attempt = tree.extend(actual, goal)

        iter_n -= 1

    if(tree.getNode(goal)!=False):
        return tree,getPath(tree, start, goal)

    lastNode = lastResort(tree, robot, obstacles, goal)
    if (lastNode == -1):
        return tree, None
    attempt = tree.extend(lastNode.point, goal)
    path = getPath(tree, start, goal)
    return tree, path

def rrt_star(robot, obstacles, start, goal, iter_n,):
    tree = Tree(robot, obstacles, start, goal)
    while (iter_n >= 0):
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
        tree.rewire(actual, r=1)

        if (tree.getNode(goal)==False and tree.distance(actual, goal) <= .25):
            attempt = tree.extend(actual, goal)

        iter_n -= 1

    if(tree.getNode(goal)!=False):
        return getPath(tree, start, goal)

    lastNode = lastResort(tree, robot, obstacles, goal)
    if (lastNode == -1):
        return None
    attempt = tree.extend(lastNode.point, goal)
    path = getPath(tree, start, goal)
    return path


def lastResort(tree, robot, obstacles, goal):
    all = tree.getAll()
    distances = []
    far = None
    added = False
    if(tree.getNode(goal)!=False):
        return tree.getNode(goal)
    for i in all:
        pointList = tree.getList(i.point, goal)
        for j in pointList:
            if (isCollisionFree(robot, j, obstacles) == False):
                distances.append(-1)
                added = True
                break
        if (added == False):
            distances.append(tree.distance(i.point, goal))
        added = False
    for i in distances:
        if i >= 0:
            added = True
            break
    if (added == True):
        index = distances.index(min([i for i in distances if i > 0]))
        return all[index]
    return -1