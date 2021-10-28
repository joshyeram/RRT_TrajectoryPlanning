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
        if(endNode.trajectoryToParent == None):
            path.append(endNode.parent)
        else:
            path.extend(endNode.trajectoryToParent)
        endNode = endNode.parent
    temp = list(reversed(path))
    #print(temp)
    return temp

def rrt(robot, obstacles, start, goal, iter_n):
    tree = Tree(robot, obstacles, start, goal)
    while(iter_n >=0):
        sampled = sample()

        near = tree.nearest(sampled)
        actual = tree.extend(near, 8, 11, .1)

        if(tree.distance(actual, goal)<=.25):
            attempt = tree.extend(actual, 8, 11, .1)

        iter_n-=1

    lastNode = lastResort(tree, robot, obstacles, goal)
    if(lastNode == -1):
        return None
    attempt = tree.extend1(lastNode.point, goal)
    path = getPath(tree, start, goal)
    return path

def rrtWithTree(robot, obstacles, start, goal, iter_n):
    tree = Tree(robot, obstacles, start, goal)
    while (iter_n >= 0):
        sampled = sample()

        near = tree.nearest(sampled)
        actual = tree.extend(near, 8, 11, .1)

        if (tree.distance(actual, goal) <= .25):
            attempt = tree.extend(actual, 8, 11, .1)

        iter_n -= 1

    lastNode = lastResort(tree, robot, obstacles, goal)
    if (lastNode == -1):
        return (None, tree)
    attempt = tree.extend1(lastNode.point, goal)
    path = getPath(tree, start, goal)
    return (path,tree)

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


temp = parse_problem("robot_env_02.txt","probs_01.txt")
robot = temp[0]
obs = temp[1]

r = rrtWithTree(robot,obs,temp[2][0][0], temp[2][0][1], 100)
print(r)