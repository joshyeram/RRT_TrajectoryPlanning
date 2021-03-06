import numpy as np
import matplotlib.pyplot as plt
import matplotlib.path as mplPath

from file_parse import *
from collision import *

"""def distance(point1, point2):
    x = (point2[0]-point1[0]) ** 2
    y = (point2[1]-point1[1]) ** 2
    return np.sqrt(x+y)"""

class Node:

    def __init__(self, point, state):
        self.point = point
        self.neighbors = []
        self.parent = None
        self.state = state
        self.right = None
        self.left = None
        self.distanceFromStart = 0

class Tree:
    robot = None
    obstacles = None
    start = None
    goal = None

    def __init__(self, robot, obstacles, start, goal):
        self.robot = robot
        self.obstacles = obstacles
        self.start = Node(start, 1)
        self.goal = Node(goal, 2)
        self.kd = self.start

    def getNode(self, point):
        if(type(point)==Node):
            point = point.point
        ptr = self.kd
        level = 0
        while (ptr != None):
            if(point == ptr.point):
                return ptr
            if ((level % 2 == 0 and point[0] < ptr.point[0]) or (level % 2 == 1 and point[1] < ptr.point[1])):
                ptr = ptr.left
            else:
                ptr = ptr.right
            level += 1
        return False

    def add(self, point1, point2):
        #print("adding = "+ str(point1)+ " " +str(point2))
        node1 = self.getNode(point1)
        node2 = Node(point2, -1)
        self.insertkd(node2)
        node1.neighbors.append(node2)
        node2.neighbors.append(node1)
        node2.parent = node1
        node2.distanceFromStart = node1.distanceFromStart + self.distance(point1, point2)

    def exists(self, point):
        if(self.getNode(point)==False):
            return False
        return True

    def parent(self, point):
        if(self.exists(point) == False):
            return None
        return self.getNode(point).parent.point

    def insertkd(self, node):
        ptr = self.kd
        prev = None
        level = 0 #if level even = compare horizontal. If level odd = compare vertically
        while(ptr!= None):
            prev = ptr
            if((level%2 ==0 and node.point[0] < ptr.point[0]) or (level%2 ==1 and node.point[1] < ptr.point[1])):
                ptr = ptr.left
            else:
                ptr = ptr.right
            level += 1
        level -= 1
        if((level % 2 ==0 and node.point[0] < prev.point[0]) or (level % 2 ==1 and node.point[1] < prev.point[1])):
            prev.left = node
        else:
            prev.right = node

    def closer(self, check, point1, point2):
        if(point1==None):
            return point2
        if(point2 == None):
            return point1
        if(self.distance(check, point1.point) < self.distance(check, point2.point)):
            return point1
        return point2

    def nearestOpt(self, head, pointC, level): #root node, point point, depth
        if(head == None):
            return None
        splitPoint = level % 2

        if(pointC[splitPoint] < head.point[splitPoint]):
            nextCheck = head.left
            oppCheck = head.right
        else:
            nextCheck = head.right
            oppCheck = head.left

        opt = self.closer(pointC, self.nearestOpt(nextCheck, pointC, splitPoint+1), head)

        if(self.distance(pointC, opt.point) > abs(pointC[splitPoint] - head.point[splitPoint])):
            opt = self.closer(pointC, self.nearestOpt(oppCheck, pointC, splitPoint+1), opt)

        return opt

    def nearest(self, point):
        return self.nearestOpt(self.kd, point, 0).point

    def nearest_star(self, point,r):
        all = self.getAll()
        trueNear = None
        for i in all:
            if(self.distance(i.point, point)>r):
                continue
            if (trueNear == None or i.distanceFromStart + self.distance(i.point, point) < trueNear.distanceFromStart + self.distance(trueNear.point, point)):
                trueNear = i
        if(trueNear==None):
            trueNear = self.getNode(self.nearest(point))
        return trueNear

    def distance(self, point1, point2):
        if(type(point1)==Node):
            point1 = point1.point
        if (type(point2) == Node):
            point2 = point2.point
        x = (point2[0]-point1[0]) ** 2
        y = (point2[1]-point1[1]) ** 2
        return np.sqrt(x+y)

    def getList(self ,point1, point2):
        if(type(point1)==Node):
            point1 = point1.point
        if (type(point2) == Node):
            point2 = point2.point
        dist = self.distance(point1, point2)
        dist = int(dist) + 1
        inc = dist * 10
        d = []
        xinc = float(point2[0] - point1[0]) / inc
        yinc = float(point2[1] - point1[1]) / inc
        for i in range(0, inc + 1):
            d.append((xinc * i + point1[0], yinc * i + point1[1]))
        return d

    def extend(self, point1, point2):
        pointList = self.getList(point1, point2)
        far = None
        for i in pointList:
            if(isCollisionFree(self.robot, i, self.obstacles)):
                far = i
                continue
            else:
                break
        if(far == None or far==point1):
            return point1
        self.add(point1,far)
        return far

    def getAll(self):
        temp = []
        stack = []
        ptr = self.kd
        while(len(stack)!=0 or ptr != None):
            if(ptr!= None):
                stack.insert(0,ptr)
                ptr = ptr.left
            else:
                cur = stack.pop()
                temp.append(cur)
                ptr = cur.right
        return temp

    def get_cost(self, point):
        return self.getNode(point).distanceFromStart

    def clearALong(self, point1, point2):
        p = self.getList(point1,point2)
        for i in p:
            if(isCollisionFree(self.robot, i, self.obstacles) == False):
                return False
        return True


    def rewire(self, point, r):
        all = self.getAll()
        neighborhoods = []
        for i in range(len(all)):
            if(self.distance(point, all[i].point) <= r):
                neighborhoods.append(all[i])
        if(len(neighborhoods)==0):
            return
        for i in neighborhoods:
            if(i.distanceFromStart  > self.getNode(point).distanceFromStart + self.distance(point, i.point) and self.clearALong(point, i.point)):
                temp = i.parent
                temp.neighbors.remove(i)
                i.neighbors.remove(i.parent)
                i.parent = self.getNode(point)
                i.neighbors.append(self.getNode(point))
                self.getNode(point).neighbors.append(i)
                i.distanceFromStart = self.getNode(point).distanceFromStart + self.distance(point, i.point)
        return
