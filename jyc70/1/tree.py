import numpy as np
import matplotlib.pyplot as plt
import matplotlib.path as mplPath

from problem1 import *

class Node:

    def __init__(self, point, state):
        self.point = point
        self.neighbors = []
        self.parent = None
        self.state = state
        self.right = None
        self.left = None

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
        print(str(point)+": not found")

    def add(self, point1, point2):
        #print("adding = "+ str(point1)+ " " +str(point2))
        node1 = self.getNode(point1)
        node2 = Node(point2, -1)
        self.insertkd(node2)
        node1.neighbors.append(node2)
        node2.neighbors.append(node1)
        node2.parent = node1

    def exists(self, point):
        if(self.getNode(point)==False):
            return False
        return True

    def parent(self, point):
        return self.getNode(point).parent

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
            return
        else:
            prev.right = node
            return

    def nearest(self, point):
        ptr = self.kd
        level = 0  # if level even = compare horizontal. If level odd = compare vertically
        temp = []
        while (ptr != None):
            temp.append(ptr)
            if ((level % 2 == 0 and point[0] < ptr.point[0]) or (level % 2 == 1 and point[1] < ptr.point[1])):
                ptr = ptr.left
            else:
                ptr = ptr.right
            level += 1
        d = []
        for i in temp:
            d.append(self.distance(point, i.point))
        return temp[d.index(min(d))].point

    def distance(self, point1, point2):
        x = (point2[0]-point1[0]) ** 2
        y = (point2[1]-point1[1]) ** 2
        return np.sqrt(x+y)

    def getList(self ,point1, point2):
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
            return False
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

