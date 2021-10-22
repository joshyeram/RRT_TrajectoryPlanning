import numpy as np
import matplotlib.pyplot as plt
import matplotlib.path as mplPath

class Node:
    point = (-1,-1)
    neighbors = []
    parent = None
    state = -1

    def __init__(self, point, state):
        self.point = point
        self.state = state
        self.kdleft = None
        self.kdright = None

    def add(self, otherPt):
    #when adding, 2 cases
    #has parent: 1. go to old parent. 2, delete curr node from neighbors 3. set parent to new parent 4. add parent to neighbor
    #or fresh node: 1. check if it has parent 2. if not, just add it to neighbors 3. set to parent
        if (otherPt.parent != None):
            temp = otherPt.parent
            temp.neighbors.remove(otherPt)
            otherPt.neighbor.remove(temp)
            otherPt.parent = self
        otherPt.neighbor.append(self)
        self.neighbors.append(otherPt)
        otherPt.parent = self

    def checkNeighbor(self, otherPt):
        return otherPt in self.neighbors


class Tree:
    robot = None
    obstacles = None
    start = None
    goal = None
    kd = None
    nodes = [] #firstpoint is self and the rest are its neighbors

    def __init__(self, robot, obstacles, start, goal):
        self.robot = robot
        self.obstacles = obstacles
        self.start = Node(start, 1)
        self.goal = Node(goal, 2)
        self.kd = start

    def add(self, point1, point2):
        pointToAdd = Node(point2,-1)
        point1.add(pointToAdd)

    def exists(self, point):
        if(len(self.nodes)==0):
            return False
        for i in self.nodes:
            if(i.point == point):
                return True
        return False

    def parent(self, point):
        if (len(self.nodes) == 0):
            return False
        for i in self.nodes:
            if (i.point == point):
                return i.point.parent.point
        return False

    def insertkd(self, node):
        ptr = self.kd
        prev = None
        level = 0 #if level even = compare horizontal. If level odd = compare vertically
        while(ptr!= None):
            prev = ptr
            if(level%2 ==0 and node.point[0] < ptr.point[0] or level%2 ==1 and node.point[1] < ptr.point[1]):
                ptr = ptr.kdleft
            else:
                ptr = ptr.kdright
            level += 1

        if(level % 2 ==0 and node.point[0] < prev.point[0] or level % 2 ==1 and node.point[1] < prev.point[1]):
            prev.kdleft = node
        else:
            prev.kdright = node

    def nearest(self, point):
        ptr = self.kd
        prev = None
        level = 0  # if level even = compare horizontal. If level odd = compare vertically
        while (ptr != None):
            prev = ptr
            if (level % 2 == 0 and point.point[0] < ptr.point[0] or level % 2 == 1 and point.point[1] < ptr.point[1]):
                ptr = ptr.kdleft
            else:
                ptr = ptr.kdright
            level += 1
        return prev

    def nearestNode(self, node):
        ptr = self.kd
        prev = None
        level = 0  # if level even = compare horizontal. If level odd = compare vertically
        while (ptr != None):
            prev = ptr
            if (level % 2 == 0 and node.point[0] < ptr.point[0] or level % 2 == 1 and node.point[1] < ptr.point[1]):
                ptr = ptr.kdleft
            else:
                ptr = ptr.kdright
            level += 1
        return prev

    def extend(self, point1, point2):
        pass

