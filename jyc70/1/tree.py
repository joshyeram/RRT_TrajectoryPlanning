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

    def closer(self, check, point1, point2):

        if(point1==None):
            return point2
        if(point2 == None):
            return point1
        if (type(point1) == Node):
            point1 = point1.point
        if (type(point2) == Node):
            point2 = point2.point
        if(distance(check, point1) < distance(check, point2)):
            return point1
        return point2

    def nearestOpt(self, root, point, depth):
        if(root == None):
            return None

        if(point[depth % 2] < root.point[depth % 2]):
            nextCheck = root.left
            oppCheck = root.right
        else:
            nextCheck = root.right
            oppCheck = root.left

        opt = self.closer(point, self.nearestOpt(nextCheck, point, depth+1), root.point)

        optX, optY = opt
        tempX, tempY = point
        s = (tempX-optX) **2 + (tempY-optY) **2

        if(s > (point[depth % 2] - root.point[depth %2]) **2):
            opt = self.closer(point, self.nearestOpt(oppCheck, point, depth+1), root)

        return opt

    def nearest1(self, point):
        return self.nearestOpt(self.kd, point, 0)
    def nearest(self, point):
        all = self.getAll()
        temp = []
        for i in all:
            temp.append(distance(point, i.point))
        return all[temp.index(min(temp))].point

    def nearest1(self, point):
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


"""start = (2,2)
end = (8,8)
tree = Tree(None, None, start, end)

for i in range (10):
    samp = (np.random.randint(0,10), np.random.randint(0,10))
    if (tree.getNode(samp) == False):
        tree.insertkd(Node(samp, -1))
        print(samp)


while(True):
    x = input()
    y = input()
    if(y =="l" or y =="r"):
        temp = x.split()
        x=(int(temp[0]),int(temp[1]))
        if(y=="l"):
            try:
                print(tree.getNode(x).left.point)
            except:
                print("not found")
        else:
            try:
                print(tree.getNode(x).right.point)
            except:
                print("not found")
    else:
        point = (float(x), float(y))
        print("for xy: "+ str(point) + "is" + str(tree.nearest(point)))"""

