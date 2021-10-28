import numpy as np
import matplotlib.pyplot as plt
import matplotlib.path as mplPath

from file_parse import *
from collision import *
from robot import *

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
    nodes = []

    def __init__(self, robot, obstacles, start, goal):
        self.robot = robot
        self.obstacles = obstacles
        self.start = Node(start, 1)
        self.goal = Node(goal, 2)
        self.kd = self.start
        self.nodes.append(self.start)

    def getNode(self, point):
        if(type(point)==Node):
            point = point.point

        for i in self.nodes:
            if(i.point == point):
                return i
        return False

    def add(self, point1, point2):
        #print("adding = "+ str(point1)+ " " +str(point2))
        node1 = self.getNode(point1)
        node2 = Node(point2, -1)
        node1.neighbors.append(node2)
        node2.neighbors.append(node1)
        node2.parent = node1
        node2.distanceFromStart = node1.distanceFromStart + self.distance(point1, point2)
        self.nodes.append(node2)

    def exists(self, point):
        if(self.getNode(point)==False):
            return False
        return True

    def parent(self, point):
        return self.getNode(point).parent

    def nearest(self, point):
        if(len(self.nodes)==1):
            return self.start.point
        temp = self.nodes[0]
        for i in self.nodes:
            if(self.distance(point,temp.point)>self.distance(point,i.point)):
                temp = i
        return temp.point

    def nearest_star(self, point, r):
        if(len(self.nodes)==1):
            return self.start.point
        temp = self.nodes[0]
        for i in self.nodes:
            if(self.distanceEuc(point,temp.point)>self.distanceEuc(point,i.point)):
                temp = i
        return temp.point

    def distance(self, point1, point2):
        if(type(point1)==Node):
            point1 = point1.point
        if (type(point2) == Node):
            point2 = point2.point
        x = (point2[0]-point1[0]) ** 2
        y = (point2[1]-point1[1]) ** 2
        tTemp = (point2[2] - point1[2]) % (2 * np.pi)
        if(tTemp< - np.pi):
            tTemp += 2 * np.pi
        elif(tTemp>np.pi):
            tTemp -= 2 * np.pi

        t = (tTemp) ** 2
        return np.sqrt(x+y+t)

    def distanceEuc(self, point1, point2):
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
        dist = self.distanceEuc(point1, point2)
        dist = int(dist) + 1
        inc = dist * 10
        d = []
        xinc = float(point2[0] - point1[0]) / inc
        yinc = float(point2[1] - point1[1]) / inc
        for i in range(0, inc + 1):
            d.append((xinc * i + point1[0], yinc * i + point1[1]))
        return d

    def getThetaList(self ,point1, point2):
        delta = (point2[2] - point1[2]) % (2 * np.pi)
        if(delta < -np.pi):
            delta +=np.pi *2
        elif(delta>np.pi):
            delta -= 2 * np.pi

        inc = delta/10
        thetas = []
        for i in range(11):
            thetas.append(inc * i + point1[2])

        return thetas

        """if((point2[2]>=0 and point1[2]>=0 and point2[2] >= point1[2]) or (point2[2]<= 0 and point1[2] <= 0 and point2[2] >= point1[2])):
            diff = np.abs(point2[2] - point1[2])
            thetas = []
            inc = diff / 10
            for i in range(11):
                thetas.append(inc * i + point1[2])
            thetas.append(point2[2])
            return thetas

        elif(point1[2]>=0 and point2[2]<=0):
            thetas = []
            diff = np.abs(np.pi - point1[2])
            inc = diff / 10
            for i in range(11):
                thetas.append(inc * i + point1[2])

            diff = np.pi - np.abs(point2[2])
            inc = diff / 10
            for i in range(11):
                thetas.append(- np.pi + inc * i )
            return thetas

        elif(point1[2]<=0 and point2[2]>=0):
            thetas = []
            diff = np.abs(point1[2])
            inc = diff / 10
            for i in range(11):
                thetas.append(inc * i + point1[2])

            diff = point2[2]
            inc = diff / 10
            for i in range(11):
                thetas.append(inc * i)
            return thetas

        elif(point2[2]>=0 and point1[2]>=0 and point2[2] <= point1[2]):
            thetas = []
            diff = np.pi - point1[2]
            inc = diff / 10
            for i in range(11):
                thetas.append(inc * i + point1[2])

            diff = np.pi
            inc = diff / 10
            for i in range(11):
                thetas.append(-np.pi + inc * i)

            diff = point2[2]
            inc = diff / 10
            for i in range(11):
                thetas.append(inc * i)
            return thetas

        elif(point2[2]<=0 and point1[2]<=0 and point2[2] <= point1[2]):
            thetas = []
            diff = np.abs(point1[2])
            inc = diff / 10
            for i in range(11):
                thetas.append(inc * i + point1[2])

            diff = np.pi
            inc = diff / 10
            for i in range(11):
                thetas.append(inc * i)

            diff = np.abs(np.pi + point2[2])
            inc = diff / 10
            for i in range(11):
                thetas.append(-np.pi + inc * i)
            return thetas"""


    def extend1(self, point1, point2):
        #see if it is clear for rotation
        tempTheta = point1[2]
        if(point1[2]!=point2[2]):
            thetas = self.getThetaList(point1, point2)
            temp = thetas[0]
            for i in thetas:
                self.robot.rotation = i
                if(isCollisionFree(self.robot, (point1[0],point1[1],i) , self.obstacles)):
                    temp = i
                else:
                    break
            self.robot.rotation = temp

        pointList = self.getList(point1, point2)
        temp = []
        for i in pointList:
            temp.append((i[0], i[1], self.robot.rotation))

        far = None
        for i in temp:
            if(isCollisionFree(self.robot, i, self.obstacles)):
                far = i
                continue
            else:
                break
        if(far == None or far==point1):
            return point1
        self.add(point1,(far[0],far[1], self.robot.rotation))

        return (far[0],far[1], self.robot.rotation)

    def get_cost(self, point):
        return self.getNode(point).distanceFromStart

    def clearALong(self, point1, point2):
        thetas = self.getThetaList(point1,point2)
        for i in thetas:
            self.robot.set_pose((point1[0],point1[1],i))
            if(isCollisionFree(self.robot,(point1[0],point1[1],i), self.obstacles)==False):
                self.robot.set_pose(point1)
                return False
        self.robot.rotation = point2[2]
        p = self.getList(point1, point2)
        for i in p:
            if(isCollisionFree(self.robot, (i[0],i[1],point2[2]), self.obstacles) == False):
                return False
        return True

    def extend(self, point, n1, n2, dt):
        dur = np.random.randint(n1, n2)
        r = Robot(0,0)

        iter = int(dur/dt)
        controls = []

        for i in range(iter):
            v = np.random.uniform(-.5,.5)
            w = np.random.uniform(-.3, .3) * np.pi
            controls.append((v,w))

        path = r.propogate(point, controls, dur, dt)

        temp = path[0]

        for i in path:
            if(isCollisionFree(self.robot, i, self.obstacles)):
                self.add(temp, i)
                temp = i
            else:
                break

        return temp


"""
start = (2,2)
end = (8,8)
tree = Tree(None, None, start, end)
print(tree.getThetaList((1,1,-1), (1,1,-2)))



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
        try:
            point = (float(x), float(y))
            print("for xy nearest: " + str(point) + "is" + str(tree.nearest(point)))
        except:
            print("error")"""


