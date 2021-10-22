import numpy as np
import matplotlib.pyplot as plt
import matplotlib.path as mplPath

class Node:
    point = (-1,-1)
    neighbors = []
    parent = None
    state = -1
    right = None
    left = None

    def __init__(self, point, state):
        self.point = point
        self.state = state

    # when adding, 2 cases
    # has parent: 1. go to old parent. 2, delete curr node from neighbors 3. set parent to new parent 4. add parent to neighbor
    # or fresh node: 1. check if it has parent 2. if not, just add it to neighbors 3. set to parent

    def add(self, otherPt):
        self.parent = otherPt
        self.neighbors.append(otherPt)
        otherPt.addNeighbor(self)

    def changeParent(self, otherPt):
        temp = self.parent
        self.neighbors.remove(self.parent)
        temp.neighbor.remove(self)
        self.addNeighbor(otherPt)
        self.parent = otherPt

    def addNeighbor(self, otherPt):
        if(otherPt not in self.neighbors):
            self.neighbors.append(otherPt)
            return True
        return False

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
        return False

    def add(self, point1, point2):
        pointToAdd = Node(point2,-1)
        pointToAdd.add(self.getNode(point1))

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
            if(level%2 ==0 and node.point[0] < ptr.point[0] or level%2 ==1 and node.point[1] < ptr.point[1]):
                ptr = ptr.left
            else:
                ptr = ptr.right
            level += 1
        if(level % 2 ==0 and node.point[0] < prev.point[0] or level % 2 ==1 and node.point[1] < prev.point[1]):
            prev.left = node
        else:
            prev.right = node

    def nearest(self, point):
        ptr = self.kd
        level = 0  # if level even = compare horizontal. If level odd = compare vertically
        temp = []
        while (ptr != None):
            temp.append(ptr)
            if (level % 2 == 0 and point[0] < ptr.point[0] or level % 2 == 1 and point[1] < ptr.point[1]):
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

    def extend(self, point1, point2):
        pass

tree = Tree(None, None, (5,5), (8,8))
node1 = Node((2.5,2.5), -1)
node2 = Node((7.5,7.5), -1)
node3 = Node((1,7.5), -1)
node4 = Node((6,3.5), -1)
node5 = Node((4,6), -1)
node6 = Node((9,5), -1)

tree.insertkd(node1)
tree.insertkd(node2)
tree.insertkd(node3)
tree.insertkd(node4)
tree.insertkd(node5)
tree.insertkd(node6)

print(tree.nearest((8,1)))
print(tree.nearest((4.5,4.5)))
print(tree.getNode((2.5,2.5)))
print(tree.getNode((4,6)))
print(tree.getNode((9,6)))
print(tree.getNode((9,1)))