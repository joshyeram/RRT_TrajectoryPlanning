import numpy as np
import matplotlib.pyplot as plt
import matplotlib.path as mplPath
from file_parse import *
from robot import *

def between(x,y,number):
    if(x<y):
        if(number>x and number<y):
            return True
    else:
        if (number > y and number < x):
            return True
    return False

def extendAppend(a):
    if(len(a)==0):
        return []
    b = []
    for i in a:
        b.append(i)
    b.append(a[0])
    return b

def isCollisionFree(robot, point, obstacles):
    #1: check if any point is outside boundary - done
    #2: check if any edges are intersecting - done
    #3: check if any midpoints or vertices are inside of each other
    #4: check if they are the same shape with the same coords

    robot.set_pose(point)
    robotO = robot.transform()
    for i in range(0, len(robotO)):
        if (robotO[i][0] < 0 or robotO[i][0] > 10):
            return False
        if (robotO[i][1] < 0 or robotO[i][1] > 10):
            return False
    pos = extendAppend(robotO)
    for i in obstacles:
        extended = extendAppend(i)
        for j in range (0,len(extended)-1):
            linePoly= []
            linePoly.append(extended[j])
            linePoly.append(extended[j+1])
            for k in range(0, len(pos)-1):
                robotLine = []
                robotLine.append(pos[k])
                robotLine.append(pos[k+1])
                if(lineToLine(linePoly,robotLine)):
                    return False

    allPolyPoints = []
    for i in obstacles:
        for j in range(0, len(i)):
            allPolyPoints.append(i[j])
            if(j == len(i)-1):
                avgx = (i[0][0]+i[-1][0]) / 2
                avgy = (i[0][1] + i[-1][1]) / 2
            else:
                avgx = (i[j][0] + i[j+1][0]) / 2
                avgy = (i[j][1] + i[j+1][1]) / 2
            allPolyPoints.append((avgx,avgy))
    for i in allPolyPoints:
        if(pointToPolygon(i,robotO)):
            return False

    allRobotPoints = []

    for i in range(0, len(pos)):
        allRobotPoints.append(pos[i])
        if (i == len(pos) - 1):
            avgx = (pos[0][0] + pos[-1][0]) / 2
            avgy = (pos[0][1] + pos[-1][1]) / 2
        else:
            avgx = (pos[i][0] + pos[i+1][0]) / 2
            avgy = (pos[i][1] + pos[i+1][1]) / 2
        allRobotPoints.append((avgx, avgy))
    for i in obstacles:
         for j in allRobotPoints:
             if (pointToPolygon(j, i)):
                 return False

    tuplesorted = sorted(robotO, key=lambda tup: tup[0])
    for i in obstacles:
        if(len(i)!=len(tuplesorted)):
            continue
        polyTupledSorted = sorted(i, key=lambda tup: tup[0])
        if(tuplesorted == polyTupledSorted):
            return False
    return True

def lineToLine(line1,line2):
    x1_1 = line1[0][0]
    y1_1 = line1[0][1]
    x1_2 = line1[1][0]
    y1_2 = line1[1][1]
    x2_1 = line2[0][0]
    y2_1 = line2[0][1]
    x2_2 = line2[1][0]
    y2_2 = line2[1][1]

    vert = 0
    hori = 0
    if(x1_1==x1_2):
        vert+=1
    if (x2_1 == x2_2):
        vert += 2
    if(vert == 3):
        return False
    if (y1_1 == y1_2):
        hori += 1
    if (y2_1 == y2_2):
        hori += 2
    if (hori == 3):
        return False
    if(hori != 0 and vert !=0):
        if(hori == 1):
            temp = line2
            line2 = line1
            line1 = temp
        if(line1[0][0]>line2[0][0] and line1[0][0]<line2[1][0] and line2[0][1]>line1[0][1] and line2[0][1]>line1[1][1]):
            return True
        return False
    elif (vert == 2):
        temp = line2
        line2 = line1
        line1 = temp
    elif (hori == 2):
        temp = line2
        line2 = line1
        line1 = temp

    x1_1 = line1[0][0]
    y1_1 = line1[0][1]
    x1_2 = line1[1][0]
    y1_2 = line1[1][1]
    x2_1 = line2[0][0]
    y2_1 = line2[0][1]
    x2_2 = line2[1][0]
    y2_2 = line2[1][1]

    m1 = -1.0
    m2 = -1.0
    b1 = -1.0
    b2 = -1.0

    if(vert !=0 or hori != 0):
        if(vert !=0):
            m2 = (y2_2-y2_1)/(x2_2-x2_1)
            b2 = -1*(m2 * x2_1 - y2_1)
            yguess = m2 * x1_1 + b2
            if(between(y1_1,y1_2, yguess) and between(x2_1,x2_2,x1_1)):
                return True
            return False
        else:
            m2 = (y2_2 - y2_1) / (x2_2 - x2_1)
            b2 = -1 * (m2 * x2_1 - y2_1)
            xguess = (y1_1 - b2)/m2
            if (between(x1_1, x1_2, xguess) and between(y2_1, y2_2, y1_1)):
                return True
            return False
    else:
        m1 = (y1_2 - y1_1) / (x1_2 - x1_1)
        b1 = -1 * (m1 * x1_1 - y1_1)
        m2 = (y2_2 - y2_1) / (x2_2 - x2_1)
        b2 = -1 * (m2 * x2_1 - y2_1)
        if (m1 == m2):
            return False
        xguess = (b1-b2)/(m2-m1)
        yguess = m1 * xguess + b1
        if(between(x1_1,x1_2,xguess) and between(y1_1,y1_2,yguess) and between(x2_1,x2_2,xguess) and between(y2_1,y2_2,yguess)):
            return True

    return False

def pointToPolygon(point, polygon):
    for i in polygon:
        if (i[0] == point[0] and i[1] == point[1]):
            return False
    poly = mplPath.Path(polygon)
    return poly.contains_point(point)