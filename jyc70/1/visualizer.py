import numpy as np
import matplotlib.pyplot as plt
import matplotlib.path as mplPath

from rrt import *
from tree import *
from collision import *
from sampler import *
from file_parse import *

def pointsAlongLines(point1, point2):
    points = []
    if(point1==point2):
        return [point1]
    d = distance(point1, point2)
    d = int(d) + 1
    if(d<2):
        inc = d * 10
    else:
        inc = d * 25
    xinc = float(point2[0] - point1[0]) / inc
    yinc = float(point2[1] - point1[1]) / inc
    for i in range(0, inc + 1):
        xTemp = xinc * i + point1[0]
        yTemp = yinc * i + point1[1]
        points.append((float("%.2f" % xTemp), float("%.2f" % yTemp)))
    return points

def robotTranslated(point, robot):
    newRobot = []
    for i in robot:
        newRobot.append((i[0]+point[0],i[1]+point[1]))
    return newRobot

def visualize_problem(robot, obstacles, start, goal):
    fig = plt.figure()
    axis = fig.gca()
    for i in obstacles:
        temp = i
        temp.append(temp[0])
        x,y = zip(*temp)
        plt.fill(x, y, color="black")

    initial = []
    final = []
    for i in range (0, len(robot)):
        initial.append([robot[i][0] + start[0],robot[i][1] + start[1]])
        final.append([robot[i][0] + goal[0], robot[i][1] + goal[1]])
    initial.append(initial[0])
    final.append(final[0])

    xi,yi = zip(*initial)
    xf,yf = zip(*final)

    plt.fill(xi, yi, color="green")
    plt.fill(xf, yf, color="red")

    axis.spines["top"].set_linewidth(2)
    axis.spines["right"].set_linewidth(2)
    axis.spines["left"].set_linewidth(2)
    axis.spines["bottom"].set_linewidth(2)

    plt.xlim(0, 10)
    plt.ylim(0, 10)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()

def visualize_points(points, robot, obstacles, start, goal):
    fig = plt.figure()
    axis = fig.gca()
    for i in obstacles:
        temp = i
        temp.append(temp[0])
        x, y = zip(*temp)
        plt.fill(x, y, color="black")

    initial = []
    final = []
    for i in range(0, len(robot)):
        initial.append([robot[i][0] + start[0], robot[i][1] + start[1]])
        final.append([robot[i][0] + goal[0], robot[i][1] + goal[1]])
    initial.append(initial[0])
    final.append(final[0])

    xi, yi = zip(*initial)
    xf, yf = zip(*final)


    plt.fill(xi, yi, color="green")
    plt.fill(xf, yf, color="red")

    axis.spines["top"].set_linewidth(2)
    axis.spines["right"].set_linewidth(2)
    axis.spines["left"].set_linewidth(2)
    axis.spines["bottom"].set_linewidth(2)

    for i in points:
        plt.plot(i[0],i[1], marker = 'o', color="blue")

    plt.xlim(0, 10)
    plt.ylim(0, 10)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()

def visualize_path(robot, obstacles, path):
    fig = plt.figure()
    axis = fig.gca()
    axis.spines["top"].set_linewidth(2)
    axis.spines["right"].set_linewidth(2)
    axis.spines["left"].set_linewidth(2)
    axis.spines["bottom"].set_linewidth(2)

    for i in obstacles:
        temp = i
        temp.append(temp[0])
        x, y = zip(*temp)
        plt.fill(x, y, color="black")

    initial = []
    final = []
    if(path ==False):
        plt.title("No Path Found")
        plt.xlim(0, 10)
        plt.ylim(0, 10)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.show()
        return

    for i in range(0, len(robot)):
        initial.append([robot[i][0] + path[0][0], robot[i][1] + path[0][1]])
        final.append([robot[i][0] + path[-1][0], robot[i][1] + path[-1][1]])
    initial.append(initial[0])
    final.append(final[0])

    xi, yi = zip(*initial)
    xf, yf = zip(*final)



    if (path != False):
        for i in range(len(path) - 1):
            x = [path[i][0], path[i + 1][0]]
            y = [path[i][1], path[i + 1][1]]
            plt.plot(x, y, 'yo', linestyle='solid')

    plt.fill(xi, yi, color="green")
    plt.fill(xf, yf, color="red")

    plt.xlim(0, 10)
    plt.ylim(0, 10)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()
    return

def visualize_configuration(robot, obstacles, start, goal):
    fig = plt.figure()
    axis = fig.gca()
    plt.title("Configuration Space")
    axis.spines["top"].set_linewidth(2)
    axis.spines["right"].set_linewidth(2)
    axis.spines["left"].set_linewidth(2)
    axis.spines["bottom"].set_linewidth(2)

    plt.plot(start[0], start[1], marker = 'o', color="green")
    plt.plot(goal[0], goal [1], marker='o', color="red")

    minRobot = []
    for i in robot:
        minRobot.append((-1*i[0], -1*i[1]))

    allPoints = []

    for i in obstacles:
        for j in range(len(i)-1):
            allPoints.extend(pointsAlongLines(i[j],i[j+1]))
        allPoints.extend(pointsAlongLines(i[0], i[-1]))

    allPoints.extend(pointsAlongLines((0,0), (0,10)))
    allPoints.extend(pointsAlongLines((0, 10), (10, 10)))
    allPoints.extend(pointsAlongLines((10, 10), (10, 0)))
    allPoints.extend(pointsAlongLines((10, 0), (0, 0)))

    for i in allPoints:
        newRobot = robotTranslated(i, minRobot)
        newRobot.append(newRobot[0])
        xi, yi = zip(*newRobot)
        plt.fill(xi, yi, color="grey")

    for i in obstacles:
        temp = i
        temp.append(temp[0])
        x, y = zip(*temp)
        plt.fill(x, y, color="black")

    plt.xlim(0, 10)
    plt.ylim(0, 10)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()

def configNotDraw(robot, obstacles, start, goal):
    fig = plt.figure()
    axis = fig.gca()
    plt.title("Configuration Space")
    axis.spines["top"].set_linewidth(2)
    axis.spines["right"].set_linewidth(2)
    axis.spines["left"].set_linewidth(2)
    axis.spines["bottom"].set_linewidth(2)

    plt.plot(start[0], start[1], marker='o', color="green")
    plt.plot(goal[0], goal[1], marker='o', color="red")

    minRobot = []
    for i in robot:
        minRobot.append((-1 * i[0], -1 * i[1]))

    allPoints = []

    for i in obstacles:
        for j in range(len(i) - 1):
            allPoints.extend(pointsAlongLines(i[j], i[j + 1]))
        allPoints.extend(pointsAlongLines(i[0], i[-1]))

    allPoints.extend(pointsAlongLines((0, 0), (0, 10)))
    allPoints.extend(pointsAlongLines((0, 10), (10, 10)))
    allPoints.extend(pointsAlongLines((10, 10), (10, 0)))
    allPoints.extend(pointsAlongLines((10, 0), (0, 0)))

    for i in allPoints:
        newRobot = robotTranslated(i, minRobot)
        newRobot.append(newRobot[0])
        xi, yi = zip(*newRobot)
        plt.fill(xi, yi, color="grey")

    for i in obstacles:
        temp = i
        temp.append(temp[0])
        x, y = zip(*temp)
        plt.fill(x, y, color="black")

    plt.xlim(0, 10)
    plt.ylim(0, 10)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show(block=False)

def visualize_rrt(robot, obstacles, start, goal, iter_n):
    tree = Tree(robot, obstacles, start, goal)
    path = []
    while (iter_n >= 0):
        # print(iter_n)
        sampled = sample()
        if (iter_n % 10 == 0):
            sampled = goal
        if (tree.getNode(sampled) != False):
            continue
        near = tree.nearest(sampled)
        actual = tree.extend(near, sampled)
        if (tree.distance(actual, goal) <= .25):
            attempt = tree.extend(actual, goal)
            if (attempt == goal):
                path = getPath(tree, start, goal)
                #drawEntireTree(tree, robot, obstacles, start, goal, path)
                configNotDraw(robot, obstacles, start, goal)
                drawTree(tree, robot, obstacles, start,goal, path)
                plt.plot(start[0], start[1], marker='o', color="green")
                plt.plot(goal[0], goal[1], marker='o', color="red")
                plt.show()
        iter_n -= 1
    lastNode = lastResort(tree, robot, obstacles, goal)
    if (lastNode == -1):
        #drawEntireTree(tree, robot, obstacles, start, goal, False)
        configNotDraw(robot, obstacles, start, goal)
        drawTree(tree, robot, obstacles, start, goal, False)
        plt.plot(start[0], start[1], marker='o', color="green")
        plt.plot(goal[0], goal[1], marker='o', color="red")
        plt.show()
    else:
        attempt = tree.extend(lastNode.point, goal)
        path = getPath(tree, start, goal)
        configNotDraw(robot, obstacles, start, goal)
        drawTree(tree, robot, obstacles, start, goal, path)
        plt.plot(start[0], start[1], marker='o', color="green")
        plt.plot(goal[0], goal[1], marker='o', color="red")
        #drawEntireTree(tree, robot, obstacles, start, goal, path)
    plt.show()

def helper(robot, obstacles, start, goal):
    configNotDraw(robot,obstacles,start,goal)



temp = parse_problem("robot_env_01.txt","probs_01.txt")
robot = temp[0]
obs = temp[1]



path = rrt(temp[0], temp[1], (3,3), (8.5,8.5), 500)
#print(path)
#visualize_path(temp[0],temp[1], path)
#visualize_configuration(robot, obs, (3,3), (8.5,8.5))
#visualize_rrt(temp[0],temp[1],(3,3), (8.5,8.5), 100)

#helper(temp[0],temp[1],(3,3), (8.5,8.5))