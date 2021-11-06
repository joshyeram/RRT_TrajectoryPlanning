import numpy as np
import matplotlib.pyplot as plt
import matplotlib.path as mplPath
import matplotlib.animation as animation
import matplotlib.patches as patches

from rrt import *
from rrt_star import *
from tree import *
from collision import *
from sampler import *
from file_parse import *

def distance(point1, point2):
    x = (point2[0]-point1[0]) ** 2
    y = (point2[1]-point1[1]) ** 2
    return np.sqrt(x+y)

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

    axis.spines["top"].set_linewidth(1.5)
    axis.spines["right"].set_linewidth(1.5)
    axis.spines["left"].set_linewidth(1.5)
    axis.spines["bottom"].set_linewidth(1.5)

    plt.xlim(0, 10)
    plt.ylim(0, 10)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.title("Visualize Problem")
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

    axis.spines["top"].set_linewidth(1.5)
    axis.spines["right"].set_linewidth(1.5)
    axis.spines["left"].set_linewidth(1.5)
    axis.spines["bottom"].set_linewidth(1.5)

    for i in points:
        t = robotTranslated(i, robot)
        t.append(t[0])
        xt, yt = zip(*t)
        plt.fill(xt,yt, color="blue")

    plt.xlim(0, 10)
    plt.ylim(0, 10)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.title("Visualize Points")
    plt.show()

def visualize_pathNOT(robot, obstacles, path):
    fig = plt.figure()
    axis = fig.gca()
    axis.spines["top"].set_linewidth(1.5)
    axis.spines["right"].set_linewidth(1.5)
    axis.spines["left"].set_linewidth(1.5)
    axis.spines["bottom"].set_linewidth(1.5)

    for i in obstacles:
        temp = i
        temp.append(temp[0])
        x, y = zip(*temp)
        plt.fill(x, y, color="black")

    initial = []
    final = []

    if(path == None):
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

    if (path != None):
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

def visualize_path(robot, obstacles, path):
    fig = plt.figure()
    axis = fig.gca()
    axis.spines["top"].set_linewidth(1.5)
    axis.spines["right"].set_linewidth(1.5)
    axis.spines["left"].set_linewidth(1.5)
    axis.spines["bottom"].set_linewidth(1.5)
    plt.xlim(0, 10)
    plt.ylim(0, 10)
    plt.gca().set_aspect('equal', adjustable='box')

    for i in obstacles:
        temp = i
        temp.append(temp[0])
        x, y = zip(*temp)
        plt.fill(x, y, color="black")

    initial = []
    final = []
    if (path == None):
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

    for i in range(len(path) - 1):
        x = [path[i][0], path[i + 1][0]]
        y = [path[i][1], path[i + 1][1]]
        plt.plot(x, y, 'yo', linestyle='solid')

    plt.fill(xi, yi, color="green")
    plt.fill(xf, yf, color="red")

    pathPoints = []
    for i in range(len(path)-1):
        pathPoints.extend(pointsAlongLines(path[i],path[i+1]))
    plt.title("Animated Path in Planning Scene")

    robotDraw = np.array(robot)
    robotPatch = patches.Polygon(robotDraw, closed=True, fc='b', ec='b')
    axis.add_patch(robotPatch)

    def animate(i):
        robotPos = robotTranslated(pathPoints[i],robot)
        robotP = np.array(robotPos)
        robotPatch.set_xy(robotP)
        return robotPatch

    ani = animation.FuncAnimation(fig, animate, frames=len(pathPoints), repeat=False, interval=20)
    plt.show()

def visualize_configuration(robot, obstacles, start, goal):
    fig = plt.figure()
    axis = fig.gca()
    plt.title("Configuration Space")
    axis.spines["top"].set_linewidth(1.5)
    axis.spines["right"].set_linewidth(1.5)
    axis.spines["left"].set_linewidth(1.5)
    axis.spines["bottom"].set_linewidth(1.5)

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
    axis.spines["top"].set_linewidth(1.5)
    axis.spines["right"].set_linewidth(1.5)
    axis.spines["left"].set_linewidth(1.5)
    axis.spines["bottom"].set_linewidth(1.5)

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
            iter_n -= 1
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
                plt.title("RRT in Configuration Space")
                plt.show()
                return
        iter_n -= 1
    lastNode = lastResort(tree, robot, obstacles, goal)
    if (lastNode == -1):
        configNotDraw(robot, obstacles, start, goal)
        drawTree(tree, robot, obstacles, start, goal, False)
        plt.plot(start[0], start[1], marker='o', color="green")
        plt.plot(goal[0], goal[1], marker='o', color="red")
        plt.title("RRT in Configuration Space: No Path Found")
        plt.show()
        return
    else:
        attempt = tree.extend(lastNode.point, goal)
        path = getPath(tree, start, goal)
        configNotDraw(robot, obstacles, start, goal)
        drawTree(tree, robot, obstacles, start, goal, path)
        plt.plot(start[0], start[1], marker='o', color="green")
        plt.plot(goal[0], goal[1], marker='o', color="red")
    plt.title("RRT in Configuration Space")
    plt.show()

def visualize_rrt_star(robot, obstacles, start, goal , iter_n):
    temp = rrt_star_vis(robot, obstacles, start, goal , iter_n)
    configNotDraw(robot, obstacles, start, goal)
    drawTree(temp[0], robot, obstacles, start, goal, temp[1])
    plt.plot(start[0], start[1], marker='o', color="green")
    plt.plot(goal[0], goal[1], marker='o', color="red")
    if(temp[1]==None):
        plt.title("No path found with RRT Star in Configuration Space")
    else:
        plt.title("RRT Star in Configuration Space")
    plt.show()

temp = parse_problem("robot_env_01.txt","probs_01.txt")
robot = temp[0]
obs = temp[1]
probs = temp[2]

#print(robot)
#print(obs)
#print(probs)
#points = [(5.2,6.7), (9.2,2.3)]

#visualize_problem(robot, obs, probs[0][0],probs[0][1])
#visualize_points(points,robot, obs, probs[0][0],probs[0][1])
#visualize_configuration(robot, obs,probs[0][0],probs[0][1])
#tempPath = rrt(robot, obs, probs[0][0],probs[0][1],99)
#print(tempPath)

#visualize_path(robot, obs, tempPath)
#visualize_rrt(robot,obs, probs[0][0],probs[0][1], 100)

#tempPathStar = rrt_star(robot, obs, probs[0][0],probs[0][1], 1000)
#print(tempPathStar)
#visualize_path(robot, obs, tempPathStar)

#visualize_rrt_star(robot, obs, probs[0][0],probs[0][1], 2000)