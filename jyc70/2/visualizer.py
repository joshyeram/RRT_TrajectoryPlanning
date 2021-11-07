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

def visualize_problem_notDraw(robot, obstacles, start, goal):
    fig = plt.figure()
    axis = fig.gca()

    for i in obstacles:
        temp = i
        temp.append(temp[0])
        x, y = zip(*temp)
        plt.fill(x, y, color="black")

    robot.translation = (start[0], start[1])
    robot.rotation = start[2]
    initial = robot.transform()

    robot.translation = (goal[0], goal[1])
    robot.rotation = goal[2]
    final = robot.transform()

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

    plt.xlim(0, 10)
    plt.ylim(0, 10)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show(block=False)

def visualize_problem(robot, obstacles, start, goal):
    visualize_problem_notDraw(robot, obstacles, start, goal)
    plt.title('Problem Planning Scene')
    plt.show()

def visualize_points(points, robot, obstacles, start, goal):
    visualize_problem_notDraw(robot, obstacles, start, goal)
    plt.title('Points in Planning Scene')
    for i in points:
        robot.set_pose(i)
        x,y = zip(*(robot.transform()))
        plt.fill(x, y, color="blue")
    plt.show()

def drawPath(robot, path):
    for i in range(len(path) - 1):
        x = [path[i][0], path[i + 1][0]]
        y = [path[i][1], path[i + 1][1]]
        plt.plot(x, y, 'yo', linestyle='solid')
    plt.show(block=False)

def pointsAlongLines(point1, point2):
    points = []
    if(point1==point2):
        return [point1]
    x = (point2[0] - point1[0]) ** 2
    y = (point2[1] - point1[1]) ** 2
    d = np.sqrt(x+y)
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
        points.append((float("%.2f" % xTemp), float("%.2f" % yTemp), point2[2]))
    return points

def drawTree(robot, tree):
    draw = tree.nodes
    while (len(draw) > 0):
        if(draw[0].point==None):
            break
        plt.plot(draw[0].point[0], draw[0].point[1], marker='o', color="blue")
        for i in draw[0].neighbors:
            # print("drawing "+ str(draw[0].point)+ " and "+ str(i.point))
            x = [draw[0].point[0], i.point[0]]
            y = [draw[0].point[1], i.point[1]]
            plt.plot(x, y, 'bo', linestyle='solid')
        draw.pop(0)

def visualize_tree(robot, obstacles, path):

    if(path[0]==None):
        visualize_problem_notDraw(robot, obstacles, (-1,-1,0), (-1,-1,0))
        plt.title('No Path Found')
        drawTree(robot, path[3])
    else:
        visualize_problem_notDraw(robot, obstacles, path[0][0], path[0][-1])
        plt.title('Path Found')
        drawTree(robot, path[1])
        drawPath(robot, path[0])
    plt.show()

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

    if (path == None):
        plt.title("No Path Found")
        plt.xlim(0, 10)
        plt.ylim(0, 10)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.show()
        return

    robot.rotation = path[0][2]
    robot.translation = (path[0][0], path[0][1])
    initial = robot.transform()

    robot.rotation = path[-1][2]
    robot.translation = (path[-1][0], path[-1][1])
    final = robot.transform()

    initial.append(initial[0])
    final.append(final[0])

    xi, yi = zip(*initial)
    xf, yf = zip(*final)

    plt.fill(xi, yi, color="green")
    plt.fill(xf, yf, color="red")

    drawPath(robot,path)
    tree = Tree(None, None,None, None)
    pathPoints = []
    for i in range(len(path)-1):
        coll = []
        thetas = tree.getThetaList(path[i], path[i+1])
        for j in thetas:
            pathPoints.append((path[i][0],path[i][1],j))
        pathPoints.extend(pointsAlongLines(path[i],path[i+1]))
    pathPoints.append(path[-1])

    plt.title("Animated Path in Planning Scene")

    robotDraw = robot.transform()
    robotPatch = patches.Polygon(robotDraw, closed=True, fc='b', ec='b')
    axis.add_patch(robotPatch)

    def animate(i):
        robot.translation = (pathPoints[i][0], pathPoints[i][1])
        robot.rotation = pathPoints[i][2]
        robotP = robot.transform()
        robotPatch.set_xy(robotP)
        return robotPatch

    ani = animation.FuncAnimation(fig, animate, frames=len(pathPoints), repeat=False, interval=20)

    plt.show()

"""temp = parse_problem("robot_env_01.txt","probs_01.txt")
robot = temp[0]
obs = temp[1]
probs = temp[2]

points = [(5,5,3),(2,8,-1),(9,1,.5)]


visualize_problem(robot, obs, probs[0][0], probs[0][1])
visualize_points(points,robot, obs, probs[0][0], probs[0][1])

tempPath = rrt(robot, obs, probs[0][0], probs[0][1], 500)
#print(tempPath)

tm = rrt_star(robot, obs, probs[0][0], probs[0][1], 100)
#print(tm)

visualize_path(robot,obs,tempPath)
visualize_path(robot,obs,tm)"""