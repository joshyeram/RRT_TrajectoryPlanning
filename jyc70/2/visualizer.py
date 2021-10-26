import numpy as np
import matplotlib.pyplot as plt
import matplotlib.path as mplPath
import matplotlib.animation as animation
import matplotlib.patches as patches

from rrt import *
#from rrt_star import *
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
        plt.plot(i[0], i[1], marker='o', color="blue")
    plt.show()

temp = parse_problem("robot_env_01.txt","probs_01.txt")
robot = temp[0]
obs = temp[1]

#visualize_problem(robot, obs, temp[2][0][0], temp[2][0][1])
#visualize_points([(1,1),(3,2), (2,8)],robot, obs, temp[2][0][0], temp[2][0][1])

