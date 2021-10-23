import numpy as np
import matplotlib.pyplot as plt
import matplotlib.path as mplPath

from problem1 import *
from tree import *

def drawTree(tree, robot, obstacles, start, goal):
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
    for i in range(0, len(robot)):
        initial.append([robot[i][0] + start[0], robot[i][1] + start[1]])
        final.append([robot[i][0] + goal[0], robot[i][1] + goal[1]])
    initial.append(initial[0])
    final.append(final[0])

    xi, yi = zip(*initial)
    xf, yf = zip(*final)

    plt.fill(xi, yi, color="green")
    plt.fill(xf, yf, color="red")

    draw = tree.getAll()

    while(len(draw)>0):
        plt.plot(draw[0].point[0], draw[0].point[1], marker='o', color="blue")
        for i in draw[0].neighbors:
            #print("drawing "+ str(draw[0].point)+ " and "+ str(i.point))
            x = [draw[0].point[0], i.point[0]]
            y = [draw[0].point[1], i.point[1]]
            plt.plot(x, y, 'bo', linestyle='solid')
        draw.pop(0)

    plt.xlim(0, 10)
    plt.ylim(0, 10)
    plt.gca().set_aspect('equal', adjustable='box')

    plt.show()

def rrt(robot, obstacles, start, goal, iter_n):
    tree = Tree(robot, obstacles, start, goal)
    while(iter_n >=0):
        #print(iter_n)
        sampled = sample()
        near = tree.nearest(sampled)
        actual = tree.extend(near, sampled)
        iter_n-=1
    drawTree(tree, robot, obstacles, start, goal)

def visualize_path(robot, obstacles, path):
    return

def visualize_configuration(robot, obstacles, start, goal):
    return

def visualize_rrt(robot, obstacles, start, goal, iter_n):
    return

temp = parse_problem("robot_env_01.txt","probs_01.txt")
rrt(temp[0], temp[1], (1,1), (9,9), 100)