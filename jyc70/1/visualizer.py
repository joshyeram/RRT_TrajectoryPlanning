from rrt import *

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

    plt.fill(xi, yi, color="green")
    plt.fill(xf, yf, color="red")

    if (path != False):
        for i in range(len(path) - 1):
            x = [path[i][0], path[i + 1][0]]
            y = [path[i][1], path[i + 1][1]]
            plt.plot(x, y, 'ro', linestyle='solid')

    plt.xlim(0, 10)
    plt.ylim(0, 10)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()
    return

def visualize_configuration(robot, obstacles, start, goal):
    return

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
                drawEntireTree(tree, robot, obstacles, start, goal, path)
                return path
        iter_n -= 1
    lastNode = lastResort(tree, robot, obstacles, goal)
    if (lastNode == -1):
        drawEntireTree(tree, robot, obstacles, start, goal, False)
        return False
    attempt = tree.extend(lastNode.point, goal)
    path = getPath(tree, start, goal)
    drawEntireTree(tree, robot, obstacles, start, goal, path)
    return

temp = parse_problem("robot_env_01.txt","probs_01.txt")
path = rrt(temp[0], temp[1], (3,3), (8.5,8.5), 500)
#print(path)
#visualize_path(temp[0],temp[1], path)
visualize_rrt(temp[0],temp[1],(3,3), (8.5,8.5), 200)