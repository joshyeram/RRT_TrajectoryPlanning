import numpy as np
import matplotlib.pyplot as plt
import matplotlib.path as mplPath

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

def parse_problem(world_file, problem_file):
    world = open(world_file,"r")
    problems = open(problem_file, "r")

    worldCoords = world.readlines()
    problem = problems.readlines()

    if(len(worldCoords)==0):
        print("no robot found")
        return
    if (len(worldCoords) == 1):
        print("no obstacles found")
        return
    if (len(problem) == 0):
        print("no problems found")
        return

    robotCoord = worldCoords[0].split()
    robot = []
    for i in range(0, len(robotCoord), 2):
        robot.append((float(robotCoord[i]),float(robotCoord[i+1])))
    #print(robot)

    obs = []
    for i in range(1, len(worldCoords)):
        temp = worldCoords[i].split()
        tempObs = []
        for i in range(0, len(temp), 2):
            tempObs.append((float(temp[i]), float(temp[i + 1])))
        obs.append(tempObs)
    #print(obs)

    probs = []
    for i in range(0, len(problem)):
        temp = problem[i].split()
        tempProbs = []
        for i in range(0, len(temp), 2):
            tempProbs.append((float(temp[i]), float(temp[i + 1])))
        probs.append(tempProbs)
    #print(probs)

    world.close()
    problems.close()
    return (robot, obs, probs)

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
    return

def sample():
    x = np.random.randint(99)/10.
    y = np.random.randint(99)/10.
    return (x+.1,y+.1)

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
    return

def isCollisionFree(robot, point, obstacles):
    #1: check if any point is outside boundary - done
    #2: check if any edges are intersecting - done
    #3: check if any midpoints or vertices are inside of each other
    #4: check if they are the same shape with the same coords

    transpPos = []
    for i in range(0, len(robot)):
        transpPos.append((robot[i][0] + point[0], robot[i][1] + point[1]))
        if (transpPos[i][0] < 0 or transpPos[i][0] > 10):
            return False
        if (transpPos[i][1] < 0 or transpPos[i][1] > 10):
            return False
    pos = extendAppend(transpPos)
    for i in obstacles:
        extended = extendAppend(i)
        for j in range (0,len(extended)-2):
            linePoly= []
            linePoly.append(extended[j])
            linePoly.append(extended[j+1])
            for k in range(0, len(pos)-2):
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
                avgx = (i[0][0]+i[len(i)-1][0])/2
                avgy = (i[0][1] + i[len(i) - 1][1]) / 2
            else:
                avgx = (i[j][0] + i[j+1][0]) / 2
                avgy = (i[j][1] + i[j+1][1]) / 2
            allPolyPoints.append((avgx,avgy))
    for i in allPolyPoints:
        if(pointToPolygon(i,transpPos)):
            return False

    allRobotPoints = []

    for i in range(0, len(transpPos)):
        allRobotPoints.append(transpPos[i])
        if (i == len(transpPos) - 1):
            avgx = (transpPos[0][0] + transpPos[len(transpPos) - 1][0]) / 2
            avgy = (transpPos[0][1] + transpPos[len(transpPos) - 1][1]) / 2
        else:
            avgx = (transpPos[i][0] + transpPos[i+1][0]) / 2
            avgy = (transpPos[i][1] + transpPos[i+1][1]) / 2
        allRobotPoints.append((avgx, avgy))
    for i in obstacles:
         for j in allRobotPoints:
             if (pointToPolygon(j, i)):
                 return False
    tuplesorted = sorted(transpPos, key=lambda tup: tup[0])
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

def distance(point1, point2):
    x = (point2[0]-point1[0]) ** 2
    y = (point2[1]-point1[1]) ** 2
    return np.sqrt(x+y)

def helper(world_file, problem_file):
    temp = parse_problem(world_file, problem_file)
    robot = temp[0]
    obs = temp[1]
    probs = temp[2]
    #print(robot)
    #print(obs)
    #print(probs)
    #visualize_problem(robot, obs, [1,1],[8,8])
    points = [(2,8),(8,4)]
    endPoint = (4,3.25)
    print(isCollisionFree(robot, endPoint, obs))
    visualize_points(points, robot, obs, [5,5],endPoint)
    #print(lineToLine(line1,line2))

#helper("robot_env_01.txt","probs_01.txt")
