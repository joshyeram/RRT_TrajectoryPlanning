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
