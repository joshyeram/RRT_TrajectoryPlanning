# RRT, RRT*, and Trajectory Planning 

## RRT ##
For the implementation of a polygonal robot in polygonal environment, all of robot and obstacle points were represented in a list of tuple such as [(0,0),(1,1),(1,0)]. In parse problem(world file, problem file), the function will first open and read individual lines from a text file and then read pairs of numbers to form a list of tuples. After interpreting the file, it will return a (robot, obstacles, problems) tuple to be used later.

For collision detection between the robot and the obstacle environment, it will check 4 different conditions.
method: isCollisionFree(robot , point , obstacles):
1. Check if any of the robot vertices are outside the 10 x 10 boundary after translating the robot to point. 
2. Check if any edges are intersecting\
  2a. If the lines are both vertical or horizontal, continue\
  2b. If one of lines is vertical and the other is horizontal, check if each other are in their intervals\
  2c. If one of the line is vertical or horizontal and the other is a sloped line , check intersection\
  2d. Get both of the lines in slope intercept form and see if the intersection is between the intervals
3. Check if any of the vertices or midpoints along the robot edges and obstacle edges are inside one another other
4. Check if any of the obstacles have coordinates same as the robot

The extensive edge cases for 2 was to take in consideration of if the robots were touching but not colliding with one another. Thus, in this implementation, touching is not considered colliding. 3 was used to see if any of the robot or the obstacles were completely inside one another as the slope intercept does not take care of that edge case. 4 was utilized to check the conditions 2 and 3 could not. The run time for one collision check is therefore O(n) + O(nm) + O(nm) + O(nm) = O(nm) where n = number of vertices of the robot and m = number of vertices of the obstacle. It will return True if it is collision free and False other wise.

### k-d tree ###
In order to find the closest node given a coordinate, a k-d tree was utilized. A k-d tree is a binary tree in which at each depth, the value which is compared is the kth coordinate. For example, it will compare x in the first layer, y in the seond layer, and z in the third layer. On the fourth layer, it will repear with comparing the x coordinate.

### RRT implementation ###
method: rrt (robot , obstacles , start , goal , iter n ):
1. Create Tree
2. For iterations under iter n : Sample a random point\
  If the iteration is divisible by 10, set sample to the goal point\
  Find the nearest node from sample Extend from nearest to the sample\
    If the extended sample is less than .25 Euclidean distance away from the goal\
      Try to extend to goal\
    If it is the goal , return path Else , continue
3. Find the closet node that has a collision free path to the goal node called Last Resort\
4. Extend Last Resort to goal node
5. return path

This implements an effective RRT and an exploration bias. At every 10 iteration, it will try to look for a straight shot to the end goal. If there is collision free path from any of the nodes, it will take that route and return a path.
![alt text](https://github.com/joshyeram/pathfinding/blob/main/jyc70/rrtmap.png?raw=true)
![alt text](https://github.com/joshyeram/pathfinding/blob/main/jyc70/animatedpaths.png?raw=true)

