# RRT, RRT*, and Trajectory Planning 

## RRT and RRT* ##
For the implementation of a polygonal robot in a polygonal environment, all robot and obstacle points were represented in a list of tuples such as [(0,0),(1,1),(1,0)]. In parse problem(world file, problem file), the function will first open and read individual lines from a text file and then read pairs of numbers to form a list of tuples. After interpreting the file, it will return a (robot, obstacles, problems) tuple to be used later.

For collision detection between the robot and the obstacle environment, it will check 4 different conditions.
method: isCollisionFree(robot , point , obstacles):
1. Check if any of the robot vertices are outside the 10 x 10 boundary after translating the robot to a point. 
2. Check if any edges are intersecting\
  2a. If the lines are both vertical or horizontal, continue\
  2b. If one of the lines is vertical and the other is horizontal, check if each other are in their intervals\
  2c. If one of the lines is vertical or horizontal and the other is a sloped line, check intersection\
  2d. Get both of the lines in slope-intercept form and see if the intersection is between the intervals
3. Check if any of the vertices or midpoints along the robot edges and obstacle edges are inside one another other
4. Check if any of the obstacles have coordinates the same as the robot

The extensive edge cases for 2 were to take into consideration if the robots were touching but not colliding with one another. Thus, in this implementation, touching is not considered colliding. 3 was used to see if any of the robots or the obstacles were completely inside one another as the slope-intercept does not take care of that edge case. 4 was utilized to check the conditions 2 and 3 could not. The run time for one collision check is therefore O(n) + O(nm) + O(nm) + O(nm) = O(nm) where n = number of vertices of the robot and m = number of vertices of the obstacle. It will return True if it is collision-free and False otherwise.

### k-d tree ###
A k-d tree was utilized to find the closest node given a coordinate. A k-d tree is a binary tree in which at each depth, the value that is compared is the kth coordinate. For example, it will compare x in the first layer, y in the second layer, and z in the third layer. On the fourth layer, it will repeat by comparing the x coordinate.

### RRT implementation ###
method: rrt (robot, obstacles, start, goal, iter n ):
1. Create a Tree
2. For iterations under iter n: Sample a random point\
  If the iteration is divisible by 10, set the sample to the goal point\
  Find the nearest node from the sample Extend from the nearest to the sample\
    If the extended sample is less than .25 Euclidean distance away from the goal\
      Try to extend to goal\
    If it is the goal, return path Else, continue
3. Find the closet node that has a collision-free path to the goal node called Last Resort\
4. Extend Last Resort to the goal node
5. return path

This implements an effective RRT and an exploration bias. At every 10 iterations, it will try to look for a straight shot to the end goal. If there is a collision-free path from any of the nodes, it will take that route and return a path.

<p align="center">
  <img src="https://github.com/joshyeram/pathfinding/blob/main/jyc70/rrtmap.png", width="295"/>
  <img src="https://github.com/joshyeram/pathfinding/blob/main/jyc70/animatedpaths.png", width="600"/>
</p>

To visualize the robot as a singular point, we can conduct a Minkowski Sum and get the configuration space.

<p align="center">
  <img src="https://github.com/joshyeram/pathfinding/blob/main/jyc70/configspace.png", width="300"/>
  <img src="https://github.com/joshyeram/pathfinding/blob/main/jyc70/rrtConfig.png", width="290"/>
</p>

To optimize rrt, we can continuously sample for a set iteration instead, of changing the closest neighbor for log n neighbors as each one is sampled.
<p align="center">
  <img src="https://github.com/joshyeram/pathfinding/blob/main/jyc70/star1.png", width="300"/>
  <img src="https://github.com/joshyeram/pathfinding/blob/main/jyc70/star2.png", width="300"/>
</p>
As the number of iterations grows, the path becomes more optimized as it finds a better path.

For a robot that can turn and is not holonomic, we can also sample a rotation aspect and utilize the same implementation of rrt
<p align="center">
  <img src="https://github.com/joshyeram/pathfinding/blob/main/jyc70/trajectoryrrt.png", width="600"/>
</p>

## Trajectory Sampling ##
Instead of sampling random coordinates the robot goes to, we can sample random trajectories the robot can follow. This gives a natural-looking path compared to a random, sporadic path you may get from rrt.
<p align="center">
  <img src="https://github.com/joshyeram/pathfinding/blob/main/jyc70/trajectory.png", width="600"/>
</p>

For kinematics(), it will simply return the velocity in the direction of the robot given the state and the control of the robot. In propagate(), the robot will utilize the controls list and apply it from the latest state position, starting from the original state. First, it will gather its velocity control using kinematics(). Second, it will apply dt to the previous state's control to simulate a discretized integral. Third, this will continue for the entire control array. Finally, it will return the trajectory of the robot after those controls have been applied from the initial state.
For the random shooting algorithm, this implementation of extending was used to utilize the propagated method created earlier. First, it will sample d between n1 and n2 (in rrt was set to 15 and 20 respectively). The dt set is between .1 and .02. For the control, a velocity between -.05 and 2 was chosen to promote forward movement while the angular velocity was between -.4 and .4 pi. The reason for this wide angle was to encourage a wider exploration space.
