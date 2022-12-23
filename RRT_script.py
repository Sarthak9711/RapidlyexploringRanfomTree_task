# Libraries used for visualisation and converting "png" into "npy" for making a 2d grid that is used as input.
from PIL import Image, ImageOps
import matplotlib.pyplot as plt

# Basic libraries used for generating random for way points and numpy as basic calculations and manipulations.
import numpy as np
import random

# Opening and grayscaling the image.
img = Image.open('sample_space.png')
img = ImageOps.grayscale(img)

# Converting the grayscale image into a form array.
np_img = np.array(img)

# Converting image into black and white.
np_img = ~np_img
plt.set_cmap('binary')
plt.imshow(np_img)

# Saving the image into ".npy" format
np.save('sample_space.npy', np_img)

# grid = np.load('sample_space.npy')
# plt.imshow(grid)
# plt.show()


# Initiating the cartesian-coordinates.
class nodes():
    def __init__(self,x,y):
        self.x = x
        self.y = y
        self.child = []
        self.parent_node = None 

# Rapidly exploring Random Tree Class
class call_rrt():

    # Initialising constructor.
    def __init__(self, start, goal, no_Iterations, grid, max_dist):

        # To give the start nodes and goal nodes.
        self.random_tree = nodes(start[0],start[1])
        self.goal = nodes(goal[0], goal[1])

        # Maximum number of steps provided to the algorithm to reach the goal position. 
        self.iterations = min(no_Iterations,200)

        # Initialising nearest_node as None.
        self.near_node = None
        self.grid = grid

        # Size of an edge.
        self.step = max_dist

        self.path_dist = 0

        # Giving a maximum possible value to the nearest_distance since we need a minimum value for way_point.
        self.nearest_dist = 10000000

        # Initial number of way points.
        self.no_way_points = 0

        # Appending each way_point into the array to create a shortest path.
        self.way_points = []


    # add the new node in way_points and add the goal point when reached.
    def add_point(self,x,y):
        if(x == self.goal.x):
            # To check if whether the random selected node is goal node.
            self.near_node.child.append(self.goal)
            self.goal.parent_node = self.near_node
        else:
            temp = nodes(x,y)
            # Add the temp node to the child of the nearest node.
            self.near_node.child.append(temp)
            temp.parent_node = self.near_node


    # To choose a point within the grid limits.
    def new_node(self):
        x = random.randint(1,grid.shape[1])
        y = random.randint(1,grid.shape[0])
        point = np.array([x,y])
        return point

    # To find a point in max_dist from start to end location.
    def leading_point(self, start_loc, end_loc):
        lead = self.step*self.direction(start_loc,end_loc)
        point = np.array([start_loc.x+lead[0], start_loc.y + lead[1]])

        # Checiking the condition if the point is outside the range of the grid
        if point[1] >= grid.shape[0]:
            point[1] = grid.shape[0]-1
        if point[0] >= grid.shape[1]:
            point[0] = grid.shape[1]-1
        
        return point

    # To check if point lies within obstacle range 
    def obstacle(self, start_loc, end_loc):
        temp1 = self.direction(start_loc, end_loc)
        test_point = np.array([0.0, 0.0])

        for i in range(self.step):
            test_point[0] = start_loc.x + i*temp1[0]
            test_point[1] = start_loc.y + i*temp1[1]

            # Check if the test point lies within obstacle. If the value at the grid is "1" then it is of black color, hence obstacle.
            if self.grid[int(round(test_point[1]).astype(np.int64)),int(round(test_point[0]).astype(np.int64))] == 1:
                return True
        
        return False
    
    # Find the nearest point from the given discrete point using eucledian distance
    def find_near_node(self, root_node, point):
        # return condition if root is NULL
        if not root_node:
            return

        # find distance between root and point
        dist = self.distance(root_node, point)

        # if distance lower than the nearest Distance, set this as the nearest node and update the nearest distance node and nearest distance value.
        if dist <= self.nearest_dist:
            self.near_node = root_node
            self.nearest_dist = dist

        # recursively called by iterating through the child 
        for child in root_node.child:
            self.find_near_node(child, point)
        
        pass

    # Find the angle for the new node within given distance
    def direction(self, start_loc, end_loc):
        vec = np.array([end_loc[0] - start_loc.x, end_loc[1] - start_loc.y])
        mag = np.sqrt(sum(pow(ele, 2) for ele in vec))
        temp1 = vec/mag
        return temp1

    # To find eucledian distance between 2 points
    def distance(self, node1, point):
        dist = np.sqrt((node1.x - point[0])**2 + (node1.y - point[1])**2)
        return dist

    # Goal reached or not within max_dist
    def reached_goal(self, point):
        if self.distance(self.goal, point) <= self.step:
            return True
        
        pass


    # Since after choosing after random point we rest the value of near_node and nearest_dist because we need to compute it for selecting next node.
    def nearest_values_reset(self):
        self.near_node = None
        self.nearest_dist = 10000

    def trace_the_path(self,goal):
        # end the recursion when goal node reaches the start node   
        if goal.x == self.random_tree.x:
            return
        
        # add 1 to number of waypoints
        self.no_way_points += 1

        # insert current point to the way points array from the start
        currentPoint = np.array([goal.x, goal.y])
        self.way_points.insert(0, currentPoint)
        self.path_dist += self.step
        self.trace_the_path(goal.parent_node)

        
grid = np.load('sample_space.npy')
start = np.array([100.0, 100.0])
goal = np.array([700.0, 250.0])
no_Iterations = 200
max_dist = 50
# goal_region = plt.circle((goal[0], goal[1]), max_dist, color='b', fill = False)

plt.imshow(grid, cmap='binary')
plt.plot(start[0], start[1], 'ro')
plt.plot(goal[0], goal[1], 'bo')
plt.xlabel('X-axis $(m)$')
plt.ylabel('Y-axis $(m)$')

rrt = call_rrt(start, goal, no_Iterations, grid, max_dist)

for i in range(rrt.iterations):
    rrt.nearest_values_reset()
    print("Iterations: ",i)
    point = rrt.new_node()
    rrt.find_near_node(rrt.random_tree, point)
    new = rrt.leading_point(rrt.near_node, point)
    bool = rrt.obstacle(rrt.near_node, new)

    if(bool == False):
        rrt.add_point(new[0], new[1])
        plt.pause(0.10)
        plt.plot([rrt.near_node.x, new[0]], [rrt.near_node.y, new[1]], 'go', linestyle='--')

        # if goal found append to path

        if (rrt.reached_goal(new)):
            rrt.add_point(goal[0], goal[1])
            print("Goal Reached")
            break

rrt.trace_the_path(rrt.goal)
rrt.way_points.insert(0, start)
print("Number of waypoints: ", rrt.no_way_points)
print("Path Distance (in m): ", rrt.path_dist)
print("Waypoints: ", rrt.way_points)

for i in range(len(rrt.way_points)-1):
    plt.plot([rrt.way_points[i][0], rrt.way_points[i+1][0]], [rrt.way_points[i][1], rrt.way_points[i+1][1]], 'ro', linestyle='--')
    plt.pause(0.10)

