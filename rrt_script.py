# from PIL import Image, ImageOps
import numpy as np
import matplotlib.pyplot as plt
import random

# img = Image.open('sample_space.png')
# img = ImageOps.grayscale(img)
# img = np.array(img)
# # Coverting black into white and vice versa.
# img = ~img

# # set_cmap was necessary unless it was taking colors other then black and white.
# plt.set_cmap('binary')
# np.save('sample_space.npy',img)

map_area = np.load('sample_space.npy')
class tree_nodes():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.child = []
        self.parent = None

class RRT_tree():

    def __init__(self, start, goal, no_iterations, grid, max_dist):
        self.tree = tree_nodes(start[0], start[1])
        self.goal = tree_nodes(goal[0], goal[1])
        self.no_iterations = no_iterations
        self.step = max_dist
        self.grid = grid
        self.nearest_node = None
        self.nearest_node_distance = grid.shape[0]*grid.shape[1]
        self.way_points = []
        self.no_way_points = 0
        self.track_dist = 0


    def new_point(self):
        x = random.randint(1, self.grid.shape[1])
        y = random.randint(1, self.grid.shape[0])
        point = np.array([x,y])
        return point

    
    def nearest_point_distance(self, point, random_point):
        
        if point == None:
            return None
        
        dist = self.distance(point, random_point)

        if dist <= self.nearest_node_distance:
            self.nearest_node = point
            self.nearest_node_distance = dist

        for i in point.child:
            self.nearest_point_distance(i,random_point)
    
    def distance(self, near_node, random_point):
        dist = np.sqrt((near_node.x-random_point[0])**2 + (near_node.y-random_point[1])**2)
        return dist
    
    def direction(self, nearest_point, random_point):
        vec = np.array([random_point[0]-nearest_point.x, random_point[1]-nearest_point.y])
        mag = np.sqrt(vec[0]**2 + vec[1]**2)
        return vec/mag

    def leading_point(self, nearest_point, random_point):
        temp = self.step*self.direction(nearest_point, random_point)
        test_point = np.array([nearest_point.x + temp[0], nearest_point.y + temp[1]])

        if test_point[0]>=self.grid.shape[1]:
            test_point[0] = self.grid.shape[1]-10
        
        if test_point[1]>=self.grid.shape[0]:
            test_point[1] = self.grid.shape[0]-10

        return test_point
    
    def obstacle(self, nearest_node, random_point):
        temp = self.direction(nearest_node, random_point)
        test_point = np.array([0.0,0.0])

        for i in range(1,self.step+1):
            test_point[0] = nearest_node.x + i*temp[0]
            test_point[1] = nearest_node.y + i*temp[1]

            if self.grid[int(test_point[1]), int(test_point[0])] == 0:
                return True
        
        return False

    
    def add_point(self, x, y):
        if(x == self.goal.x and y == self.goal.y):
            self.nearest_node.child.append(self.goal)
            self.goal.parent = self.nearest_node
        
        else:
            temp = tree_nodes(x,y)
            self.nearest_node.child.append(temp)
            temp.parent = self.nearest_node

    def nearest_value_reset(self):
        self.nearest_node = None
        self.nearest_node_distance = self.grid.shape[0]*self.grid.shape[1]
    
    def reached_goal(self, point):
        if self.distance(self.goal, point) <= self.step:
            return True
    
    def trace_the_path(self,goal):

        if (goal.x == self.tree.x):
            return
        
        self.no_way_points += 1

        currentPoint = np.array([goal.x, goal.y])
        self.way_points.insert(0, currentPoint)
        self.track_dist += self.step
        self.trace_the_path(goal.parent)


print("Range for x co-ordinate: 0 to " + str(map_area.shape[1]))
print("Range for y co-ordinate: 0 to " + str(map_area.shape[0]))
start_x = int(input("Enter the x co-ordinate for start point: "))
start_y = int(input("Enter the y co-ordinate for start point: "))
start = np.array([start_x, start_y])

end_x = int(input("Enter the x co-ordinate for end point: "))
end_y = int(input("Enter the y co-ordinate for end point: "))
end = np.array([end_x, end_y])

no_iterations = int(input("Enter the number of iterations: "))

max_dist = int(input("Enter the max distance: "))

plt.imshow(map_area, cmap='binary')
plt.plot(start[0], start[1], 'ro')
plt.plot(end[0], end[1], 'bo')
plt.xlabel('X-axis (m)')
plt.ylabel('Y-axis (m)')

rrt = RRT_tree(start, end, no_iterations, map_area, max_dist)

for i in range(no_iterations):
    rrt.nearest_value_reset()
    point = rrt.new_point()
    print("Iterations: ", i)
    rrt.nearest_point_distance(rrt.tree, point)
    new = rrt.leading_point(rrt.nearest_node, point)
    obstacle_bool = rrt.obstacle(rrt.nearest_node, new)

    if(obstacle_bool == False):
        rrt.add_point(new[0], new[1])
        plt.pause(0.10)
        plt.plot([rrt.nearest_node.x, new[0]], [rrt.nearest_node.y, new[1]], 'go', linestyle='--')
    
    if (rrt.reached_goal(new)):
            rrt.add_point(end[0], end[1])
            print("Goal Reached")
            break

rrt.trace_the_path(rrt.goal)
rrt.way_points.insert(0, start)
print("Number of waypoints: ", rrt.no_way_points)
print("Path Distance (in m): ", rrt.track_dist)
print("Waypoints : ")

for i in rrt.way_points:
    print(i, end='\n')

for i in range(len(rrt.way_points)-1):
    plt.plot([rrt.way_points[i][0], rrt.way_points[i+1][0]], [rrt.way_points[i][1], rrt.way_points[i+1][1]], 'ro', linestyle='--')
    plt.pause(0.10)