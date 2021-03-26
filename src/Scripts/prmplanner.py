# prmplanner.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the authors.
# 
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)

from graph import RoadmapVertex, RoadmapEdge, Roadmap
from utils import *
import random
import numpy as np
import csv

disk_robot = True #(change this to False for the advanced extension)


obstacles = None # the obstacles
robot_radius = None # the radius of the robot
robot_width = None # the width of the OBB robot (advanced extension)
robot_height = None # the height of the OBB robot (advanced extension)


# ----------------------------------------
# modify the code below
# ----------------------------------------

# Construction phase: Build the roadmap
# You should incrementally sample configurations according to a strategy and add them to the roadmap,
# select the neighbors of each sample according to a distance function and strategy, and
# attempt to connect the sample to its neighbors using a local planner, leading to corresponding edges
# See graph.py to get familiar with the Roadmap class

def build_roadmap(q_range, robot_dim, scene_obstacles):

    global obstacles, robot_width, robot_height, robot_radius

    obstacles = scene_obstacles # setting the global obstacle variable

    x_limit = q_range[0] # the range of x-positions for the robot
    y_limit = q_range[1] # the range of y-positions for the robot
    theta_limit = q_range[2] # the range of orientations for the robot (advanced extension)

    robot_width, robot_height = robot_dim[0], robot_dim[1] # the dimensions of the robot, represented as an oriented bounding box

    robot_radius = max(robot_width, robot_height)/2.

    # the roadmap
    graph = Roadmap()

    #### Sampling 1 ####
    # p = robot_radius
    p = 0.02

    x_pos = np.arange(x_limit[0]+3, x_limit[1]-3, p)
    y_pos = np.arange(y_limit[0]+3, y_limit[1]-3, p)

    samples = []

    vertex = [0, 0]
    samples.append(vertex)
    first = samples.pop()
    check = collision(first)
    if not check:
        graph.addVertex(first)
    k = 1
    l = 1
    while k <= 60:
        if l == 500:
            break
        vertex = [np.random.choice(x_pos), np.random.choice(y_pos)]
        samples.append(vertex)
        trial = samples.pop()
        check1 = collision(trial)
        if not check1:
            trial_put = graph.addVertex(trial)
            k += 1
            l += 1
            vertices = graph.getVertices()
            for point in vertices:
                if not point == trial_put:
                    d = distance(point, trial_put)
                    if d < 0.2:
                        graph.removeVertex(trial_put.id)
                        k -= 1

    vertices = graph.getVertices()
    visited = OrderedSet()
    stacked = OrderedSet()
    stacked.add(vertices[0])
    while len(stacked) != 0:
        point = stacked.pop(len(stacked) - 1)
        visited.add(point)
        current_position_x, current_position_y = point.q
        tot_neighbor = 0
        for v in vertices:
            neighbor_position_x, neighbor_position_y = v.q
            if point == v:
                continue
            neighbor_distance = distance(point, v)
            if neighbor_distance <= 0.8 and v not in stacked and v not in visited:
                stacked.add(v)
                graph.addEdge(point, v, neighbor_distance)
                tot_neighbor = tot_neighbor + 1
                for m in range(len(obstacles)):
                    step = 15
                    for x in range(1, step):
                        z = step - x
                        check_x = (x * neighbor_position_x + z * current_position_x) / (x + z)
                        check_y = (x * neighbor_position_y + z * current_position_y) / (x + z)
                        y = [check_x, check_y]
                        if collision(y) == True:
                            if v in stacked:
                                stacked.remove(v)

                            graph.removeEdge(point, v)
                            tot_neighbor = tot_neighbor - 1

            if tot_neighbor > 5:
                break

    # uncomment this to export the roadmap to a file
    # graph.saveRoadmap("prm_roadmap.txt")
    return graph

# ----------------------------------------
# modify the code below
# ----------------------------------------

# Query phase: Connect start and goal to roadmap and find a path using A*
# (see utils for Value, PriorityQueue, OrderedSet classes that you can use as in project 3)
# The returned path should be a list of configurations, including the local paths along roadmap edges
# Make sure that start and goal configurations are collision-free. Otherwise return None

def find_path(q_start, q_goal, graph):
    path = []

    ## Add start and goal to roadmap as vertex ##
    coll1 = collision(q_start)
    coll2 = collision(q_goal)
    if (not coll1) and (not coll2):
        graph.addVertex((q_start[0], q_start[1]))
        graph.addVertex((q_goal[0], q_goal[1]))
    else:
        tk.messagebox.showinfo("", "Queries are colliding with obstacle")
        return None

    ## Connect roadmap to nearest edge ##
    roadmapvertices = graph.getVertices()
    q_startV = roadmapvertices[-2]
    q_goalV = roadmapvertices[-1]

    for vertex in [q_startV, q_goalV]:
        neighbors, neighborsdist = nearest_neighbors(graph, vertex, max_dist=0.7)
        k = 0
        for k in range(len(neighbors)):
            steps = 10
            interpolate(vertex, neighbors[k], neighborsdist[k], graph, steps)
            if vertex.getEdges == None:
                continue
            else:
                break

    # Adding edge from start to nearest Roadmap vertex
    # dist_arr = []
    # for node in roadmapvertices:
    #     if node.id != q_startV.id:
    #         dist = distance(q_startV, node)
    #         dist_arr.append(dist)
    # min_dist_idx = np.argmin(dist_arr)
    # d1 = dist_arr[min_dist_idx]
    # closest_s = roadmapvertices[min_dist_idx]
    # graph.addEdge(q_startV, closest_s, d1)
    #
    # # Adding edge from end to nearest roadmap vertex
    # dist_arr = []
    # for node in roadmapvertices:
    #     if node.id != q_goalV.id:
    #         dist = distance(q_goalV, node)
    #         dist_arr.append(dist)
    # min_dist_idx = np.argmin(dist_arr)
    # d1 = dist_arr[min_dist_idx]
    # closest_s = roadmapvertices[min_dist_idx]
    # graph.addEdge(q_goalV, closest_s, d1)


    ###########################
    ## A star implementation ##
    # Use the OrderedSet for your closed list
    closed_set = OrderedSet()

    # Use the PriorityQueue for the open list
    open_set = PriorityQueue(order=min, f=lambda v: v.f)

    parent = [0] * len(roadmapvertices)

    g = 0
    h = distance(q_startV, q_goalV)
    f = g + h
    open_set.put(q_startV.id, Value(f=f, g=g))
    lastId = q_startV.id

    while open_set:
        nodeId, nodeVal = open_set.pop()
        closed_set.add(nodeId)

        if nodeId == q_goalV.id:
            lastId = nodeId
            break
        else:
            node = roadmapvertices[nodeId]
            edges = node.getEdges()
            for edge in edges:
                childId = edge.getId()
                child = roadmapvertices[childId]
                if childId not in closed_set:
                    childg = nodeVal.g + distance(node, child)
                    childh = distance(child, q_goalV)
                    childf = childg + childh
                    childVal = Value(f=childf, g=childg)
                    if (childId not in open_set) or (open_set.get(childId).g > childg):
                        parent[childId] = nodeId
                        open_set.put(childId, childVal)

    ## Path Finding ##
    roadmapvertices[lastId].q = q_goal
    if lastId == q_goalV.id:
        while lastId != q_startV.id:
            parId = parent[lastId]
            theta = abs(math.atan2((roadmapvertices[lastId].q[1]-roadmapvertices[parId].q[1]),(roadmapvertices[lastId].q[0]-roadmapvertices[parId].q[0])))
            roadmapvertices[parId].q = [roadmapvertices[parId].q[0],roadmapvertices[parId].q[1],theta+math.pi/2]
            path.append(roadmapvertices[lastId].q)

            ## For adding path between two configurations ##
            # x1, y1, t1 = roadmapvertices[lastId].q
            # x2, y2, t2 = roadmapvertices[parId].q
            # n = 3
            # for i in range(n):
            #     j = n - i
            #     x = (i * x2 + j * x1) / (i + j)
            #     y = (i * y2 + j * y1) / (i + j)
            #     graph.addVertex((x, y))
            #     path.append([x, y,(i*t1+j*t2)/n])
                # path.append([x,y])
            lastId = parId

    # path.append(q_start)

    ## To convert from goal-start to start-goal
    path.reverse()

    print(path)
    with open('path.csv', 'w') as file:
        writer = csv.writer(file)
        writer.writerow(path)

    return path


# ----------------------------------------
# below are some functions that you may want to populate/modify and use above
# ----------------------------------------

def nearest_neighbors(graph, q, max_dist):
    """
        Returns all the nearest roadmap vertices for a given configuration q that lie within max_dist units
        You may also want to return the corresponding distances
    """
    neighbors = []
    neighborsdist = []
    Vertices = graph.getVertices()
    for Vertex in Vertices:
        if Vertex.getId() != q.getId():
            if distance(Vertex, q) <= max_dist:
                neighbors.append(Vertex)
                neighborsdist.append(distance(Vertex, q))

    return neighbors, neighborsdist


def k_nearest_neighbors(graph, q, K=10):
    """
        Returns the K-nearest roadmap vertices for a given configuration q.
        You may also want to return the corresponding distances
    """

    return None

def distance (q1, q2):
    """
        Returns the distance between two configurations.
        You may want to look at the getRobotPlacement function in utils.py that returns the OBB for a given configuration
    """
    x1, y1 = q1.q
    x2, y2 = q2.q
    dist = np.sqrt((x2-x1)**2 + (y2-y1)**2)

    return dist


def collision(q):
    """
        Determines whether the robot placed at configuration q will collide with the list of AABB obstacles.
    """
    for obstacle in obstacles:
        if q[0] + robot_radius*1.2 > obstacle.x_min and q[0] - robot_radius*1.2 < obstacle.x_max and\
                q[1] + robot_radius*1.2 > obstacle.y_min and q[1] - robot_radius*1.2 < obstacle.y_max:
            return True

    return False


def interpolate(q1, q2, ndist, graph, steps):
    """
        Returns an interpolated local path between two given configurations.
        It can be used to determine whether an edge between vertices is collision-free.
    """
    x1, y1 = q1.q
    x2, y2 = q2.q
    n = steps

    for i in range(n):
        j = n - i
        x = (i*x2 + j * x1)/(i+j)
        y = (i * y2 + j * y1) / (i + j)
        coll = collision((x, y))
        if coll:
            break
    if not coll:
        graph.addEdge(q1, q2, ndist)

    return None


if __name__ == "__main__":
    from scene import Scene
    import tkinter as tk

    win = tk.Tk()
    Scene('prm_obstacles.csv', disk_robot, (build_roadmap, find_path), win)
    win.mainloop()
