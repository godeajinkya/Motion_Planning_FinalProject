# scene.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the authors.
#
# Authors: Ioannis Karamouzas (ioannis@g.clemson.edu)
#

import time, random
import sys
import numpy as np
from obstacles import BoxObstacle
from utils import *
from graph import *
import copy

class Scene():
    random.seed('cpsc8810')

    def __init__(self, filename, disk_robot, build_fn, master=None, resolution = 700):
        # super().__init__(master)
        #self.master = tk.Tk()
        self.filename = filename
        self.resolution = resolution
        self.disk_robot = disk_robot

        # parameters related to the problem
        self.scene_width, self.scene_height = None, None
        self.scene_xmin, self.scene_xmax, self.scene_ymin, self.scene_ymax = None, None, None, None
        self.default_start, self.default_goal = None, None
        self.start, self.goal = None, None
        self.obstacles = None
        self.robot_width, self.robot_height = None, None
        self.roadmap = None
        if not self.loadProblem(): sys.exit("Failed to load problem file")

    def loadProblem(self):
        """
            Read a scenario from a file
        """
        try:
            fp = open(self.filename, 'r')
            lines = fp.readlines()
            fp.close()

            # first line reads the dimension
            # second line reads the dimension of the robot
            scene_parameters = lines[0].split(',')
            robot_parameters = lines[1].split(',')
            query_parameters = lines[2].split(',')
            self.scene_xmin, self.scene_xmax, self.scene_ymin, self.scene_ymax = int(scene_parameters[0]), int(scene_parameters[1]), int(scene_parameters[2]), int(scene_parameters[3])
            self.scene_width = self.scene_xmax -self.scene_xmin
            self.scene_height = self.scene_ymax -self.scene_ymin
            self.robot_width = float(robot_parameters[0])
            self.robot_height = float(robot_parameters[1])
            self.default_start = (float(query_parameters[0]), -float(query_parameters[1]), float(query_parameters[2]))
            self.default_goal = (float(query_parameters[3]), -float(query_parameters[4]), float(query_parameters[5]))


            self.obstacles = []
            for line in lines[3:]:
                parameters = line.split(',')
                vertices = []
                vertices.append((float(parameters[0]), -float(parameters[1]))) #y-axis is flipped
                vertices.append((float(parameters[2]), -float(parameters[3])))
                vertices.append((float(parameters[4]), -float(parameters[5])))
                vertices.append((float(parameters[6]), -float(parameters[7])))
                self.obstacles.append(BoxObstacle(vertices))
        except:
            return False

        return True


    def getObstacles(self):
        return self.obstacles

    def getRobot(self):
        return self.robot_width, self.robot_height

    def default_query(self):
        self.start = self.default_start
        self.goal = self.default_goal


    def random_query(self):
        self.start = (random.uniform(self.scene_xmin, self.scene_xmax), random.uniform(self.scene_ymin, self.scene_ymax), random.uniform(0, np.pi))
        self.goal = (random.uniform(self.scene_xmin, self.scene_xmax), random.uniform(self.scene_ymin, self.scene_ymax), random.uniform(0, np.pi))
