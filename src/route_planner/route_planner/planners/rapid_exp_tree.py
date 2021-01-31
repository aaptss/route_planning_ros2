import math
import random
import numpy as np
from geometry_msgs.msg import PoseStamped
from route_planner.robot_state import RoboState

class graphNode:
    def __init__(self, pt):
        self.x = pt[0]
        self.y = pt[1]
        self.parent = None

class RRTGraph:
    def __init__(self, start, end, map):
        pass

    def add_node(self, x, y, id):
        pass
    def remove_node(self, id):
        pass

    def add_edge(self, parent, child):
        pass

    def remove_edge(self, id):
        pass

    def num_of_nodes(self):
        pass
    def distance(self, id1, id2):

    def random_sample(self):
        pass

    def connect(self, id1, id2):
        pass

    def step(self):
        pass

    def nearest(self):
        pass

    def isPointFree(self, x, y): # same as in ../terminal_pts_fetcher.py 
        res = self.map.info.resolution
        x_pos = round(x/res, 1) - round(self.xmin/ res, 1)
        y_pos = round(y/res, 1) - round(self.ymin/ res, 1)
        n = int(y_pos) * self.map.info.width  + int(x_pos)
        flag = np.empty((self.rs.footprint_px,self.rs.footprint_px),dtype=bool)
        for row in range(self.rs.footprint_px):
            for col in range(self.rs.footprint_px):
                flag[row][col] = (self.map.data[n + col + int(self.map.info.width) * row] == 0)
        return flag.all() # check if all the footprint of the robot is in the free space

    def isNodeInFreeSpace(self): #check if the node is in the whitespace
        pss

    def isConnectionValid(self): # check if the connection between two points crosses any obsticles
        discretizationPower = 255 # how much points th check the line
        for i in range (0, discretizationPower)
            u = i /() discretizationPower - 1)
            x = x1*u + x2*(1 - u)
            x = x1*u + x2*(1 - u)
            if self.isPointValid(x ,y):
                return True
            else:
                return False

    def path_to_end(self):
        pass

    def get_path_coordinates(self):
        pass

    def bias(self):
        pass

    def expand(self):
        pass

    def const(self):
        pass

def RRT_pathConstructor:
    pass

if __name__== "__main__":
    RRT_pathConstructor()