#!/usr/bin/env python3
import math
import rclpy
import random
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from route_planner.robot_state import RoboParams

class graphNode:
    def __init__(self, pt):
        self.x = pt[0]
        self.y = pt[1]
        self.parent = None

class RRTGraph(Node):
    def __init__(self, step_size, max_iterations, map_meta, ocgrid):
        self.rs = RoboParams()

        self.startnode_ = graphNode((0,0))
        self.goal_node_ = graphNode((0,0))
        self.all_nodes_ = [self.startnode_]

        self.map_meta = map_meta
        self.ocgrid = ocgrid

        self.iter_max = max_iterations
        self.step_size = step_size
        self.search_radius = 5 * self.map_meta.resolution

        good_points = [x==0 for x in self.ocgrid]
        self.good_points = np.where(good_points)[0]

        self.route = []

    def path_constructor(self, start, end):
        self.startnode_.x = start[0]
        self.startnode_.y = start[1]
        self.goal_node_.x = end[0]
        self.goal_node_.y = end[1]
        print("start in RTT: " + str(self.startnode_.x) + " " + str(self.startnode_.y))
        print("end   in RTT: " + str(self.goal_node_.x) + " " + str(self.goal_node_.y))
        self.all_nodes_ = [self.startnode_]

        # if self.isLos(self.startnode_,self.goal_node_):
        #     dist, phi = self.get_dist_and_angle(self.startnode_,self.goal_node_)
        #     self.iter_max = int(2 * dist % self.step_size)

        for iterator in range(self.iter_max):
            node_attractor_ = self.get_new_node(0.1)
            node_near_ = self.get_nearest_neighbour(self.all_nodes_, node_attractor_)
            node_new_ = self.get_new_state(node_near_, node_attractor_)
            if self.isLos(node_near_, node_new_) and node_new_:
                self.all_nodes_.append(node_new_)
                dist, _ = self.get_dist_and_angle(node_new_, self.goal_node_)
                
                if dist <= self.step_size and self.isLos(node_new_, self.goal_node_):
                    self.get_new_state(node_new_,self.goal_node_)
                    print(len(self.all_nodes_))
                    return self.get_path_coordinates(node_new_)
        return None

    def get_new_node(self, bias_chance):
        if np.random.random() >= bias_chance:
            return self.get_random_node()
        else:
            return self.goal_node_

    def get_random_node(self):
        x, y = self.coords_from_ocgrid_idx(self.map_meta, random.choice(self.good_points))
        return graphNode((x,y))

    @staticmethod
    def get_nearest_neighbour(all_nodes_, n):
        return all_nodes_[int(np.argmin([math.hypot(n_test.x - n.x, n_test.y - n.y) 
                                        for n_test in all_nodes_]))]

    def get_new_state(self, node_a_, node_b_):
        dist, phi = self.get_dist_and_angle(node_a_, node_b_)
        node_new_ = graphNode((0,0))
        node_new_.x = node_a_.x + min(self.step_size, dist) * math.cos(phi)
        node_new_.y = node_a_.y + min(self.step_size, dist) * math.sin(phi)
        node_new_.parent = node_a_
        return node_new_

    def get_path_coordinates(self, node_end_):
        path = [(self.goal_node_.x, self.goal_node_.y)]
        node_now_ = node_end_

        while node_now_.parent is not None:
            node_now_ = node_now_.parent
            path.append((node_now_.x, node_now_.y))
        return path

    def isPointFree(self, x, y): # same as in ../terminal_pts_fetcher.py 
        res = self.map_meta.resolution
        x_pos = round(x/res, 1) - round(self.map_meta.origin.position.x/ res, 1)
        y_pos = round(y/res, 1) - round(self.map_meta.origin.position.y/ res, 1)
        n = int(y_pos) * self.map_meta.width  + int(x_pos)
        flag = np.empty((self.rs.footprint_px,self.rs.footprint_px),dtype=bool)
        for row in range(self.rs.footprint_px):
            for col in range(self.rs.footprint_px):
                flag[row][col] = (self.ocgrid[n + col + int(self.map_meta.width) * row] == 0)
        return flag.all() # check if all the footprint of the robot is in the free space
        # return (self.ocgrid[n] == 0) # check if all the footprint of the robot is in the free space

    def isLos(self,node1,node2): # check if the connection between two points crosses any obsticles
        discretizationPower = int(1/self.map_meta.resolution + 10) # how much points to check the line of sight
        for i in range (0, discretizationPower):
            u = i /(discretizationPower - 1)
            x = node1.x*u + node2.x*(1 - u)
            y = node1.y*u + node2.y*(1 - u)
            if not self.isPointFree(x ,y):
                return False
        return True

    @staticmethod
    def coords_from_ocgrid_idx(mapmeta, idx):
        # input is random.choice(good_points)
        x_id = idx % mapmeta.width # pixel index by axis from 1D occupancy grid 
        y_id = idx // mapmeta.width # pixel index by axis from 1D occupancy grid 

        x = x_id * mapmeta.resolution + mapmeta.origin.position.x # coordinate from pixel index
        y = y_id * mapmeta.resolution + mapmeta.origin.position.y # coordinate from pixel index
        
        x = round(round(x/mapmeta.resolution)*mapmeta.resolution, 2)
        y = round(round(y/mapmeta.resolution)*mapmeta.resolution, 2)
        return x, y
    
    @staticmethod
    def get_dist_and_angle(n1, n2):
        return math.hypot(n1.x - n2.x, n1.y - n2.y), math.atan2(n1.x - n2.x, n1.y - n2.y)

    @staticmethod
    def round_to_map_res(coord,resolution):
        # get clean coordinate value inline with map's resolution
        # inner round() to actually round the valuse
        # the second round is needed to get rid of float poind division imperfections
        return round(round(coord/resolution)*resolution,2)

    def num_of_nodes(self):
        return len(self.all_nodes_)

    def scan_neighbourhood(self, candidate_):# for rrt*
        # the bigger the amount of nodes, the easier to find a neighbour. from 2 radii at zero nodes to zero-radius at 10k
        r = self.search_radius *(1 + math.cos(0.0003 * self.num_of_nodes()))
        if r < self.step_size:
            r = self.step_size
        distances = [math.hypot(stored_.x - candidate_.x, stored_.y - candidate_.y) for stored_ in self.all_nodes_]
        dist_idx = [idx for idx in range(len(distances)) if distances[idx] <= r 
                            and self.isLos(candidate_, self.all_nodes_[idx])] # pick all satisfying nodes from 
        return dist_idx

    def pick_a_parent(self, dist_idx):# for rrt*
        pass

    def rewire(self):# for rrt*
        pass
