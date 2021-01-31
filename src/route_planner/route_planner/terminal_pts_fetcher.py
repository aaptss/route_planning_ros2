#!/usr/bin/env python3
import rclpy
import random
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from route_planner.robot_state import RoboParams

class PintFetcherNode(Node):
    def __init__(self):
        # the NPC gives the quest so you know the start and the end points of the journey
        super().__init__("npc_with_quest")

        self.map = OccupancyGrid()
        self.x_start = 0.0
        self.y_start = 0.0
        self.x_end = 0.0
        self.y_end = 0.0
        self.noInputFlag = False
        self.isMapFlag = False
        self.isGoodPtArrayRdy = False
        self.rs = RoboParams()
        self.timer_ = self.create_timer(3, self.publish_both)
        self.start_publisher_ = self.create_publisher(
            PoseStamped,
            "start",
            1)
        self.end_publisher_ = self.create_publisher(
            PoseStamped,
            "end",
            1)
        self.map_subscriber_ = self.create_subscription(
            OccupancyGrid,
            "map",
            self.new_map_cb,
            1)

    def new_map_cb(self, msg):
        self.isMapFlag = True
        self.map = msg
        if not self.isGoodPtArrayRdy:
            self.good_pts = [x==0 for x in self.map.data] # data(data)
            self.good_pts = np.where(self.good_pts)[0]
            self.isGoodPtArrayRdy = True

    def fetch_points(self):
        self.get_logger().info("If you want to turn off keybard input and take random pts, type in 1024")
        self.buf = [self.x_start, self.y_start, self.x_end, self.y_end]
        self.x_start = float(input("Please enter the start X coordinate: "))
        self.y_start = float(input("Please enter the start Y coordinate: "))
        self.get_logger().info("Your start input is (X, Y) = (" +
            str(self.x_start)+", " + str(self.y_start) + ")\r\n")

        self.x_end = float(input("Please enter the end X coordinate: "))
        self.y_end = float(input("Please enter the end Y coordinate: "))

        self.get_logger().info("Your end input is (X, Y) = (" +
            str(self.x_end)+", " + str(self.y_end) + ")\r\n")
# 
        # x_start == 1024 means keyboard input turned off, the points gonna go random.
        if self.x_start == 1024.0:
            self.noInputFlag = True
        else:
            self.x_start = self.round_to_map_res(self.x_start, self.map.info.resolution)
            self.y_start = self.round_to_map_res(self.y_start, self.map.info.resolution)
            self.x_end = self.round_to_map_res(self.x_end, self.map.info.resolution)
            self.y_end = self.round_to_map_res(self.y_end, self.map.info.resolution)

    def get_random_points(self):
        # while True:
        #     ranstart = random.choice(list(enumerate(self.map.data)))
        #     ranend = random.choice(list(enumerate(self.map.data)))
        #     if ((ranstart[1] == 0) and(ranend[1] == 0)):
        #         break
        ranstart = random.choice(self.good_pts) # choose an index of a random free point 
        ranend = random.choice(self.good_pts)

        # start_x_id = ranstart[0] % self.map.info.width # use with while-do loop 
        # start_y_id = ranstart[0] // self.map.info.width # use with while-do loop 
        # end_x_id =  ranend[0] % self.map.info.width # use with while-do loop 
        # end_y_id = ranend[0] // self.map.info.width # use with while-do loop 
        start_x_id = ranstart % self.map.info.width # pixel index by axis from 1D occupancy grid 
        start_y_id = ranstart // self.map.info.width # pixel index by axis from 1D occupancy grid 
        end_x_id =  ranend % self.map.info.width # pixel index by axis from 1D occupancy grid 
        end_y_id = ranend // self.map.info.width # pixel index by axis from 1D occupancy grid 

        self.x_start = start_x_id * self.map.info.resolution + self.map.info.origin.position.x # coordinate from pixel index
        self.y_start = start_y_id * self.map.info.resolution + self.map.info.origin.position.y # coordinate from pixel index
        self.x_end = end_x_id * self.map.info.resolution + self.map.info.origin.position.x # coordinate from pixel index
        self.y_end = end_y_id * self.map.info.resolution + self.map.info.origin.position.y # coordinate from pixel index
        
        self.x_start = self.round_to_map_res(self.x_start,self.map.info.resolution)
        self.y_start = self.round_to_map_res(self.y_start,self.map.info.resolution)
        self.x_end = self.round_to_map_res(self.x_end,self.map.info.resolution)
        self.y_end = self.round_to_map_res(self.y_end,self.map.info.resolution)

    def isPointInBoundaries(self, x, y):
        self.xmin = self.map.info.origin.position.x
        self.ymin = self.map.info.origin.position.y
        self.xmax = round(self.xmin + self.map.info.width * self.map.info.resolution, 3) 
        self.ymax = round(self.ymin + self.map.info.height * self.map.info.resolution, 3)
        
        return ((x < (self.xmax - self.rs.diam)) 
        and (x > self.xmin) 
        and (y < (self.ymax - self.rs.diam)) 
        and (y > self.ymin))

    def isPointFree(self, x, y): # same as in planners/rapid_exp_tree.py
        res = self.map.info.resolution
        x_pos = round(x/res, 1) - round(self.xmin/ res, 1)
        y_pos = round(y/res, 1) - round(self.ymin/ res, 1)
        n = int(y_pos) * self.map.info.width  + int(x_pos)
        flag = np.empty((self.rs.footprint_px,self.rs.footprint_px),dtype=bool)
        for row in range(self.rs.footprint_px):
            for col in range(self.rs.footprint_px):
                flag[row][col] = (self.map.data[n + col + int(self.map.info.width) * row] == 0)
        return flag.all() # check if all the footprint of the robot is in the free space

    def arePointsOk(self):
        # TODO: make an util.py with this and use this method across the workspace to avoid copypase
        return (self.isPointInBoundaries(self.x_start, self.y_start) and
            self.isPointInBoundaries(self.x_end, self.y_end) and
            self.isPointFree(self.x_start, self.y_start) and
            self.isPointFree(self.x_end, self.y_end))

    def publish_start(self):
        start_pt = PoseStamped()
        if self.x_start == 1024:
            start_pt.pose.position.x = self.buf[0]
            start_pt.pose.position.y = self.buf[1]
        else:
            start_pt.pose.position.x = self.x_start
            start_pt.pose.position.y = self.y_start
        start_pt.pose.position.z = 0.1
        start_pt.header.frame_id = self.map.header.frame_id
        start_pt.header.stamp = self.t
        self.start_publisher_.publish(start_pt)
        self.get_logger().info("start_pt(" + str(self.x_start)+", " + str(self.y_start) + ") published")

    def publish_end(self):
        end_pt = PoseStamped()
        if self.x_start == 1024:
            end_pt.pose.position.x = self.buf[2]
            end_pt.pose.position.y = self.buf[3]
        else:
            end_pt.pose.position.x = self.x_end
            end_pt.pose.position.y = self.y_end
        end_pt.pose.position.z = 0.1
        end_pt.header.frame_id = self.map.header.frame_id
        end_pt.header.stamp = self.t
        self.end_publisher_.publish(end_pt)
        self.get_logger().info("end_pt(" + str(self.x_end)+", " + str(self.y_end) + ") published")

    def publish_both(self):
        if self.isMapFlag:
            if not self.noInputFlag: # see fetch_points()
                while True:
                    self.fetch_points()
                    if self.noInputFlag:
                        self.get_logger().info("go for random")
                        self.get_random_points()
                        break
                    if self.arePointsOk():
                        break
            else:
                self.get_logger().info("go for random")
                self.noInputFlag = True
                self.get_random_points()

            if self.arePointsOk():
                self.get_logger().info("points are ok!")
                self.get_logger().info("prepping to publish!")
                self.t = self.get_clock().now().to_msg()
                self.publish_start()
                self.publish_end()
            else:
                self.get_logger().info("points not okay, need new pair!")

    @staticmethod
    def round_to_map_res(a,b):
        # get clean coordinate value inline with map's resolution
        # inner round() to actually round the valuse
        # the second round is needed to get rid of float poind division imperfections
        return round(round(a/b)*b,2)


def main(args=None):
    rclpy.init(args=args)

    node = PintFetcherNode()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
