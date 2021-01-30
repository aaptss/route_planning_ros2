#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import random


class PintFetcherNode(Node):
    def __init__(self):
        # the NPC gives the quest so you know the start and the end points of the journey
        super().__init__("npc_with_quest")

        self.map = OccupancyGrid()
        self.x_start = 0.0
        self.y_start = 0.0
        self.x_end = 0.0
        self.y_end = 0.0
        self.isMap = False
        self.noInputFlag = False

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
        # self.get_logger().info(msg.header.frame_id + " map frame received")
        # self.get_logger().info("    " + self.map.header.frame_id + " was previous")

        if self.map.header.frame_id != msg.header.frame_id:
            self.isMap = True
        self.map = msg

        if (self.noInputFlag == False): # see fetch_points()
        # x_start == 1024 means keyboard input turned off, the points gonna go random.
            while True:
                self.fetch_points()
                if self.x_start == 1024.0:
                    break
                if self.arePointsOk():
                    break
        else:
            self.noInputFlag = True
            self.get_random_points()

        if self.isMap and self.arePointsOk():
            self.isMap = False

        if self.arePointsOk():
            self.publish_start()
            self.get_logger().info("start_pt(" + str(self.x_start)+", " + str(self.y_start) + ") published")
            self.publish_end()
            self.get_logger().info("end_pt(" + str(self.x_end)+", " + str(self.y_end) + ") published")

    def publish_start(self):
        start_pt = PoseStamped()
        if self.x_start == 1024:
            start_pt.pose.position.x = self.buf[0]
            start_pt.pose.position.y = self.buf[1]
        else:
            start_pt.pose.position.x = self.x_start
            start_pt.pose.position.y = self.y_start
        start_pt.pose.position.z = 0.1
        self.start_publisher_.publish(start_pt)

    def publish_end(self):
        end_pt = PoseStamped()
        if self.x_start == 1024:
            end_pt.pose.position.x = self.buf[2]
            end_pt.pose.position.y = self.buf[3]
        else:
            end_pt.pose.position.x = self.x_end
            end_pt.pose.position.y = self.y_end
        end_pt.pose.position.z = 0.1

        self.end_publisher_.publish(end_pt)

    def fetch_points(self):
        self.get_logger().info("If you want to turn off keybard input and take random pts, type in 1024")
        self.buf = [self.x_start, self.y_start, self.x_end, self.y_end]
        self.x_start = float(input("Please enter the starting X coordinate: "))
        self.y_start = float(input("Please enter the starting Y coordinate: "))
        self.get_logger().info("Your input is (X, Y) = (" +
            str(self.x_start)+", " + str(self.y_start) + ")\r\n")

        self.x_end = float(input("Please enter the terminal X coordinate: "))
        self.y_end = float(input("Please enter the terminal Y coordinate: "))

        self.get_logger().info("Your input is (X, Y) = (" +
            str(self.x_end)+", " + str(self.y_end) + ")\r\n")

        if self.x_start == 1024.0:
            self.noInputFlag = True

        self.x_start = self.my_rounding(self.x_start, self.map.info.resolution)
        self.y_start = self.my_rounding(self.y_start, self.map.info.resolution)
        self.x_end = self.my_rounding(self.x_end, self.map.info.resolution)
        self.y_end = self.my_rounding(self.y_end, self.map.info.resolution)

    def get_random_points(self):
        while True:
            ranstart = random.choice(list(enumerate(self.map.data)))
            ranend = random.choice(list(enumerate(self.map.data)))
            if ((ranstart[1] == 0) and(ranend[1] == 0)):
                break

        start_x_id = ranstart[0] % self.map.info.width
        start_y_id = ranstart[0] // self.map.info.width
        end_x_id =  ranend[0] % self.map.info.width
        end_y_id = ranend[0] // self.map.info.width

        self.x_start = start_x_id * self.map.info.resolution + self.map.info.origin.position.x
        self.y_start = start_y_id * self.map.info.resolution + self.map.info.origin.position.y
        self.x_end = end_x_id * self.map.info.resolution + self.map.info.origin.position.x
        self.y_end = end_y_id * self.map.info.resolution + self.map.info.origin.position.y
        
        self.x_start = self.my_rounding(self.x_start,self.map.info.resolution)
        self.y_start = self.my_rounding(self.y_start,self.map.info.resolution)
        self.x_end = self.my_rounding(self.x_end,self.map.info.resolution)
        self.y_end = self.my_rounding(self.y_end,self.map.info.resolution)

    def arePointsOk(self):
        # TODO: make an util.py with this and use this method across the workspace to avoid copypase
        return (self.isPointInBoundaries(self.x_start, self.y_start) and
            self.isPointInBoundaries(self.x_end, self.y_end) and
            self.isPointFree(self.x_start, self.y_start) and
            self.isPointFree(self.x_end, self.y_end))

    def isPointInBoundaries(self, x, y):
        self.xmin = self.map.info.origin.position.x
        self.ymin = self.map.info.origin.position.y
        self.xmax = round(self.xmin + self.map.info.width * self.map.info.resolution, 3) 
        self.ymax = round(self.ymin + self.map.info.height * self.map.info.resolution, 3)
        return (x < self.xmax) and (x > self.xmin) and (y < self.ymax) and (y > self.ymin)

    def isPointFree(self, x, y):
        res = self.map.info.resolution
        x_pos = round(x/res, 1) - round(self.xmin/ res, 1)
        y_pos = round(y/res, 1) - round(self.ymin/ res, 1)
        n = int(y_pos) * self.map.info.width  + int(x_pos)
        return  self.map.data[n] == 0

    @staticmethod
    def my_rounding(a,b):
        # get clean division when perform operations with the resolution of the map
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
