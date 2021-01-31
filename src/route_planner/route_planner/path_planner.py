#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from route_planner.robot_state import RoboState
from route_planner.planners.rapid_exp_tree import RRTGraph
from route_planner.planners.rapid_exp_tree import RRT_pathConstructor

import cv2


class PathPlannerNode(Node):
# TODO: make it a server
# # i should not because the assignement is to publish in topic
# # however, making this node a server is more comprehensive
    def __init__(self):
        super().__init__("path_planner")

        self.start = None
        self.end = None
        self.ocgrid = None
        self.isMapFlag = False
        self.rs = RoboState()

        self.ocgrid_subscriber_ = self.create_subscription(
            OccupancyGrid, "map",
            self.new_map_cb, 1)
        self.start_subscriber_ = self.create_subscription(
            PoseStamped, "start",
            self.new_start_cb, 1)
        self.end_subscriber_ = self.create_subscription(
            PoseStamped, "end",
            self.new_end_cb, 1)
        self.path_publisher_ = self.create_publisher(
            Path, 
            "path", 
            1)
        self.get_logger().info("Path planning node is up")

    def new_map_cb(self, msg):
        self.get_logger().info("map " + msg.header.frame_id + " received")
        self.map_header = msg.header
        self.map_meta = msg.info
        self.ocgrid = msg.data
        self.isMapFlag = True

    def new_start_cb(self, msg):
        self.get_logger().info(hex(msg.header.stamp) + "   " + str(msg.pose.position))
        self.start_header = msg.header
        self.start = (msg.pose.position.x, msg.pose.position.y)

    def new_end_cb(self, msg):
        self.get_logger().info(hex(msg.header.stamp) + "   " + str(msg.pose.position))
        self.end_header = msg.header
        self.end = (msg.pose.position.x, msg.pose.position.y)

    def isReadyToConstruct(self):
        if self.isMapFlag:
            if ((self.map_header.frame_id == self.start_header.frame_id)
            and (self.map_header.frame_id == self.end_header.frame_id)):
                return True
        return False

    def call_algo(self):
        if self.isReadyToConstruct():
            pass

    def configure_msg(self):
        message = Path()
        message.header = self.end_header
        return message

    def publish_path(self):
        msg = self.configure_msg()
        self.map_publisher_.publish(msg)
        self.get_logger().info("path " + msg.header.stamp.nanosec + " is published")


def main(args=None):
    rclpy.init(args=args)

    node = PathPlannerNode()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__== "__main__":
    main()