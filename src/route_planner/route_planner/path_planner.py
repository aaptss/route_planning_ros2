#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from route_planner.robot_state import RoboState
import cv2


class PathPlannerNode(Node):
# TODO: make it a server
# # i should not because the assignement is to publish in topic
# # however, making this node a server is more comprehensive
    def __init__(self):
        super().__init__("path_planner")

        self.start_id = None
        self.start = None
        self.end_id = None
        self.end = None
        self.map = None
        rs = RoboState()

        self.map_subscriber_ = self.create_subscription(
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
        self.get_logger().info(msg.header.frame_id)
        self.ocgrid = msg.info
        if self.isReadyToConstruct():
            yml = rs.yaml_parse(rs.folder + rs.mapname + ".yaml")
            imgloc = rs.folder + yml['image']
            self.get_logger().info(imgloc)
            map_image = cv2.imread(imgloc, 0) # load 1 channel, white-gray-black


    def new_start_cb(self, msg):
        self.get_logger().info(msg.header.frame_id + "   " + str(msg.pose.position))
        self.start = msg.pose.position

    def new_end_cb(self, msg):
        self.get_logger().info(msg.header.frame_id + "   " + str(msg.pose.position))
        self.end = msg.pose.position

    def isReadyToConstruct(self):
        if self.map is not None:
            if self.start is not None and self.end is not None:
                if self.start_id == self.end_id:
                    return True
        return False

    def isAbleToConstruct(self):
        # TODO: check if start and end points are in one region (both or none are encircled)
        return True


def main(args=None):
    rclpy.init(args=args)

    node = PathPlannerNode()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__== "__main__":
    main()