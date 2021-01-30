#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from route_planner.robot_state import RoboState

class UserNode(Node):
    def __init__(self):
        super().__init__("user")

        self.start_subscriber_ = self.create_subscription(
            PoseStamped, "start",
            self.new_start_cb, 1)
        self.end_subscriber_ = self.create_subscription(
            PoseStamped, "end",
            self.new_end_cb, 1)
        self.map_subscriber_ = self.create_subscription(
            OccupancyGrid, "map",
            self.new_map_cb, 1)
        self.path_subscriber_ = self.create_subscription(
            Path, "path",
            self.new_path_cb,1)
        self.get_logger().info("User node is running")

    def new_map_cb(self, msg):
        self.get_logger().info(msg.header.frame_id)
    
    def new_start_cb(self, msg):
        self.get_logger().info(msg.header.frame_id + "   " + str(msg.pose.position))
    
    def new_end_cb(self, msg):
        self.get_logger().info(msg.header.frame_id + "   " + str(msg.pose.position))

    def new_path_cb(self, msg):
        self.get_logger().info(msg.header.frame_id + "   " + str(msg.poses[1].position))


def main(args=None):
    rclpy.init(args=args)

    node = UserNode()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__== "__main__":
    main()