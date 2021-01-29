#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from route_planner.robot_state import RoboState

class UserNode(Node):
    def __init__(self):
        super().__init__("user") 
        
        self.path_subscriber_ = self.create_subscription(
            Path, "end",
            self.new_path_cb, 1)
    self.get_logger().info("User node is running")

    def new_path_cb():
        poses = RoboState.get_new_state(msg.poses)
        self.get_logger().info(poses)

def main(args=None):
    rclpy.init(args=args)
    
    node = UserNode()
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__== "__main__":
    main()