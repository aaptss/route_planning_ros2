#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from route_planner.robot_state import RoboParams
from route_planner.rapid_exp_tree import RRTGraph
from route_planner.euler_from_q import q_from_eu

import cv2

class PathPlannerNode(Node):
# TODO: make it a server
# # i should not because the assignement is to publish in topic
# # however, making this node a server is more comprehensive
    def __init__(self):
        super().__init__("path_planner")

        self.isMapFlag = False
        self.start_header = None
        self.map_header = None
        self.end_header = None
        self.start = None
        self.end = None
        self.rs = RoboParams()
        self.seq = 0
        
        self.constructionFaliedFlag = False
        
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
        # self.get_logger().info("map " + msg.header.frame_id + " received")
        self.map_header = msg.header
        self.map_meta = msg.info
        self.ocgrid = msg.data
        if not self.isMapFlag:
            step_size = self.rs.diam * 3
            max_iterations = 5000
            self.rrt = RRTGraph(step_size, max_iterations, self.map_meta, self.ocgrid)
        self.isMapFlag = True

    def new_start_cb(self, msg):
        # self.get_logger().info(hex(msg.header.stamp.nanosec) + "   " + str(msg.pose.position))
        self.start_header = msg.header
        self.start = (msg.pose.position.x, msg.pose.position.y)
        self.constructionFaliedFlag = False

    def new_end_cb(self, msg):
        # self.get_logger().info(hex(msg.header.stamp.nanosec) + "   " + str(msg.pose.position))
        self.end_header = msg.header
        self.end = (msg.pose.position.x, msg.pose.position.y)
        self.constructionFaliedFlag = False
        self.publish_path()
    
    def isReadyToConstruct(self):
        # self.get_logger().info(str(self.isMapFlag))
        if self.isMapFlag:
            if not self.constructionFaliedFlag:
                if (self.map_header is not None 
                and self.start_header is not None 
                and self.end_header is not None):
                    if ((self.map_header.frame_id == self.start_header.frame_id)
                    and (self.map_header.frame_id == self.end_header.frame_id)):
                        return True
        return False

    def publish_path(self):
        if self.isReadyToConstruct():
            msg = Path()
            self.get_logger().info("start prepping the path, calling rrt algo for start " + str(self.start) + " and end " + str(self.end))
            self.call_rrt()
            if self.path is not None:
                msg = self.configure_msg()
                self.path_publisher_.publish(msg)
                self.get_logger().info("path " + str(msg.header.stamp.nanosec) + " is published")
            else:
                self.get_logger().info("RRT algo returned " + str(self.path) + ". Message not published")
                self.constructionFaliedFlag = True

    def call_rrt(self):
        self.path = self.rrt.path_constructor(self.end, self.start)
        self.get_logger().info("RRT finished it's job: " + str(self.path))

    def configure_msg(self):
        message = Path()
        message.header.frame_id = self.start_header.frame_id
        message.header.stamp = self.start_header.stamp
        one_of_poses = PoseStamped()
        for i in range(0, len(self.path)):
            prev_pose = one_of_poses 
            one_of_poses = PoseStamped()
            one_of_poses.pose.position.x = self.path[i][0]
            one_of_poses.pose.position.y = self.path[i][1]
            # message.poses[i].position.x = self.path[i][0]
            # message.poses[i].position.y = self.path[i][1]
            if i < len(self.path):
                _ , phi = self.rrt.get_dist_and_angle(prev_pose.pose.position, one_of_poses.pose.position)
            else:
                phi = 0
            qx, qy, qz, qw = q_from_eu(0,0,phi)
            one_of_poses.pose.orientation.x = qx
            one_of_poses.pose.orientation.y = qy
            one_of_poses.pose.orientation.z = qz
            one_of_poses.pose.orientation.w = qw
            message.poses.append(one_of_poses)
        return message

def main(args=None):
    rclpy.init(args=args)

    node = PathPlannerNode()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__== "__main__":
    main()