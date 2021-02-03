#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from route_planner.robot_state import RoboParams

class UserNode(Node):
    def __init__(self):
        super().__init__("user")
        self.rs = RoboParams()
        self.path = None
        self.path_header = None
        self.seq = 0
        self.isMapFlag = False
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
        # self.get_logger().info("map " + msg.header.frame_id + " received")
        self.map_header = msg.header
        self.map_meta = msg.info
        self.ocgrid = msg.data
        self.isMapFlag = True

    def new_start_cb(self, msg):
        # self.get_logger().info(hex(msg.header.stamp.nanosec) + "   " + str(msg.pose.position))
        self.start_header = msg.header
        self.start = (msg.pose.position.x, msg.pose.position.y)

    def new_end_cb(self, msg):
        # self.get_logger().info(hex(msg.header.stamp.nanosec) + "   " + str(msg.pose.position))
        self.end_header = msg.header
        self.end = (msg.pose.position.x, msg.pose.position.y)

    def new_path_cb(self, msg):
        # self.get_logger().info(hex(msg.header.stamp.nanosec) + "   " + str(msg.poses[0].position))
        self.path_header = msg.header
        self.path = msg.poses
        # self.get_logger().info(str(msg))
        self.draw_output()

    def draw_output(self):
        if self.path is not None:
            self.seq += 1
            start = self.start
            end = self.end
            map_meta = self.map_meta
            yml = self.rs.yaml_parse(self.rs.folder + self.rs.mapname + ".yaml")
            imgloc = self.rs.folder + yml['image']
            canvas = cv2.imread(imgloc) # load 1 channel, white-gray-black

            # self.draw_a_point(start, canvas, map_meta, self.rs.footprint_px, [255, 0, 0])
            # self.draw_a_point(end, canvas, map_meta, self.rs.footprint_px, [0, 255, 0])
            
            for i in range(0, len(self.path)):
                color = [0, 255 - int(255*i/len(self.path)), int(255*i/len(self.path))]
                pt = (self.path[i].pose.position.x, self.path[i].pose.position.y)
                self.draw_a_point(pt, canvas, map_meta, self.rs.footprint_px, color)
            cv2.imwrite(self.rs.folder + "geterated_len_" + str(len(self.path)) + "_id_" + hex(self.path_header.stamp.nanosec) + ".png", canvas)
            self.get_logger().info("geterated_len_" + str(len(self.path)) + "_id_" + hex(self.path_header.stamp.nanosec) + ".png saved")

    @staticmethod
    def draw_a_point(pt, canvas, params, pt_thickness, color):
        pt_idx_x = int(round(pt[0] / params.resolution, 2)
        - round(params.origin.position.x / params.resolution, 2))
        pt_idx_y = int(round(pt[1] / params.resolution, 2)
        - round(params.origin.position.y / params.resolution, 2))
        for row in range(pt_thickness):
            for col in range(pt_thickness):
                canvas[pt_idx_y + row,pt_idx_x + col] = color


def main(args=None):
    rclpy.init(args=args)

    node = UserNode()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__== "__main__":
    main()