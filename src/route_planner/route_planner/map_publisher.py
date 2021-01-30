#!/usr/bin/env python3
import cv2
import yaml
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion, Pose
from route_planner.robot_state import RoboState

class MapHostNode(Node):
    def __init__(self):
        super().__init__("map_publisher")

        self.folder = "/home/rosubuntu//ws_ros2/src/route_planner/test_map/"
        self.mapname = "test"
        self.frame_id = 0
        self.map = OccupancyGrid()
        self.map_publisher_ = self.create_publisher(
            OccupancyGrid,
            "map",
            1)
        self.map_timer_ = self.create_timer(3, self.publish_map) # publish once in 3 secs
        self.get_logger().info("map publisher node is running")

    def publish_map(self):
        msg = self.configure_msg()
        self.map_publisher_.publish(msg)
        self.get_logger().info("map sent")

    def parse_yaml(self):
        yamloc = self.folder + self.mapname + ".yaml"
        with open(yamloc) as file:
            self.yml = yaml.load(file,Loader=yaml.FullLoader)

    def parse_and_configure_map(self):
        self.parse_yaml()

        imgloc = self.folder + self.yml['image']
        img_np_data = cv2.imread(imgloc, 0) # load 1 channel, white-gray-black

        p = (255 - img_np_data) / 255.0
        p[p > self.yml['occupied_thresh']] = 100
        p[p < self.yml['free_thresh']] = 0
        p[np.logical_and(p >= self.yml['free_thresh'],
                         p <= self.yml['occupied_thresh'])] = -1

        p = p.astype('int8')
        data = np.empty(0, dtype = 'int8')
        for i in range (0, p.shape[0]):
            data = np.concatenate([data, p[i,:]])

        self.map_shape = img_np_data.shape

        return data

    def configure_msg(self):
        data = self.parse_and_configure_map()
        self.frame_id += 1

        msg = OccupancyGrid()
        msg.header.frame_id = hex(self.frame_id)

        msg.info.origin.position.x = self.yml['origin'][0]
        msg.info.origin.position.y = self.yml['origin'][1]
        msg.info.resolution = round(self.yml['resolution'],4)
        msg.info.height = self.map_shape[0]
        msg.info.width = self.map_shape[1]
        msg.data = data.tolist()
        return msg


def main(args=None):
    rclpy.init(args=args)

    node = MapHostNode()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__== "__main__":
    main()
