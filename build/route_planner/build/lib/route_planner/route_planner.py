#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped


class PathPlanner:
    def __init__(self):
        self.start = None
        self.end = None
        self.map = None
        self.is_ok = False

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
            Path, "path", 1)

    def is_ready_to_construct(self):
        return self.map is not None and self.start is not None and self.goal is not None
