#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid


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
        self.get_logger().info(msg.header.frame_id + " map frame received")
        self.get_logger().info("        " + self.map.header.frame_id + " was previous")

        if self.map.header.frame_id != msg.header.frame_id:
            self.isMap = True
        self.map = msg

        self.fetch_points()
        while(not self.arePointsOk):
            self.fetch_points()

        if self.isMap and self.arePointsOk:
            self.isMap = False
            self.publish_start()
            self.publish_end()

    def fetch_points(self):
        self.x_start = float(input("Please enter the starting X coordinate: "))
        self.y_start = float(input("Please enter the starting Y coordinate: "))
        self.get_logger().info("Your input is (X, Y) = (" +
                               str(self.x_start)+", " + str(self.y_start) + " )\r\n")

        self.x_end = float(input("Please enter the terminal X coordinate: "))
        self.y_end = float(input("Please enter the terminal Y coordinate: "))

        self.get_logger().info("Your input is (X, Y) = (" +
                               str(self.x_end)+", " + str(self.y_end) + " )\r\n")

        self.x_start = self.my_rounding(self.x_start, self.map.info.resolution)
        self.y_start = self.my_rounding(self.y_start, self.map.info.resolution)
        self.x_end = self.my_rounding(self.x_end, self.map.info.resolution)
        self.y_end = self.my_rounding(self.y_end, self.map.info.resolution)

    def publish_start(self):
        start_pt = PoseStamped()
        start_pt.pose.position.x = self.x_start
        start_pt.pose.position.y = self.y_start
        start_pt.pose.position.z = 0.1
        self.start_publisher_.publish(start_pt)

    def publish_end(self):
        end_pt = PoseStamped()
        end_pt.pose.position.x = self.x_end
        end_pt.pose.position.y = self.y_end
        end_pt.pose.position.z = 0.1

        self.end_publisher_.publish(end_pt)

    def arePointsOk(self):
        # TODO: make an util.py with this and use this method across the workspace to avoid copypase
        return (isPointInBoundaries(self.x_start, self.y_start) and
            isPointInBoundaries(self.x_end, self.y_end) and
            isPointFree(self.x_start, self.y_start) and
            isPointFree(self.x_end, self.y_end))

    def isPointInBoundaries(self, x, y):
        self.xmin = self.map.info.origin.position.x
        self.ymin = self.map.info.origin.position.y
        self.xmax = self.xmin + self.map.info.width
        self.ymax = self.ymin + self.map.info.height
        return (x < xmax) & (x > xmin) & (y < ymax) & (y > ymin)

    def isPointFree(self, x, y):
        res = self.map.info.resolution
        x_pos = self.my_rounding(x, res) - self.my_rounding(self.xmin, res)
        y_pos = self.my_rounding(y, res) - self.my_rounding(self.ymin, res)
        n = int(y_pos) * self.map.info.width  + int(x_pos)
        return  self.msg.data[n] == 0

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
