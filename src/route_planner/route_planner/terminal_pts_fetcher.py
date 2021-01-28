#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

class PintFetcherNode(Node):
    def __init__(self):
        super().__init__("npc_with_quest") # the NPC gives the quest so you know the start and the end points of the journey

        self.x_start = None
        self.y_start = None
        self.x_end = None
        self.y_end = None
        
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

        self.get_logger().info("This node polls for input of start point and end point")

    def fetch_points(self):
        self.x_start = input("Please enter the starting X coordinate: ")
        self.y_start = input("Please enter the starting Y coordinate: ")
        print("Your input is (X, Y) = (" + str(self.x_start)+", " + str(self.y_start) + " \r\n")
        
        self.x_end = input("Please enter the terminal X coordinate: ")
        self.y_end = input("Please enter the terminal Y coordinate: ")
        print("Your input is (X, Y) = (" + str(self.x_end)+", " + str(self.y_end) + " \r\n")
        
        # TODO: check againts the map if the points are valid
        ##TODO: connect the node to the map server (request     )
        # TODO: code callback functions. the'll check if points are ok (in white on the map)

        
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

def main(args=None):
    rclpy.init(args=args)
    
    node = PintFetcherNode()
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__== "__main__":
    main()