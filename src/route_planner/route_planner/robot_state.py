#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from route_planner.euler_from_q import eu_from_q
import math
import yaml


class RoboPose:
    def __init__(self, x=0.0, y=0.0, phi=0.0):
        self.id = None
        self.x = None
        self.y = None
        self.phi = None
        self.dt = None

    def get_new_state(self, pose):
        self.x = pose.position.x
        self.y = pose.position.y

        qtrnion = (pose.orientation.x,
                   pose.orientation.y,
                   pose.orientation.z,
                   pose.orientation.w)
        (roll, pitch, yaw) = eu_from_q(qtrnion)
        self.phi = yaw

class RoboParams:
    def __init__(self, diam=0.2,folder="/home/rosubuntu/ws_ros2/src/route_planner/test_map/",mapname="test"):
        self.diam = diam
        self.folder = folder
        self.mapname = mapname
        
        yamloc = self.folder + self.mapname + ".yaml"
        yml = self.yaml_parse(yamloc)
        self.footprint_px = int(self.diam / round(yml['resolution'],4))

    def yaml_parse(self,location):
        with open(location) as file:
            yml = yaml.load(file,Loader=yaml.FullLoader)
        return yml
