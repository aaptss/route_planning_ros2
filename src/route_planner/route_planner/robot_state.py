#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from route_planner.euler_from_q import eu_from_q
import math
import yaml


class RoboState:
    def __init__(self, x=0.0, y=0.0, phi=0.0, diam=0.2,folder="/home/rosubuntu/ws_ros2/src/route_planner/test_map/",mapname="test"):
        self.x = x
        self.y = y
        self.phi = phi
        self.diam = diam
        self.folder = folder
        self.mapname = mapname
        
        yamloc = self.folder + self.mapname + ".yaml"
        self.yml = self.yaml_parse(yamloc)
        self.resolution = round(self.yml['resolution'],4)
        self.footprint_px = int(self.diam / self.resolution)

    def yaml_parse(self,location):
        with open(location) as file:
            yml = yaml.load(file,Loader=yaml.FullLoader)
        return yml

    @staticmethod  
    # need to parse external data and reshape it
    def get_new_state(pose):
        new_state = RoboState()
        new_state.x = pose.position.x
        new_state.y = pose.position.y

        qtrnion = (pose.orientation.x,
                   pose.orientation.y,
                   pose.orientation.z,
                   pose.orientation.w)
        (roll, pitch, yaw) = eu_from_q(qtrnion)
        new_state.phi = yaw

        return new_state