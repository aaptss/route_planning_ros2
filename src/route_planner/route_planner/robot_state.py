#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from route_planner.euler_from_q import eu_from_q
import math


class RoboState:
    def __init__(self, x=0.0, y=0.0, phi=0.0, radius_mm=160):
        self.x = x
        self.y = y
        self.phi = phi
        self.radius_mm = radius_mm

    @staticmethod  # use of static method because we
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