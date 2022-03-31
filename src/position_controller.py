#!/usr/bin/python

"""
From an initial position and target (Poses in lat/lon), calculates a new goal
and the distance to the new goal. Returns goal (Pose) and distance (float)
in the body frame.
"""

from utilities import Utilities

from geometry_msgs.msg import PoseStamped

import math
import tf

#Original constant for MAX_DISTANCE chosen by Ian was 5.0 (why?)
MAX_DISTANCE = 5.0 # trying out a bigger value to allow the rover to set goal positions on the other side of the map

class PositionController:

    def __init__(self):

        self.goal = PoseStamped()

        self.utilities = Utilities()

    def calculate_new_goal(self, init, target):
        """Calculates a new goal given an initial position and target position."""

        # if the goal is stationary:
        if target.latitude > 180 or target.longitude > 180:
            x = 0.0
            y = 0.0
            theta = (2.0 * math.pi - (target.heading - init.heading))
            distance = 0.0

        else:
            calc_x, calc_y, calc_distance = self.utilities.calculate_distance(init, target)

            if calc_distance > MAX_DISTANCE:
                x = (MAX_DISTANCE / calc_distance) * calc_x
                y = (MAX_DISTANCE / calc_distance) * calc_y
                distance = MAX_DISTANCE

            else:
                x = calc_x
                y = calc_y
                distance = calc_distance

            if target.heading < 0.0:
                theta = math.atan2(y, x)

            else:
                theta = (2.0 * math.pi - (target.heading - init.heading))

        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, theta)

        self.goal.header.frame_id = "base_link"

        self.goal.pose.position.x = x
        self.goal.pose.position.y = y

        self.goal.pose.orientation.x = quaternion[0]
        self.goal.pose.orientation.y = quaternion[1]
        self.goal.pose.orientation.z = quaternion[2]
        self.goal.pose.orientation.w = quaternion[3]

        return self.goal, distance
