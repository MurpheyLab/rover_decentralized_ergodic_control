#!/usr/bin/python
"""Utilities.py contains a couple of utility functions for
calculating the distance between an initial position and target
position in the body frame, as well as the distance between
an initial pose and target pose."""

import utm
import math

EAST = 0
NORTH = 1

class Utilities:

    def __init__(self):

        pass

    def calculate_distance(self, init, target):
        """ Finds the distance to the target and goal position in the body frame."""
        theta = init.heading

        init_utm = utm.from_latlon(init.latitude, init.longitude)

        target_utm = utm.from_latlon(target.latitude, target.longitude)

        east = (target_utm[EAST] - init_utm[EAST])

        north = (target_utm[NORTH] - init_utm[NORTH])

        # calculating the distance to the target
        distance = math.sqrt(math.pow(east, 2.0) + math.pow(north, 2.0))

        # calculating the goal position in the body frame
        x = north * math.cos(theta) + east * math.sin(theta)

        y = north * math.sin(theta) - east * math.cos(theta)

        return x, y, distance

    def calculate_pose_distance(self, init, target, threshold):
        """ Finds the distance between the initial pose and the target pose."""
        dx = target.pose.position.x - init.pose.position.x

        dy = target.pose.position.y - init.pose.position.y

        distance = math.sqrt(math.pow(dx, 2.0) + math.pow(dy, 2.0))

        return distance
