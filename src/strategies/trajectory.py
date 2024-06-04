from __future__ import annotations

from scripts import utils

import numpy as np
import math

import rospy

class TrajectoryTracking:
    """
    Class that implements a trajectory tracking strategy
    """

    def __init__(
            self
    ) -> None:
        pass

    def plan(
            self,
            pos: tuple[int, int],
            trajectory: np.array
    ):
        
        posx, posy = pos
        rospy.loginfo(f'crosshair: {posx}, {posy}')
        pry, prx = TrajectoryTracking._closest_point(pos, trajectory)
        errtheta, theta = utils.get_angle(pos, (prx, pry))
        errx = posx-prx
        
        return (prx, pry), errx, errtheta

    @staticmethod
    def _closest_point(
            pos: tuple[int, int],
            trajectory: np.array
    ) -> tuple[int, int]:
        if trajectory.size == 0:
            return pos
        
        posx, posy = pos

        closest_dist = float("inf")
        closest = 0
        for i, (y, x) in enumerate(trajectory):
            if y > posy-30:
                continue

            dist = math.sqrt(
                pow(posx-x, 2) + pow(posy-y, 2)
            )
            rospy.loginfo(f'point: ({x}, {y}), dist: {dist}')
            if dist < closest_dist:
                closest_dist = dist
                closest = i
            rospy.loginfo(f'selected point: {trajectory[closest]}')
        
        return trajectory[closest]