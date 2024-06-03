from __future__ import annotations

from scripts import utils

import numpy as np
import math

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
        prx, pry = TrajectoryTracking._closest_point(pos, trajectory)
        theta = utils.get_angle(pos, (prx, pry))
        
        return posx-prx, theta

    @staticmethod
    def _closest_point(
            pos: tuple[int, int],
            trajectory: np.array
    ) -> tuple[int, int]:
        posx, posy = pos

        closest_dist = float("inf")
        closest = 0
        for i, (x, y) in enumerate(trajectory):
            dist = math.sqrt(
                pow(posx-x, 2) + pow(posy-y, 2)
            )
            if closest_dist > dist:
                closest = i
        
        return trajectory[closest]

    def _compute_velocity(
            self
    ):
        pass
    