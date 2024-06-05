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
            width: int,
            trajectory: np.array
    ):
        
        posx, posy = pos
        pry, prx = TrajectoryTracking._closest_point(pos, trajectory)
        errtheta, theta = utils.get_angle(pos, (prx, pry))
        x = posx-prx
        errx = 2*(x + width/2)/width - 1
        
        return (prx, pry), errx, theta, errtheta

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
            if dist < closest_dist:
                closest_dist = dist
                closest = i
        
        return trajectory[closest]