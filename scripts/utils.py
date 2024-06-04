from __future__ import annotations

import numpy as np

import math

def get_angle(
        p: tuple[int, int],
        p1: tuple[int, int]
) -> tuple[float, float]:
    
    """
    Function that returns the angle beetween two segment.
    """

    d = math.sqrt(
        pow(p1[0]-p[0], 2) + pow(p1[1]-p[1], 2)
    )
    angle = math.asin((p1[0]-p[0])/d)
    deg = angle*180/math.pi

    return (deg+90)/90-1, deg