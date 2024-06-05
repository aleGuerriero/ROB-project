from __future__ import annotations
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
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

def distance(pos, postarget):
    posx, posy = pos
    postargetx, postargety = postarget
    return np.sqrt(
        (posx - postargetx)**2 + (posy - postargety)**2 
    )
