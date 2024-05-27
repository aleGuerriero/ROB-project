#!/usr/bin/pyhton3

from __future__ import annotations

from cv_bridge import CvBridge

from project.scripts.colors import mask, LOWER_YELLOW, UPPER_YELLOW

import numpy as np
import cv2 as cv

class CameraProcessor:
    """
    Class for visualizing the processed camera info.
    """
    def __init__(
            self
    ) -> None:
        self.canvas = None
        self.cv_bridge = CvBridge()

    def process(
            self,
            img_msg,
    ) -> None:
        """
        Method that porcesses the images got from camera and returns the track borders and center.
        
        Args:
            img: the ROS image to process.

        Return:
            An ndarray with ...
        """
        img = self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        height, width, _ = img.shape

    def _get_track_outline(
            self
    ) -> np.ndarray:
        pass

    def show(self):
        cv.imshow("Visualize", self.canvas)
        cv.waitKey(1)

    def reset(self):
        self.canvas = None