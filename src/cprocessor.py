#!/usr/bin/pyhton3

from __future__ import annotations

from cv_bridge import CvBridge

from scripts.colors import mask, LOWER_YELLOW, UPPER_YELLOW, MAGENTA

import numpy as np
import cv2 as cv

class CameraProcessor:
    """
    Class for visualizing the processed camera info.
    """
    def __init__(
            self,
            show_img: bool = False
    ) -> None:
        self.canvas = None
        self.show_img = show_img
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
        height, width, img = self._get_track_outline(
            self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        )
        
        if self.show:
            if self.canvas is None:
                self.canvas = img
            self.show()

    def _get_track_outline(
            self,
            img: np.array,
    ) -> np.ndarray:
        height, width, _ = img.shape
        track_outline = np.zeros((height, width), dtype=np.uint8)
        img_mask = mask(img, np.array(LOWER_YELLOW), np.array(UPPER_YELLOW))

        # Detect track outline and draw it on a new image
        contours, _ = cv.findContours(img_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        cv.drawContours(track_outline, contours, 0, MAGENTA)

        return height, width, track_outline

    def show(self):
        cv.imshow("Visualize", self.canvas)
        cv.waitKey(1)

    def reset(self):
        self.canvas = None