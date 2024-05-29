from __future__ import annotations

from cv_bridge import CvBridge

import scripts.colors as colors

import numpy as np
import cv2 as cv
import math

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
        img = self._get_track_outline(
            self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        )
        centerline = self._get_centerline(img)
        
        if self.show_img:
            self.canvas = img
            self._draw_centerline(centerline)
            self.show()

        return centerline

    def _get_track_outline(
            self,
            img: np.array,
    ) -> np.ndarray:
        img = img[int(img.shape[0]/2):img.shape[0]-10, 100:img.shape[1]-100]
        img = cv.resize(img, (640, 480))
        track_outline = np.zeros((img.shape[0], img.shape[1]), dtype=np.uint8)
        img_mask = colors.mask(img, np.array(colors.LOWER_YELLOW), np.array(colors.UPPER_YELLOW))

        # Detect track outline and draw it on a new image
        contours, _ = cv.findContours(img_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        cv.drawContours(track_outline, contours, 0, colors.MAGENTA)

        return track_outline
    
    def _get_centerline(
            self,
            track_outline: np.ndarray
    ) -> np.array:
        centerline = []
        left, right = self._get_track_limits(track_outline)
        for (x1, y1), (x2, y2) in list(zip(left, right)):
            xc = math.floor((x1+x2)/2)
            yc = math.floor((y1+y2)/2)
            centerline.append((xc, yc))
        return np.array(centerline)

    def _get_track_limits(
            self,
            track_outline: np.ndarray
    ):
        _, labels = cv.connectedComponents(track_outline)
        left_limit = np.column_stack(
            np.where(labels==1)
        )
        right_limit = np.column_stack(
            np.where(labels==2)
        )

        return left_limit, right_limit
    
    def _draw_centerline(
            self,
            centerline: np.array
    ) -> None:
        for point in centerline:
            cv.circle(self.canvas, point, colors.MAGENTA, 1)

    def show(self):
        cv.imshow("Visualize", self.canvas)
        cv.waitKey(1)

    def reset(self):
        self.canvas = None