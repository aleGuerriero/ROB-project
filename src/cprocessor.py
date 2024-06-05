from __future__ import annotations

from cv_bridge import CvBridge

import scripts.colors as colors

import numpy as np
import cv2 as cv
import math

class CameraProcessor:
    """
    Class for processing and visualizing camera info.
    """

    def __init__(
            self,
            debug: bool = False
    ) -> None:
        self.canvas = None
        self.debug = debug
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
        self.height, self.width = img.shape
        self.left, self.right, self.centerline = self._get_centerline(img)

        crosshair = (math.floor(self.width / 2), math.floor(self.height/2))
        position = (math.floor(self.width/2), self.height-1)
        
        return position, crosshair, self.centerline

    def _get_track_outline(
            self,
            img: np.array,
    ) -> np.ndarray:

        track_outline = np.zeros((img.shape[0], img.shape[1]), dtype=np.uint8)
        img_mask = colors.mask(img, np.array(colors.LOWER_YELLOW), np.array(colors.UPPER_YELLOW))

        # Detect track outline and draw it on a new image
        contours, _ = cv.findContours(img_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        cv.drawContours(track_outline, contours, 0, colors.MAGENTA)

        track_outline = track_outline[int(img.shape[0]/2):img.shape[0]-10, 100:img.shape[1]-100]

        return track_outline
    
    @staticmethod
    def _get_centerline(
            track_outline: np.ndarray
    ) -> tuple[np.array, np.array, np.array]:
        centerline = []
        left, right = CameraProcessor._get_track_limits(track_outline)
        for (x1, y1), (x2, y2) in list(zip(left[::10], right[::10])):
            xc = math.floor((x1+x2)/2)
            yc = math.floor((y1+y2)/2)
            centerline.append((xc, yc))
        return left, right, np.array(centerline)

    @staticmethod
    def _get_track_limits(
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
    
    def draw(
            self,
            position,
            crosshair,
            waypoint: tuple[int, int],
            angle
    ) -> None:
        self.canvas = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        for (x, y) in self.left:
            cv.circle(self.canvas, (y, x), 1, colors.WHITE)

        for (x, y) in self.right:
            cv.circle(self.canvas, (y, x), 1, colors.WHITE)

        for (x, y) in self.centerline:
            cv.circle(self.canvas, (y, x), 1, colors.MAGENTA)

        cv.line(
            self.canvas,
            position,
            crosshair,
            colors.RED
        )

        cv.line(
            self.canvas,
            crosshair,
            waypoint,
            colors.RED
        )

        cv.line(
            self.canvas,
            position,
            waypoint,
            colors.LIGHT_BLUE
        )

        cv.ellipse(
            self.canvas,
            position,
            (60, 60),
            180,
            90,
            angle,
            colors.WHITE,
            2
        )

    @classmethod
    def width(self):
        return self.width

    def show(self):
        cv.imshow("Visualize", self.canvas)
        cv.waitKey(1)

    def reset(self):
        self.canvas = None