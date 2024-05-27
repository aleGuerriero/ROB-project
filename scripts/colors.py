import cv2 as cv
import numpy as np

# In openCV hue ranges from 0 to 179
MAX_HUE = 179

# HSV thresholds
LOWER_YELLOW = (20, 50, 50)
UPPER_YELLOW = (30, 255, 255)

# Common colors expressed in BGR format
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (0, 0, 255)
LIGHT_BLUE = (255, 150, 0)
YELLOW = (0, 255, 255)
MAGENTA = (255, 0, 255)

def mask(
        src: np.array,
        lowerb: np.array,
        upperb: np.array
) -> np.array:
    """
    Function to convert an image to HSV and bound it to lower bound and upper bound colors.
    See cv.inRange for further documentation.

    Args:
        src: input image.
        lowerb: lower bound color.
        upperb: upper bound color.

    Returns:
        A new image array bounded to the chosen colors.
    """

    return cv.inRange(cv.cvtColor(src, cv.COLOR_BGR2HSV), lowerb, upperb)