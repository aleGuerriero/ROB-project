from enum import Enum

class ErrorType(Enum):
    """
    Class for defining the error used during the planning phase.

        ORIENTATION: Error computed based on the orientation of the car in respect to the path to follow.
        POSITION: Error computed based on the position of the car
    """
    ORIENTATION = 0
    POSITION = 1
    NON_LINEAR = 2

class ErrorTypeException(Exception):
    pass