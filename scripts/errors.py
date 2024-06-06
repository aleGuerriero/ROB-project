from enum import Enum

class ErrorType(Enum):
    """
    Class for defining the error used during the planning phase.

    """

    LINEAR = 0
    NON_LINEAR = 1

class ErrorTypeException(Exception):
    pass