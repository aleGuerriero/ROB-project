from __future__ import annotations

from scripts.errors import ErrorType, ErrorTypeException

class PathFollowing:

    """
    Class that implements a path following strategy.
    """

    def __init__(
            self,
            debug: bool,
            error_type: ErrorType
    ) -> None:
        self.debug = debug

        if error_type not in ErrorType:
            raise ErrorTypeException
        self.error_type = error_type

    def plan(
            self,
            pos: tuple[int, int],
            path: tuple[int, int]
    ) -> None:
        """
        Method that implements the planning strategy.

        Args:
            pos: An approximate position of the position of the car.
        """
        pass

    