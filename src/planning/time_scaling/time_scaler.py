import numpy as np
from abc import ABC, abstractmethod
from planning.primitives.trajectory import Trajectory


class TimeScaler(ABC):
    """
    Base class for objects which scale a path of kinematic states through time.
    """

    def __init__(self, *args, **kwargs):
        """Constructor.

        Args:
            *args: tuple
            **kwargs: dict
        """
        ...

    @abstractmethod
    def scale(self, path: np.ndarray) -> Trajectory:
        """Scales the provided series of kinematic states through time.

        Args:
            path: numpy.ndarray
                Series of kinematic states.

        Returns:
            Trajectory:
                Contains a schedule for when the robot should be in each kinematic state.
        """
        ...
