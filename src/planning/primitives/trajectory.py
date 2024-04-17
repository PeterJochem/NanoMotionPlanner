import numpy as np


class Trajectory:
    """
    Represents a schedule for when the robot should be in each kinematic state.
    """

    def __init__(self, path: np.ndarray, schedule: np.ndarray):
        """Constructor.

        Args:
            path: numpy.ndarray
                Series of kinematic states.
            schedule: numpy.ndarray
                Indicates the time that robot should be in each kinematic state of the path.
        """
        self.path = path
        self.schedule = schedule
