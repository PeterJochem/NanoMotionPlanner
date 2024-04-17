import numpy as np


class PlanningProblem:
    """
    Base class for implementing planning requests. These are objects which parameterize the planning problem.
    """

    def __init__(self, start_state: np.ndarray, end_state: np.ndarray):
        """Constructor.

        Args:
            start_state: numpy.ndarray
                Encodes the starting state. Format depends on concrete implementation.
            end_state: numpy.ndarray
                Encodes the goal state. Format depends on concrete implementation.
        """
        self.start_state = start_state
        self.end_state = end_state


class JointStateToJointStatePlanningProblem(PlanningProblem):
    """
    Represents a request to plan from one joint state to another.
    """
    def __init__(self, start_state: np.ndarray, end_state: np.ndarray):
        """Constructor.

        Args:
            start_state: numpy.ndarray
                Joint angles in radians of the starting state.
            end_state: numpy.ndarray
                Joint angles in radians of the goal state.
        """
        super().__init__(start_state, end_state)


class JointStateToPoseStatePlanningProblem(PlanningProblem):
    """
    Represents a request to plan from a joint state to a pose.
    """
    def __init__(self, start_state: np.ndarray, end_state: np.ndarray):
        """Constructor.

        Args:
            start_state: numpy.ndarray
                Joint angles in radians of the starting state.
            end_state: numpy.ndarray
                Goal state pose. 6 vector. [x, y, z, roll, pitch, yaw]. Angles are in radians.
        """
        super().__init__(start_state, end_state)


class PoseStateToPoseStatePlanningProblem(PlanningProblem):
    """
    Represents a request to plan from one pose to another pose.
    """

    def __init__(self, start_state: np.ndarray, end_state: np.ndarray):
        """Constructor.

        Args:
            start_state: numpy.ndarray
                Starting state pose. 6 vector. [x, y, z, roll, pitch, yaw]. Angles are in radians.
            end_state: numpy.ndarray
                Goal state pose. 6 vector. [x, y, z, roll, pitch, yaw]. Angles are in radians.
        """
        super().__init__(start_state, end_state)