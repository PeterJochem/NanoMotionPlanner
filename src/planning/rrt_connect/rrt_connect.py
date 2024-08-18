import numpy as np
from planning.planner import Planner
from planning.planning_problem import PlanningProblem
from planning.time_scaling.time_scaler import TimeScaler


class RRTConnect(Planner):
    """
    Implements the RRT Connect algorithm as presented in the link below.
    https://www.cs.cmu.edu/afs/cs/academic/class/15494-s14/readings/kuffner_icra2000.pdf
    """

    def __init__(self, problem: PlanningProblem, time_scaler: TimeScaler):
        """Constructor.

        Args:
            problem: PlanningProblem
                Parameterizes the planning problem.
            time_scaler: TimeScaler
                Determines how the path is converted to a trajectory.
        """
        super().__init__(problem, time_scaler)

    def plan_path(self) -> np.ndarray:
        """Plans a series of kinematic states which move the robot from the start state to the goal state.

        Returns:
            numpy.ndarray:
                A series of kinematic states which move the robot from the start state to the goal state.
        """
        ...
