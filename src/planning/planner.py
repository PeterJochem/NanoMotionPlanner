from abc import ABC, abstractmethod
import numpy as np
from planning.planning_problem import PlanningProblem
from planning.primitives.trajectory import Trajectory
from planning.time_scaling.time_scaler import TimeScaler


class Planner(ABC):
    """
    Base class for implementing motion planning algorithms.
    """

    def __init__(self, problem: PlanningProblem, time_scaler: TimeScaler):
        """Constructor.

        Args:
            problem: PlanningProblem
            time_scaler: TimeScaler
        """
        self.problem = problem
        self.time_scaler = time_scaler

    @abstractmethod
    def plan_trajectory(self) -> Trajectory:
        """Plans a trajectory in accordance with the problem's parameters.

        Returns:
            Trajectory:
                Moves the robot from the start state to the goal state.
        """

        path = self.plan_path()
        return self.time_scaler.scale(path)

    @abstractmethod
    def plan_path(self) -> np.ndarray:
        """Plans a series of kinematic states which move the robot from the start state to the goal state.

        Returns:
            numpy.ndarray:
                A series of kinematic states which move the robot from the start state to the goal state.
        """
        ...
