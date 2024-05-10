from typing import List
import numpy as np
import heapq
from planning.planner import Planner
from planning.planning_problem import PlanningProblem
from planning.time_scaling.time_scaler import TimeScaler


class AStar(Planner):
    """
    Implements the AStar Planning algorithm as presented in the links below.
    https://brilliant.org/wiki/a-star-search/
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

    def plan_path(self):
        """..."""
        ...

        # [0, 2 * pi]
        range = np.pi * 2
        discretization_factor = 10
        num_joints = 6

        shape = (range / discretization_factor) * num_joints
        grid = np.zeros(shape, dtype=bool)

        frontier = []

        # Adding elements to the heap using heappush
        cost = 10  # g(N) + h(N) where g is the cost to reach the node and h is the estimated cost to reach the end.
        heapq.heappush(frontier, cost)
        heapq.heappush(frontier, cost)

        lowest_estimated_cost_node = heapq.heappop(frontier)

        visited = {}



    def euclidean_distance(self, point1: np.ndarray, point2: np.ndarray) -> float:
        """Calculates the Euclidean distance between two N-dimensional points.

        Args:
            point1: numpy.ndarray
            point2: numpy.ndarray

        Returns:
            float:
                Distance in RN space.
        """
        return np.linalg.norm(point1 - point2)

    def neighbors(self, grid: np.ndarray, coordinate: np.ndarray) -> List[np.ndarray]:
        """..."""

        coordinates = []
        n = len(grid)

        for i in range(n):

            delta = np.zeros(n, dtype=int)
            delta[i] = 1

            coordinates.append(coordinate - delta)
            coordinates.append(coordinate + delta)

        return coordinates


