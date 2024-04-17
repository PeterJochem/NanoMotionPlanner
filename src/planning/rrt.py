import numpy as np

from planning.planner import Planner
from planning.planning_problem import PlanningProblem
from planning.time_scaling.time_scaler import TimeScaler


class RRT(Planner):
    """
    Implements the Rapidly Exploring Random Tree algorithm.
    Description: https://en.wikipedia.org/wiki/Rapidly_exploring_random_tree
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

        robot = self.problem.robot

        termination_distance = 0.1
        distance = 0.2

        current_state = self.problem.start_state
        goal_state = self.problem.end_state

        graph = []

        current_distance = np.linalg.norm(current_state - goal_state)

        while current_distance > termination_distance:

            random_joint_state = ...
            nearest_neighbor = self.nearest(random_joint_state)

            direction = random_joint_state - nearest_neighbor
            new_state = nearest_neighbor + (distance * direction)

            # Add the new state to the graph.








