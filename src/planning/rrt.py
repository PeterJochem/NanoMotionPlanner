import numpy as np
from planning.graphs.graph import NaiveListGraph
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

        termination_distance = 0.2
        distance = 0.001
        max_number_of_iterations = 1000

        current_state = self.problem.start_state
        goal_state = self.problem.end_state
        current_iteration = 0

        graph = NaiveListGraph()
        graph.add(current_state)

        current_distance = np.linalg.norm(current_state - goal_state)
        distances = [current_distance]

        while current_distance > termination_distance:

            if current_iteration > max_number_of_iterations:
                breakpoint()
                raise RuntimeError("Failed to solve the path planning problem in the given number of iterations.")

            random_joint_state = robot.random_joint_state()
            nearest_neighbor = graph.nearest_neighbor(random_joint_state)

            direction = random_joint_state - nearest_neighbor
            normalized_direction = direction / np.linalg.norm(direction)
            new_state = nearest_neighbor + (distance * normalized_direction)

            # Add the new state to the graph.
            graph.add(new_state)

            current_distance = np.linalg.norm(new_state - goal_state)
            distances.append(current_distance)

            current_iteration += 1
            #print(f"Added a new state to the graph: {new_state}")
            #breakpoint()



        breakpoint()