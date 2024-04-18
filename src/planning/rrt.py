import numpy as np
from planning.graphs.graph import NaiveListGraph, Graph
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

        termination_distance = 0.21
        distance = 0.1
        max_number_of_iterations = 1000
        sample_goal_state_rate = 10

        current_state = self.problem.start_state
        goal_state = self.problem.end_state
        current_iteration = 0

        graph = NaiveListGraph()
        graph.add(current_state, None)

        current_distance = np.linalg.norm(current_state - goal_state)
        distances = [current_distance]

        while current_distance > termination_distance:

            if current_iteration > max_number_of_iterations:
                raise RuntimeError("Failed to solve the path planning problem in the given number of iterations.")

            random_joint_state = robot.random_joint_state()
            if current_iteration % sample_goal_state_rate == 0:
                random_joint_state = goal_state
            nearest_neighbor = graph.nearest_neighbor(random_joint_state)

            direction = random_joint_state - nearest_neighbor
            normalized_direction = direction / np.linalg.norm(direction)
            new_state = nearest_neighbor + (distance * normalized_direction)

            # Add the new state to the graph.
            if robot.is_legal(new_state):
                graph.add(new_state, nearest_neighbor)
                current_distance = np.linalg.norm(new_state - goal_state)
                distances.append(current_distance)

            current_iteration += 1

        graph.add(goal_state, new_state)
        return self.convert_graph_to_path(graph, goal_state)

    def convert_graph_to_path(self, graph: Graph, goal_state: np.ndarray) -> np.ndarray:
        """Converts the provided graph into a set of

        Args:
            graph: Graph
            goal_state: numpy.ndarray

        Returns:
            numpy.ndarray:
                The computed path.
        """

        current_state = tuple(goal_state)
        path = []
        while current_state is not None:
            path.append(current_state)
            current_state = tuple(current_state)
            current_state = graph.parents[current_state]
        return np.array(path)
