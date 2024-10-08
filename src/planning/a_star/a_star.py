from typing import Optional
import numpy as np
import heapq
from planning.a_star.a_star_planning_parameters import AStarPlanningParameters
from planning.planner import Planner
from planning.planning_problem import PlanningProblem
from planning.time_scaling.time_scaler import TimeScaler
from planning.utilities import HeapNode, euclidean_distance


class AStar(Planner):
    """
    Implements the AStar Planning algorithm as presented in the links below.
    https://brilliant.org/wiki/a-star-search/
    """

    def __init__(self, problem: PlanningProblem, time_scaler: TimeScaler, parameters: AStarPlanningParameters):
        """Constructor.

        Args:
            problem: PlanningProblem
                Parameterizes the planning problem.
            time_scaler: TimeScaler
                Determines how the path is converted to a trajectory.
        """
        super().__init__(problem, time_scaler)
        self.parameters = parameters
        self.grid = None

    def successors(self, grid: np.ndarray, indices: np.ndarray) -> np.ndarray:
        """Generates the neighbors of a given point in configuration space.

        Args:
            grid: numpy.ndarray
                Represents the configuration space.
            indices: numpy.ndarray
                Specifies a point in configuration space.

        Returns:
            numpy.ndarray:
                Each entry is a new set of indices indicating one position
                in the robot's configuration space.
        """

        num_joints = len(grid)
        neighbors = []
        for joint_index in range(num_joints):

            decrease_joint_angle_neighbor = indices.copy()
            decrease_joint_angle_neighbor[joint_index] -= 1

            increase_joint_angle_neighbor = indices.copy()
            increase_joint_angle_neighbor[joint_index] += 1

            new_neighbors = []
            if indices[joint_index] > 0:
                new_neighbors.append(decrease_joint_angle_neighbor)

            if indices[joint_index] < len(grid[joint_index]) - 1:
                new_neighbors.append(increase_joint_angle_neighbor)

            neighbors.extend(new_neighbors)

        return np.array(neighbors)

    def plan_path(self) -> Optional[np.ndarray]:
        """..."""

        num_entries_in_row = self.parameters.discretization_factor
        self.grid = np.zeros((self.parameters.num_joints, num_entries_in_row))

        starting_idxs = self.joint_angles_to_indices(self.grid, self.problem.start_state)
        final_idxs = self.joint_angles_to_indices(self.grid, self.problem.end_state)

        frontier_list = [HeapNode(starting_idxs, 0.0)]
        frontier_dict = {tuple(starting_idxs): True}
        closed_set = {}
        min_cost_parents = {}
        current_iteration = 0

        while current_iteration < self.parameters.max_num_iterations:

            searching_from_heap_node = heapq.heappop(frontier_list)

            if np.linalg.norm(np.array(searching_from_heap_node.state - final_idxs)) < 0.1:
                break

            del frontier_dict[tuple(searching_from_heap_node.state)]
            closed_set[tuple(searching_from_heap_node.state)] = True

            neighbors = self.successors(self.grid, searching_from_heap_node.state)

            # Add the neighbors to the frontier.
            neighbors = [neighbor for neighbor in neighbors if (tuple(neighbor) not in closed_set
                                                                and tuple(neighbor) not in frontier_dict)]

            searching_from_joint_angles = self.indices_to_joint_angle(self.grid, searching_from_heap_node.state)

            for neighbor in neighbors:

                neighbor_joint_angles = self.indices_to_joint_angle(self.grid, neighbor)

                cost_to_reach = searching_from_heap_node.distance
                cost_to_reach += euclidean_distance(searching_from_joint_angles, neighbor_joint_angles)

                estimated_remaining_cost = euclidean_distance(searching_from_joint_angles, self.problem.end_state)
                neighbor_path_distance = cost_to_reach + estimated_remaining_cost

                min_cost_parents[tuple(neighbor)] = searching_from_heap_node.state

                heapq.heappush(frontier_list, HeapNode(neighbor, neighbor_path_distance))
                frontier_dict[tuple(neighbor)] = True

            current_iteration += 1

        if current_iteration >= self.parameters.max_num_iterations:
            return None
        return self.convert_graph_to_path(min_cost_parents, final_idxs)

    def convert_graph_to_path(self, min_cost_parents: dict, goal_state_idxs: np.ndarray) -> np.ndarray:
        """Converts the provided graph (represented as a dict) into a path from the start state to the goal state.

        Args:
            min_cost_parents: dict
                Maps a c-space point to the c-space point who is the min cost neighbor from the start state.
            goal_state_idxs: numpy.ndarray
                The goal state represented in grid index space.

        Returns:
            numpy.ndarray:
                Path from the start state to the goal state.
        """

        current_state = tuple(goal_state_idxs)
        path = []
        while current_state is not None:
            path.append(current_state)
            if current_state in min_cost_parents:
                current_state = tuple(min_cost_parents[current_state])
            else:
                current_state = None

        final_path = []
        for path_entry in path:
            final_path.append(self.indices_to_joint_angle(self.grid, path_entry))

        path = list(reversed(final_path))
        return np.array(path)


    def indices_to_joint_angle(self,
                               grid: np.ndarray,
                               indices: np.ndarray,
                               lower_limit=0.,
                               upper_limit=np.pi * 2) -> np.ndarray:
        """Maps c-space grid indices to the corresponding joint angles.

        Args:
            grid: numpy.ndarray
            indices: numpy.ndarray
            lower_limit
                Joint limit in radians.
            upper_limit
                Joint limit in radians.

        Returns:
            numpy.ndarray:
                Joint angles in radians.
        """

        range = upper_limit - lower_limit
        delta = range / len(grid[0])
        return np.array([index * delta for index in indices])

    def joint_angles_to_indices(self,
                                grid: np.ndarray,
                                joint_angles: np.ndarray,
                                lower_limit=0.,
                                upper_limit=np.pi * 2) -> np.ndarray:
        """Maps joint angles to the corresponding C-space grid indices.

        Args:
            grid: numpy.ndarray
            joint_angles: numpy.ndarray
                Expressed in radians.
            lower_limit
                Joint limit in radians.
            upper_limit
                Joint limit in radians.

        Returns:
            numpy.ndarray:
                The joint angles corresponding C-space grid indices.
        """

        range = upper_limit - lower_limit
        delta = range / len(grid[0])
        return np.array([int(joint_angle / delta) for joint_angle in joint_angles])
