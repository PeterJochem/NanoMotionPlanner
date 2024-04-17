from planning.planning_problem import PlanningProblem
from robots.robot import Robot


class RRT:
    """
    Implements the Rapidly Exploring Random Tree algorithm.
    Description: https://en.wikipedia.org/wiki/Rapidly_exploring_random_tree
    """

    def __init__(self, robot: Robot, problem: PlanningProblem):
        """Constructor.

        Args:
            robot: Robot
                Plan for this robot.
            problem: PlanningProblem
                Parameterizes the planning problem.
        """
        self.robot = robot
        self.problem = problem

    def plan(self) -> :