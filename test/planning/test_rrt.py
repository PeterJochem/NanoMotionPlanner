import numpy as np
import pytest
from planning.planning_problem import JointStateToJointStatePlanningProblem
from planning.rrt import RRT
from robots.ur5 import UR5


def test_rrt_planner():

    robot = UR5()
    start_state = np.zeros(6)
    delta = np.array([0.1, 0.2, 0., 0., 0., 0.])
    end_state = start_state + (delta * 1)
    problem = JointStateToJointStatePlanningProblem(robot, start_state, end_state)
    time_scaler = None
    planner = RRT(problem, time_scaler)

    traj = planner.plan_path()
    assert traj is not None and len(traj) > 2


