import numpy as np
from planning.a_star.a_star import AStar
from planning.a_star.a_star_planning_parameters import AStarPlanningParameters
from planning.planning_problem import JointStateToJointStatePlanningProblem
from robots.ur5 import UR5


def test_a_star_planner():

    robot = UR5()
    start_state = np.zeros(6)
    delta = np.array([0.1, 0.2, 0.1, 0.2, 0.3, 0.4])
    end_state = start_state + (delta * 1.0)
    problem = JointStateToJointStatePlanningProblem(robot, start_state, end_state)
    time_scaler = None
    parameters = AStarPlanningParameters(6, 100, 50000)
    planner = AStar(problem, time_scaler, parameters)
    traj = planner.plan_path()
    assert traj is not None and len(traj) > 2
    assert np.linalg.norm(traj[-1] - end_state) < 0.25
