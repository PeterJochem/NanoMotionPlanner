

class AStarPlanningParameters:
    """Holds parameters for the A Star planner."""

    def __init__(self,
                 num_joints: int,
                 discretization_factor: int = 100,
                 max_num_iterations: int = 500):
        """Constructor."""

        self.num_joints = num_joints
        self.discretization_factor = discretization_factor
        self.max_num_iterations = max_num_iterations
