# NanoMotionPlanner
This is a motion planner for robotic arms written entirely in Python. It only depends on Numpy and Pytest!

# Results
I have implemented a lot of fundamental robotics and planning algorithms.

## Robotics Algorithms
1. Triangle-Triangle Collision Checking
2. Mesh-Mesh Collision Checking
3. Robot Self Collision Checking
4. Forward Kinematics via Screw Theory
5. Inverse Kinematics via the Newton Raphson Method

## Planning Algorithms
1. RRT
2. RRT Connect
3. A*
4. Probabilistic Roadmap
5. More to Come!

# Running the Tests
```
export PYTHONPATH="${PYTHONPATH}:<path to repo>/nano_motion_planner/src"  
pytest ./tests
```

# Future Improvements
1. Create a C-Module to do triangle-triangle collision detection. This would greatly reduce the time spent doing collision detection.
