from typing import Any
import numpy as np


class HeapNode:
    """Wrapper around entries in a min heap."""

    def __init__(self, state: Any, cost: float):
        """Constructor.

        Args:
            state: Any
                The item to be sorted via the heap.
            cost: float
                Used to compute this object's place in the heap.
        """
        self.state = state
        self.distance = cost

    def __lt__(self, other: "HeapNode") -> bool:
        return self.distance < other.distance


def euclidean_distance(point1: np.ndarray, point2: np.ndarray) -> float:
    """Calculates the Euclidean distance between two N-dimensional points.

    Args:
        point1: numpy.ndarray
        point2: numpy.ndarray

    Returns:
        float:
            Distance in RN space.
    """
    return np.linalg.norm(point1 - point2)
