from abc import ABC, abstractmethod
from typing import List

import numpy as np


class Graph(ABC):
    """
    Base class for representing a set of robot states in some abstract state space.
    """

    def __init__(self):
        """Constructor."""
        ...

    @abstractmethod
    def nearest_neighbor(self, state: np.ndarray) -> np.ndarray:
        """Gets the nearest neighbor to the provided state.

        Args:
            state: numpy.ndarray

        Returns:
            numpy.ndarray
                The nearest state to the provided state.
        """
        ...

    @abstractmethod
    def add(self, state: np.ndarray, **kwargs):
        """..."""
        ...


class NaiveListGraph:
    """
    Represents a set of robotic states as a list. This makes nearest neighbor computations quite slow but it is
    very easy to implement.
    """

    def __init__(self):
        """Constructor."""
        super().__init__()
        self.states = []
        self.edges = {}  # tuple(state) -> connected states

    def nearest_neighbor(self, state: np.ndarray) -> np.ndarray:
        """Gets the nearest neighbor to the provided state.

        Args:
            state: numpy.ndarray

        Returns:
            numpy.ndarray
                The nearest state to the provided state.
        """

        if self.states is None or len(self.states) == 0:
            raise RuntimeError("The graph does not have any states in it.")

        distances = [(np.linalg.norm(state - neighbor), neighbor) for neighbor in self.states]
        return min(distances)[1]

    def add(self, state: np.ndarray, edges: List[np.ndarray] = None):
        """..."""
        self.states.append(state)
        edge_key = tuple(state)

        if edges is not None:
            if edge_key in self.edges:
                self.edges[edge_key] += edges
            else:
                self.edges[edge_key] = edges
