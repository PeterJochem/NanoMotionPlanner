from typing import Any


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
