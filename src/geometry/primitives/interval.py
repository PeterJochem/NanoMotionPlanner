

class Interval:
    """
    Defines a finite section of the number line.
    """

    def __init__(self, a: float, b: float):
        """Constructor.

        Args:
            a: float
            b: float
        """
        self.smaller = min(a, b)
        self.larger = max(a, b)

    def is_in_interval(self, num: float) -> bool:
        """Checks if the provided number lies in this interval.

        Args:
            num: float

        Returns:
            bool:
                True iff the provided number lies in this interval.
        """
        return self.smaller <= num <= self.larger

    def intersects(self, other: "Interval") -> bool:
        """Checks if this interval and the other one overlap.

        Args:
            other: "Interval"

        Returns:
            bool:
                True iff this interval and the other one overlap.
        """
        return (self.is_in_interval(other.smaller) or
                self.is_in_interval(other.larger) or
                other.is_in_interval(self.smaller) or
                other.is_in_interval(self.larger))
