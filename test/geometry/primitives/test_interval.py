import pytest
from geometry.primitives.interval import Interval


@pytest.mark.parametrize("interval_1, interval_2, expected", [(Interval(-10, 10), Interval(5, 20), True),
                                                              (Interval(-10, -10), Interval(-10, 20), True),
                                                              (Interval(-10, -10), Interval(-10, -10), True),
                                                              (Interval(-5, 5), Interval(-2, -1), True),
                                                              (Interval(0, -5), Interval(20, 22), False)])
def test_overlap(interval_1: Interval, interval_2: Interval, expected: bool):

    assert interval_1.intersects(interval_2) == expected
    assert interval_2.intersects(interval_1) == expected
