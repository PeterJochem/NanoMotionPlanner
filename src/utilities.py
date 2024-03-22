import numpy as np


def row_to_string(row: np.ndarray) -> str:
    """Converts the numbers into a format which is easier to read.

    Args:
        row: numpy.ndarray

    Returns:
        str:
            The array's contents reformatted to be easier to read.
    """

    rounded_row = np.round(row, 2)
    return "[" + ",  ".join(str(num) for num in rounded_row) + "]"
