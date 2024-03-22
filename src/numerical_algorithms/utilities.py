from typing import Callable, Union, Any, Tuple
import numpy as np


def one_hot(length: int, hot_index: int, scale: float) -> np.ndarray:
    """..."""

    array = np.zeros(length, dtype=float)
    array[hot_index] = 1. * scale
    return array


def gradient(f: Callable,
             x: np.ndarray,
             range_index: int,
             delta: float = 1e-4) -> float:
    """..."""

    f_at_point = f(x)
    domain = len(x)

    values = np.zeros(domain)

    for index in range(domain):
        perturbed_x = x + one_hot(domain, index, delta)
        values[index] = (f(perturbed_x)[range_index] - f_at_point[range_index]) / delta

    return values


def derivative_of_function_in_rn(f: Callable, x: Union[float, int], range_index: int, delta: float = 1e-4) -> float:
    """..."""

    return (f(x + delta)[range_index] - f(x)[range_index]) / delta

def derivative(f: Callable, x: Union[float, int], delta: float = 1e-4) -> float:
    """Computes the numerical derivative.

    Args:
        f: Callable
            The function to compute the derivative of.
        x: Union[float, int]
            The location of the derivative in f's domain.
        delta: float
    """

    return (f(x + delta) - f(x)) / delta
