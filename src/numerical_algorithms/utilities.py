from typing import Callable, Union
import numpy as np


def normalized_cosine_similarity(vector_1: np.ndarray, vector_2: np.ndarray) -> float:
    """Calculates the normalized cosine similarity metric between the two provided vectors.

    Notes:
        Normalized denotes that two parallel vectors would have a ncs of 1.0.

    Args:
        vector_1: numpy.ndarray
        vector_2: numpy.ndarray

    Returns:
        float:
            cosine of angle between the two provided vectors.
    """
    return abs(np.dot(vector_1, vector_2) / (np.linalg.norm(vector_1) * np.linalg.norm(vector_2)))


def angle_between_vectors(vector_1: np.ndarray, vector_2: np.ndarray) -> float:
    """Calculates the angle (in radians) between the two provided vectors.

    Args:
        vector_1: numpy.ndarray
        vector_2: numpy.ndarray

    Returns:
        float:
            The angle, in radians, between the two vectors.
    """
    return np.arccos(normalized_cosine_similarity(vector_1, vector_2))


def one_hot(length: int, hot_index: int, scale: float) -> np.ndarray:
    """Creates a one hot array.

    Args:
        length: int
            The length of the one hot array.
        hot_index: int
            The index where the one non-zero value goes.
        scale: float
            Scales the hot entry.
    """

    array = np.zeros(length, dtype=float)
    array[hot_index] = 1. * scale
    return array


def gradient(f: Callable,
             x: np.ndarray,
             range_index: int,
             delta: float = 1e-4) -> np.ndarray:
    """Computes the gradient of f evaluated at x.

    Args:
        f: Callable
            Computes the gradient of this function.
        x: numpy.ndarray
            Location to evaluate the gradient.
        range_index: int
            The output vector is M-dimensional. This indicates which value to compute the gradient of.
        delta: float

    Returns:
        numpy.ndarray:
            [df()[range_index] / dx[0], df()[range_index] / dx[1], ..., df[range_index] / dx[N]]
    """

    f_at_point = f(x)
    domain = len(x)
    values = np.zeros(domain, dtype=float)
    for index in range(domain):
        perturbed_x = x + one_hot(domain, index, delta)
        values[index] = (f(perturbed_x)[range_index] - f_at_point[range_index]) / delta
    return values


def derivative_of_function_in_rn(f: Callable, x: Union[float, int], range_index: int, delta: float = 1e-4) -> float:
    """Computes the derivative of f at x.

    Args:
        f: Callable
            Takes the derivative of this function.
        x: Union[float, int]
            Domain variable.
        range_index: int
            The output is M dimensional. This indicates which of the M outputs to take the derivative of.
        delta: float

    Returns:
        float:
            df()[range_index] / dx
    """

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
