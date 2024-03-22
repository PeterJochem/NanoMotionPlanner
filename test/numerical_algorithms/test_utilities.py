from typing import Union, Callable
import numpy as np
import pytest
from numerical_algorithms.utilities import derivative, gradient


@pytest.mark.parametrize("f, x, expected", [(np.cos, 0., -np.sin(0.))])
def test_derivative(f: Callable, x: Union[float, int], expected: float):

    assert np.isclose(derivative(f, x, delta=1e-8), expected, atol=1e-5)


@pytest.mark.parametrize("f, x, expected", [(lambda x: np.array([2 * x, np.sin(x)]), np.array([5.]), np.array([2, np.cos(5.)])),
                                            (lambda x: np.array([2 * x[1], np.sin(x[0])]), np.array([1., 2.]), np.array([[0., 2.], [np.cos(1.), 0.]]))
                                            ])
def test_gradient(f: Callable, x: np.ndarray, expected: np.ndarray):

    n = len(f(x))
    for index in range(n):
        gradient_value = gradient(f, x, index, delta=1e-8)
        assert np.allclose(gradient_value, expected[index], atol=1e-5)
