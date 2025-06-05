import numpy as np
import numpy.typing as npt

def probabilities_from_values(values: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]: 
    if np.sum(values) == 0:
        return np.zeros(len(values))
    return values / np.sum(values)


def random_decision(probabilities: npt.NDArray[np.float64]) -> int:
    """Compute a decsion based on the decision probability."""
    return np.random.choice(len(probabilities), p=probabilities)


def greedy_decision(probabilities: npt.NDArray[np.float64]) -> int:
    """Compute a decision based on the greedy policy."""
    return int(np.argmax(probabilities))