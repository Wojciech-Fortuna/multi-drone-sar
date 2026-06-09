import numpy as np

from controllers.probability_map import ProbabilityMap


def hotspot_probability(x, y, z):
    hotspot = np.array([10.0, 10.0])

    pos = np.array([x, y])
    dist_sq = np.sum((pos - hotspot) ** 2)

    return np.exp(-dist_sq / (2 * 15.0 ** 2))


def create_probability_map(terrain):
    return ProbabilityMap(
        terrain=terrain,
        probability_function=hotspot_probability,
    )