from controllers.terrain import Terrain
from controllers.probability_map import ProbabilityMap


def default_test_height(x, y):
    import numpy as np

    h1 = 12 * np.exp(-((x - 20) ** 2 + (y - 20) ** 2) / 300)
    h2 = 18 * np.exp(-((x + 15) ** 2 + (y + 10) ** 2) / 200)
    h3 = 8 * np.exp(-((x - 5) ** 2 + (y + 25) ** 2) / 150)

    return h1 + h2 + h3


def default_test_probability(x, y, z):
    import numpy as np

    hotspot = np.array([10.0, 10.0])
    pos = np.array([x, y])
    dist_sq = np.sum((pos - hotspot) ** 2)

    return np.exp(-dist_sq / (2 * 15.0 ** 2))


def create_test_terrain():
    return Terrain(
        x_min=-30,
        x_max=30,
        y_min=-30,
        y_max=30,
        resolution=10.0,
        height_function=default_test_height,
    )


def create_test_probability_map(terrain):
    return ProbabilityMap(
        terrain=terrain,
        probability_function=default_test_probability,
    )