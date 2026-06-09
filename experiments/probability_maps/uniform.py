from controllers.probability_map import ProbabilityMap


def uniform_probability(x, y, z):
    return 1.0


def create_probability_map(terrain):
    return ProbabilityMap(
        terrain=terrain,
        probability_function=uniform_probability,
    )