import numpy as np

class ProbabilityMap:
    def __init__(
        self,
        terrain,
        probability_function,
    ):
        self.terrain = terrain
        self.cells = []
        self.probabilities = []
        self.probability_function = probability_function

        self._generate_probability_map()

    def _generate_probability_map(self):
        for x in self.terrain.x_values:
            for y in self.terrain.y_values:
                z = self.terrain.get_height(x, y)

                cell = np.array([x, y, z])

                p = self.probability_function(x, y, z)

                self.cells.append(cell)
                self.probabilities.append(p)

        self.probabilities = np.array(
            self.probabilities
        )

        total_probability = np.sum(
            self.probabilities
        )

        if total_probability <= 0:
            raise ValueError(
                "Probability map has zero total probability."
            )

        self.probabilities = (
            self.probabilities / total_probability
        )