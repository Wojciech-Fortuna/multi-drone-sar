import numpy as np

class ProbabilityMap:
    def __init__(self, terrain, sigma=15.0):
        self.terrain = terrain
        self.cells = []
        self.probabilities = []

        self._generate_probability_map(sigma)

    def _generate_probability_map(self, sigma):
        hotspot = np.array([10.0, 10.0])

        for x in self.terrain.x_values:
            for y in self.terrain.y_values:
                pos = np.array([x, y])
                dist_sq = np.sum((pos - hotspot) ** 2)

                p = np.exp(-dist_sq / (2 * sigma ** 2))

                z = self.terrain.get_height(x, y)
                self.cells.append(np.array([x, y, z]))
                self.probabilities.append(p)

        self.probabilities = np.array(self.probabilities)
        self.probabilities = self.probabilities / np.sum(self.probabilities)
