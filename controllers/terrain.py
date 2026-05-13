import numpy as np

class Terrain:
    def __init__(self, x_min, x_max, y_min, y_max, resolution):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.resolution = resolution

        self.x_values = np.arange(x_min, x_max + 0.5 * resolution, resolution)

        self.y_values = np.arange(y_min, y_max + 0.5 * resolution, resolution)

    def get_height(self, x, y):
        h1 = 12 * np.exp(-((x - 20)**2 + (y - 20)**2) / 300)
        h2 = 18 * np.exp(-((x + 15)**2 + (y + 10)**2) / 200)
        h3 = 8 * np.exp(-((x - 5)**2 + (y + 25)**2) / 150)

        return h1 + h2 + h3
