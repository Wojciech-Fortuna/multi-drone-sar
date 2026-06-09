import numpy as np

class Terrain:
    def __init__(
        self,
        x_min,
        x_max,
        y_min,
        y_max,
        resolution,
        height_function,
    ):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.resolution = resolution

        self.x_values = np.arange(
            x_min,
            x_max + 0.5 * resolution,
            resolution,
        )

        self.y_values = np.arange(
            y_min,
            y_max + 0.5 * resolution,
            resolution,
        )

        self.height_function = height_function

    def get_height(self, x, y):
        return self.height_function(x, y)