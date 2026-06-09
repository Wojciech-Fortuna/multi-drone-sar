import numpy as np

from controllers.terrain import Terrain


def default_hills_height(x, y):
    h1 = 12 * np.exp(-((x - 20) ** 2 + (y - 20) ** 2) / 300)
    h2 = 18 * np.exp(-((x + 15) ** 2 + (y + 10) ** 2) / 200)
    h3 = 8 * np.exp(-((x - 5) ** 2 + (y + 25) ** 2) / 150)

    return h1 + h2 + h3


def create_terrain(map_config):
    return Terrain(
        x_min=map_config["x_min"],
        x_max=map_config["x_max"],
        y_min=map_config["y_min"],
        y_max=map_config["y_max"],
        resolution=map_config["resolution"],
        height_function=default_hills_height,
    )