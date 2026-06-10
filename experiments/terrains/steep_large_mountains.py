import numpy as np

from controllers.terrain import Terrain


def steep_large_mountains_height(x, y):
    mountains = [
        (85, -130, -120, 450),
        (95, -70, -90, 380),
        (110, -120, -20, 420),
        (90, -40, -20, 360),
        (120, -90, 70, 500),
        (100, -20, 110, 430),
        (115, 50, 90, 420),
        (130, 120, 120, 520),
        (105, 100, 20, 380),
        (95, 40, -60, 340),
        (125, 130, -90, 460),
        (80, 0, -140, 360),
    ]

    height = 0.0

    for amplitude, cx, cy, spread in mountains:
        height += amplitude * np.exp(
            -((x - cx) ** 2 + (y - cy) ** 2) / spread
        )

    ridges = (
        10.0 * np.sin(0.08 * x)
        + 8.0 * np.cos(0.075 * y)
        + 7.0 * np.sin(0.055 * (x + y))
        + 5.0 * np.cos(0.06 * (x - y))
    )

    height = height + ridges

    return max(0.0, height)


def create_terrain(map_config):
    return Terrain(
        x_min=map_config["x_min"],
        x_max=map_config["x_max"],
        y_min=map_config["y_min"],
        y_max=map_config["y_max"],
        resolution=map_config["resolution"],
        height_function=steep_large_mountains_height,
    )