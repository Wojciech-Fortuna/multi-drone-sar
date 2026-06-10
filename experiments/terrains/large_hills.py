import numpy as np

from controllers.terrain import Terrain


def large_hills_height(x, y):
    hills = [
        (35, -120, -120, 1800),
        (28, -60, -40, 1200),
        (32, -100, 80, 1600),
        (25, -20, 120, 1400),
        (30, 60, 90, 1500),
        (36, 120, 120, 1800),
        (24, 110, 20, 1100),
        (34, 70, -80, 1500),
        (27, 0, -130, 1400),
        (31, 130, -120, 1600),
        (22, -140, 30, 1200),
        (26, 20, 20, 1000),
    ]

    height = 0.0

    for amplitude, cx, cy, spread in hills:
        height += amplitude * np.exp(
            -((x - cx) ** 2 + (y - cy) ** 2) / spread
        )

    waves = (
        4.0 * np.sin(0.045 * x)
        + 3.0 * np.cos(0.04 * y)
        + 2.5 * np.sin(0.025 * (x + y))
    )

    return max(0.0, height + waves)


def create_terrain(map_config):
    return Terrain(
        x_min=map_config["x_min"],
        x_max=map_config["x_max"],
        y_min=map_config["y_min"],
        y_max=map_config["y_max"],
        resolution=map_config["resolution"],
        height_function=large_hills_height,
    )