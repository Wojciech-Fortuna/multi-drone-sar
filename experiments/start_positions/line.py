import numpy as np


def generate(
    num_drones: int,
    altitude: float = 30.0,
):
    spacing = 15.0

    start_x = -spacing * (num_drones - 1) / 2

    return [
        np.array([
            start_x + i * spacing,
            0.0,
            altitude,
        ])
        for i in range(num_drones)
    ]