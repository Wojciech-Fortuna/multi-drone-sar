import numpy as np


def generate(
    num_drones: int,
    altitude: float = 30.0,
):
    if num_drones == 1:
        return [
            np.array([0.0, 0.0, altitude])
        ]

    radius = 20.0

    return [
        np.array([
            radius * np.cos(2 * np.pi * i / num_drones),
            radius * np.sin(2 * np.pi * i / num_drones),
            altitude,
        ])
        for i in range(num_drones)
    ]