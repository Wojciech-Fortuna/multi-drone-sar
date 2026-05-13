import numpy as np


def trajectory_distance(trajectory):
    trajectory = np.asarray(trajectory, dtype=float)

    if len(trajectory) < 2:
        return 0.0

    distance = 0.0

    for i in range(1, len(trajectory)):
        distance += np.linalg.norm(trajectory[i] - trajectory[i - 1])

    return distance


def multi_trajectory_distance(trajectories):
    return sum(
        trajectory_distance(trajectory)
        for trajectory in trajectories
    )


def count_waypoints(trajectory):
    return len(trajectory)


def count_multi_waypoints(trajectories):
    return sum(len(trajectory) for trajectory in trajectories)
