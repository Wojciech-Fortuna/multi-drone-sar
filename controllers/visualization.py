from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


def save_plot(save_path):
    path = Path(save_path)

    if path.parent != Path("."):
        path.parent.mkdir(parents=True, exist_ok=True)

    plt.savefig(path, dpi=300, bbox_inches="tight")


def plot_trajectory_2d(terrain, probability_map, trajectory, save_path=None):
    cells = np.array(probability_map.cells)
    probabilities = np.array(probability_map.probabilities)

    x = cells[:, 0]
    y = cells[:, 1]

    trajectory = np.array(trajectory)

    plt.figure(figsize=(8, 6))

    plt.scatter(x, y, c=probabilities, s=80, alpha=0.8)

    plt.plot(trajectory[:, 0], trajectory[:, 1], marker="o", linewidth=2)

    plt.scatter(trajectory[0, 0], trajectory[0, 1], marker="s", s=120, label="Start")

    plt.scatter(trajectory[-1, 0], trajectory[-1, 1], marker="X", s=120, label="End")

    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("Greedy trajectory on probability map")
    plt.colorbar(label="Target probability")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")

    if save_path is not None:
        save_plot(save_path)
        plt.close()
    else:
        plt.show()

def plot_multi_trajectory_2d(terrain, probability_map, trajectories, save_path=None):
    cells = np.array(probability_map.cells)
    probabilities = np.array(probability_map.probabilities)

    x = cells[:, 0]
    y = cells[:, 1]

    plt.figure(figsize=(8, 6))

    plt.scatter(x, y, c=probabilities, s=80, alpha=0.8)

    for drone_idx, trajectory in enumerate(trajectories):
        trajectory = np.array(trajectory)

        plt.plot(trajectory[:, 0], trajectory[:, 1], marker="o", linewidth=2, label=f"Drone {drone_idx}")

        plt.scatter(trajectory[0, 0], trajectory[0, 1], marker="s", s=120)

        plt.scatter(trajectory[-1, 0], trajectory[-1, 1], marker="X", s=120)

    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("Multi-drone greedy trajectories")
    plt.colorbar(label="Target probability")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")

    if save_path is not None:
        save_plot(save_path)
        plt.close()
    else:
        plt.show()
