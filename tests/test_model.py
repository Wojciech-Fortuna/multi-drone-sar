import numpy as np

from controllers.terrain import Terrain
from controllers.detection import (can_detect, target_position_on_terrain)
from controllers.probability_map import ProbabilityMap


def main():
    terrain = Terrain(x_min=-50, x_max=50, y_min=-50, y_max=50, resolution=2.0)

    prob_map = ProbabilityMap(terrain)

    drone_pos = np.array([0.0, 0.0, 30.0])

    target_pos = target_position_on_terrain(x=10.0, y=10.0, terrain=terrain)

    detected = can_detect(drone_pos=drone_pos, target_pos=target_pos, terrain=terrain, detection_radius=40.0)

    print("\n=== TEST RESULTS ===")

    print(f"Number of cells: {len(prob_map.cells)}")

    print(f"Probability sum: "f"{np.sum(prob_map.probabilities):.6f}")

    print(f"Drone position: {drone_pos}")

    print(f"Target position: {target_pos}")

    print(f"Detected: {detected}")


if __name__ == "__main__":
    main()
