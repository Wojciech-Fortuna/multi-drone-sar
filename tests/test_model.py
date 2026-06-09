import numpy as np

from controllers.detection import (
    can_detect,
    target_position_on_terrain,
)

from tests.fixtures import (
    create_test_terrain,
    create_test_probability_map,
)


def main():
    terrain = create_test_terrain()

    prob_map = create_test_probability_map(
        terrain
    )

    drone_pos = np.array([0.0, 0.0, 30.0])

    target_pos = target_position_on_terrain(
        x=10.0,
        y=10.0,
        terrain=terrain,
    )

    detected = can_detect(
        drone_pos=drone_pos,
        target_pos=target_pos,
        terrain=terrain,
        detection_radius=40.0,
    )

    print("\n=== TEST RESULTS ===")

    print(
        f"Number of cells: "
        f"{len(prob_map.cells)}"
    )

    print(
        f"Probability sum: "
        f"{np.sum(prob_map.probabilities):.6f}"
    )

    print(f"Drone position: {drone_pos}")

    print(f"Target position: {target_pos}")

    print(f"Detected: {detected}")


if __name__ == "__main__":
    main()