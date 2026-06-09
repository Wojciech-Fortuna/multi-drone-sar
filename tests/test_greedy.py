import numpy as np

from controllers.greedy_planner import GreedyPlanner
from controllers.visualization import plot_trajectory_2d

from tests.fixtures import (
    create_test_terrain,
    create_test_probability_map,
)


def main():
    terrain = create_test_terrain()

    probability_map = create_test_probability_map(
        terrain
    )

    planner = GreedyPlanner(
        terrain=terrain,
        probability_map=probability_map,
        detection_radius=35.0,
        drone_altitude=25.0,
        v_max=5.0,
    )

    start_pos = np.array([0.0, 0.0, 30.0])

    result = planner.plan_single_drone(
        start_pos=start_pos,
        time_budget=60.0,
        candidate_step=20.0,
    )

    print("\n=== GREEDY TEST RESULTS ===")
    print(f"Trajectory length: {len(result['trajectory'])}")
    print(f"Mission time: {result['time']:.2f}")
    print(
        f"Observed cells: "
        f"{result['observed_cells']} / {result['total_cells']}"
    )
    print(f"Coverage: {result['coverage']:.2%}")

    print("\nTrajectory:")
    for point in result["trajectory"]:
        print(point)

    plot_trajectory_2d(
        terrain=terrain,
        probability_map=probability_map,
        trajectory=result["trajectory"],
        save_path="plots/greedy_trajectory.png",
    )


if __name__ == "__main__":
    main()