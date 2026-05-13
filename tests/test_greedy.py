import numpy as np

from controllers.terrain import Terrain
from controllers.probability_map import ProbabilityMap
from controllers.greedy_planner import GreedyPlanner
from controllers.visualization import plot_trajectory_2d


def main():
    terrain = Terrain(x_min=-30, x_max=30, y_min=-30, y_max=30, resolution=10.0)

    probability_map = ProbabilityMap(terrain)

    planner = GreedyPlanner(
        terrain=terrain,
        probability_map=probability_map,
        detection_radius=35.0,
        drone_altitude=25.0,
        v_max=5.0
    )

    start_pos = np.array([0.0, 0.0, 30.0])

    result = planner.plan_single_drone(start_pos=start_pos, time_budget=60.0, candidate_step=20.0)

    print("\n=== GREEDY TEST RESULTS ===")
    print(f"Trajectory length: {len(result['trajectory'])}")
    print(f"Mission time: {result['time']:.2f}")
    print(f"Observed cells: {result['observed_cells']} / {result['total_cells']}")
    print(f"Coverage: {result['coverage']:.2%}")

    print("\nTrajectory:")
    for point in result["trajectory"]:
        print(point)

    plot_trajectory_2d(
        terrain=terrain,
        probability_map=probability_map,
        trajectory=result["trajectory"],
        save_path="plots/greedy_trajectory.png"
    )


if __name__ == "__main__":
    main()
