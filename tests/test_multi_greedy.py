import numpy as np

from controllers.terrain import Terrain
from controllers.probability_map import ProbabilityMap
from controllers.greedy_planner import GreedyPlanner
from controllers.visualization import plot_multi_trajectory_2d
from controllers.metrics import multi_trajectory_distance, count_multi_waypoints
from controllers.webots_export import (
    export_trajectories_to_json,
    export_webots_world,
)
from controllers.webots_terrain_export import export_terrain_to_webots_proto

from tests.test_utils import reset_webots_state, reset_mission_status


def main():
    reset_webots_state()

    terrain = Terrain(
        x_min=-30,
        x_max=30,
        y_min=-30,
        y_max=30,
        resolution=10.0
    )

    export_terrain_to_webots_proto(
        terrain,
        output_path="protos/GeneratedTerrain.proto"
    )

    probability_map = ProbabilityMap(terrain)

    planner = GreedyPlanner(
        terrain=terrain,
        probability_map=probability_map,
        detection_radius=35.0,
        drone_altitude=25.0,
        v_max=5.0
    )

    start_positions = [
        np.array([0.0, 0.0, 30.0]),
        np.array([20.0, 0.0, 30.0]),
        np.array([-20.0, 0.0, 30.0])
    ]

    result = planner.plan_multi_drone(
        start_positions=start_positions,
        time_budget=60.0,
        candidate_step=20.0
    )

    print("\n=== MULTI DRONE RESULTS ===")
    print(f"Coverage: {result['coverage']:.2%}")

    total_distance = multi_trajectory_distance(result["trajectories"])
    total_waypoints = count_multi_waypoints(result["trajectories"])

    print(f"Total distance: {total_distance:.2f}")
    print(f"Total waypoints: {total_waypoints}")
    print(f"Drone times: {result['times']}")

    for idx, trajectory in enumerate(result["trajectories"]):
        print(f"\nDrone {idx} trajectory:")
        for point in trajectory:
            print(point)

    export_trajectories_to_json(
        result["trajectories"],
        "results/trajectories.json"
    )

    export_webots_world(
        result["trajectories"],
        "worlds/sar_minimal.wbt"
    )

    plot_multi_trajectory_2d(
        terrain=terrain,
        probability_map=probability_map,
        trajectories=result["trajectories"],
        save_path="plots/multi_greedy_trajectory.png"
    )

    reset_mission_status()


if __name__ == "__main__":
    main()
