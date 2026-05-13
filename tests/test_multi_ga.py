import numpy as np

from controllers.terrain import Terrain
from controllers.probability_map import ProbabilityMap
from controllers.greedy_planner import GreedyPlanner
from controllers.genetic_algorithm import MultiDroneGeneticAlgorithmPlanner
from controllers.visualization import plot_multi_trajectory_2d
from controllers.webots_export import export_trajectories_to_json
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

    helper_planner = GreedyPlanner(
        terrain=terrain,
        probability_map=probability_map,
        detection_radius=35.0,
        drone_altitude=25.0,
        v_max=5.0
    )

    candidates = helper_planner.create_candidate_points(step=20.0)
    visibility_sets = helper_planner.precompute_visibility_sets(candidates)

    start_positions = [
        np.array([0.0, 0.0, 30.0]),
        np.array([20.0, 0.0, 30.0]),
        np.array([-20.0, 0.0, 30.0])
    ]

    ga = MultiDroneGeneticAlgorithmPlanner(
        terrain=terrain,
        probability_map=probability_map,
        candidate_points=candidates,
        visibility_sets=visibility_sets,
        start_positions=start_positions,
        time_budget=60.0,
        v_max=5.0,
        population_size=40,
        chromosome_length=12,
        mutation_rate=0.2,
        crossover_rate=0.8,
        elite_size=2
    )

    result = ga.evolve(generations=40)

    print("\n=== MULTI-DRONE GA RESULTS ===")
    print(f"Best fitness: {result['best_fitness']:.4f}")
    print(f"Covered probability: {result['covered_probability']:.4f}")
    print(f"Total distance: {result['distance']:.2f}")
    print(f"Drone times: {result['drone_times']}")
    print(f"Total waypoints: {result['num_waypoints']}")

    for idx, trajectory in enumerate(result["trajectories"]):
        print(f"\nDrone {idx} trajectory:")
        for point in trajectory:
            print(point)

    export_trajectories_to_json(
        result["trajectories"],
        "results/trajectories.json"
    )

    plot_multi_trajectory_2d(
        terrain=terrain,
        probability_map=probability_map,
        trajectories=result["trajectories"],
        save_path="plots/multi_ga_trajectory.png"
    )

    reset_mission_status()


if __name__ == "__main__":
    main()
