import numpy as np

from controllers.greedy_planner import GreedyPlanner
from controllers.genetic_algorithm import GeneticAlgorithmPlanner
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

    greedy = GreedyPlanner(
        terrain=terrain,
        probability_map=probability_map,
        detection_radius=35.0,
        drone_altitude=25.0,
        v_max=5.0,
    )

    candidates = greedy.create_candidate_points(
        step=20.0
    )

    visibility_sets = (
        greedy.precompute_visibility_sets(
            candidates
        )
    )

    start_pos = np.array([0.0, 0.0, 30.0])

    ga = GeneticAlgorithmPlanner(
        terrain=terrain,
        probability_map=probability_map,
        candidate_points=candidates,
        visibility_sets=visibility_sets,
        start_pos=start_pos,
        time_budget=60.0,
        v_max=5.0,
        population_size=30,
        chromosome_length=6,
        mutation_rate=0.2,
    )

    result = ga.evolve(
        generations=30
    )

    print("\n=== GA TEST RESULTS ===")
    print(
        f"Best fitness: "
        f"{result['best_fitness']:.4f}"
    )

    print(
        f"Best individual: "
        f"{result['best_individual']}"
    )

    print("\nBest trajectory:")

    for point in result["best_trajectory"]:
        print(point)

    plot_trajectory_2d(
        terrain=terrain,
        probability_map=probability_map,
        trajectory=result["best_trajectory"],
        save_path="plots/ga_trajectory.png",
    )


if __name__ == "__main__":
    main()