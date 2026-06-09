from __future__ import annotations

from controllers.greedy_planner import GreedyPlanner
from controllers.genetic_algorithm import (
    MultiDroneGeneticAlgorithmPlanner,
)
from controllers.metrics import (
    multi_trajectory_distance,
    count_multi_waypoints,
)
from experiments.loaders import (
    load_terrain,
    load_probability_map,
)


def run(
    config: dict,
    start_positions,
    detection_radius: float,
    time_budget: float,
) -> dict:

    terrain = load_terrain(config)

    probability_map = load_probability_map(
        config=config,
        terrain=terrain,
    )

    helper = GreedyPlanner(
        terrain=terrain,
        probability_map=probability_map,
        detection_radius=detection_radius,
        drone_altitude=config["drone_altitude"],
        v_max=config["v_max"],
    )

    candidates = helper.create_candidate_points(
        step=config["candidate_step"]
    )

    visibility_sets = (
        helper.precompute_visibility_sets(
            candidates
        )
    )

    ga_cfg = config.get("ga", {})

    planner = MultiDroneGeneticAlgorithmPlanner(
        terrain=terrain,
        probability_map=probability_map,
        candidate_points=candidates,
        visibility_sets=visibility_sets,
        start_positions=start_positions,
        time_budget=time_budget,
        v_max=config["v_max"],
        population_size=ga_cfg.get(
            "population_size", 40
        ),
        chromosome_length=ga_cfg.get(
            "chromosome_length", 12
        ),
        mutation_rate=ga_cfg.get(
            "mutation_rate", 0.2
        ),
        crossover_rate=ga_cfg.get(
            "crossover_rate", 0.8
        ),
        elite_size=ga_cfg.get(
            "elite_size", 2
        ),
    )

    result = planner.evolve(
        generations=ga_cfg.get(
            "generations", 40
        )
    )

    trajectories = result["trajectories"]

    total_distance = multi_trajectory_distance(
        trajectories
    )

    coverage = result.get(
        "covered_probability",
        result.get("coverage", 0.0),
    )

    return {
        "trajectories": trajectories,
        "coverage": coverage,
        "total_distance": total_distance,
        "total_waypoints": count_multi_waypoints(
            trajectories
        ),
        "coverage_per_distance": (
            coverage / total_distance
            if total_distance > 0.0
            else 0.0
        ),
        "drone_times": str(
            result.get("drone_times", "")
        ),
        "best_fitness": result.get(
            "best_fitness", None
        ),
    }