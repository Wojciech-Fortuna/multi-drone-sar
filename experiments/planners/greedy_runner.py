from __future__ import annotations

from controllers.greedy_planner import GreedyPlanner
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

    planner = GreedyPlanner(
        terrain=terrain,
        probability_map=probability_map,
        detection_radius=detection_radius,
        drone_altitude=config["drone_altitude"],
        v_max=config["v_max"],
    )

    result = planner.plan_multi_drone(
        start_positions=start_positions,
        time_budget=time_budget,
        candidate_step=config["candidate_step"],
    )

    trajectories = result["trajectories"]

    total_distance = multi_trajectory_distance(
        trajectories
    )

    return {
        "trajectories": trajectories,
        "coverage": result["coverage"],
        "total_distance": total_distance,
        "total_waypoints": count_multi_waypoints(
            trajectories
        ),
        "coverage_per_distance": (
            result["coverage"] / total_distance
            if total_distance > 0.0
            else 0.0
        ),
        "drone_times": str(
            result.get("times", "")
        ),
    }