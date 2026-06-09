import numpy as np

from controllers.greedy_planner import GreedyPlanner
from controllers.simulation import SimulationManager
from controllers.webots_export import (
    export_trajectories_to_json,
    export_webots_world,
)
from controllers.webots_terrain_export import export_terrain_to_webots_proto

from tests.fixtures import (
    create_test_terrain,
    create_test_probability_map,
)
from tests.test_utils import (
    reset_webots_state,
    reset_mission_status,
)


def main():
    reset_webots_state()

    terrain = create_test_terrain()

    export_terrain_to_webots_proto(
        terrain,
        output_path="protos/GeneratedTerrain.proto",
    )

    probability_map = create_test_probability_map(
        terrain
    )

    detection_radius = 25.0

    planner = GreedyPlanner(
        terrain=terrain,
        probability_map=probability_map,
        detection_radius=detection_radius,
        drone_altitude=25.0,
        v_max=5.0,
    )

    start_positions = [
        np.array([0.0, 0.0, 30.0]),
        np.array([20.0, 0.0, 30.0]),
        np.array([-20.0, 0.0, 30.0]),
        np.array([-10.0, 0.0, 30.0]),
    ]

    planning_result = planner.plan_multi_drone(
        start_positions=start_positions,
        time_budget=60.0,
        candidate_step=20.0,
    )

    export_trajectories_to_json(
        planning_result["trajectories"],
        "results/trajectories.json",
    )

    export_webots_world(
        planning_result["trajectories"],
        "worlds/sar_minimal.wbt",
    )

    simulation = SimulationManager(
        terrain=terrain,
        probability_map=probability_map,
        trajectories=planning_result["trajectories"],
        target_xy=(10.0, 20.0),
        detection_radius=detection_radius,
        time_budget=60.0,
        time_step=1.0,
        v_max=5.0,
    )

    result = simulation.run()

    print("\n=== SIMULATION RESULTS ===")
    print(f"Target found: {result['target_found']}")
    print(f"Stop reason: {result['stop_reason']}")
    print(f"Mission time: {result['mission_time']:.2f}")
    print(f"Observed cells: {result['observed_cells']}")
    print(
        f"Remaining probability: "
        f"{result['remaining_probability']:.4f}"
    )

    reset_mission_status()


if __name__ == "__main__":
    main()