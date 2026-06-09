from __future__ import annotations

import importlib
import json
import sys
from pathlib import Path

from controllers.webots_export import (
    export_trajectories_to_json,
    export_webots_world,
)
from controllers.webots_terrain_export import export_terrain_to_webots_proto
from experiments.loaders import load_terrain

from tests.test_utils import reset_webots_state, reset_mission_status


def load_scenario(scenario_name: str) -> dict:
    module = importlib.import_module(
        f"experiments.scenarios.{scenario_name}"
    )
    return module.CONFIG


def run_planner(
    planner_name: str,
    config: dict,
    start_positions,
    detection_radius: float,
    time_budget: float,
) -> dict:
    module = importlib.import_module(
        f"experiments.planners.{planner_name}_runner"
    )

    return module.run(
        config=config,
        start_positions=start_positions,
        detection_radius=detection_radius,
        time_budget=time_budget,
    )


def resolve_target_xyz(
    config: dict,
    terrain,
) -> list[float] | None:
    if "target_xyz" in config:
        return [
            float(config["target_xyz"][0]),
            float(config["target_xyz"][1]),
            float(config["target_xyz"][2]),
        ]

    if "target_xy" not in config:
        return None

    x = float(config["target_xy"][0])
    y = float(config["target_xy"][1])

    height_above_ground = float(
        config.get("target_height_above_ground", 2.0)
    )

    ground_z = float(
        terrain.get_height(x, y)
    )

    return [
        x,
        y,
        ground_z + height_above_ground,
    ]


def export_target_status(
    target_xyz,
    visible_in_webots: bool = True,
    output_path: str | Path = "results/target_status.json",
) -> Path:
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with output_path.open("w", encoding="utf-8") as file:
        json.dump(
            {
                "target_xyz": target_xyz,
                "visible_in_webots": visible_in_webots,
            },
            file,
            indent=4,
        )

    print(f"[Scenario] Saved target status to: {output_path}")
    return output_path


def run_scenario(config: dict) -> None:
    reset_webots_state()

    terrain = load_terrain(config)

    export_terrain_to_webots_proto(
        terrain,
        output_path="protos/GeneratedTerrain.proto",
    )

    planner_name = config.get("planner", "greedy")

    result = run_planner(
        planner_name=planner_name,
        config=config,
        start_positions=config["start_positions"],
        detection_radius=config["detection_radius"],
        time_budget=config["time_budget"],
    )

    if "trajectories" not in result:
        raise KeyError(
            f"Planner runner '{planner_name}' must return "
            "'trajectories' for Webots scenario export."
        )

    trajectories = result["trajectories"]

    export_trajectories_to_json(
        trajectories,
        "results/trajectories.json",
    )

    export_webots_world(
        trajectories,
        "worlds/sar_minimal.wbt",
    )

    reset_mission_status()

    target_xyz = resolve_target_xyz(
        config=config,
        terrain=terrain,
    )

    if target_xyz is not None:
        export_target_status(
            target_xyz=target_xyz,
            visible_in_webots=config.get("show_target", True),
        )

    print()
    print(f"Scenario generated: {config['name']}")
    print(f"Planner: {planner_name}")
    print(f"Drones: {len(config['start_positions'])}")

    if "coverage" in result:
        print(f"Coverage: {result['coverage']:.2%}")

    if target_xyz is not None:
        print(f"Target XYZ: {target_xyz}")
        print(f"Target visible: {config.get('show_target', True)}")

    print()
    print("Now open/reload worlds/sar_minimal.wbt in Webots.")


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python -m experiments.run_webots_scenario basic_3_drones")
        print("  python -m experiments.run_webots_scenario many_drones")
        return

    scenario_name = sys.argv[1]
    config = load_scenario(scenario_name)

    run_scenario(config)


if __name__ == "__main__":
    main()