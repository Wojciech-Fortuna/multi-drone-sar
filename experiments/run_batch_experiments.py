from __future__ import annotations

import csv
import importlib
import sys
from itertools import product
from pathlib import Path


def load_config(config_name: str) -> dict:
    module = importlib.import_module(
        f"experiments.batch_configs.{config_name}"
    )
    return module.CONFIG


def load_start_positions(
    mode: str,
    num_drones: int,
    altitude: float,
):
    module = importlib.import_module(
        f"experiments.start_positions.{mode}"
    )

    return module.generate(
        num_drones=num_drones,
        altitude=altitude,
    )


def run_planner(
    planner_name: str,
    config: dict,
    start_positions,
    detection_radius: float,
    time_budget: float,
):
    module = importlib.import_module(
        f"experiments.planners.{planner_name}_runner"
    )

    return module.run(
        config=config,
        start_positions=start_positions,
        detection_radius=detection_radius,
        time_budget=time_budget,
    )


def as_list(value):
    if isinstance(value, list):
        return value
    return [value]


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage:")
        print(
            "python -m experiments.run_batch_experiments CONFIG_NAME"
        )
        return

    config_name = sys.argv[1]
    config = load_config(config_name)

    planners = as_list(
        config.get("planner", ["greedy"])
    )

    start_position_modes = as_list(
        config.get("start_positions_mode", ["circle"])
    )

    combinations = list(
        product(
            planners,
            start_position_modes,
            config["num_drones"],
            config["detection_radius"],
            config["time_budget"],
        )
    )

    output_dir = (
        Path("results")
        / "experiments"
        / config["name"]
    )

    output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    output_csv = output_dir / "summary.csv"

    rows = []

    print()
    print(f"Running batch config: {config['name']}")
    print(f"Total experiments: {len(combinations)}")
    print()

    for idx, (
        planner_name,
        start_positions_mode,
        num_drones,
        detection_radius,
        time_budget,
    ) in enumerate(combinations, start=1):

        print(
            f"[{idx}/{len(combinations)}] "
            f"planner={planner_name} "
            f"mode={start_positions_mode} "
            f"drones={num_drones} "
            f"radius={detection_radius} "
            f"time_budget={time_budget}"
        )

        start_positions = load_start_positions(
            mode=start_positions_mode,
            num_drones=num_drones,
            altitude=30.0,
        )

        result = run_planner(
            planner_name=planner_name,
            config=config,
            start_positions=start_positions,
            detection_radius=detection_radius,
            time_budget=time_budget,
        )

        row = {
            "planner": planner_name,
            "start_positions_mode": start_positions_mode,
            "num_drones": num_drones,
            "detection_radius": detection_radius,
            "time_budget": time_budget,
            **result,
        }

        rows.append(row)

    if not rows:
        print("No results generated.")
        return

    with output_csv.open(
        "w",
        newline="",
        encoding="utf-8",
    ) as f:
        writer = csv.DictWriter(
            f,
            fieldnames=rows[0].keys(),
        )

        writer.writeheader()
        writer.writerows(rows)

    print()
    print(f"Saved results to: {output_csv}")


if __name__ == "__main__":
    main()