from __future__ import annotations

import json
from pathlib import Path

from controller import Supervisor


TIME_STEP = 32


def find_project_root(start: Path) -> Path:
    current = start.resolve()

    for parent in [current, *current.parents]:
        if (parent / "results").exists() or (parent / "worlds").exists():
            return parent

    return start.resolve().parents[1]


def reset_mission_status(project_root: Path) -> None:
    path = project_root / "results" / "mission_status.json"

    status = {
        "mission_finished": False,
        "target_found": False,
        "stop_reason": None,
        "mission_time": 0.0,
        "remaining_probability": 1.0,
        "observed_cells": 0,
    }

    with path.open("w", encoding="utf-8") as file:
        json.dump(status, file, indent=4)

    print("[mission_manager] Mission status reset.")


def main() -> None:
    robot = Supervisor()

    controller_dir = Path(__file__).resolve().parent
    project_root = find_project_root(controller_dir)

    reset_mission_status(project_root)

    while robot.step(TIME_STEP) != -1:
        pass


if __name__ == "__main__":
    main()
    