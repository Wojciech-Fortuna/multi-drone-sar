import json
from pathlib import Path


RESULTS_DIR = Path("results")

MISSION_STATUS_PATH = RESULTS_DIR / "mission_status.json"
TARGET_STATUS_PATH = RESULTS_DIR / "target_status.json"


def reset_mission_status() -> None:
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    with MISSION_STATUS_PATH.open("w", encoding="utf-8") as file:
        json.dump(
            {
                "mission_finished": False,
                "target_found": False,
                "stop_reason": None,
                "mission_time": 0.0,
                "remaining_probability": 1.0,
                "observed_cells": 0,
            },
            file,
            indent=4,
        )


def reset_target_status() -> None:
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    with TARGET_STATUS_PATH.open("w", encoding="utf-8") as file:
        json.dump({"target_xyz": None, "visible_in_webots": False}, file, indent=4,)


def reset_webots_state() -> None:
    reset_mission_status()
    reset_target_status()
