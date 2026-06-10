from __future__ import annotations

import json
import math
from pathlib import Path

from controller import Supervisor


TIME_STEP = 32
DEFAULT_SPEED = 8.0
WAYPOINT_TOLERANCE = 0.6
WEBOTS_DETECTION_RADIUS = 25.0


def find_project_root(start: Path) -> Path:
    current = start.resolve()

    for parent in [current, *current.parents]:
        if (parent / "results").exists() or (parent / "worlds").exists():
            return parent

    return start.resolve().parents[1]


def load_waypoints(robot_name: str) -> list[list[float]]:
    controller_dir = Path(__file__).resolve().parent
    project_root = find_project_root(controller_dir)

    json_path = project_root / "results" / "trajectories.json"

    if not json_path.exists():
        print(f"[{robot_name}] No trajectory file found: {json_path}")
        return []

    with json_path.open("r", encoding="utf-8") as file:
        trajectories = json.load(file)

    raw_waypoints = trajectories.get(robot_name, [])

    if not raw_waypoints:
        print(f"[{robot_name}] No waypoints found in JSON.")
        return []

    waypoints = []

    for point in raw_waypoints:
        if len(point) != 3:
            print(f"[{robot_name}] Skipping invalid waypoint: {point}")
            continue

        waypoints.append([
            float(point[0]),
            float(point[1]),
            float(point[2]),
        ])

    print(f"[{robot_name}] Loaded {len(waypoints)} waypoints.")
    return waypoints


def load_mission_status(project_root: Path) -> dict:
    status_path = project_root / "results" / "mission_status.json"

    if not status_path.exists():
        return {
            "mission_finished": False,
            "target_found": False,
            "stop_reason": None,
        }

    try:
        with status_path.open("r", encoding="utf-8") as file:
            return json.load(file)
    except json.JSONDecodeError:
        return {
            "mission_finished": False,
            "target_found": False,
            "stop_reason": None,
        }


def load_target_status(project_root: Path) -> dict:
    path = project_root / "results" / "target_status.json"

    if not path.exists():
        return {
            "target_xyz": None,
            "visible_in_webots": False,
        }

    try:
        with path.open("r", encoding="utf-8") as file:
            return json.load(file)
    except json.JSONDecodeError:
        return {
            "target_xyz": None,
            "visible_in_webots": False,
        }


def export_target_found_status(project_root: Path) -> None:
    path = project_root / "results" / "mission_status.json"

    status = {
        "mission_finished": True,
        "target_found": True,
        "stop_reason": "target_found",
        "mission_time": None,
        "remaining_probability": None,
        "observed_cells": None,
    }

    with path.open("w", encoding="utf-8") as file:
        json.dump(status, file, indent=4)


def distance(a: list[float], b: list[float]) -> float:
    return math.sqrt(
        (a[0] - b[0]) ** 2 +
        (a[1] - b[1]) ** 2 +
        (a[2] - b[2]) ** 2
    )


def check_webots_target_detection(
    drone_position: list[float],
    project_root: Path,
) -> bool:
    target_status = load_target_status(project_root)

    if not target_status.get("visible_in_webots", False):
        return False

    target_xyz = target_status.get("target_xyz")

    if not isinstance(target_xyz, list) or len(target_xyz) != 3:
        return False

    target_position = [
        float(target_xyz[0]),
        float(target_xyz[1]),
        float(target_xyz[2]),
    ]

    detection_radius = float(
        target_status.get("detection_radius", WEBOTS_DETECTION_RADIUS)
    )

    return distance(drone_position, target_position) <= detection_radius


def move_towards(
    current: list[float],
    target: list[float],
    max_step: float,
) -> list[float]:
    d = distance(current, target)

    if d <= max_step or d == 0:
        return target[:]

    ratio = max_step / d

    return [
        current[0] + (target[0] - current[0]) * ratio,
        current[1] + (target[1] - current[1]) * ratio,
        current[2] + (target[2] - current[2]) * ratio,
    ]


def drone_index(robot_name: str) -> int:
    try:
        return int(robot_name.split("_")[-1])
    except ValueError:
        return 0


def set_status_label(
    robot: Supervisor,
    robot_name: str,
    text: str,
    color: int,
) -> None:
    robot.setLabel(
        0,
        f"{robot_name}: {text}",
        0.02,
        0.02 + 0.04 * drone_index(robot_name),
        0.06,
        color,
        0.0,
        "Arial",
    )


def main() -> None:
    robot = Supervisor()
    robot_name = robot.getName()

    controller_dir = Path(__file__).resolve().parent
    project_root = find_project_root(controller_dir)

    self_node = robot.getSelf()
    if self_node is None:
        raise RuntimeError(
            "Could not access self node. Make sure Robot has supervisor TRUE."
        )

    translation_field = self_node.getField("translation")
    if translation_field is None:
        raise RuntimeError("Could not access translation field.")

    waypoints = load_waypoints(robot_name)

    if not waypoints:
        print(f"[{robot_name}] Waiting without movement.")

        stopped_by_mission_status = False

        while robot.step(TIME_STEP) != -1:
            status = load_mission_status(project_root)

            if status.get("mission_finished", False):
                if not stopped_by_mission_status:
                    reason = status.get("stop_reason", "unknown")
                    print(f"[{robot_name}] Mission finished: {reason}")
                    set_status_label(
                        robot,
                        robot_name,
                        f"stopped ({reason})",
                        0x990000,
                    )
                    stopped_by_mission_status = True

                continue

        return

    dt = TIME_STEP / 1000.0
    max_step = DEFAULT_SPEED * dt

    translation_field.setSFVec3f(waypoints[0])
    self_node.resetPhysics()

    waypoint_index = 1
    finished = False
    stopped_by_mission_status = False

    while robot.step(TIME_STEP) != -1:
        status = load_mission_status(project_root)

        if status.get("mission_finished", False):
            if not stopped_by_mission_status:
                reason = status.get("stop_reason", "unknown")
                print(f"[{robot_name}] Mission finished: {reason}")

                set_status_label(
                    robot,
                    robot_name,
                    f"stopped ({reason})",
                    0x990000,
                )

                stopped_by_mission_status = True

            continue

        current = list(translation_field.getSFVec3f())

        if check_webots_target_detection(current, project_root):
            print(f"[{robot_name}] Target detected in Webots.")

            export_target_found_status(project_root)

            set_status_label(
                robot,
                robot_name,
                "stopped (target_found)",
                0x006600,
            )

            stopped_by_mission_status = True
            continue

        if waypoint_index >= len(waypoints):
            if not finished:
                print(f"[{robot_name}] All waypoints executed.")

                set_status_label(
                    robot,
                    robot_name,
                    "completed",
                    0x003366,
                )

                finished = True

            continue

        target = waypoints[waypoint_index]

        if distance(current, target) <= WAYPOINT_TOLERANCE:
            print(f"[{robot_name}] Reached waypoint {waypoint_index}: {target}")
            waypoint_index += 1
            continue

        new_position = move_towards(current, target, max_step)

        translation_field.setSFVec3f(new_position)
        self_node.resetPhysics()


if __name__ == "__main__":
    main()
