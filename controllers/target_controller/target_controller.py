from __future__ import annotations

import json
from pathlib import Path

from controller import Supervisor


TIME_STEP = 32
TARGET_HEIGHT_OFFSET = 1.5


def find_project_root(start: Path) -> Path:
    current = start.resolve()

    for parent in [current, *current.parents]:
        if (parent / "results").exists() or (parent / "worlds").exists():
            return parent

    return start.resolve().parents[1]


def load_target_status() -> dict:
    controller_dir = Path(__file__).resolve().parent
    project_root = find_project_root(controller_dir)

    path = project_root / "results" / "target_status.json"

    if not path.exists():
        return {
            "target_xyz": None,
            "visible_in_webots": False,
        }

    try:
        with path.open("r", encoding="utf-8") as file:
            return json.load(file)
    except Exception as error:
        print(f"[target_controller] Could not read target_status.json: {error}")
        return {
            "target_xyz": None,
            "visible_in_webots": False,
        }


def main() -> None:
    robot = Supervisor()

    target_node = robot.getFromDef("TARGET_PERSON")
    appearance_node = robot.getFromDef("TARGET_APPEARANCE")

    if target_node is None:
        print("[target_controller] TARGET_PERSON not found.")
        while robot.step(TIME_STEP) != -1:
            pass
        return

    if appearance_node is None:
        print("[target_controller] TARGET_APPEARANCE not found.")
        while robot.step(TIME_STEP) != -1:
            pass
        return

    translation_field = target_node.getField("translation")
    transparency_field = appearance_node.getField("transparency")

    if translation_field is None:
        print("[target_controller] TARGET_PERSON translation field not found.")
        while robot.step(TIME_STEP) != -1:
            pass
        return

    if transparency_field is None:
        print("[target_controller] TARGET_APPEARANCE transparency field not found.")
        while robot.step(TIME_STEP) != -1:
            pass
        return

    last_visible = None
    last_position = None

    while robot.step(TIME_STEP) != -1:
        status = load_target_status()

        target_xyz = status.get("target_xyz")
        visible = status.get("visible_in_webots", False)

        if visible and isinstance(target_xyz, list) and len(target_xyz) == 3:
            x = float(target_xyz[0])
            y = float(target_xyz[1])
            z = float(target_xyz[2])

            webots_position = [x, y, z + TARGET_HEIGHT_OFFSET]

            if last_visible is not True or last_position != webots_position:
                translation_field.setSFVec3f(webots_position)
                transparency_field.setSFFloat(0.0)

                print(f"[target_controller] TARGET_PERSON placed at {webots_position}")

                last_visible = True
                last_position = webots_position

        else:
            if last_visible is not False:
                transparency_field.setSFFloat(1.0)

                print("[target_controller] TARGET_PERSON hidden.")

                last_visible = False
                last_position = None


if __name__ == "__main__":
    main()
