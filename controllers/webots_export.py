from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, Iterable, Sequence, Any


def _point_to_list(point: Any) -> list[float]:
    values = list(point)
    if len(values) != 3:
        raise ValueError(f"Waypoint must have 3 values, got: {values}")
    return [float(values[0]), float(values[1]), float(values[2])]


def _normalize_trajectories(
    trajectories: Dict[str, Iterable[Sequence[float]]] | list,
) -> dict[str, list[list[float]]]:
    if isinstance(trajectories, dict):
        return {
            str(drone_id): [_point_to_list(p) for p in path]
            for drone_id, path in trajectories.items()
        }

    return {
        f"drone_{i}": [_point_to_list(p) for p in path]
        for i, path in enumerate(trajectories)
    }


def export_trajectories_to_json(
    trajectories: Dict[str, Iterable[Sequence[float]]] | list,
    output_path: str | Path = "results/trajectories.json",
) -> Path:
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    data = _normalize_trajectories(trajectories)

    with output_path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)

    print(f"[Webots export] Saved trajectories to: {output_path}")
    return output_path


def export_webots_world(
    trajectories: Dict[str, Iterable[Sequence[float]]] | list,
    output_path: str | Path = "worlds/sar_minimal.wbt",
) -> Path:
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    data = _normalize_trajectories(trajectories)

    if not data:
        raise ValueError("Cannot export Webots world without any drone trajectories.")

    colors = [
        "0.1 0.3 1",
        "1 0.25 0.1",
        "0.1 0.8 0.2",
        "0.8 0.1 0.8",
        "0.1 0.8 0.8",
        "0.9 0.9 0.1",
    ]

    drone_blocks: list[str] = []

    for i, (drone_name, trajectory) in enumerate(data.items()):
        if not trajectory:
            raise ValueError(f"Drone {drone_name} has an empty trajectory.")

        start_x, start_y, start_z = trajectory[0]
        color = colors[i % len(colors)]

        drone_blocks.append(
            f"""
DEF DRONE_{i} Robot {{
  translation {start_x:.3f} {start_y:.3f} {start_z:.3f}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor {color}
        roughness 0.4
      }}
      geometry Sphere {{
        radius 2
      }}
    }}
  ]
  name "{drone_name}"
  boundingObject Sphere {{
    radius 2
  }}
  controller "drone_controller"
  supervisor TRUE
}}
"""
        )

    world_content = f"""#VRML_SIM R2025a utf8

EXTERNPROTO "../protos/GeneratedTerrain.proto"

WorldInfo {{
  basicTimeStep 32
}}

Viewpoint {{
  orientation -0.0709 0.996 0.0587 0.597
  position -183 -6.78 141
  followType "None"
}}

Background {{
  skyColor [
    0.7 0.85 1
  ]
}}

DirectionalLight {{
  direction -0.5 -1 -0.5
  intensity 2
}}

DEF TERRAIN GeneratedTerrain {{
}}

DEF TARGET_PERSON Solid {{
  translation 0 0 0
  name "target_person"
  children [
    Transform {{
      children [
        Shape {{
          appearance DEF TARGET_APPEARANCE PBRAppearance {{
            baseColor 1 0.85 0
            roughness 0.4
            transparency 1
          }}
          geometry Cylinder {{
            radius 1.2
            height 2.0
          }}
        }}
      ]
    }}
  ]
}}

{''.join(drone_blocks)}

DEF TARGET_MANAGER Robot {{
  translation 0 0 0
  supervisor TRUE
  controller "target_controller"
  name "target_manager"
  children [
  ]
}}

DEF MISSION_MANAGER Robot {{
  translation 0 0 0
  supervisor TRUE
  controller "mission_manager_controller"
  name "mission_manager"
  children [
  ]
}}
"""

    output_path.write_text(world_content, encoding="utf-8")

    print(f"[Webots export] Saved Webots world to: {output_path}")
    print(f"[Webots export] Exported {len(data)} drones.")

    return output_path