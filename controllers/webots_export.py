from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, Iterable, Sequence, Any


def _point_to_list(point: Any) -> list[float]:
    values = list(point)
    if len(values) != 3:
        raise ValueError(f"Waypoint must have 3 values, got: {values}")
    return [float(values[0]), float(values[1]), float(values[2])]


def export_trajectories_to_json(
    trajectories: Dict[str, Iterable[Sequence[float]]] | list,
    output_path: str | Path = "results/trajectories.json",
) -> Path:

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    if isinstance(trajectories, dict):
        data = {
            str(drone_id): [_point_to_list(p) for p in path]
            for drone_id, path in trajectories.items()
        }
    else:
        data = {
            f"drone_{i}": [_point_to_list(p) for p in path]
            for i, path in enumerate(trajectories)
        }

    with output_path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)

    print(f"[Webots export] Saved trajectories to: {output_path}")
    return output_path
