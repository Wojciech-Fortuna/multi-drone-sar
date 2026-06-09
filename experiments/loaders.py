from __future__ import annotations

import importlib


def load_terrain(config: dict):
    terrain_name = config["terrain"]

    module = importlib.import_module(
        f"experiments.terrains.{terrain_name}"
    )

    return module.create_terrain(config["map"])


def load_probability_map(config: dict, terrain):
    probability_map_name = config["probability_map"]

    module = importlib.import_module(
        f"experiments.probability_maps.{probability_map_name}"
    )

    return module.create_probability_map(terrain)