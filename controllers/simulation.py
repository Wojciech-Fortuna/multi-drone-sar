from __future__ import annotations

import json
import os

import numpy as np

from controllers.detection import can_detect, target_position_on_terrain


class SimulationManager:
    def __init__(
        self,
        terrain,
        trajectories,
        target_xy=None,
        detection_radius=35.0,
        time_budget=120.0,
        time_step=1.0,
        v_max=5.0,
        remaining_probability_threshold=0.01,
        probability_map=None,
        mission_status_path="results/mission_status.json",
        target_status_path="results/target_status.json"
    ):
        self.terrain = terrain
        self.trajectories = trajectories
        self.target_xy = target_xy
        self.detection_radius = detection_radius
        self.time_budget = time_budget
        self.time_step = time_step
        self.v_max = v_max
        self.remaining_probability_threshold = remaining_probability_threshold
        self.probability_map = probability_map

        self.mission_status_path = mission_status_path
        self.target_status_path = target_status_path

        self.current_time = 0.0
        self.target_found = False
        self.stop_reason = None

        self.drone_positions = [np.asarray(trajectory[0], dtype=float) for trajectory in trajectories]

        self.current_waypoint_indices = [1 for _ in trajectories]

        if target_xy is not None:
            self.target_pos = target_position_on_terrain(x=target_xy[0], y=target_xy[1], terrain=terrain)
        else:
            self.target_pos = None

        self.observed_cells = set()

        self.export_mission_status()
        self.export_target_status()

    @staticmethod
    def load_trajectories_from_json(path):
        with open(path, "r", encoding="utf-8") as file:
            data = json.load(file)

        trajectories = []

        for drone_name in sorted(data.keys()):
            trajectory = [np.asarray(point, dtype=float) for point in data[drone_name]]

            trajectories.append(trajectory)

        return trajectories

    def export_mission_status(self):
        directory = os.path.dirname(self.mission_status_path)

        if directory:
            os.makedirs(directory, exist_ok=True)

        status = {
            "mission_finished": self.stop_reason is not None,
            "target_found": self.target_found,
            "stop_reason": self.stop_reason,
            "mission_time": self.current_time,
            "remaining_probability": self.remaining_probability(),
            "observed_cells": len(self.observed_cells)
        }

        with open(self.mission_status_path, "w", encoding="utf-8") as file:
            json.dump(status, file, indent=4)

    def export_target_status(self):
        directory = os.path.dirname(self.target_status_path)

        if directory:
            os.makedirs(directory, exist_ok=True)

        status = {
            "target_xyz": None,
            "visible_in_webots": False
        }

        if self.target_xy is not None:
            status = {
                "target_xyz": [
                    float(self.target_pos[0]),
                    float(self.target_pos[1]),
                    float(self.target_pos[2])
                ],
                "visible_in_webots": True
            }

        with open(self.target_status_path, "w", encoding="utf-8") as file:
            json.dump(status, file, indent=4)

    def move_drone_towards_waypoint(self, drone_idx):
        trajectory = self.trajectories[drone_idx]
        waypoint_idx = self.current_waypoint_indices[drone_idx]

        if waypoint_idx >= len(trajectory):
            return

        current_pos = self.drone_positions[drone_idx]

        target_waypoint = np.asarray(trajectory[waypoint_idx], dtype=float)

        direction = target_waypoint - current_pos
        distance = np.linalg.norm(direction)

        if distance < 1e-6:
            self.current_waypoint_indices[drone_idx] += 1
            return

        max_step_distance = self.v_max * self.time_step

        if distance <= max_step_distance:
            self.drone_positions[drone_idx] = target_waypoint
            self.current_waypoint_indices[drone_idx] += 1
        else:
            direction = direction / distance

            self.drone_positions[drone_idx] = (current_pos + direction * max_step_distance)

    def update_observed_cells(self):
        if self.probability_map is None:
            return

        for cell_idx, cell in enumerate(self.probability_map.cells):
            if cell_idx in self.observed_cells:
                continue

            target_pos = np.array([cell[0], cell[1], cell[2] + 1.7], dtype=float)

            for drone_pos in self.drone_positions:
                if can_detect(
                    drone_pos=drone_pos,
                    target_pos=target_pos,
                    terrain=self.terrain,
                    detection_radius=self.detection_radius
                ):
                    self.observed_cells.add(cell_idx)
                    break

    def remaining_probability(self):
        if self.probability_map is None:
            return 1.0

        remaining = 0.0

        for cell_idx, probability in enumerate(self.probability_map.probabilities):
            if cell_idx not in self.observed_cells:
                remaining += probability

        return remaining

    def check_target_detection(self):
        if self.target_pos is None:
            return False

        for drone_pos in self.drone_positions:
            if can_detect(
                drone_pos=drone_pos,
                target_pos=self.target_pos,
                terrain=self.terrain,
                detection_radius=self.detection_radius
            ):
                return True

        return False

    def all_trajectories_finished(self):
        for drone_idx, trajectory in enumerate(self.trajectories):
            if self.current_waypoint_indices[drone_idx] < len(trajectory):
                return False

        return True

    def step(self):
        for drone_idx in range(len(self.trajectories)):
            self.move_drone_towards_waypoint(drone_idx)

        self.update_observed_cells()

        if self.check_target_detection():
            self.target_found = True
            self.stop_reason = "target_found"

            self.export_mission_status()

            return False

        if self.current_time >= self.time_budget:
            self.stop_reason = "time_budget_exceeded"

            self.export_mission_status()

            return False

        if self.remaining_probability() < self.remaining_probability_threshold:
            self.stop_reason = "no_useful_search_remaining"

            self.export_mission_status()

            return False

        if self.all_trajectories_finished():
            self.stop_reason = "trajectories_finished"

            self.export_mission_status()

            return False

        self.current_time += self.time_step

        self.export_mission_status()

        return True

    def run(self):
        running = True

        while running:
            running = self.step()

        return {
            "target_found": self.target_found,
            "stop_reason": self.stop_reason,
            "mission_time": self.current_time,
            "observed_cells": len(self.observed_cells),
            "remaining_probability": self.remaining_probability(),
            "final_drone_positions": self.drone_positions
        }
