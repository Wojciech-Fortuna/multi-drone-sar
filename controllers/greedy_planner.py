import numpy as np

from controllers.detection import can_detect


class GreedyPlanner:
    def __init__(
        self,
        terrain,
        probability_map,
        detection_radius=30.0,
        drone_altitude=25.0,
        v_max=5.0
    ):
        self.terrain = terrain
        self.probability_map = probability_map
        self.detection_radius = detection_radius
        self.drone_altitude = drone_altitude
        self.v_max = v_max

    def create_candidate_points(self, step=10.0):
        candidates = []

        x_values = np.arange(
            self.terrain.x_min,
            self.terrain.x_max + 0.5 * step,
            step
        )

        y_values = np.arange(
            self.terrain.y_min,
            self.terrain.y_max + 0.5 * step,
            step
        )

        for x in x_values:
            for y in y_values:
                z = self.terrain.get_height(x, y) + self.drone_altitude
                candidates.append(np.array([x, y, z], dtype=float))

        return candidates

    def compute_visible_cells(self, observation_point):
        visible_cells = []

        for idx, cell in enumerate(self.probability_map.cells):
            target_pos = np.array([cell[0], cell[1], cell[2] + 1.7], dtype=float)

            if can_detect(
                drone_pos=observation_point,
                target_pos=target_pos,
                terrain=self.terrain,
                detection_radius=self.detection_radius
            ):
                visible_cells.append(idx)

        return visible_cells

    def precompute_visibility_sets(self, candidates):
        visibility_sets = []

        print("Precomputing visibility sets...")

        for idx, candidate in enumerate(candidates):
            visible_cells = self.compute_visible_cells(candidate)
            visibility_sets.append(visible_cells)

            print(f"Candidate {idx + 1}/{len(candidates)}: "f"{len(visible_cells)} visible cells")

        return visibility_sets

    def plan_single_drone(
        self,
        start_pos,
        time_budget=120.0,
        candidate_step=10.0
    ):
        candidates = self.create_candidate_points(step=candidate_step)
        visibility_sets = self.precompute_visibility_sets(candidates)

        unobserved = set(range(len(self.probability_map.cells)))

        trajectory = [np.asarray(start_pos, dtype=float)]
        current_pos = np.asarray(start_pos, dtype=float)
        current_time = 0.0

        while current_time < time_budget and len(unobserved) > 0:
            best_candidate = None
            best_score = 0.0
            best_visible_cells = []
            best_travel_time = 0.0

            for candidate_idx, candidate in enumerate(candidates):
                distance = np.linalg.norm(candidate - current_pos)
                travel_time = distance / self.v_max

                if current_time + travel_time > time_budget:
                    continue

                visible_cells = visibility_sets[candidate_idx]

                newly_visible = [
                    cell_idx
                    for cell_idx in visible_cells
                    if cell_idx in unobserved
                ]

                gain = sum(
                    self.probability_map.probabilities[cell_idx]
                    for cell_idx in newly_visible
                )

                score = gain / (travel_time + 1e-6)

                if score > best_score:
                    best_score = score
                    best_candidate = candidate
                    best_visible_cells = newly_visible
                    best_travel_time = travel_time

            if best_candidate is None:
                print(
                    f"[GreedyPlanner] Stopped: "
                    f"remaining_cells={len(unobserved)}, "
                    f"drone_times={current_time}"
                )
                break

            if best_score <= 0:
                print(
                    f"[GreedyPlanner] Stopped: "
                    f"no additional coverage possible, "
                    f"remaining_cells={len(unobserved)}"
                )
                break

            trajectory.append(best_candidate)

            current_pos = best_candidate
            current_time += best_travel_time

            for cell_idx in best_visible_cells:
                unobserved.remove(cell_idx)

        return {
            "trajectory": trajectory,
            "time": current_time,
            "observed_cells": len(self.probability_map.cells) - len(unobserved),
            "total_cells": len(self.probability_map.cells),
            "coverage": 1.0 - len(unobserved) / len(self.probability_map.cells)
        }

    def plan_multi_drone(
        self,
        start_positions,
        time_budget=120.0,
        candidate_step=10.0
    ):
        candidates = self.create_candidate_points(step=candidate_step)
        visibility_sets = self.precompute_visibility_sets(candidates)

        num_drones = len(start_positions)

        trajectories = [[np.asarray(pos, dtype=float)] for pos in start_positions]

        current_positions = [np.asarray(pos, dtype=float) for pos in start_positions]

        current_times = [0.0 for _ in range(num_drones)]

        unobserved = set(range(len(self.probability_map.cells)))

        while len(unobserved) > 0:
            best_drone = None
            best_candidate = None
            best_visible_cells = []
            best_score = 0.0
            best_travel_time = 0.0

            for drone_idx in range(num_drones):
                current_pos = current_positions[drone_idx]
                current_time = current_times[drone_idx]

                if current_time >= time_budget:
                    continue

                for candidate_idx, candidate in enumerate(candidates):
                    distance = np.linalg.norm(candidate - current_pos)
                    travel_time = distance / self.v_max

                    if current_time + travel_time > time_budget:
                        continue

                    visible_cells = visibility_sets[candidate_idx]

                    newly_visible = [
                        cell_idx
                        for cell_idx in visible_cells
                        if cell_idx in unobserved
                    ]

                    gain = sum(
                        self.probability_map.probabilities[cell_idx]
                        for cell_idx in newly_visible
                    )

                    score = gain / (travel_time + 1e-6)

                    if score > best_score:
                        best_score = score
                        best_drone = drone_idx
                        best_candidate = candidate
                        best_visible_cells = newly_visible
                        best_travel_time = travel_time

            if best_candidate is None:
                print(
                    f"[GreedyPlanner] Stopped: "
                    f"remaining_cells={len(unobserved)}, "
                    f"drone_times={current_times}"
                )
                break

            if best_score <= 0:
                print(
                    f"[GreedyPlanner] Stopped: "
                    f"no additional coverage possible, "
                    f"remaining_cells={len(unobserved)}"
                )
                break

            trajectories[best_drone].append(best_candidate)

            current_positions[best_drone] = best_candidate
            current_times[best_drone] += best_travel_time

            for cell_idx in best_visible_cells:
                unobserved.remove(cell_idx)

            print(f"Drone {best_drone} -> "f"observed {len(best_visible_cells)} new cells")

        return {
            "trajectories": trajectories,
            "times": current_times,
            "coverage": (1.0 - len(unobserved) / len(self.probability_map.cells))
        }
