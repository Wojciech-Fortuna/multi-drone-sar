import numpy as np

from controllers.detection import can_detect
from controllers.visibility import is_visible


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
            target_pos = np.array(
                [cell[0], cell[1], cell[2] + 1.7],
                dtype=float
            )

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

            print(
                f"Candidate {idx + 1}/{len(candidates)}: "
                f"{len(visible_cells)} visible cells"
            )

        return visibility_sets

    def _travel_time(self, from_pos, to_pos):
        distance = np.linalg.norm(to_pos - from_pos)
        return distance / self.v_max

    def _get_newly_visible_cells(self, visible_cells, unobserved):
        return [
            cell_idx
            for cell_idx in visible_cells
            if cell_idx in unobserved
        ]

    def _coverage_gain(self, newly_visible):
        return sum(
            self.probability_map.probabilities[cell_idx]
            for cell_idx in newly_visible
        )

    def _xy_key(self, pos):
        return (round(float(pos[0]), 6), round(float(pos[1]), 6))

    def _observation_point_for_cell(self, cell_idx):
        cell = self.probability_map.cells[cell_idx]

        x = float(cell[0])
        y = float(cell[1])
        z = self.terrain.get_height(x, y) + self.drone_altitude

        return np.array([x, y, z], dtype=float)

    def _can_fly_straight(self, from_pos, to_pos):
        return is_visible(
            drone_pos=from_pos,
            target_pos=to_pos,
            terrain=self.terrain
        )

    def _find_best_observation_move(
        self,
        current_pos,
        current_time,
        time_budget,
        candidates,
        visibility_sets,
        unobserved,
        visited_observation_xy
    ):
        best_candidate = None
        best_visible_cells = []
        best_score = 0.0
        best_travel_time = 0.0

        for candidate_idx, candidate_original in enumerate(candidates):
            candidate = np.asarray(candidate_original, dtype=float).copy()

            xy_key = self._xy_key(candidate)

            if xy_key in visited_observation_xy:
                continue

            travel_time = self._travel_time(current_pos, candidate)

            if travel_time <= 1e-9:
                continue

            if current_time + travel_time > time_budget:
                continue

            if not self._can_fly_straight(current_pos, candidate):
                continue

            newly_visible = self._get_newly_visible_cells(
                visibility_sets[candidate_idx],
                unobserved
            )

            if not newly_visible:
                continue

            gain = self._coverage_gain(newly_visible)
            score = gain / (travel_time + 1e-6)

            if score > best_score:
                best_score = score
                best_candidate = candidate
                best_visible_cells = newly_visible
                best_travel_time = travel_time

        return {
            "candidate": best_candidate,
            "visible_cells": best_visible_cells,
            "travel_time": best_travel_time,
            "mode": "observe"
        }

    def _find_nearest_unobserved_cell(
        self,
        current_pos,
        unobserved,
        visited_navigation_cells
    ):
        best_cell_idx = None
        best_distance = np.inf

        for cell_idx in unobserved:
            if cell_idx in visited_navigation_cells:
                continue

            target_pos = self._observation_point_for_cell(cell_idx)

            distance = np.linalg.norm(target_pos[:2] - current_pos[:2])

            if distance <= 1e-9:
                continue

            if distance < best_distance:
                best_distance = distance
                best_cell_idx = cell_idx

        return best_cell_idx

    def _find_navigation_move(
        self,
        current_pos,
        current_time,
        time_budget,
        unobserved,
        visited_navigation_cells,
        climb_step=20.0,
        max_extra_altitude=200.0
    ):
        if not unobserved:
            return None

        target_cell_idx = self._find_nearest_unobserved_cell(
            current_pos=current_pos,
            unobserved=unobserved,
            visited_navigation_cells=visited_navigation_cells
        )

        if target_cell_idx is None:
            return None

        observation_pos = self._observation_point_for_cell(target_cell_idx)

        direct_time = self._travel_time(current_pos, observation_pos)

        if (
            direct_time > 1e-9
            and current_time + direct_time <= time_budget
            and self._can_fly_straight(current_pos, observation_pos)
        ):
            return {
                "waypoints": [observation_pos],
                "final_pos": observation_pos,
                "visible_cells": [],
                "travel_time": direct_time,
                "mode": "navigate",
                "target_cell_idx": target_cell_idx
            }

        extra_altitude = climb_step

        while extra_altitude <= max_extra_altitude:
            high_pos = observation_pos.copy()
            high_pos[2] = observation_pos[2] + extra_altitude

            time_to_high = self._travel_time(current_pos, high_pos)
            time_down = self._travel_time(high_pos, observation_pos)
            total_time = time_to_high + time_down

            if current_time + total_time > time_budget:
                return None

            if self._can_fly_straight(current_pos, high_pos):
                return {
                    "waypoints": [high_pos, observation_pos],
                    "final_pos": observation_pos,
                    "visible_cells": [],
                    "travel_time": total_time,
                    "mode": "navigate_high_then_descend",
                    "target_cell_idx": target_cell_idx
                }

            extra_altitude += climb_step

        return None

    def _mark_observed_from_current_position(
        self,
        current_pos,
        unobserved
    ):
        visible_cells = self.compute_visible_cells(current_pos)

        newly_visible = self._get_newly_visible_cells(
            visible_cells,
            unobserved
        )

        for cell_idx in newly_visible:
            unobserved.remove(cell_idx)

        return newly_visible

    def _apply_move(
        self,
        trajectory,
        current_pos,
        current_time,
        move,
        unobserved,
        visited_observation_xy,
        visited_navigation_cells=None
    ):
        if move["mode"] == "observe":
            waypoints = [move["candidate"]]
            final_pos = move["candidate"]
        else:
            waypoints = move["waypoints"]
            final_pos = move["final_pos"]

        for waypoint in waypoints:
            trajectory.append(waypoint)

        current_pos = final_pos
        current_time += move["travel_time"]

        if move["mode"] == "observe":
            visited_observation_xy.add(self._xy_key(final_pos))
            newly_visible = move["visible_cells"]

            for cell_idx in newly_visible:
                if cell_idx in unobserved:
                    unobserved.remove(cell_idx)
        else:
            if visited_navigation_cells is not None:
                visited_navigation_cells.add(move["target_cell_idx"])

            newly_visible = self._mark_observed_from_current_position(
                current_pos,
                unobserved
            )

        return current_pos, current_time, newly_visible

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

        visited_observation_xy = set()
        visited_navigation_cells = set()

        start_visible = self._mark_observed_from_current_position(
            current_pos,
            unobserved
        )

        if start_visible:
            print(
                f"Drone 0 -> observed {len(start_visible)} cells from start, "
                f"remaining={len(unobserved)}"
            )

        while len(unobserved) > 0 and current_time < time_budget:
            move = self._find_best_observation_move(
                current_pos=current_pos,
                current_time=current_time,
                time_budget=time_budget,
                candidates=candidates,
                visibility_sets=visibility_sets,
                unobserved=unobserved,
                visited_observation_xy=visited_observation_xy
            )

            if move["candidate"] is None:
                move = self._find_navigation_move(
                    current_pos=current_pos,
                    current_time=current_time,
                    time_budget=time_budget,
                    unobserved=unobserved,
                    visited_navigation_cells=visited_navigation_cells
                )

            if move is None:
                print(
                    f"[GreedyPlanner] Stopped: "
                    f"no reachable observation/navigation move, "
                    f"remaining_cells={len(unobserved)}, "
                    f"drone_time={current_time}"
                )
                break

            current_pos, current_time, newly_visible = self._apply_move(
                trajectory=trajectory,
                current_pos=current_pos,
                current_time=current_time,
                move=move,
                unobserved=unobserved,
                visited_observation_xy=visited_observation_xy,
                visited_navigation_cells=visited_navigation_cells
            )

            print(
                f"Drone 0 -> {move['mode']}, "
                f"observed {len(newly_visible)} cells, "
                f"time={current_time:.2f}, "
                f"remaining={len(unobserved)}"
            )

        if len(unobserved) == 0:
            print("[GreedyPlanner] Finished: all cells covered.")

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

        trajectories = [
            [np.asarray(pos, dtype=float)]
            for pos in start_positions
        ]

        current_positions = [
            np.asarray(pos, dtype=float)
            for pos in start_positions
        ]

        current_times = [0.0 for _ in range(num_drones)]

        visited_observation_xy = [
            set()
            for _ in range(num_drones)
        ]

        # Wspólne dla wszystkich dronów:
        # jak jeden dron wybierze komórkę jako cel nawigacyjny,
        # pozostałe nie będą leciały do tego samego miejsca.
        visited_navigation_cells = set()

        unobserved = set(range(len(self.probability_map.cells)))

        for drone_idx in range(num_drones):
            start_visible = self._mark_observed_from_current_position(
                current_positions[drone_idx],
                unobserved
            )

            if start_visible:
                print(
                    f"Drone {drone_idx} -> observed "
                    f"{len(start_visible)} cells from start, "
                    f"remaining={len(unobserved)}"
                )

        round_idx = 0

        while len(unobserved) > 0:
            round_idx += 1
            any_action = False

            print(f"[GreedyPlanner] Round {round_idx}")

            for drone_idx in range(num_drones):
                current_pos = current_positions[drone_idx]
                current_time = current_times[drone_idx]

                if current_time >= time_budget:
                    continue

                move = self._find_best_observation_move(
                    current_pos=current_pos,
                    current_time=current_time,
                    time_budget=time_budget,
                    candidates=candidates,
                    visibility_sets=visibility_sets,
                    unobserved=unobserved,
                    visited_observation_xy=visited_observation_xy[drone_idx]
                )

                if move["candidate"] is None:
                    move = self._find_navigation_move(
                        current_pos=current_pos,
                        current_time=current_time,
                        time_budget=time_budget,
                        unobserved=unobserved,
                        visited_navigation_cells=visited_navigation_cells
                    )

                if move is None:
                    continue

                current_positions[drone_idx], current_times[drone_idx], newly_visible = (
                    self._apply_move(
                        trajectory=trajectories[drone_idx],
                        current_pos=current_pos,
                        current_time=current_time,
                        move=move,
                        unobserved=unobserved,
                        visited_observation_xy=visited_observation_xy[drone_idx],
                        visited_navigation_cells=visited_navigation_cells
                    )
                )

                any_action = True

                print(
                    f"Drone {drone_idx} -> {move['mode']}, "
                    f"observed {len(newly_visible)} cells, "
                    f"time={current_times[drone_idx]:.2f}, "
                    f"remaining={len(unobserved)}"
                )

                if len(unobserved) == 0:
                    break

            if len(unobserved) == 0:
                print("[GreedyPlanner] Finished: all cells covered.")
                break

            if not any_action:
                print(
                    f"[GreedyPlanner] Stopped: "
                    f"no drone can observe or navigate within time budget, "
                    f"remaining_cells={len(unobserved)}, "
                    f"drone_times={current_times}"
                )
                break

            if all(current_time >= time_budget for current_time in current_times):
                print(
                    f"[GreedyPlanner] Stopped: "
                    f"all drones reached time budget, "
                    f"remaining_cells={len(unobserved)}, "
                    f"drone_times={current_times}"
                )
                break

        return {
            "trajectories": trajectories,
            "times": current_times,
            "coverage": 1.0 - len(unobserved) / len(self.probability_map.cells)
        }