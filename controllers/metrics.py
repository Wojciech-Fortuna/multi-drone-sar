import numpy as np

from controllers.detection import can_detect


def trajectory_distance(trajectory):
    trajectory = np.asarray(trajectory, dtype=float)

    if len(trajectory) < 2:
        return 0.0

    distance = 0.0

    for i in range(1, len(trajectory)):
        distance += np.linalg.norm(trajectory[i] - trajectory[i - 1])

    return distance


def multi_trajectory_distance(trajectories):
    return sum(
        trajectory_distance(trajectory)
        for trajectory in trajectories
    )


def count_waypoints(trajectory):
    return len(trajectory)


def count_multi_waypoints(trajectories):
    return sum(len(trajectory) for trajectory in trajectories)


def trajectory_arrival_times(trajectory, v_max):
    trajectory = np.asarray(trajectory, dtype=float)

    if len(trajectory) == 0:
        return []

    arrival_times = [0.0]
    current_time = 0.0

    for i in range(1, len(trajectory)):
        segment_distance = np.linalg.norm(
            trajectory[i] - trajectory[i - 1]
        )
        current_time += segment_distance / v_max
        arrival_times.append(current_time)

    return arrival_times


def expected_target_detection_time(
    trajectories,
    terrain,
    probability_map,
    detection_radius,
    v_max,
    time_budget,
    target_height_offset=1.7,
):
    detection_times = np.full(
        len(probability_map.cells),
        np.inf,
        dtype=float,
    )

    for trajectory in trajectories:
        arrival_times = trajectory_arrival_times(
            trajectory,
            v_max,
        )

        for drone_pos, arrival_time in zip(
            trajectory,
            arrival_times,
        ):
            for cell_idx, cell in enumerate(
                probability_map.cells
            ):
                if arrival_time >= detection_times[cell_idx]:
                    continue

                target_pos = np.array(
                    [
                        cell[0],
                        cell[1],
                        cell[2] + target_height_offset,
                    ],
                    dtype=float,
                )

                if can_detect(
                    drone_pos=drone_pos,
                    target_pos=target_pos,
                    terrain=terrain,
                    detection_radius=detection_radius,
                ):
                    detection_times[cell_idx] = arrival_time

    capped_detection_times = np.where(
        np.isfinite(detection_times),
        detection_times,
        float(time_budget),
    )

    return float(
        np.sum(
            probability_map.probabilities
            * capped_detection_times
        )
    )
