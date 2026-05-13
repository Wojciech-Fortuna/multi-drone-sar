import numpy as np

from controllers.visibility import is_visible


def euclidean_distance(a, b):
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)

    return np.linalg.norm(a - b)


def can_detect(drone_pos, target_pos, terrain, detection_radius=30.0):
    distance = euclidean_distance(drone_pos, target_pos)

    if distance > detection_radius:
        return False

    if not is_visible(drone_pos, target_pos, terrain):
        return False

    return True


def target_position_on_terrain(x, y, terrain, height_offset=1.7):
    z = terrain.get_height(x, y) + height_offset

    return np.array([x, y, z], dtype=float)
