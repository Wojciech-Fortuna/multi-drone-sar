import numpy as np

def is_visible(drone_pos, target_pos, terrain, samples=50, safety_margin=0.5):
    drone_pos = np.asarray(drone_pos, dtype=float)
    target_pos = np.asarray(target_pos, dtype=float)

    for alpha in np.linspace(0, 1, samples)[1:-1]:
        point = (1 - alpha) * drone_pos + alpha * target_pos

        x, y, z_line = point
        z_terrain = terrain.get_height(x, y)

        if z_terrain + safety_margin > z_line:
            return False

    return True
