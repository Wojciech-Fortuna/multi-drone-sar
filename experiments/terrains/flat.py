from controllers.terrain import Terrain


def flat_height(x, y):
    return 0.0


def create_terrain(map_config):
    return Terrain(
        x_min=map_config["x_min"],
        x_max=map_config["x_max"],
        y_min=map_config["y_min"],
        y_max=map_config["y_max"],
        resolution=map_config["resolution"],
        height_function=flat_height,
    )