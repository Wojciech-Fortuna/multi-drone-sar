import numpy as np

CONFIG = {
    "name": "basic_3_drones",

    "planner": "greedy",

    "terrain": "default_hills",
    "probability_map": "hotspot_10_10",

    "map": {
        "x_min": -30,
        "x_max": 30,
        "y_min": -30,
        "y_max": 30,
        "resolution": 10.0,
    },

    "start_positions": [
        np.array([0.0, 0.0, 30.0]),
        np.array([20.0, 0.0, 30.0]),
        np.array([-20.0, 0.0, 30.0]),
    ],

    "detection_radius": 25.0,
    "drone_altitude": 25.0,
    "v_max": 5.0,
    "time_budget": 60.0,
    "candidate_step": 10.0,

    "target_xy": [10.0, 20.0],
    "target_height_above_ground": 2.0,
    "show_target": True,
}