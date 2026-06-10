import numpy as np

CONFIG = {
    "name": "large_terrain_test",

    "planner": "greedy",

    "terrain": "steep_large_mountains",
    "probability_map": "uniform",

    "map": {
        "x_min": -150,
        "x_max": 150,
        "y_min": -150,
        "y_max": 150,
        "resolution": 10.0,
    },

    "start_positions": [
        np.array([-120.0, -120.0, 30.0]),
        np.array([0.0, -120.0, 30.0]),
        np.array([120.0, -120.0, 30.0]),
    ],

    "detection_radius": 50.0,
    "drone_altitude": 35.0,
    "v_max": 5.0,
    "time_budget": 480.0,
    "candidate_step": 20.0,

    "target_xy": [100.0, 100.0],
    "target_height_above_ground": 2.0,
    "show_target": True,
}