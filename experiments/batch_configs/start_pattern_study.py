CONFIG = {
    "name": "start_pattern_study",

    "planner": ["greedy", "ga"],

    "start_positions_mode": ["circle", "line"],

    "num_drones": [2, 3, 4, 5],
    "detection_radius": [25.0],
    "time_budget": [60.0],

    "drone_altitude": 25.0,
    "v_max": 5.0,
    "candidate_step": 20.0,

    "terrain": "default_hills",
    "probability_map": "hotspot_10_10",

    "map": {
        "x_min": -30,
        "x_max": 30,
        "y_min": -30,
        "y_max": 30,
        "resolution": 10.0,
    },

    "ga": {
        "population_size": 40,
        "chromosome_length": 12,
        "mutation_rate": 0.2,
        "crossover_rate": 0.8,
        "elite_size": 2,
        "generations": 40,
    },
}
