CONFIG = {
    "name": "terrain_difficulty_study",

    "planner": ["greedy", "ga"],

    "start_positions_mode": ["circle"],

    "num_drones": [3],
    "detection_radius": [45.0],
    "time_budget": [120.0],

    "drone_altitude": 35.0,
    "v_max": 5.0,
    "candidate_step": 40.0,

    "terrain": [
        "flat",
        "default_hills",
        "large_hills",
        "steep_large_mountains",
    ],
    "probability_map": "uniform",

    "map": {
        "x_min": -150,
        "x_max": 150,
        "y_min": -150,
        "y_max": 150,
        "resolution": 30.0,
    },

    "ga": {
        "population_size": 50,
        "chromosome_length": 14,
        "mutation_rate": 0.2,
        "crossover_rate": 0.8,
        "elite_size": 2,
        "generations": 50,
    },
}
