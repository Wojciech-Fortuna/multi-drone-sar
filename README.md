# Multi-Drone SAR Simulation

A multi-drone Search and Rescue (SAR) simulation project with 3D visualization in Webots.

## Requirements

- Python 3.11+
- Webots
- NumPy
- Matplotlib

---

## Running Tests

### Detection Model Test

```bash
python -m tests.test_model
```

### Greedy Planner

```bash
python -m tests.test_greedy
```

### Multi-Drone Greedy Planner (Webots)

```bash
python -m tests.test_multi_greedy
```

### Genetic Algorithm

```bash
python -m tests.test_ga
```

### Multi-Drone Genetic Algorithm (Webots)

```bash
python -m tests.test_multi_ga
```

### Full Simulation with Target Detection (Webots)

```bash
python -m tests.test_simulation
```

---

## Experiments

The `experiments` package provides a configurable framework for running SAR scenarios and batch experiments.

### Running a Scenario

Available scenarios are defined in:

```text
experiments/scenarios/
```

Run a scenario with:

```bash
python -m experiments.run_webots_scenario SCENARIO_NAME
```

Example:

```bash
python -m experiments.run_webots_scenario basic_3_drones
```

Running a scenario generates a Webots world containing:

- generated terrain,
- drone trajectories,
- mission target (optional).

The generated world can be opened in Webots for visualization and simulation.

---

### Running Batch Experiments

Batch configurations are defined in:

```text
experiments/batch_configs/
```

Run a batch experiment with:

```bash
python -m experiments.run_batch_experiments CONFIG_NAME
```

Example:

```bash
python -m experiments.run_batch_experiments detection_radius_study
```

Results are saved to:

```text
results/experiments/
```

---

## Experiment Architecture

### Scenarios

Scenario configurations are stored in:

```text
experiments/scenarios/
```

A scenario defines:

- planner type,
- terrain,
- probability map,
- map geometry,
- drone start positions,
- mission parameters,
- target location.

---

### Terrains

Terrain generators are stored in:

```text
experiments/terrains/
```

Each module must implement:

```python
create_terrain(map_config)
```

---

### Probability Maps

Probability map generators are stored in:

```text
experiments/probability_maps/
```

Each module must implement:

```python
create_probability_map(terrain)
```

---

### Start Position Generators

Drone deployment patterns are stored in:

```text
experiments/start_positions/
```

Each module must implement:

```python
generate(num_drones, altitude)
```

---

### Planner Runners

Planner-specific experiment runners are stored in:

```text
experiments/planners/
```

---

## Webots

The following tests generate Webots simulation data:

- `test_multi_greedy.py`
- `test_multi_ga.py`
- `test_simulation.py`

Scenarios generated through:

```bash
python -m experiments.run_webots_scenario ...
```

also generate Webots worlds.

To open the Webots world:

```text
File → Open World → worlds/sar_minimal.wbt
```

After generating a new world:

```text
Webots → File → Reload World
```