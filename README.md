# Multi-Drone SAR Simulation

A multi-drone Search and Rescue (SAR) simulation project with 3D visualization in Webots.

## Requirements

- Python 3.11+
- Webots
- NumPy
- Matplotlib

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

## Webots

The following tests generate Webots simulation data:

- `test_multi_greedy.py`
- `test_multi_ga.py`
- `test_simulation.py`

To open the Webots world:

```text
File → Open World → worlds/sar_minimal.wbt
```

After running a Webots-related test:

```text
Webots → File → Reload World
```
