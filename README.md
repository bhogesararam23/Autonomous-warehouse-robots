# Warehouse A* Simulation

A browser demo and Python command-line simulator for time-aware warehouse path planning. The simulated robot navigates around fixed shelves and predictable moving obstacles; this project is an educational simulation and does not connect to or control physical robots.

## Requirements

- **Web UI:** Node.js 20.19+ or 22.12+, with npm.
- **Python CLI and tests:** Python 3.10+.

## Run the web UI

From the repository root:

```bash
cd ui
npm install
npm run dev
```

Open the local URL printed by Vite (usually `http://localhost:5173/`). The UI runs in the browser; no separate backend is needed. See [`ui/README.md`](ui/README.md) for UI controls and frontend checks.

## Run the Python simulator

From the repository root:

```bash
# Standard warehouse (the default)
python3 warehouse_astar_sim.py

# Peak cross-aisle traffic
python3 warehouse_astar_sim.py --scenario peak-traffic

# Skip the SVG artifact (useful for automated smoke tests)
python3 warehouse_astar_sim.py --no-svg
```

The CLI prints a planning summary and ASCII map. By default, a successful plan also writes `warehouse_path.svg` beside `warehouse_astar_sim.py`, replacing that output file on each successful run. Pass `--no-svg` to skip the visualization file. Use `python3 warehouse_astar_sim.py --help` to see the available CLI options.

## Scenarios and map

Both planners read the versioned [`scenarios.json`](scenarios.json) catalog. Its [`scenarios.schema.json`](scenarios.schema.json) defines field shapes, coordinate/time semantics, and structural limits; the TypeScript tests validate the checked-in catalog against it. Matching Python and TypeScript runtime validators additionally enforce cross-field invariants before planning, including in-grid endpoints/routes, non-overlapping shelves and movers, adjacent mover steps, a bounded map/horizon, and a bounded joint route period. Invalid catalogs fail before either planner runs.

| Scenario | CLI ID | Moving obstacles |
| --- | --- | --- |
| **Standard warehouse** (default) | `standard` | A forklift and another mobile robot following repeating routes. |
| **Peak cross-aisle traffic** | `peak-traffic` | The standard obstacles plus a pallet cart patrolling the central cross-aisle. |

The default warehouse is 24 columns by 14 rows. Coordinates use `(x, y)`, with `x` increasing left to right and `y` increasing top to bottom. The robot starts at `(1, 1)` and plans to reach `(22, 12)` around three fixed shelf blocks. These values now come from the shared catalog rather than duplicated planner constants.

## How planning works

The planner uses time-aware A*: each search state includes both a grid location and a time step. It can move one cell up, down, left, or right, or wait in place. Moving obstacle routes repeat cyclically. Candidate moves are rejected if they would cause either a same-cell collision or a head-on edge swap with a moving obstacle. A goal arrival is accepted only when the goal remains safe for a complete joint cycle of all moving-obstacle routes.

Both planner APIs support custom grid dimensions and a larger planning horizon. The current UI and Python CLI launch with the standard 24×14 map and the default 120-step planning limit; the CLI does not provide command-line flags for custom dimensions or horizon. The Python `astar` function accepts `grid_width`, `grid_height`, and `max_planning_time`; the TypeScript `astar` function accepts the corresponding `gridWidth`, `gridHeight`, and `maxPlanningTime` options.

For callers that need explicit failure handling, Python `plan_warehouse` and TypeScript `planWarehouse` return a discriminated result: `status: "success"` with the path and expanded-node count, or `status: "failure"` with a stable `code` (`invalid-input` or `no-safe-path`) and message. The low-level `astar` functions remain available and raise typed exceptions for compatibility.

The Python and TypeScript episode APIs use the same unsigned 32-bit xorshift seed rules (zero maps to `0x6d2b79f5`) and row-major safe-cell selection. Each replayable event stream records the seed and endpoints, every robot position by time step, and the final structured outcome. `python3 scripts/test_episode_replay.py` runs shared seeds through both implementations, repeats generation, and verifies replay reproduces the same event sequence.

The Python terminal map uses `S` for the start, `G` for the goal, `#` for shelves, `M` for a moving obstacle at time zero, `*` for the planned path, and `.` for open floor. After installing the UI dependencies, run `python3 scripts/test_planner_parity.py` from the repository root to compare both planners against shared deterministic fixtures, verify safe optimal path lengths and matching structured failure codes, and check that repeat runs are stable.

## Tests and CI checks

Run the discoverable Python unit tests from the repository root:

```bash
python3 -m unittest discover -v
```

The collision demos are named `collision_demo.py` and `collision_demo_after_arrival.py`, so unittest does not discover them by filename. The Python test suite runs both explicitly: one demonstrates an edge-swap collision, and the other demonstrates a collision after an unsafe arrival.

Run the UI test, lint, and production build from `ui/`:

```bash
cd ui
npm run test:ts
npm run lint
npm run build
```

CI runs Python tests and smoke-tests both scenarios on Python 3.10 and 3.12. It runs the TypeScript tests, cross-language planner parity and seeded episode replay fixtures, ESLint, and production build on Node.js 22, and has a separate ROS 2 Jazzy job that builds both workspace packages, runs their tests, and checks the Open-RMF assets. The UI regression tests also exercise a larger custom grid and horizon, collision avoidance, keyboard-shortcut focus guards, and stable priority-queue ordering. From a ROS-enabled environment, run `./scripts/test_ros2_workspace.sh` for the same ROS checks locally. The real headless Gazebo regressions are available in a separate [manual-only GitHub Actions workflow](docs/LIVE_SIMULATION_CI.md) and are not part of push/pull-request CI.

## ROS 2 / Gazebo simulation

The repository also contains a ROS 2 Jazzy Nav2 A* planner plugin for the Clearpath Husky A300 simulation, using the vendor-supported Gazebo Harmonic warehouse world. See [`docs/HUSKY_A300_SIMULATION.md`](docs/HUSKY_A300_SIMULATION.md) for the planner design, A300 specifications, setup steps, launch commands, and simulation-only safety boundary. After installing the documented dependencies, run `./scripts/test_ros2_workspace.sh` from the repository root to build both ROS packages, run their package tests, and validate the Open-RMF graph and fleet configuration. The existing Python and browser demonstrations remain available as before.

For a repeatable headless `NavigateToPose` run through the custom planner and the standard controller, use [`scripts/test_husky_nav2_e2e.sh`](scripts/test_husky_nav2_e2e.sh); the procedure and its collision-check limits are documented in the simulation guide.

The [manual live-simulation workflow](docs/LIVE_SIMULATION_CI.md) can run the real Nav2 and two-A300 Open-RMF regressions on demand, or the opt-in blocked-aisle, stale-status, stale-TF, and unavailable-Nav2 fault-injection suite, and preserve logs and result JSON as downloadable artifacts.

## Open-RMF A300 fleet

The simulation workspace now includes a two-robot Open-RMF fleet slice with namespaced A300s, RMF task dispatch and traffic scheduling, the existing Nav2 A* planner, and a shared-aisle navigation graph. See [`docs/OPEN_RMF_A300_FLEET.md`](docs/OPEN_RMF_A300_FLEET.md) for the architecture, coordinate-frame model, setup, and usage. With ROS 2 Jazzy and the workspace dependencies installed, [`scripts/test_husky_rmf_fleet.sh`](scripts/test_husky_rmf_fleet.sh) runs the headless two-A300 smoke; [`scripts/test_rmf_fleet_assets.sh`](scripts/test_rmf_fleet_assets.sh) validates the generated graph and RMF configuration.
