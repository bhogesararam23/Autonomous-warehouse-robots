# Warehouse A* Simulation — Web UI

An interactive React and TypeScript demo of a robot planning a safe route through a warehouse with fixed shelves and repeating moving-obstacle routes. It runs entirely in the browser and does not connect to or control physical robots.

## Requirements and setup

Use Node.js **20.19+** or **22.12+** with npm. From the repository root:

```bash
cd ui
npm install
npm run dev
```

Open the local URL printed by Vite (usually `http://localhost:5173/`). No separate backend is required.

## Scenarios and simulation

The UI displays a 24-column by 14-row warehouse. The robot starts at `(1, 1)` and plans to reach `(22, 12)` around three fixed shelf blocks. Select a scenario from **Warehouse scenario**:

- **Standard warehouse** (default): the original forklift and mobile-robot routes.
- **Peak cross-aisle traffic**: adds a pallet cart patrolling the central cross-aisle.

Scenario IDs, names, descriptions, and obstacle routes are defined once in the repository-root [`scenarios.json`](../scenarios.json) and shared by the UI and Python simulator. Changing the selection stops playback, returns the timeline to the start, and calculates a new plan.

The time-aware A* planner can move one cell in a cardinal direction or wait in place. It avoids both same-cell collisions and head-on edge swaps with moving obstacles. It only accepts a goal arrival if the goal stays safe for one complete joint cycle of all obstacle routes. The panel shows the current time step, path length, expanded-node count, status, and a map legend.

The planner function also accepts custom grid dimensions and planning horizon through its options (`gridWidth`, `gridHeight`, and `maxPlanningTime`). The displayed app uses the default 24×14 map and 120-step limit; there are no UI fields for changing those values. The TypeScript tests cover a larger custom grid and horizon.

## Playback controls

- **Play / Pause** starts or pauses animation. Starting from the end restarts at the beginning; **Reset** stops playback and returns to step zero.
- Use the **Time step** slider to seek. Seeking pauses playback.
- **Step back** and **Step forward** move one time step at a time.
- **Playback speed** offers **0.5×**, **1×**, and **2×**.
- Keyboard shortcuts work when focus is outside interactive controls and editable text: **Space** toggles play/pause, **←** steps backward, and **→** steps forward. Shortcuts are ignored while typing, when an interactive control has focus, or for modified/repeated key presses.

## Tests and checks

From this `ui/` directory, run:

```bash
npm run test:ts
npm run lint
npm run build
```

`test:ts` runs the TypeScript simulation regression tests, including both scenarios, safe-goal and collision cases, large-map planning, priority-queue ordering, and keyboard-shortcut guards. `build` runs the TypeScript project build and creates the production UI in `dist/`.

The Python tests are run separately from the repository root:

```bash
cd ..
python3 -m unittest discover -v
```

That command discovers the Python unit tests in `test_warehouse.py`. The continuous-integration workflow also smoke-tests the Python CLI with both scenario IDs.
