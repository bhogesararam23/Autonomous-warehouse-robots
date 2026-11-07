# Two-A300 Open-RMF Fleet Slice

## What it adds

This simulation-only slice uses Open-RMF's traffic schedule, blockade moderator, mutex-group supervisor, task dispatcher, and Easy Full Control fleet adapter to coordinate two Clearpath Husky A300 robots. Each robot remains namespaced and keeps the existing `warehouse_astar_planner::AStarPlanner` plus Nav2 MPPI at the robot-navigation layer, with a tighter supported Nav2 goal checker for this integration test. RMF owns task dispatch, shared traffic planning, mutex locking, and itinerary publication; this project does not add a bespoke central robot coordinator.

The setup targets ROS 2 Jazzy, Ubuntu 24.04, and Gazebo Harmonic, a combination listed in the [Open-RMF installation guidance](https://osrf.github.io/ros2multirobotbook/installation.html). The adapter design follows the [official fleet-adapter integration tutorial](https://osrf.github.io/ros2multirobotbook/integration_fleets_adapter_tutorial.html) and the [Jazzy RMF demo adapter](https://github.com/open-rmf/rmf_demos/tree/jazzy/rmf_demos_fleet_adapter). The navigation graph follows [Traffic Editor graph conventions](https://osrf.github.io/ros2multirobotbook/traffic-editor.html) and Open-RMF's [mutex-group strategy](https://osrf.github.io/ros2multirobotbook/integration_nav-maps-strategies.html).

## Fleet and traffic model

The validated RMF graph is a four-waypoint straight aisle at x=`0`, `1`, `2.5`, and `3.5 m` (y=`0`). A300 0 starts at `(0, 0)` facing east; A300 1 starts at `(3.5, 0)` facing west. Their distinct dispatch goals are `goal_west` `(1, 0)` and `goal_east` `(2.5, 0)`, so each robot has a one-metre navigation leg toward the aisle. All three bidirectional lanes between the starts share the exclusive `a300_shared_aisle` mutex. Before dispatch, the smoke checks each goal route against that robot's live SLAM occupancy grid; unknown cells remain blocked by the existing Nav2 A* planner. The asset validator checks the graph's exact protected lanes, goal separation, route reachability, and parked-start clearance. Its 1.35 m layout threshold is only a regression check for this simulation—not a safety rating.

The adapter requires each robot to name a graph vertex marked `is_charger`. Both start vertices carry that RMF flag so robot registration has valid charger metadata; battery drain is disabled, so they are virtual placeholders—not physical chargers or a charging workflow.

The disposable Gazebo profiles disable the Clearpath EKF and enable TF from the differential-drive controller's simulated wheel odometry. The configured Phidgets Spatial IMU has no Gazebo data bridge in this profile, so this smoke test does not exercise IMU-fused localization; the checked-in single-robot profile remains unchanged.

Each simulated robot's SLAM map is local to its spawn pose. A300 0 starts facing east at yaw `0`; A300 1 starts at facility x=`3.5 m` facing west at yaw `π`, so its local map x-axis points west in the facility frame. The adapter transforms each live `map → base_link` pose into facility/RMF coordinates and transforms each RMF waypoint back into that robot's local `map` before sending `NavigateToPose`. The offsets are explicit in `config/rmf_fleet.yaml`; the asset validator checks their alignment with the graph starts, and the smoke checks both initial headings. The graph source is `config/warehouse_a300_fleet.building.yaml`, and the checked-in `config/nav_graphs/0.yaml` is its generated RMF navigation graph.

The smoke client sends two fleet-level `dispatch_task_request` jobs to `goal_east` and `goal_west`, using Open-RMF's task dispatcher to bid and assign them to distinct available A300s. It waits for the first assignment and for an active RMF task plus at least 0.15 m of measured pose progress before sending the second request, creating overlapping fleet demand. The request schema follows the [official RMF `dispatch_go_to_place` client](https://github.com/open-rmf/rmf_demos/blob/jazzy/rmf_demos_tasks/rmf_demos_tasks/dispatch_go_to_place.py). The test records native participant, itinerary, negotiation, `DispatchStates`, fleet-state, and `/mutex_group_states` data. A goal counts as complete only after its assigned robot is idle, its RMF fleet-state task ID is empty, and its measured final pose is within the smoke tolerances below. Success also requires both robots to be observed as the exclusive mutex owner at different times during the run and no non-floor Gazebo contact. Any contact, incomplete dispatch, or incomplete navigation is a hard failure, not a safety pass.

## Goal accuracy and completion criteria

Clearpath's shipped A300 Nav2 profile uses a `SimpleGoalChecker` with 0.25 m XY and 0.25 rad yaw tolerances. A previous smoke result reported 0.482 m XY error for `a300_00000` at `goal_west`, near its then-current 0.50 m acceptance limit. Inspection of that run's evidence showed a non-empty RMF task ID, and its Nav2 log did not report a successful goal for that robot; the smoke had mistaken a temporary idle-mode pose for completed navigation. That 0.482 m observation was therefore not a validated final-goal error.

The launch overlays the supported Clearpath `general_goal_checker` settings to 0.05 m XY and 0.10 rad yaw. Before dispatch, the repeatable runner reads these values from the project overlay and verifies both live Nav2 controller parameter services match. The smoke allows up to 0.15 m XY and 0.25 rad yaw, leaving 0.10 m and 0.15 rad of acceptance margin over the controller thresholds. It records the exact goal and measured final XY/yaw errors, and it will not stop the run while an RMF task ID remains active. These are simulation regression targets, not safety ratings. The existing route coordinates, live-map known-free route preflight, 17 collision geometries per robot, and zero-non-floor-contact requirement remain unchanged.

## Headless run measurements

Three sequential runs passed. Run 1 was the tuning run with the 0.10 m / 0.15 rad Nav2 checker and 0.20 m / 0.30 rad smoke bounds. Runs 2 and 3 used the final 0.05 m / 0.10 rad checker and 0.15 m / 0.25 rad smoke bounds; the runner verified both live controllers before dispatch.

- **Run 1:** `a300_00000` final pose `(0.8953597, 0.0002951, 0.0005876)` vs goal `(1.0, 0.0, 0.0)`; XY error `0.1046407 m`, yaw error `0.0005876 rad`. `a300_00001` final pose `(2.5861096, -0.0078602, -3.1410997)` vs goal `(2.5, 0.0, π)`; XY error `0.0864676 m`, yaw error `0.0004930 rad`. Evidence: `/tmp/husky-rmf-fleet.v0pJNr/fleet_result.json`.
- **Run 2:** `a300_00000` final pose `(0.9475781, 0.0002582, 0.0002310)`; XY error `0.0524225 m`, yaw error `0.0002310 rad`. `a300_00001` final pose `(2.5473800, 0.0001842, -3.1407323)`; XY error `0.0473803 m`, yaw error `0.0008604 rad`. Evidence: `/tmp/husky-rmf-fleet.PpjNgf/fleet_result.json`.
- **Run 3:** `a300_00000` final pose `(0.9453426, 0.0002330, -0.0001953)`; XY error `0.0546579 m`, yaw error `0.0001953 rad`. `a300_00001` final pose `(2.5478528, -0.0012739, -3.1410022)`; XY error `0.0478697 m`, yaw error `0.0005905 rad`. Evidence: `/tmp/husky-rmf-fleet.xDVYDW/fleet_result.json`.

In every run both RMF task IDs cleared and both robots were idle at the final sample. The two robots were assigned distinct goals while the first task was active; itinerary data and ownership changes for `a300_shared_aisle` were observed. Each run measured a minimum sampled center separation of `2.42 m` versus the modeled `1.22 m` footprint diameter, instrumented 17 collision geometries per robot, and recorded zero non-floor Gazebo contacts. These are measured simulation outcomes only, not safety certification.

## Build and validate

From the repository root, source ROS 2 Jazzy and build the ROS workspace:

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
cd ..
```

Validate that the generated four-waypoint graph, charger points, shared mutex zone, and RMF parser agree:

```bash
./scripts/test_rmf_fleet_assets.sh
```

Then run the complete two-robot simulation smoke test:

```bash
./scripts/test_husky_rmf_fleet.sh
```

The launch stages the second Clearpath spawn, each SLAM/Nav2 stack, and the RMF adapter to reduce controller-manager contention during startup. It also launches Open-RMF's `mutex_group_supervisor`, as in the official demo launch; the RMF schedule and fleet-adapter model remain the coordination path, with no bespoke scheduler.

The runner creates disposable A300 profiles and a contact-instrumented warehouse world under `/tmp`; it does not modify the checked-in Clearpath profile. Before dispatch, it checks every segment and destination on each assigned A300's route for known-free live SLAM occupancy. On completion it prints the run directory and writes `fleet_result.json` with those map checks, task outcomes, final poses, schedule evidence, and contact counts. Logs and generated evidence are preserved when a test fails.

## Manual use

The runner's printed `/tmp/husky-rmf-fleet.<id>` directory contains the generated profiles and contact-enabled world. To run the fleet launch manually, source the built workspace and reuse that directory:

```bash
RUN_DIR=/tmp/husky-rmf-fleet.<id> # replace with the path printed by the runner
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch warehouse_rmf_fleet a300_fleet.launch.py \
  setup_path_0:="$RUN_DIR/profiles/a300_00000" \
  setup_path_1:="$RUN_DIR/profiles/a300_00001" \
  world_path:="$RUN_DIR/warehouse_contacts.sdf" contact_count:=17
```

Then send fleet-dispatch requests with the official demo CLI; RMF selects available robots through its dispatcher:

```bash
ros2 run rmf_demos_tasks dispatch_go_to_place \
  --fleet warehouse_a300_fleet --place goal_east --use_sim_time
ros2 run rmf_demos_tasks dispatch_go_to_place \
  --fleet warehouse_a300_fleet --place goal_west --use_sim_time
```

Native schedule and robot-state messages can be inspected with:

```bash
ros2 topic echo /rmf_traffic/itinerary_set
ros2 topic echo /fleet_states
ros2 topic echo /mutex_group_states
```

## Limits

This is an engineering simulation, not a safety-certified fleet deployment. The 1.35 m graph-layout threshold, footprint, and speed values are simulation checks/approximations, not safety ratings; the battery model is not connected to live battery telemetry, and the adapter currently supports two A300s on one flat map with Nav2 `NavigateToPose`. It does not validate real-world human interaction, radio/network failure behavior, lifts, charging, emergency stops, or physical hardware. Keep physical robots out of this workflow.
