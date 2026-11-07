# Husky A300 Warehouse Navigation Simulation

## What this adds

This project now includes a ROS 2 Nav2 global-planner plugin that runs A* over a snapshot of Nav2's 2D costmap. The planner uses 8-connected moves, prevents diagonal corner-cutting, treats lethal and inscribed-obstacle cells as blocked, and applies a tunable cost penalty so inflated cells are less attractive than clear space. Unknown cells are blocked by default; that policy is configurable. Search cancellation and a bounded expansion count are supported.

The plugin is an integration layer around a dependency-free C++ core, so the search algorithm and its tests can run without ROS. The original Python and TypeScript time-aware demonstrations remain intact. The C++ Nav2 plugin is a spatial planner over the current costmap; it does not use the old time-indexed route reservations or predict moving people and vehicles.

## Reference platform and compatibility choice

The simulated platform is Clearpath's Husky A300, using the vendor's robot descriptions and simulator assets rather than a hand-built chassis. Clearpath's current product specifications list dimensions of **990 × 698 × 381 mm**, **80 kg** base mass, **100 kg** maximum payload, **2.0 m/s** maximum speed, **IP54** ingress rating, and a **30°** maximum climb grade. Typical runtime is listed as **8 hours** (standard battery), **16 hours** (upgraded), or **24 hours** (extended); the official product page lists **ROS 2 Jazzy** compatibility. These are platform ratings, not guarantees about every sensor, payload, battery, drivetrain option, or controller configuration, and the simulation does not validate those real-world ratings.

The supported stack for this build is **Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic**. Clearpath's Jazzy simulator documentation uses Harmonic and provides a `warehouse` Gazebo world with shelves and people; Gazebo's compatibility table identifies Jazzy + Harmonic as the recommended pairing. We intentionally follow the vendor-supported A300 stack rather than switching to a newer general ROS/Gazebo release that the A300 docs do not target. Clearpath recommends a dedicated GPU for good simulator performance.

The included `config/robot.yaml` is based on Clearpath's A300 Nav2 setup example and selects the vendor's A300 model, Phidgets IMU profile, and Hokuyo 2D lidar profile. The `nav2.launch.py` wrapper loads Clearpath's A300 Nav2 defaults and rewrites the global planner class to `warehouse_astar_planner::AStarPlanner`; it does not replace the vendor robot description or its platform-specific Nav2 defaults.

## Install and build

Use an Ubuntu 24.04 host with ROS 2 Jazzy installed. The official [ROS 2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html) explains how to configure the ROS package repository. Then install the ROS and Clearpath packages:

```bash
sudo apt update
sudo apt install ros-jazzy-desktop ros-dev-tools
source /opt/ros/jazzy/setup.bash
sudo apt install \
  ros-jazzy-ros-gz \
  ros-jazzy-clearpath-simulator \
  ros-jazzy-clearpath-nav2-demos \
  python3-rosdep
```

Build this repository's ROS package:

```bash
cd Autonomous-warehouse-robots/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

If Clearpath's simulator packages are not available as binaries for the host, use the [official Clearpath source-install steps](https://docs.clearpathrobotics.com/docs/ros/tutorials/simulator/install) and build their Jazzy workspace with `vcstool`, `rosdep`, and `colcon`.

## Prepare the A300 simulation profile

The simulator generator reads a Clearpath `robot.yaml` from a setup directory. Copy the included profile and generate the setup files:

```bash
mkdir -p "$HOME/clearpath"
cp ros2_ws/src/warehouse_astar_planner/config/robot.yaml "$HOME/clearpath/robot.yaml"
source /opt/ros/jazzy/setup.bash
ros2 run clearpath_generator_common generate_bash -s "$HOME/clearpath"
source "$HOME/clearpath/setup.bash"
```

The included sensors are selected from Clearpath's configuration catalog; they are simulation sensor profiles and do not claim to represent a purchased robot's physical sensor bill of materials.

## Launch the warehouse and navigation

Run each command in a separate terminal from the repository root, sourcing the same ROS setup in each terminal.

**Terminal 1 — Gazebo warehouse and A300:**

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/clearpath/setup.bash"
ros2 launch clearpath_gz simulation.launch.py \
  setup_path:="$HOME/clearpath" world:=warehouse
```

**Terminal 2 — SLAM:**

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/clearpath/setup.bash"
source ros2_ws/install/setup.bash
ros2 launch clearpath_nav2_demos slam.launch.py \
  setup_path:="$HOME/clearpath" use_sim_time:=true
```

**Terminal 3 — Nav2 with this A* plugin:**

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/clearpath/setup.bash"
source ros2_ws/install/setup.bash
ros2 launch warehouse_astar_planner nav2.launch.py \
  setup_path:="$HOME/clearpath" use_sim_time:=true
```

**Terminal 4 — navigation view:**

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/clearpath/setup.bash"
ros2 launch clearpath_viz view_navigation.launch.py \
  namespace:=/a300_00000 use_sim_time:=true
```

In RViz, set the initial pose and then send a 2D navigation goal. The custom launch assumes the included A300 profile and uses the Clearpath A300 Nav2 parameter set. The optional `scan_topic:=...` argument can select a different 2D scan topic.

## Planner parameters

The launch wrapper loads Clearpath's current A300 Nav2 defaults, then merges the package-owned [`planner_params.yaml`](../ros2_ws/src/warehouse_astar_planner/config/planner_params.yaml) over the `GridBased` planner block. This preserves the vendor's controller, costmap, and behavior settings while ensuring the plugin class and each custom parameter are present in the final YAML. Composition is disabled in this fixed A300 profile so it follows the same non-composed namespacing path as the standard Clearpath launch.

The planner parameters under `GridBased` are:

| Parameter | Default | Meaning |
| --- | ---: | --- |
| `allow_unknown` | `false` | Reject unknown costmap cells unless explicitly enabled. |
| `cost_penalty` | `2.0` | Increase travel cost through known inflated cells. |
| `unknown_cost_multiplier` | `5.0` | Relative step cost for unknown cells when allowed. |
| `max_expansions` | `250000` | Bound the number of A* expansions per request. |

Known cells with cost values at or above Nav2's inscribed-obstacle threshold are rejected. The planner copies the current costmap while holding its mutex, then searches the immutable snapshot so planning does not hold the live costmap lock for the entire search. Returned poses use the costmap's global frame and map resolution.

## Checks

The dependency-free C++ core suite runs on a standard C++17 compiler:

```bash
./scripts/test_cpp_core.sh
```

It covers an optimal diagonal path, a blocked passage, inflated-cost preference, unreachable and blocked endpoints, start-equals-goal, forbidden corner-cutting, unknown-space policy, malformed grids, and cancellation. In a ROS 2 Jazzy environment, also run:

```bash
cd ros2_ws
colcon test --packages-select warehouse_astar_planner
colcon test-result --verbose
ros2 plugin list | grep warehouse_astar_planner
ros2 param get /a300_00000/planner_server GridBased.plugin
```

The parameter query should return `warehouse_astar_planner::AStarPlanner`; it verifies that Nav2 received the plugin override in addition to confirming that pluginlib can discover the package.

## End-to-end path-following smoke test

Run the complete software-only navigation path with:

```bash
./scripts/test_husky_nav2_e2e.sh
```

The runner creates a disposable Clearpath setup under `/tmp`, starts Gazebo Harmonic in server-only/headless-rendering mode, starts SLAM and Nav2, and sends a `NavigateToPose` action in the `map` frame. By default it deterministically selects a goal 1.25–3.0 m from the measured start pose, inside a known-free 0.65 m radius in the live SLAM map, with a known-free 0.25 m-radius approach corridor. To use a specific goal instead, provide both `--goal-x` and `--goal-y`, and optionally `--goal-yaw`; the same map-clearance check applies. The runner verifies the namespaced TF tree, confirms Nav2 is configured with `warehouse_astar_planner::AStarPlanner` and Clearpath's standard `nav2_mppi_controller::MPPIController`, and checks that the action succeeds with error code zero and the measured final pose is within 0.35 m / 0.35 rad of the goal.

For this test only, the runner copies the world and generated A300 profile to a temporary directory, enables Gazebo's contact system, and adds a contact sensor to each converted robot collision geometry. It verifies an active Gazebo `Contacts` publisher for every sensor, then monitors contact events throughout the run; any contact with a non-floor model fails the test. Gazebo contact sensors emit events rather than periodic empty “clear” messages, so the test does not require an event from every sensor. This is evidence for the tested simulation episode, not a general collision-avoidance guarantee.

Before running, build and source the workspace as described above. The generated Clearpath profile and instrumented world are temporary; the repository's robot configuration, installed world, and generated user profile are not modified. ROS domain 57 and a unique Gazebo Transport partition are used by default; set `HUSKY_SMOKE_DOMAIN_ID` or `HUSKY_SMOKE_GZ_PARTITION` to override them. Logs, the contact-geometry manifest, and `result.json` are retained in the printed run directory on success or failure. The runner shuts down the simulator, SLAM, and Nav2 process groups when it exits.

This regression is also available in the [manual live-simulation GitHub Actions workflow](LIVE_SIMULATION_CI.md), which uploads logs, runner diagnostics, and generated result JSON. Hosted-runner timing and graphics resources can vary; see that guide for the limits and how to interpret a failed run.

The test was exercised in the real headless Gazebo simulation: `NavigateToPose` returned `SUCCEEDED` with error code 0, the goal was `(1.30, 0.00)` and the final pose was `(1.054, 0.008)` (0.247 m XY error, within tolerance), all 17 contact publishers were verified, and no non-floor contacts were observed.

## Model and safety boundary

This is a **simulation-only** project. It does not connect to a physical robot, does not validate real stopping distance or protective separation, and is not a safety-certified controller. The global A* planner uses the latest available costmap snapshot; it is not a time-aware fleet scheduler and does not guarantee collision avoidance around people or other robots. Nav2's normal costmap, controller, and recovery components remain responsible for their own behavior, but neither they nor this planner replace manufacturer safety functions, safety-rated sensing, an emergency-stop chain, or a professional risk assessment. Do not use this software to control a physical vehicle.

The A300's published 2.0 m/s maximum is a hardware capability, not a required simulation target. The vendor's Nav2 example has its own conservative controller settings (for example, a 0.5 m/s MPPI forward-speed limit), so those navigation parameters should not be mistaken for the chassis rating.

## Primary references

- [Clearpath Husky A300 specifications](https://clearpathrobotics.com/husky-a300-unmanned-ground-vehicle-robot/)
- [Clearpath simulator install for ROS 2 Jazzy](https://docs.clearpathrobotics.com/docs/ros/tutorials/simulator/install)
- [Clearpath Gazebo simulator and warehouse world](https://github.com/clearpathrobotics/clearpath_simulator/tree/jazzy)
- [Clearpath simulation launch arguments](https://docs.clearpathrobotics.com/docs/ros/tutorials/simulator/simulate)
- [Clearpath A300 Nav2 configuration example](https://docs.clearpathrobotics.com/docs/ros/tutorials/navigation_demos/configuration)
- [Clearpath Nav2 launch and simulation instructions](https://docs.clearpathrobotics.com/docs/ros/tutorials/navigation_demos/nav2/)
- [Nav2 planner plugin tutorial](https://docs.nav2.org/plugin_tutorials/docs/writing_new_nav2planner_plugin.html)
- [ROS 2 Jazzy `GlobalPlanner` interface](https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_core/include/nav2_core/global_planner.hpp)
- [Gazebo ROS compatibility table](https://gazebosim.org/docs/latest/ros_installation/)
