# Manual live simulation workflow

## Run it on GitHub

The real headless simulator regression is intentionally separate from push and pull-request CI. In the repository's **Actions** tab, choose **Live A300 simulation regression**, select **Run workflow**, and choose one of:

- **both** (the default): run the single-A300 Nav2 end-to-end smoke and the two-A300 Open-RMF fleet smoke.
- **nav2**: run only the single-A300 `NavigateToPose` regression.
- **rmf-fleet**: run only the two-A300 Open-RMF task-dispatch and traffic-scheduling regression.
- **fault-injection**: run a Nav2 blocked-aisle detour check and an Open-RMF stale-status detection check.

The workflow uses the ROS 2 Jazzy / Ubuntu 24.04 environment, installs the repository's ROS dependencies, builds and tests the workspace, validates the RMF assets, then runs the requested simulation test scripts. It uses software OpenGL rendering because GitHub-hosted runners do not promise a dedicated simulator GPU.

## Fault-injection option

The blocked-aisle case first selects a known-free staging goal at least 1.25 m away, verifies that direct route, and navigates there with no obstacle present. It waits for new SLAM and odometry messages, then selects and verifies the fault route from the updated pose. That route must be at least 1.8 m long; the minimum is derived from the A300's 0.495 m half-length and the box's 0.275 m half-length, leaving 0.13 m of endpoint body clearance. Only after this second known-free preflight does it spawn a separately validated static box at the route midpoint, aligned across the route. The runner passes explicit world X/Y/Z and yaw arguments because `ros_gz_sim create` otherwise submits its zero-valued pose defaults over the SDF pose. It verifies a fresh lidar return on the obstacle's expected bearing and distance, allows the Nav2 costmap to refresh, and sends the A300 to the already-known-free goal. Nav2's global obstacle layer consumes live lidar scans, so a new SLAM occupancy cell is recorded when available but is not a pass condition. The smoke requires a post-dispatch global plan whose endpoints match that request and whose points on the start-to-goal segment deviate at least 0.45 m from the direct line. It retains the route points, endpoint errors, warmup and final poses, Gazebo spawn response, lidar evidence, optional SLAM evidence, and contact-sensor data. The source Clearpath world and checked-in robot profile are not modified.

The stale-status case first observes both simulated robots, then the smoke-test consumer deliberately discards subsequent `FleetState` entries for one robot. It passes only if the consumer detects that robot's receipt age exceeded 15 seconds before dispatching any task. This is a deterministic observer-side data-loss injection—not a DDS/network outage, an RMF adapter watchdog test, or a claim that the fleet stack itself stops a robot when status is stale. The age uses the local monotonic receipt clock because `FleetState` has no per-robot source timestamp.

Both fault cases run only when `fault-injection` is selected. The workflow requires each expected detection to pass and includes the fault metadata and result JSON in the usual downloadable artifact.

## Results and artifacts

A run is green only when dependency installation, the workspace build/asset checks, and every requested simulation regression complete successfully. A failed or timed-out suite remains a failure; it is never reported as a pass merely because the other selected suite succeeded.

The workflow uploads a downloadable `live-a300-simulation-<run ID>-<attempt>` artifact even when setup or a simulation check fails. It contains runner CPU/memory/disk and graphics-device diagnostics, ROS logs, simulator and smoke-test logs, generated evidence, and `result.json` / `fleet_result.json` when those files were produced. Artifacts are retained for 14 days. The workflow summary lists the outcome of each stage and any result JSON files found.

Each simulator step has a 45-minute limit and the full job has a 180-minute limit. The blocked-aisle NavigateToPose goal has a 600-second wall-clock timeout by default, overridable with `HUSKY_SMOKE_ACTION_TIMEOUT`; the longer budget accommodates the measured detour execution time without weakening the required global-plan detour, action success, final-pose, or no-contact checks. Hosted CPU, memory, disk, and graphics availability can affect startup and runtime. When a run fails, use the step log and uploaded runner diagnostics to identify whether it stopped during setup, hit a test assertion, or was constrained by the hosted environment; do not infer a simulator defect or a pass from a timeout alone.

## Simulation-only boundary

These checks exercise software in Gazebo using simulated Clearpath A300s. They do not connect to physical robots and are not safety certification, physical collision testing, or proof of real-world fleet behavior. The live simulator workflow is an engineering regression and is not a required per-push CI gate.
