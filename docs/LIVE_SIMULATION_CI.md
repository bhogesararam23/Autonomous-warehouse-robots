# Manual live simulation workflow

## Run it on GitHub

The real headless simulator regression is intentionally separate from push and pull-request CI. In the repository's **Actions** tab, choose **Live A300 simulation regression**, select **Run workflow**, and choose one of:

- **both** (the default): run the single-A300 Nav2 end-to-end smoke and the two-A300 Open-RMF fleet smoke.
- **nav2**: run only the single-A300 `NavigateToPose` regression.
- **rmf-fleet**: run only the two-A300 Open-RMF task-dispatch and traffic-scheduling regression.

The workflow uses the ROS 2 Jazzy / Ubuntu 24.04 environment, installs the repository's ROS dependencies, builds and tests the workspace, validates the RMF assets, then runs the requested simulation test scripts. It uses software OpenGL rendering because GitHub-hosted runners do not promise a dedicated simulator GPU.

## Results and artifacts

A run is green only when dependency installation, the workspace build/asset checks, and every requested simulation regression complete successfully. A failed or timed-out suite remains a failure; it is never reported as a pass merely because the other selected suite succeeded.

The workflow uploads a downloadable `live-a300-simulation-<run ID>-<attempt>` artifact even when setup or a simulation check fails. It contains runner CPU/memory/disk and graphics-device diagnostics, ROS logs, simulator and smoke-test logs, generated evidence, and `result.json` / `fleet_result.json` when those files were produced. Artifacts are retained for 14 days. The workflow summary lists the outcome of each stage and any result JSON files found.

Each simulator step has a 45-minute limit and the full job has a 180-minute limit. Hosted CPU, memory, disk, and graphics availability can affect startup and runtime. When a run fails, use the step log and uploaded runner diagnostics to identify whether it stopped during setup, hit a test assertion, or was constrained by the hosted environment; do not infer a simulator defect or a pass from a timeout alone.

## Simulation-only boundary

These checks exercise software in Gazebo using simulated Clearpath A300s. They do not connect to physical robots and are not safety certification, physical collision testing, or proof of real-world fleet behavior. The live simulator workflow is an engineering regression and is not a required per-push CI gate.
