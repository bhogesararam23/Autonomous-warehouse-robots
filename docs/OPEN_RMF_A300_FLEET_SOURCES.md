# Open-RMF A300 Fleet Research Sources

The architecture is based on Open-RMF's own components rather than a custom traffic coordinator. These official sources were reviewed while implementing the Jazzy simulation slice:

- [Open-RMF installation guide](https://osrf.github.io/ros2multirobotbook/installation.html) — ROS distribution and supported installation guidance.
- [Open-RMF core overview](https://osrf.github.io/ros2multirobotbook/rmf-core.html) — fleet adapters, traffic schedule, and task dispatch model.
- [Fleet-adapter integration tutorial](https://osrf.github.io/ros2multirobotbook/integration_fleets_adapter_tutorial.html) — the robot/fleet adapter integration pattern.
- [Traffic Editor guide](https://osrf.github.io/ros2multirobotbook/traffic-editor.html) — named waypoints, holding points, lane graphs, map scaling, and the need to transform robot poses/goals into a shared RMF frame.
- [Navigation Graph Strategies: Mutex Groups](https://osrf.github.io/ros2multirobotbook/integration_nav-maps-strategies.html) — the same named group can cover waypoints and lanes; only one robot may occupy a group at a time, and groups should be placed to avoid blocking exits.
- [Jazzy `rmf_demos` source](https://github.com/open-rmf/rmf_demos/tree/jazzy) — current-branch examples and common launch composition for the schedule and dispatcher.
- [Open-RMF mutex supervisor guidance](https://github.com/open-rmf/rmf/issues/514) — maintainers advise using one consistent group name across the protected area and launching `rmf_fleet_adapter`'s `mutex_group_supervisor`.
- [Official RMF demo launch with `mutex_group_supervisor`](https://github.com/open-rmf/rmf_demos/blob/febbc659e3b33866349c20ccdce461fa767ab7cf/rmf_demos/launch/common.launch.xml) — upstream launch example for the supervisor executable.
- [`MutexGroupStates` message](https://docs.ros.org/en/ros2_packages/jazzy/api/rmf_fleet_msgs/msg/MutexGroupStates.html) and [Jazzy topic names](https://raw.githubusercontent.com/open-rmf/rmf_ros2/jazzy/rmf_fleet_adapter/include/rmf_fleet_adapter/StandardNames.hpp) — schedule-side ownership telemetry used by the smoke test.
- [Jazzy Easy Full Control demo adapter](https://github.com/open-rmf/rmf_demos/blob/jazzy/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/fleet_adapter.py) — adapter lifecycle, initial robot registration, navigation callbacks, and state updates.
- [Jazzy `dispatch_go_to_place` source](https://github.com/open-rmf/rmf_demos/blob/jazzy/rmf_demos_tasks/rmf_demos_tasks/dispatch_go_to_place.py) — the smoke uses its `dispatch_task_request` path with a fleet name, not the optional direct `robot_task_request` path.
- [RMF installation overview](https://open-rmf.org/installation) — project-level installation entry point.

The installed Jazzy Python bindings were exercised locally: `FleetConfiguration.from_config_files` accepted the generated navigation graph and two-robot fleet YAML. The packaged smoke test is still the integration authority for whether the full adapter/runtime path works in this specific workspace.
