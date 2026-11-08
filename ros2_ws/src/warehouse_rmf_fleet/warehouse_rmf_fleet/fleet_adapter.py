"""Open-RMF bridge for two namespaced Clearpath A300 Nav2 simulations."""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from datetime import timedelta
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
import rmf_adapter
import rmf_adapter.easy_full_control as rmf_easy
import yaml
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.clock import Clock
from rclpy.clock_type import ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from .coordinate_transform import local_to_rmf, rmf_to_local
from .robot_availability import (
    DEFAULT_TRANSFORM_MAX_AGE_SECONDS,
    RMFRobotAvailabilityGate,
    RMFUpdateHandleNotReady,
    TransformFreshnessMonitor,
)


BATTERY_SOC = 0.95


def yaw_from_quaternion(q: Any) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _frame_candidates(namespace: str, frame: str) -> tuple[str, ...]:
    bare = frame.strip("/")
    prefix = namespace.strip("/")
    candidates = [bare]
    if prefix:
        candidates.append(f"{prefix}/{bare}")
    return tuple(dict.fromkeys(candidates))


class RobotNav2Bridge(Node):
    """One RMF robot backed by one namespaced Nav2 action server and TF tree."""

    def __init__(
        self,
        robot_name: str,
        interface: dict[str, Any],
        map_name: str,
        fleet_handle: Any,
        robot_configuration: Any,
        *,
        use_sim_time: bool,
        state_update_period: float,
    ) -> None:
        namespace = str(interface["namespace"]).strip("/")
        super().__init__(
            f"{robot_name}_rmf_bridge",
            namespace=namespace,
            cli_args=["--ros-args", "-r", "/tf:=tf", "-r", "/tf_static:=tf_static"],
            parameter_overrides=[Parameter("use_sim_time", value=use_sim_time)],
            use_global_arguments=False,
        )
        self.robot_name = robot_name
        self.namespace = namespace
        self.map_frame = str(interface.get("map_frame", "map")).strip("/")
        self.base_frame = str(interface.get("base_frame", "base_link")).strip("/")
        self.map_origin = tuple(float(v) for v in interface["map_origin_in_rmf"])
        if len(self.map_origin) != 3:
            raise ValueError(f"{robot_name}: map_origin_in_rmf must be [x,y,yaw]")
        self.map_name = map_name
        self.fleet_handle = fleet_handle
        self.robot_configuration = robot_configuration
        self._tf_buffer = Buffer(node=self)
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._update_handle = None
        self._execution = None
        self._goal_handle = None
        self._generation = 0
        self._lock = threading.RLock()
        self._last_tf_warning = 0.0
        self._last_action_warning = 0.0
        self._last_registration_attempt = 0.0
        self._last_pose_warning = 0.0
        self._last_commission_error = 0.0
        self._last_gate_pending_warning = 0.0
        self._tf_freshness = TransformFreshnessMonitor(
            DEFAULT_TRANSFORM_MAX_AGE_SECONDS
        )
        self._availability_gate: RMFRobotAvailabilityGate | None = None
        self._callbacks = rmf_easy.RobotCallbacks(
            self.navigate,
            self.stop,
            self.execute_action,
        )
        self._timer = self.create_timer(
            state_update_period,
            self._tick,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        self.get_logger().info(
            f"Waiting for TF {self.map_frame}->{self.base_frame} and "
            f"action server /{self.namespace}/navigate_to_pose; "
            f"facility origin={self.map_origin}"
        )

    def _local_pose(self) -> tuple[tuple[float, float, float], int] | None:
        for map_frame in _frame_candidates(self.namespace, self.map_frame):
            for base_frame in _frame_candidates(self.namespace, self.base_frame):
                try:
                    tf = self._tf_buffer.lookup_transform(map_frame, base_frame, Time())
                    t = tf.transform.translation
                    stamp_ns = (
                        int(tf.header.stamp.sec) * 1_000_000_000
                        + int(tf.header.stamp.nanosec)
                    )
                    return (
                        (float(t.x), float(t.y), yaw_from_quaternion(tf.transform.rotation)),
                        stamp_ns,
                    )
                except TransformException:
                    continue
        now = time.monotonic()
        if now - self._last_tf_warning > 10.0:
            try:
                available_frames = self._tf_buffer.all_frames_as_yaml()
            except Exception as error:
                available_frames = f"<could not enumerate TF frames: {error}>"
            self.get_logger().info(
                f"Waiting for the robot's local map transform "
                f"({self.map_frame} -> {self.base_frame}); frames seen: {available_frames}"
            )
            self._last_tf_warning = now
        return None

    def _rmf_state(self, local_pose: tuple[float, float, float]) -> Any:
        x, y, yaw = local_to_rmf(local_pose, self.map_origin)
        return rmf_easy.RobotState(
            self.map_name,
            np.asarray([x, y, yaw], dtype=np.float64),
            BATTERY_SOC,
        )

    def _tick(self) -> None:
        if self._update_handle is not None and self._availability_gate is None:
            try:
                self._availability_gate = RMFRobotAvailabilityGate(
                    self._update_handle
                )
            except RMFUpdateHandleNotReady:
                now = time.monotonic()
                if now - self._last_gate_pending_warning > 10.0:
                    self.get_logger().info(
                        f"Waiting for RMF to finish initializing the update "
                        f"handle for {self.robot_name}"
                    )
                    self._last_gate_pending_warning = now
                return
        pose_sample = self._local_pose()
        if pose_sample is None:
            self._mark_pose_unavailable("map-to-base transform is unavailable")
            return
        local_pose, transform_stamp_ns = pose_sample
        now_ros_ns = self.get_clock().now().nanoseconds
        if not self._tf_freshness.observe(
            transform_stamp_ns,
            now_ros_ns,
            monotonic_now=time.monotonic(),
        ):
            self._mark_pose_unavailable(
                "map-to-base transform is stale, future-dated, or not advancing"
            )
            return
        if not self._action_client.server_is_ready():
            now = time.monotonic()
            if now - self._last_action_warning > 10.0:
                self.get_logger().info(
                    f"Waiting for Nav2 action server /{self.namespace}/navigate_to_pose"
                )
                self._last_action_warning = now
            return
        state = self._rmf_state(local_pose)
        with self._lock:
            if self._update_handle is None:
                now = time.monotonic()
                if now - self._last_registration_attempt < 2.0:
                    return
                self._last_registration_attempt = now
                update_handle = self.fleet_handle.add_robot(
                    self.robot_name,
                    state,
                    self.robot_configuration,
                    self._callbacks,
                )
                if update_handle is None:
                    self.get_logger().error(
                        f"RMF did not add robot {self.robot_name!r}; verify its "
                        "charger waypoint and initial graph location"
                    )
                    return
                self._update_handle = update_handle
                self.get_logger().info(
                    f"Successfully registered with RMF at facility pose "
                    f"{tuple(float(v) for v in state.position)}"
                )
                return
            activity = self._execution.identifier if self._execution is not None else None
            self._update_handle.update(state, activity)
            if self._availability_gate is not None:
                if self._availability_gate.available:
                    self._availability_gate.refresh_baseline()
                else:
                    try:
                        if self._availability_gate.mark_available():
                            self.get_logger().info(
                                f"Fresh TF restored for {self.robot_name}; "
                                "restored its prior RMF task-intake policy"
                            )
                    except Exception as error:
                        self._log_commission_error("restore", error)

    def _mark_pose_unavailable(self, reason: str) -> None:
        with self._lock:
            if self._availability_gate is None:
                return
            try:
                changed = self._availability_gate.mark_unavailable()
            except Exception as error:
                self._log_commission_error("decommission", error)
                return
            if changed:
                self.get_logger().warning(
                    f"{self.robot_name} pose unavailable ({reason}); stopped "
                    "publishing pose updates and disabled new RMF task intake"
                )
                self._last_pose_warning = time.monotonic()
            elif time.monotonic() - self._last_pose_warning > 10.0:
                self.get_logger().warning(
                    f"{self.robot_name} remains unavailable: {reason}"
                )
                self._last_pose_warning = time.monotonic()

    def _log_commission_error(self, action: str, error: Exception) -> None:
        now = time.monotonic()
        if now - self._last_commission_error <= 10.0:
            return
        self.get_logger().fatal(
            f"Could not {action} {self.robot_name} in RMF after a TF freshness "
            f"change; task-intake state may not match telemetry: {error}"
        )
        self._last_commission_error = now

    def navigate(self, destination: Any, execution: Any) -> None:
        """Convert the next RMF waypoint into a local Nav2 goal."""
        if str(destination.map) != self.map_name:
            self.get_logger().error(
                f"Refusing destination on unknown RMF map {destination.map!r}; "
                f"expected {self.map_name!r}"
            )
            execution.finished()
            return
        if not self._action_client.server_is_ready():
            self.get_logger().error("Nav2 NavigateToPose server is not ready")
            execution.finished()
            return

        rmf_position = tuple(float(v) for v in destination.position)
        local_position = rmf_to_local(rmf_position, self.map_origin)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = local_position[0]
        goal.pose.pose.position.y = local_position[1]
        goal.pose.pose.orientation.z = math.sin(local_position[2] / 2.0)
        goal.pose.pose.orientation.w = math.cos(local_position[2] / 2.0)

        with self._lock:
            previous_goal = self._goal_handle
            self._generation += 1
            generation = self._generation
            self._execution = execution
            self._goal_handle = None
        if previous_goal is not None:
            previous_goal.cancel_goal_async()

        self.get_logger().info(
            f"RMF command: {tuple(round(v, 3) for v in rmf_position)} on "
            f"{destination.map} -> Nav2 local pose "
            f"{tuple(round(v, 3) for v in local_position)} "
            f"(waypoint={destination.name!r}, speed_limit={destination.speed_limit})"
        )
        try:
            future = self._action_client.send_goal_async(goal)
            future.add_done_callback(lambda result: self._on_goal_response(generation, result))
        except Exception as error:  # command submission is an integration boundary
            self.get_logger().error(f"Could not submit Nav2 goal: {error}")
            self._finish(generation)

    def _on_goal_response(self, generation: int, future: Any) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:
            self.get_logger().error(f"Nav2 goal request failed: {error}")
            self._finish(generation)
            return
        with self._lock:
            if generation != self._generation:
                if goal_handle.accepted:
                    goal_handle.cancel_goal_async()
                return
            if not goal_handle.accepted:
                self.get_logger().error("Nav2 rejected the RMF waypoint goal")
                self._goal_handle = None
                self._finish_locked(generation)
                return
            self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda result: self._on_goal_result(generation, result))

    def _on_goal_result(self, generation: int, future: Any) -> None:
        try:
            wrapped = future.result()
            status = int(wrapped.status)
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("Nav2 waypoint completed successfully")
            else:
                self.get_logger().error(f"Nav2 waypoint ended with action status {status}")
        except Exception as error:
            self.get_logger().error(f"Nav2 action result failed: {error}")
        self._finish(generation)

    def _finish_locked(self, generation: int) -> None:
        if generation != self._generation:
            return
        execution = self._execution
        self._execution = None
        self._goal_handle = None
        if execution is not None:
            execution.finished()

    def _finish(self, generation: int) -> None:
        with self._lock:
            self._finish_locked(generation)

    def stop(self, activity: Any) -> None:
        """Cancel the matching Nav2 goal when RMF stops or replans an activity."""
        with self._lock:
            if self._execution is None:
                return
            if not self._execution.identifier.is_same(activity):
                return
            self._generation += 1
            goal_handle = self._goal_handle
            self._goal_handle = None
            self._execution = None
        if goal_handle is not None:
            goal_handle.cancel_goal_async()
        self.get_logger().info("Canceled the active Nav2 goal at RMF's request")

    def execute_action(self, category: str, description: Any, execution: Any) -> None:
        del description
        self.get_logger().warning(
            f"No simulator action handler is configured for RMF action {category!r}"
        )
        execution.finished()


def _arguments(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--nav-graph", required=True, type=Path)
    parser.add_argument("--use-sim-time", action="store_true")
    return parser.parse_args(rclpy.utilities.remove_ros_args(argv)[1:])


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv if argv is None else argv)
    rclpy.init(args=argv)
    rmf_adapter.init_rclcpp()
    try:
        args = _arguments(argv)
        fleet_config = rmf_easy.FleetConfiguration.from_config_files(
            str(args.config), str(args.nav_graph)
        )
        if fleet_config is None:
            raise RuntimeError("RMF rejected the fleet configuration or navigation graph")
        raw_config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
        interfaces = raw_config.get("robot_interfaces", {})
        names = sorted(str(name) for name in fleet_config.known_robots)
        if len(names) != 2:
            raise RuntimeError(f"This simulation slice requires exactly two robots; found {names}")
        if set(names) != set(interfaces):
            raise RuntimeError(
                "robot_interfaces must exactly match rmf_fleet.robots; "
                f"robots={names}, interfaces={sorted(interfaces)}"
            )
        namespaces = [str(interfaces[name]["namespace"]).strip("/") for name in names]
        if len(set(namespaces)) != 2 or any(not namespace for namespace in namespaces):
            raise RuntimeError("Each A300 must have its own non-empty ROS namespace")

        adapter = rmf_adapter.Adapter.make(
            f"{fleet_config.fleet_name}_nav2_adapter",
            wait_time=timedelta(seconds=30),
        )
        if adapter is None:
            raise RuntimeError(
                "Open-RMF could not connect to the traffic schedule. Start "
                "rmf_traffic_schedule before this adapter."
            )
        if args.use_sim_time:
            adapter.node.use_sim_time()
        adapter.start()
        fleet_handle = adapter.add_easy_fleet(fleet_config)

        node = Node(
            f"{fleet_config.fleet_name}_fleet_bridge",
            parameter_overrides=[Parameter("use_sim_time", value=args.use_sim_time)],
            use_global_arguments=False,
        )
        node.get_logger().info(
            f"Open-RMF fleet {fleet_config.fleet_name!r} is ready; waiting for "
            "each A300's namespaced TF and NavigateToPose server"
        )
        bridges = [
            RobotNav2Bridge(
                name,
                interfaces[name],
                "L1",
                fleet_handle,
                fleet_config.get_known_robot_configuration(name),
                use_sim_time=args.use_sim_time,
                state_update_period=0.2,
            )
            for name in names
        ]
        executor = MultiThreadedExecutor(num_threads=8)
        executor.add_node(node)
        for bridge in bridges:
            executor.add_node(bridge)
        try:
            executor.spin()
        finally:
            for bridge in bridges:
                executor.remove_node(bridge)
                bridge.destroy_node()
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_node()
        return 0
    except Exception as error:
        print(f"warehouse_a300_fleet_adapter failed: {error}", file=sys.stderr)
        return 1
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
