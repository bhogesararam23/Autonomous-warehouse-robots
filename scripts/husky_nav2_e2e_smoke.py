#!/usr/bin/env python3
"""Exercise NavigateToPose and verify result, final pose, and Gazebo contacts."""

from __future__ import annotations

import argparse
import json
import math
import re
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from ros_gz_interfaces.msg import Contacts
from tf2_ros import Buffer, TransformException, TransformListener


ROBOT_NS = "/a300_00000"
EXPECTED_PLANNER = "warehouse_astar_planner::AStarPlanner"
EXPECTED_CONTROLLER = "nav2_mppi_controller::MPPIController"


class SmokeTestError(RuntimeError):
    """Raised when an end-to-end precondition or assertion fails."""


def yaw_from_quaternion(q: Any) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angular_error(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


class NavigationSmokeNode(Node):
    def __init__(self, contact_count: int, base_frame: str) -> None:
        super().__init__(
            "husky_nav2_e2e_smoke",
            namespace=ROBOT_NS,
            cli_args=["--ros-args", "-r", "/tf:=tf", "-r", "/tf_static:=tf_static"],
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.base_frame = base_frame
        self.map_msg: OccupancyGrid | None = None
        self.odom_msg: Odometry | None = None
        self.contact_message_counts = [0] * contact_count
        self.contact_messages_total = 0
        self.gazebo_contact_topics_advertised = 0
        self.contact_hits: list[dict[str, Any]] = []
        self.last_map_wall_time: float | None = None
        self.last_odom_wall_time: float | None = None
        self.last_feedback_wall_time = 0.0
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_subscription = self.create_subscription(
            OccupancyGrid, f"{ROBOT_NS}/map", self._on_map, 10
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            f"{ROBOT_NS}/platform/odom/filtered",
            self._on_odom,
            10,
        )
        self.contact_subscriptions = []
        self.contact_topics = []
        for index in range(contact_count):
            topic = f"{ROBOT_NS}/smoke/contact/sensor_{index:02d}"
            self.contact_topics.append(topic)
            self.contact_subscriptions.append(
                self.create_subscription(
                    Contacts,
                    topic,
                    lambda message, sensor_index=index: self._on_contacts(
                        sensor_index, message
                    ),
                    10,
                )
            )
        self.action_client = ActionClient(
            self, NavigateToPose, f"{ROBOT_NS}/navigate_to_pose"
        )

    def _on_map(self, message: OccupancyGrid) -> None:
        self.map_msg = message
        self.last_map_wall_time = time.monotonic()

    def _on_odom(self, message: Odometry) -> None:
        self.odom_msg = message
        self.last_odom_wall_time = time.monotonic()

    def _on_contacts(self, sensor_index: int, message: Contacts) -> None:
        self.contact_message_counts[sensor_index] += 1
        self.contact_messages_total += 1
        for contact in message.contacts:
            first = contact.collision1.name
            second = contact.collision2.name
            if self._is_floor(first) or self._is_floor(second):
                continue
            if ROBOT_NS.strip("/") in first and ROBOT_NS.strip("/") in second:
                continue
            self.contact_hits.append(
                {
                    "sensor_index": sensor_index,
                    "collision1": first,
                    "collision2": second,
                    "sim_stamp": {
                        "sec": int(message.header.stamp.sec),
                        "nanosec": int(message.header.stamp.nanosec),
                    },
                }
            )

    @staticmethod
    def _is_floor(name: str) -> bool:
        normalized = name.lower()
        return any(token in normalized for token in ("ground_plane", "floor"))

    def wait_until(self, description: str, condition, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if condition():
                return
        raise SmokeTestError(f"Timed out waiting for {description} after {timeout:.0f}s")

    def wait_for_clock(self, timeout: float) -> None:
        self.wait_until(
            "a non-zero Gazebo /clock",
            lambda: self.get_clock().now().nanoseconds > 0,
            timeout,
        )

    def wait_for_robot_state(self, timeout: float) -> None:
        self.wait_until(
            "fresh A300 map and odometry",
            lambda: self.map_msg is not None
            and self.odom_msg is not None
            and self.last_map_wall_time is not None
            and self.last_odom_wall_time is not None
            and time.monotonic() - self.last_map_wall_time < 3.0
            and time.monotonic() - self.last_odom_wall_time < 3.0,
            timeout,
        )

    def wait_for_contact_instrumentation(self, timeout: float) -> None:
        def all_publishers_visible() -> bool:
            return all(self.count_publishers(topic) > 0 for topic in self.contact_topics)

        self.wait_until("all Gazebo contact bridge publishers", all_publishers_visible, timeout)
        topic_listing = subprocess.run(
            ["gz", "topic", "-l"],
            check=True,
            capture_output=True,
            text=True,
            timeout=10.0,
        ).stdout
        advertised_topics = set(topic_listing.splitlines())
        missing = [topic for topic in self.contact_topics if topic not in advertised_topics]
        if missing:
            raise SmokeTestError(
                "Gazebo is not advertising every instrumented contact sensor: "
                + ", ".join(missing)
            )
        for topic in self.contact_topics:
            info = subprocess.run(
                ["gz", "topic", "-i", "-t", topic],
                capture_output=True,
                text=True,
                timeout=5.0,
            )
            details = (info.stdout + info.stderr).lower()
            if info.returncode != 0 or "publisher" not in details or "gz.msgs.contacts" not in details:
                raise SmokeTestError(
                    f"Gazebo did not confirm an active Contacts publisher for {topic}: "
                    f"{info.stdout.strip()} {info.stderr.strip()}"
                )
        self.gazebo_contact_topics_advertised = len(self.contact_topics)
        if self.contact_hits:
            raise SmokeTestError(
                "The robot made contact with a non-floor model before motion: "
                + json.dumps(self.contact_hits[:5])
            )

    def pose_in_map(self, timeout: float) -> tuple[float, float, float]:
        deadline = time.monotonic() + timeout
        last_error: Exception | None = None
        while time.monotonic() < deadline:
            try:
                transform = self.tf_buffer.lookup_transform(
                    "map", self.base_frame, Time()
                )
                translation = transform.transform.translation
                return (
                    float(translation.x),
                    float(translation.y),
                    yaw_from_quaternion(transform.transform.rotation),
                )
            except TransformException as error:
                last_error = error
                rclpy.spin_once(self, timeout_sec=0.1)
        try:
            frame_tree = self.tf_buffer.all_frames_as_yaml()
        except Exception as error:  # diagnostics must not hide the transform failure
            frame_tree = f"<could not enumerate TF frames: {error}>"
        raise SmokeTestError(
            f"Timed out looking up map->{self.base_frame}: {last_error}; "
            f"available TF frames: {frame_tree}"
        )

    def assert_map_goal_clear(
        self, goal_x: float, goal_y: float, clearance: float, max_occupancy: int
    ) -> None:
        if self.map_msg is None:
            raise SmokeTestError("No SLAM occupancy grid is available")
        grid = self.map_msg
        resolution = float(grid.info.resolution)
        if resolution <= 0 or grid.info.width == 0 or grid.info.height == 0:
            raise SmokeTestError("SLAM published an empty or invalid occupancy grid")

        origin = grid.info.origin
        origin_yaw = yaw_from_quaternion(origin.orientation)
        dx = goal_x - origin.position.x
        dy = goal_y - origin.position.y
        local_x = math.cos(origin_yaw) * dx + math.sin(origin_yaw) * dy
        local_y = -math.sin(origin_yaw) * dx + math.cos(origin_yaw) * dy
        map_x = math.floor(local_x / resolution)
        map_y = math.floor(local_y / resolution)
        radius_cells = math.ceil(clearance / resolution)
        data = grid.data

        for offset_y in range(-radius_cells, radius_cells + 1):
            for offset_x in range(-radius_cells, radius_cells + 1):
                if (offset_x * resolution) ** 2 + (offset_y * resolution) ** 2 > clearance**2:
                    continue
                cell_x = map_x + offset_x
                cell_y = map_y + offset_y
                if not (0 <= cell_x < grid.info.width and 0 <= cell_y < grid.info.height):
                    raise SmokeTestError(
                        f"Goal clearance disk extends outside the SLAM map at cell ({cell_x}, {cell_y})"
                    )
                occupancy = int(data[cell_y * grid.info.width + cell_x])
                if occupancy < 0 or occupancy > max_occupancy:
                    raise SmokeTestError(
                        "Requested goal is not known clear in the SLAM map: "
                        f"cell ({cell_x}, {cell_y}) occupancy={occupancy}; "
                        f"allowed range is 0..{max_occupancy}"
                    )

    def assert_map_corridor_clear(
        self,
        start_pose: tuple[float, float, float],
        goal_x: float,
        goal_y: float,
        clearance: float,
        max_occupancy: int,
    ) -> None:
        distance = math.hypot(goal_x - start_pose[0], goal_y - start_pose[1])
        heading = math.atan2(goal_y - start_pose[1], goal_x - start_pose[0])
        step = max(float(self.map_msg.info.resolution), 0.1)
        sample_distance = min(0.6, distance)
        while sample_distance < distance:
            x = start_pose[0] + sample_distance * math.cos(heading)
            y = start_pose[1] + sample_distance * math.sin(heading)
            self.assert_map_goal_clear(x, y, clearance, max_occupancy)
            sample_distance += step

    def select_goal_pose(
        self,
        start_pose: tuple[float, float, float],
        args: argparse.Namespace,
    ) -> tuple[float, float, float, str]:
        if (args.goal_x is None) != (args.goal_y is None):
            raise SmokeTestError("Specify both --goal-x and --goal-y, or neither")
        if args.goal_x is not None:
            self.assert_map_goal_clear(
                args.goal_x, args.goal_y, args.clearance_m, args.max_occupancy
            )
            goal_yaw = args.goal_yaw if args.goal_yaw is not None else 0.0
            return args.goal_x, args.goal_y, goal_yaw, "explicit"

        if args.max_auto_goal_distance_m < args.minimum_goal_distance_m:
            raise SmokeTestError(
                "Maximum automatic goal distance must be at least the minimum distance"
            )
        resolution = float(self.map_msg.info.resolution)
        distance_step = max(2.0 * resolution, 0.1)
        distances = []
        distance = args.max_auto_goal_distance_m
        while distance >= args.minimum_goal_distance_m:
            distances.append(distance)
            distance -= distance_step
        if not distances or distances[-1] > args.minimum_goal_distance_m:
            distances.append(args.minimum_goal_distance_m)

        corridor_clearance = min(0.25, args.clearance_m)
        for distance in distances:
            for direction_index in range(32):
                heading = start_pose[2] + direction_index * (2.0 * math.pi / 32.0)
                goal_x = start_pose[0] + distance * math.cos(heading)
                goal_y = start_pose[1] + distance * math.sin(heading)
                try:
                    self.assert_map_goal_clear(
                        goal_x, goal_y, args.clearance_m, args.max_occupancy
                    )
                    self.assert_map_corridor_clear(
                        start_pose,
                        goal_x,
                        goal_y,
                        corridor_clearance,
                        args.max_occupancy,
                    )
                except SmokeTestError:
                    continue
                goal_yaw = args.goal_yaw if args.goal_yaw is not None else heading
                return goal_x, goal_y, goal_yaw, "auto_known_free_corridor"

        raise SmokeTestError(
            "No known-free, map-clear approach corridor was found between "
            f"{args.minimum_goal_distance_m:.2f} m and "
            f"{args.max_auto_goal_distance_m:.2f} m from the current pose "
            f"(goal clearance {args.clearance_m:.2f} m). Let SLAM observe more of "
            "the warehouse or adjust the explicit goal/clearance arguments."
        )

    def read_parameter(self, node_name: str, parameter_name: str, timeout: float) -> str:
        client = self.create_client(GetParameters, f"{node_name}/get_parameters")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not client.wait_for_service(timeout_sec=0.25):
            rclpy.spin_once(self, timeout_sec=0.1)
        if not client.service_is_ready():
            raise SmokeTestError(f"Nav2 parameter service is unavailable: {node_name}")
        request = GetParameters.Request()
        request.names = [parameter_name]
        future = client.call_async(request)
        while time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            raise SmokeTestError(f"Timed out reading {node_name}.{parameter_name}")
        response = future.result()
        if response is None or not response.values:
            raise SmokeTestError(f"Nav2 returned no value for {node_name}.{parameter_name}")
        value = response.values[0]
        if value.type != ParameterType.PARAMETER_STRING:
            raise SmokeTestError(
                f"Nav2 parameter {node_name}.{parameter_name} is not a string"
            )
        return value.string_value

    def verify_nav2_plugins(self, timeout: float) -> dict[str, str]:
        planner = self.read_parameter(
            f"{ROBOT_NS}/planner_server", "GridBased.plugin", timeout
        )
        controller = self.read_parameter(
            f"{ROBOT_NS}/controller_server", "FollowPath.plugin", timeout
        )
        if planner != EXPECTED_PLANNER:
            raise SmokeTestError(
                f"Unexpected global planner plugin: {planner!r}; expected {EXPECTED_PLANNER!r}"
            )
        if controller != EXPECTED_CONTROLLER:
            raise SmokeTestError(
                f"Unexpected controller plugin: {controller!r}; expected {EXPECTED_CONTROLLER!r}"
            )
        return {"global_planner": planner, "controller": controller}

    def feedback(self, wrapped_feedback) -> None:
        now = time.monotonic()
        if now - self.last_feedback_wall_time >= 5.0:
            remaining = float(wrapped_feedback.feedback.distance_remaining)
            print(f"NavigateToPose active: {remaining:.2f} m remaining", flush=True)
            self.last_feedback_wall_time = now


def write_result(path: Path, result: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def run_test(args: argparse.Namespace) -> dict[str, Any]:
    rclpy.init()
    node = NavigationSmokeNode(args.contact_count, args.base_frame)
    start_pose: tuple[float, float, float] | None = None
    try:
        node.wait_for_clock(args.startup_timeout)
        node.wait_for_robot_state(args.startup_timeout)
        node.wait_for_contact_instrumentation(args.startup_timeout)
        plugins = node.verify_nav2_plugins(args.startup_timeout)

        node.wait_until(
            "Nav2 NavigateToPose action server",
            lambda: node.action_client.wait_for_server(timeout_sec=0.1),
            args.startup_timeout,
        )
        start_pose = node.pose_in_map(args.startup_timeout)
        goal_x, goal_y, goal_yaw, goal_selection = node.select_goal_pose(
            start_pose, args
        )
        distance_to_goal = math.hypot(goal_x - start_pose[0], goal_y - start_pose[1])
        if distance_to_goal < args.minimum_goal_distance_m:
            raise SmokeTestError(
                f"Goal is only {distance_to_goal:.2f} m from the start; "
                f"minimum is {args.minimum_goal_distance_m:.2f} m"
            )
        if node.contact_hits:
            raise SmokeTestError(
                "A non-floor contact was detected before goal dispatch: "
                + json.dumps(node.contact_hits[:5])
            )

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_x
        goal.pose.pose.position.y = goal_y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = math.sin(goal_yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(goal_yaw / 2.0)

        print(
            "Sending NavigateToPose through "
            f"{plugins['global_planner']} with {plugins['controller']}: "
            f"map({goal_x:.2f}, {goal_y:.2f}, yaw={goal_yaw:.2f}) "
            f"[selection={goal_selection}]",
            flush=True,
        )
        send_future = node.action_client.send_goal_async(
            goal, feedback_callback=node.feedback
        )
        deadline = time.monotonic() + args.action_timeout
        while time.monotonic() < deadline and not send_future.done():
            rclpy.spin_once(node, timeout_sec=0.1)
        if not send_future.done():
            raise SmokeTestError("Timed out while submitting NavigateToPose")
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise SmokeTestError("Nav2 rejected the NavigateToPose goal")

        result_future = goal_handle.get_result_async()
        while time.monotonic() < deadline and not result_future.done():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.contact_hits:
                goal_handle.cancel_goal_async()
                raise SmokeTestError(
                    "Gazebo reported robot contact with a non-floor model during navigation: "
                    + json.dumps(node.contact_hits[:5])
                )
        if not result_future.done():
            goal_handle.cancel_goal_async()
            raise SmokeTestError(
                f"NavigateToPose did not finish within {args.action_timeout:.0f}s"
            )
        wrapped_result = result_future.result()
        if wrapped_result is None:
            raise SmokeTestError("NavigateToPose returned no action result")
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            error_code = getattr(wrapped_result.result, "error_code", "unknown")
            error_msg = getattr(wrapped_result.result, "error_msg", "")
            raise SmokeTestError(
                "NavigateToPose action did not succeed: "
                f"status={wrapped_result.status}, error_code={error_code}, error_msg={error_msg!r}"
            )
        action_error_code = getattr(wrapped_result.result, "error_code", 0)
        if int(action_error_code) != 0:
            raise SmokeTestError(
                f"NavigateToPose status succeeded but error_code={action_error_code}"
            )

        final_pose = node.pose_in_map(10.0)
        xy_error = math.hypot(final_pose[0] - goal_x, final_pose[1] - goal_y)
        yaw_error = angular_error(final_pose[2], goal_yaw)
        if xy_error > args.xy_tolerance_m:
            raise SmokeTestError(
                f"Final xy error {xy_error:.3f} m exceeds {args.xy_tolerance_m:.3f} m"
            )
        if yaw_error > args.yaw_tolerance_rad:
            raise SmokeTestError(
                f"Final yaw error {yaw_error:.3f} rad exceeds {args.yaw_tolerance_rad:.3f} rad"
            )

        # Continue receiving contacts briefly after the result to catch late physics updates.
        settle_deadline = time.monotonic() + args.settle_seconds
        while time.monotonic() < settle_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.contact_hits:
            raise SmokeTestError(
                "Gazebo reported robot contact with a non-floor model: "
                + json.dumps(node.contact_hits[:5])
            )

        return {
            "status": "passed",
            "simulation": "Gazebo Harmonic headless; Clearpath Husky A300 warehouse world",
            "action": "NavigateToPose",
            "action_status": "SUCCEEDED",
            "action_error_code": int(action_error_code),
            "plugins": plugins,
            "goal_selection": goal_selection,
            "start_pose_map": {
                "x": start_pose[0],
                "y": start_pose[1],
                "yaw": start_pose[2],
            },
            "goal_pose_map": {
                "x": goal_x,
                "y": goal_y,
                "yaw": goal_yaw,
            },
            "final_pose_map": {
                "x": final_pose[0],
                "y": final_pose[1],
                "yaw": final_pose[2],
            },
            "final_xy_error_m": xy_error,
            "final_yaw_error_rad": yaw_error,
            "xy_tolerance_m": args.xy_tolerance_m,
            "yaw_tolerance_rad": args.yaw_tolerance_rad,
            "collision_check": {
                "instrumented_robot_collision_geometries": len(node.contact_topics),
                "gazebo_contact_publishers_verified": node.gazebo_contact_topics_advertised,
                "contact_messages_received": node.contact_messages_total,
                "per_geometry_messages": node.contact_message_counts,
                "non_floor_contacts": node.contact_hits,
                "result": "no non-floor Gazebo contacts observed",
            },
            "goal_clearance_check": {
                "radius_m": args.clearance_m,
                "approach_corridor_radius_m": (
                    min(0.25, args.clearance_m)
                    if goal_selection == "auto_known_free_corridor"
                    else None
                ),
                "maximum_occupancy": args.max_occupancy,
                "result": "goal neighborhood and selected approach corridor are known free in SLAM occupancy grid",
            },
        }
    finally:
        node.destroy_node()
        rclpy.shutdown()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contact-count", required=True, type=int)
    parser.add_argument("--result-file", required=True, type=Path)
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--goal-x", type=float)
    parser.add_argument("--goal-y", type=float)
    parser.add_argument("--goal-yaw", type=float)
    parser.add_argument("--startup-timeout", type=float, default=150.0)
    parser.add_argument("--action-timeout", type=float, default=180.0)
    parser.add_argument("--settle-seconds", type=float, default=2.0)
    parser.add_argument("--minimum-goal-distance-m", type=float, default=1.25)
    parser.add_argument("--max-auto-goal-distance-m", type=float, default=3.0)
    parser.add_argument("--clearance-m", type=float, default=0.65)
    parser.add_argument("--max-occupancy", type=int, default=20)
    parser.add_argument("--xy-tolerance-m", type=float, default=0.35)
    parser.add_argument("--yaw-tolerance-rad", type=float, default=0.35)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.contact_count <= 0:
        print("contact-count must be positive", file=sys.stderr)
        return 2
    result_path = args.result_file.resolve()
    try:
        result = run_test(args)
    except Exception as error:  # persist concise test failure evidence
        result = {"status": "failed", "reason": str(error)}
        write_result(result_path, result)
        print(f"FAIL: {error}", file=sys.stderr, flush=True)
        return 1
    write_result(result_path, result)
    print("PASS: " + json.dumps(result, sort_keys=True), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
