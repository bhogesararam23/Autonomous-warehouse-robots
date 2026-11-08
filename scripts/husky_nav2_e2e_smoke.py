#!/usr/bin/env python3
"""Exercise NavigateToPose and verify result, final pose, and Gazebo contacts."""

from __future__ import annotations

import argparse
import hashlib
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
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from ros_gz_interfaces.msg import Contacts
from tf2_ros import Buffer, TransformException, TransformListener
from fault_injection import (
    BLOCKED_AISLE_MIN_DETOUR_M,
    blocked_aisle_model_sdf,
    max_cross_track_deviation,
    validate_blocked_aisle_model,
    world_has_blocked_aisle,
)


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
        self.plan_message_count = 0
        self.latest_plan_points: list[tuple[float, float]] = []
        self.plan_history: list[list[tuple[float, float]]] = []
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
        self.plan_subscription = self.create_subscription(
            NavPath, f"{ROBOT_NS}/plan", self._on_plan, 10
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

    def _on_plan(self, message: NavPath) -> None:
        self.plan_message_count += 1
        self.latest_plan_points = [
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in message.poses
        ]
        self.plan_history.append(self.latest_plan_points.copy())

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

    def map_point_in_odom(self, x: float, y: float) -> tuple[float, float, float]:
        if self.odom_msg is None:
            raise SmokeTestError("Odometry is unavailable while transforming obstacle pose")
        odom_frame = self.odom_msg.header.frame_id or "odom"
        try:
            transform = self.tf_buffer.lookup_transform(odom_frame, "map", Time())
        except TransformException as error:
            raise SmokeTestError(f"Could not transform obstacle pose from map to {odom_frame}: {error}") from error
        translation = transform.transform.translation
        yaw = yaw_from_quaternion(transform.transform.rotation)
        return (
            math.cos(yaw) * x - math.sin(yaw) * y + float(translation.x),
            math.sin(yaw) * x + math.cos(yaw) * y + float(translation.y),
            yaw,
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

    def map_contains_occupied_near(
        self, x: float, y: float, radius: float, max_occupancy: int
    ) -> bool:
        if self.map_msg is None:
            return False
        grid = self.map_msg
        resolution = float(grid.info.resolution)
        if resolution <= 0 or grid.info.width == 0 or grid.info.height == 0:
            return False
        origin = grid.info.origin
        origin_yaw = yaw_from_quaternion(origin.orientation)
        dx = x - origin.position.x
        dy = y - origin.position.y
        local_x = math.cos(origin_yaw) * dx + math.sin(origin_yaw) * dy
        local_y = -math.sin(origin_yaw) * dx + math.cos(origin_yaw) * dy
        center_x = math.floor(local_x / resolution)
        center_y = math.floor(local_y / resolution)
        radius_cells = math.ceil(radius / resolution)
        for offset_y in range(-radius_cells, radius_cells + 1):
            for offset_x in range(-radius_cells, radius_cells + 1):
                if (offset_x * resolution) ** 2 + (offset_y * resolution) ** 2 > radius**2:
                    continue
                cell_x = center_x + offset_x
                cell_y = center_y + offset_y
                if not (0 <= cell_x < grid.info.width and 0 <= cell_y < grid.info.height):
                    continue
                if int(grid.data[cell_y * grid.info.width + cell_x]) > max_occupancy:
                    return True
        return False

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


def spawn_blocked_aisle(
    node: NavigationSmokeNode,
    args: argparse.Namespace,
    initial_pose: tuple[float, float, float],
    goal_xy: tuple[float, float],
    fault_manifest: dict[str, Any],
) -> dict[str, Any]:
    if args.fault_model_file is None:
        raise SmokeTestError("blocked-aisle requires a validated --fault-model-file")
    dx = goal_xy[0] - initial_pose[0]
    dy = goal_xy[1] - initial_pose[1]
    distance = math.hypot(dx, dy)
    if distance < 2.0:
        raise SmokeTestError("Blocked-aisle fault needs at least 2.0 m of known-free route")
    center_map = ((initial_pose[0] + goal_xy[0]) / 2.0, (initial_pose[1] + goal_xy[1]) / 2.0)
    map_heading = math.atan2(dy, dx)
    center_world_x, center_world_y, map_to_odom_yaw = node.map_point_in_odom(*center_map)
    world_yaw = map_heading + map_to_odom_yaw
    model_sdf = blocked_aisle_model_sdf((center_world_x, center_world_y), world_yaw)
    runtime_metadata = validate_blocked_aisle_model(
        model_sdf,
        expected_center_xy=(center_world_x, center_world_y),
        expected_yaw_rad=world_yaw,
    )
    model_digest = hashlib.sha256(model_sdf.encode("utf-8")).hexdigest()
    args.fault_model_file.write_text(model_sdf, encoding="utf-8")
    fault_manifest.update(runtime_metadata)
    fault_manifest["center_map_xy_m"] = list(center_map)
    fault_manifest["injection_timing"] = "after-known-free-map-preflight"
    fault_manifest["model_sha256"] = model_digest
    fault_manifest["model_file"] = str(args.fault_model_file.resolve())
    fault_manifest.pop("model_template_sha256", None)
    if args.fault_manifest is not None:
        complete_manifest = json.loads(args.fault_manifest.read_text(encoding="utf-8"))
        complete_manifest["fault_injection"] = fault_manifest
        args.fault_manifest.write_text(
            json.dumps(complete_manifest, indent=2) + "\n", encoding="utf-8"
        )

    spawn_time = time.monotonic()
    completed = subprocess.run(
        [
            "ros2",
            "run",
            "ros_gz_sim",
            "create",
            "-world",
            "warehouse",
            "-file",
            str(args.fault_model_file),
        ],
        capture_output=True,
        text=True,
        timeout=25.0,
        check=False,
    )
    response = (completed.stdout + completed.stderr).strip()
    if completed.returncode != 0:
        raise SmokeTestError(
            "Gazebo failed to spawn the blocked-aisle model: "
            f"exit={completed.returncode}; response={response[-1200:]}"
        )

    node.wait_until(
        "SLAM occupancy evidence of the spawned aisle obstacle",
        lambda: node.last_map_wall_time is not None
        and node.last_map_wall_time >= spawn_time
        and node.map_contains_occupied_near(
            center_map[0], center_map[1], radius=0.40, max_occupancy=args.max_occupancy
        ),
        25.0,
    )
    return {
        "spawn_command": "ros2 run ros_gz_sim create -world warehouse -file <validated model>",
        "spawn_response": response[-1200:],
        "map_center_xy_m": list(center_map),
        "world_center_xy_m": [center_world_x, center_world_y],
        "world_orientation_yaw_rad": world_yaw,
        "occupied_map_evidence": True,
    }


def run_test(args: argparse.Namespace) -> dict[str, Any]:
    fault_manifest: dict[str, Any] = {"scenario": "none"}
    if args.fault_manifest is not None:
        fault_manifest = json.loads(args.fault_manifest.read_text(encoding="utf-8"))
        fault_manifest = fault_manifest.get("fault_injection", {"scenario": "none"})
    if fault_manifest.get("scenario") != args.fault_injection:
        raise SmokeTestError(
            "Prepared world fault does not match the requested scenario: "
            f"manifest={fault_manifest.get('scenario')!r}, requested={args.fault_injection!r}"
        )
    if args.fault_injection != "none" and args.fault_manifest is None:
        raise SmokeTestError("A fault-injection run requires --fault-manifest evidence")
    if args.world_file is not None:
        if world_has_blocked_aisle(args.world_file.read_text(encoding="utf-8")):
            raise SmokeTestError(
                "The launched world already contains the blocked-aisle model; "
                "it must be spawned after the known-free map preflight"
            )
    if args.fault_injection == "blocked-aisle":
        if args.world_file is None or args.fault_model_file is None:
            raise SmokeTestError(
                "blocked-aisle requires --world-file and --fault-model-file evidence"
            )
        model_sdf = args.fault_model_file.read_text(encoding="utf-8")
        model_metadata = validate_blocked_aisle_model(model_sdf)
        for key in (
            "scenario",
            "model",
            "size_xyz_m",
            "static",
            "minimum_planner_cross_track_deviation_m",
        ):
            value = model_metadata[key]
            if fault_manifest.get(key) != value:
                raise SmokeTestError(
                    f"Fault manifest/model mismatch for {key}: "
                    f"manifest={fault_manifest.get(key)!r}, model={value!r}"
                )
        model_template_digest = hashlib.sha256(model_sdf.encode("utf-8")).hexdigest()
        if fault_manifest.get("model_template_sha256") != model_template_digest:
            raise SmokeTestError("Fault manifest SHA-256 does not match the obstacle model template")
        if fault_manifest.get("model_file") != str(args.fault_model_file.resolve()):
            raise SmokeTestError("Fault manifest points to a different obstacle model file")
        if fault_manifest.get("injection_timing") != "after-known-free-map-preflight":
            raise SmokeTestError("Fault manifest does not record post-map obstacle injection")

    rclpy.init()
    node = NavigationSmokeNode(args.contact_count, args.base_frame)
    start_pose: tuple[float, float, float] | None = None
    obstacle_spawn_evidence: dict[str, Any] | None = None
    associated_plan: list[tuple[float, float]] = []
    associated_plan_count = 0
    plan_start_error = None
    plan_goal_error = None
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
        corridor_clearance = (
            min(0.25, args.clearance_m)
            if goal_selection == "auto_known_free_corridor"
            else None
        )
        if args.fault_injection == "blocked-aisle":
            corridor_clearance = min(0.25, args.clearance_m)
            node.assert_map_corridor_clear(
                start_pose,
                goal_x,
                goal_y,
                corridor_clearance,
                args.max_occupancy,
            )
            print(
                "Verified the requested goal and direct corridor are known free "
                "before spawning the test obstacle.",
                flush=True,
            )
            obstacle_spawn_evidence = spawn_blocked_aisle(
                node, args, start_pose, (goal_x, goal_y), fault_manifest
            )
            if node.contact_hits:
                raise SmokeTestError(
                    "A non-floor contact was detected after obstacle spawn: "
                    + json.dumps(node.contact_hits[:5])
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
        dispatch_plan_count = node.plan_message_count
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

        max_plan_deviation = None
        if args.fault_injection == "blocked-aisle":
            matching_plans = []
            endpoint_tolerance = max(0.5, args.xy_tolerance_m)
            for plan in node.plan_history[dispatch_plan_count:]:
                if len(plan) < 2:
                    continue
                start_error = math.hypot(
                    plan[0][0] - start_pose[0], plan[0][1] - start_pose[1]
                )
                goal_error = math.hypot(
                    plan[-1][0] - goal_x, plan[-1][1] - goal_y
                )
                if start_error <= endpoint_tolerance and goal_error <= endpoint_tolerance:
                    matching_plans.append((plan, start_error, goal_error))
            if not matching_plans:
                latest = node.plan_history[-1] if node.plan_history else []
                raise SmokeTestError(
                    "No post-dispatch Nav2 global plan matched the requested start/goal; "
                    f"captured_after_dispatch={len(node.plan_history[dispatch_plan_count:])}, "
                    f"latest_first={latest[0] if latest else None}, "
                    f"latest_last={latest[-1] if latest else None}"
                )
            associated_plan, plan_start_error, plan_goal_error = matching_plans[-1]
            associated_plan_count = len(matching_plans)
            max_plan_deviation = max_cross_track_deviation(
                associated_plan,
                (start_pose[0], start_pose[1]),
                (goal_x, goal_y),
            )
            if max_plan_deviation < BLOCKED_AISLE_MIN_DETOUR_M:
                raise SmokeTestError(
                    "The global planner did not measurably route around the injected aisle block: "
                    f"cross-track deviation={max_plan_deviation:.3f} m, "
                    f"required>={BLOCKED_AISLE_MIN_DETOUR_M:.3f} m"
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
            "fault_injection": {
                **fault_manifest,
                "scenario": args.fault_injection,
                "global_plan_topic": f"{ROBOT_NS}/plan",
                "plan_topic_messages_total": node.plan_message_count,
                "captured_plan_messages": associated_plan_count,
                "global_plan_points_map": (
                    [{"x": x, "y": y} for x, y in associated_plan]
                    if args.fault_injection == "blocked-aisle"
                    else []
                ),
                "plan_start_error_m": plan_start_error,
                "plan_goal_error_m": plan_goal_error,
                "max_cross_track_deviation_m": max_plan_deviation,
                "obstacle_spawn": obstacle_spawn_evidence,
                "result": (
                    "global plan detoured around the injected static obstacle"
                    if args.fault_injection == "blocked-aisle"
                    else "no fault injected"
                ),
            },
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
                "approach_corridor_radius_m": corridor_clearance,
                "maximum_occupancy": args.max_occupancy,
                "result": (
                    "goal neighborhood and direct corridor were known free immediately before obstacle injection"
                    if args.fault_injection == "blocked-aisle"
                    else "goal neighborhood and selected approach corridor are known free in SLAM occupancy grid"
                ),
            },
        }
    finally:
        node.destroy_node()
        rclpy.shutdown()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contact-count", required=True, type=int)
    parser.add_argument("--result-file", required=True, type=Path)
    parser.add_argument("--fault-manifest", type=Path)
    parser.add_argument("--world-file", type=Path)
    parser.add_argument("--fault-model-file", type=Path)
    parser.add_argument(
        "--fault-injection", choices=("none", "blocked-aisle"), default="none"
    )
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
    if args.fault_injection != "none" and args.fault_manifest is None:
        print("fault-injection requires --fault-manifest", file=sys.stderr)
        return 2
    if args.fault_injection == "blocked-aisle" and (
        args.world_file is None or args.fault_model_file is None
    ):
        print(
            "blocked-aisle requires --world-file and --fault-model-file evidence",
            file=sys.stderr,
        )
        return 2
    result_path = args.result_file.resolve()
    try:
        result = run_test(args)
    except Exception as error:  # persist concise test failure evidence
        result = {
            "status": "failed",
            "reason": str(error),
            "fault_injection": args.fault_injection,
        }
        write_result(result_path, result)
        print(f"FAIL: {error}", file=sys.stderr, flush=True)
        return 1
    write_result(result_path, result)
    print("PASS: " + json.dumps(result, sort_keys=True), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
