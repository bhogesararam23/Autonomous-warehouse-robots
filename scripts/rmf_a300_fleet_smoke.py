#!/usr/bin/env python3
"""Fleet-dispatch two RMF goals and verify schedule reporting and contact-free execution."""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import time
import uuid
from pathlib import Path
from typing import Any

import rclpy
from builtin_interfaces.msg import Time as RosTime
from rmf_fleet_msgs.msg import FleetState, MutexGroupStates
from rmf_task_msgs.msg import ApiRequest, ApiResponse, DispatchState, DispatchStates
from rmf_traffic_msgs.msg import ItinerarySet, NegotiationNotice, Participants
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from ros_gz_interfaces.msg import Contacts
from warehouse_rmf_fleet.coordinate_transform import rmf_to_local
from warehouse_rmf_fleet.fleet_status_freshness import stale_status_ages

FLEET = "warehouse_a300_fleet"
ROBOTS = ("a300_00000", "a300_00001")
GOAL_NAMES = ("goal_east", "goal_west")
STARTS = {"a300_00000": (0.0, 0.0), "a300_00001": (3.5, 0.0)}
START_YAWS = {"a300_00000": 0.0, "a300_00001": math.pi}
GOAL_POSES = {"goal_east": (2.5, 0.0), "goal_west": (1.0, 0.0)}
NAV2_GOAL_XY_TOLERANCE_M = 0.05
NAV2_GOAL_YAW_TOLERANCE_RAD = 0.10
GOAL_XY_TOLERANCE_M = 0.15
GOAL_YAW_TOLERANCE_RAD = 0.25
GOAL_YAWS = {"a300_00000": 0.0, "a300_00001": math.pi}
MAP_ORIGINS_IN_RMF = {
    "a300_00000": (0.0, 0.0, 0.0),
    "a300_00001": (3.5, 0.0, math.pi),
}
PREFLIGHT_PATHS = {
    "a300_00000": (("goal_west", (1.0, 0.0)),),
    "a300_00001": (("goal_east", (2.5, 0.0)),),
}
FOOTPRINT_DIAMETER_M = 1.22
MUTEX_GROUP = "a300_shared_aisle"
UNCLAIMED_PARTICIPANT = (1 << 64) - 1
MAX_SLAM_OCCUPANCY = 20
PREFLIGHT_CLEARANCE_M = 0.25


def _find_task_id(payload: Any) -> str | None:
    if isinstance(payload, dict):
        value = payload.get("task_id")
        if isinstance(value, str) and value:
            return value
        booking = payload.get("booking")
        if isinstance(booking, dict):
            booking_id = booking.get("id")
            if isinstance(booking_id, str) and booking_id:
                return booking_id
        for child in payload.values():
            found = _find_task_id(child)
            if found:
                return found
    elif isinstance(payload, list):
        for child in payload:
            found = _find_task_id(child)
            if found:
                return found
    return None


class FleetSmokeError(RuntimeError):
    """A precondition or end-to-end assertion failed."""


def _angle_error(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def _yaw_from_quaternion(q: Any) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _waypoint_tuple(waypoint: Any) -> tuple[int, tuple[float, float, float]]:
    position = tuple(float(value) for value in waypoint.position)
    return int(waypoint.time), (position[0], position[1], position[2])


def _trajectory_samples(routes: list[dict[str, Any]]) -> list[tuple[int, float, float]]:
    samples: list[tuple[int, float, float]] = []
    for route in routes:
        points = route.get("waypoints", [])
        if not points:
            continue
        samples.extend((stamp, pose[0], pose[1]) for stamp, pose in points)
        for (t0, p0), (t1, p1) in zip(points, points[1:]):
            duration = t1 - t0
            if duration <= 0:
                continue
            steps = max(1, math.ceil(duration / 250_000_000))
            for step in range(1, steps):
                ratio = step / steps
                samples.append(
                    (
                        round(t0 + ratio * duration),
                        p0[0] + ratio * (p1[0] - p0[0]),
                        p0[1] + ratio * (p1[1] - p0[1]),
                    )
                )
    return sorted(samples, key=lambda item: item[0])


def _minimum_simultaneous_distance(
    first: list[tuple[int, float, float]], second: list[tuple[int, float, float]]
) -> float | None:
    """Estimate closest center distance when scheduled trajectory samples overlap in time."""
    if not first or not second:
        return None
    index = 0
    minimum = math.inf
    tolerance_ns = 350_000_000
    for stamp, x, y in first:
        while index + 1 < len(second) and second[index + 1][0] <= stamp:
            index += 1
        for candidate in (second[index], second[min(index + 1, len(second) - 1)]):
            other_stamp, other_x, other_y = candidate
            if abs(stamp - other_stamp) <= tolerance_ns:
                minimum = min(minimum, math.hypot(x - other_x, y - other_y))
    return None if math.isinf(minimum) else minimum


class FleetSmokeNode(Node):
    def __init__(
        self, contact_count: int, status_stale_after: float, use_sim_time: bool = True
    ) -> None:
        super().__init__(
            "a300_rmf_fleet_smoke",
            parameter_overrides=[Parameter("use_sim_time", value=use_sim_time)],
        )
        transient_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.api_pub = self.create_publisher(ApiRequest, "task_api_requests", transient_qos)
        self.responses: dict[str, dict[str, Any]] = {}
        self.dispatches: dict[str, dict[str, Any]] = {}
        self.request_ids: dict[str, str] = {}
        self.fleet_states: dict[str, Any] = {}
        self.fleet_state_wall_times: dict[str, float] = {}
        self.status_stale_after = status_stale_after
        self.stale_injection_robot: str | None = None
        self.stale_injection_drops: dict[str, int] = {robot: 0 for robot in ROBOTS}
        self.maps: dict[str, OccupancyGrid] = {}
        self.map_wall_times: dict[str, float] = {}
        self.initial_map_checks: dict[str, dict[str, Any]] = {}
        self.participant_names: dict[int, str] = {}
        self.itinerary_events = 0
        self.itineraries: dict[int, dict[str, Any]] = {}
        self.negotiation_notices = 0
        self.mutex_state_events = 0
        self.mutex_assignments: dict[str, int] = {}
        self.mutex_owner_history: list[dict[str, Any]] = []
        self.second_dispatch_precondition: dict[str, Any] | None = None
        self.contact_counts = {robot: [0] * contact_count for robot in ROBOTS}
        self.contact_messages = 0
        self.contact_hits: list[dict[str, Any]] = []
        self.contact_topics = []

        self.map_subscriptions = []
        for robot in ROBOTS:
            self.map_subscriptions.append(
                self.create_subscription(
                    OccupancyGrid,
                    f"/{robot}/map",
                    lambda message, robot_name=robot: self._on_map(robot_name, message),
                    10,
                )
            )

        self.create_subscription(ApiResponse, "task_api_responses", self._on_response, 30)
        self.create_subscription(DispatchStates, "dispatch_states", self._on_dispatch_states, 10)
        self.create_subscription(FleetState, "fleet_states", self._on_fleet_state, 20)
        self.create_subscription(Participants, "/rmf_traffic/participants", self._on_participants, transient_qos)
        self.create_subscription(MutexGroupStates, "/mutex_group_states", self._on_mutex_group_states, 10)
        self.create_subscription(ItinerarySet, "/rmf_traffic/itinerary_set", self._on_itinerary, 100)
        self.create_subscription(NegotiationNotice, "/rmf_traffic/negotiation_notice", self._on_negotiation_notice, 100)
        self.contact_subscriptions = []
        for robot in ROBOTS:
            for index in range(contact_count):
                topic = f"/{robot}/smoke/contact/sensor_{index:02d}"
                self.contact_topics.append(topic)
                self.contact_subscriptions.append(
                    self.create_subscription(
                        Contacts,
                        topic,
                        lambda message, robot_name=robot, sensor=index: self._on_contacts(
                            robot_name, sensor, message
                        ),
                        10,
                    )
                )

    def _on_response(self, message: ApiResponse) -> None:
        try:
            payload = json.loads(message.json_msg)
        except json.JSONDecodeError:
            payload = {"raw": message.json_msg}
        self.responses[message.request_id] = {"type": int(message.type), "payload": payload}

    def _on_dispatch_states(self, message: DispatchStates) -> None:
        for state in [*message.active, *message.finished]:
            assignment = state.assignment
            self.dispatches[state.task_id] = {
                "status": int(state.status),
                "is_assigned": bool(assignment.is_assigned),
                "fleet_name": assignment.fleet_name,
                "expected_robot_name": assignment.expected_robot_name,
                "errors": list(state.errors),
            }

    def dispatch_state_for_request(self, request_id: str) -> dict[str, Any] | None:
        response = self.responses.get(request_id)
        payload = response.get("payload", {}) if response else {}
        task_id = _find_task_id(payload)
        for key in (task_id, request_id):
            if key and key in self.dispatches:
                return self.dispatches[key]
        return None

    def _on_fleet_state(self, message: FleetState) -> None:
        for robot in message.robots:
            name = str(robot.name)
            if name == self.stale_injection_robot and name in self.fleet_states:
                self.stale_injection_drops[name] += 1
                continue
            self.fleet_states[name] = robot
            self.fleet_state_wall_times[name] = time.monotonic()

    def stale_fleet_statuses(self, now: float | None = None) -> dict[str, float]:
        return stale_status_ages(
            self.fleet_state_wall_times,
            now=time.monotonic() if now is None else now,
            max_age_seconds=self.status_stale_after,
        )

    def _on_map(self, robot: str, message: OccupancyGrid) -> None:
        self.maps[robot] = message
        self.map_wall_times[robot] = time.monotonic()

    def _on_participants(self, message: Participants) -> None:
        for participant in message.participants:
            self.participant_names[int(participant.id)] = str(participant.description.name)

    def _on_mutex_group_states(self, message: MutexGroupStates) -> None:
        self.mutex_state_events += 1
        for assignment in message.assignments:
            group = str(assignment.group)
            claimant = int(assignment.claimant)
            previous = self.mutex_assignments.get(group)
            if previous != claimant:
                self.mutex_owner_history.append(
                    {
                        "group": group,
                        "claimant": claimant,
                        "claim_time": {
                            "sec": int(assignment.claim_time.sec),
                            "nanosec": int(assignment.claim_time.nanosec),
                        },
                    }
                )
                self.mutex_assignments[group] = claimant

    def _on_itinerary(self, message: ItinerarySet) -> None:
        self.itinerary_events += 1
        routes = []
        for route in message.itinerary:
            trajectory = getattr(route, "trajectory", None)
            waypoints = []
            if trajectory is not None:
                waypoints = [_waypoint_tuple(point) for point in trajectory.waypoints]
            routes.append({"map": str(route.map), "waypoints": waypoints})
        record = {
            "participant": int(message.participant),
            "plan": int(message.plan),
            "itinerary_version": int(message.itinerary_version),
            "routes": routes,
        }
        if routes:
            self.itineraries[int(message.participant)] = record

    def _on_negotiation_notice(self, message: NegotiationNotice) -> None:
        self.negotiation_notices += 1

    def _on_contacts(self, robot: str, sensor: int, message: Contacts) -> None:
        self.contact_counts[robot][sensor] += 1
        self.contact_messages += 1
        for contact in message.contacts:
            first = contact.collision1.name
            second = contact.collision2.name
            if self._is_floor(first) or self._is_floor(second):
                continue
            first_robot = next((name for name in ROBOTS if name in first), None)
            second_robot = next((name for name in ROBOTS if name in second), None)
            if first_robot == robot and second_robot == robot:
                continue
            self.contact_hits.append(
                {
                    "sensor_robot": robot,
                    "sensor_index": sensor,
                    "collision1": first,
                    "collision2": second,
                    "sim_time": {
                        "sec": int(message.header.stamp.sec),
                        "nanosec": int(message.header.stamp.nanosec),
                    },
                }
            )

    @staticmethod
    def _is_floor(collision_name: str) -> bool:
        text = collision_name.lower()
        return any(token in text for token in ("ground_plane", "floor"))

    def spin_until(
        self,
        description: str,
        predicate,
        timeout: float,
        *,
        require_fresh_status: bool = False,
    ) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if require_fresh_status:
                stale = self.stale_fleet_statuses()
                if stale:
                    ages = {robot: round(age, 3) for robot, age in stale.items()}
                    raise FleetSmokeError(
                        f"RMF robot status became stale (age seconds): {ages}"
                    )
            if predicate():
                return
        raise FleetSmokeError(f"Timed out waiting for {description} after {timeout:.0f}s")

    def verify_contact_publishers(self, timeout: float) -> None:
        self.spin_until(
            "ROS bridges for every instrumented contact sensor",
            lambda: all(self.count_publishers(topic) > 0 for topic in self.contact_topics),
            timeout,
        )
        listing = subprocess.run(
            ["gz", "topic", "-l"], check=True, capture_output=True, text=True, timeout=15.0
        ).stdout.splitlines()
        advertised = set(listing)
        missing = [topic for topic in self.contact_topics if topic not in advertised]
        if missing:
            raise FleetSmokeError("Gazebo is missing contact topics: " + ", ".join(missing[:8]))
        for robot in ROBOTS:
            topic = f"/{robot}/smoke/contact/sensor_00"
            details = subprocess.run(
                ["gz", "topic", "-i", "-t", topic],
                check=True,
                capture_output=True,
                text=True,
                timeout=6.0,
            ).stdout.lower()
            if "publisher" not in details or "gz.msgs.contacts" not in details:
                raise FleetSmokeError(f"Gazebo did not confirm the Contacts publisher for {topic}")

    def _known_clear_disk(
        self, robot: str, grid: OccupancyGrid, rmf_x: float, rmf_y: float, clearance: float
    ) -> tuple[int, int]:
        resolution = float(grid.info.resolution)
        if resolution <= 0.0 or grid.info.width == 0 or grid.info.height == 0:
            raise FleetSmokeError(f"{robot} published an invalid SLAM occupancy grid")
        local_x, local_y, _ = rmf_to_local(
            (rmf_x, rmf_y, 0.0), MAP_ORIGINS_IN_RMF[robot]
        )
        grid_origin = grid.info.origin
        origin_yaw = _yaw_from_quaternion(grid_origin.orientation)
        dx = local_x - float(grid_origin.position.x)
        dy = local_y - float(grid_origin.position.y)
        map_x_m = math.cos(origin_yaw) * dx + math.sin(origin_yaw) * dy
        map_y_m = -math.sin(origin_yaw) * dx + math.cos(origin_yaw) * dy
        center_x = math.floor(map_x_m / resolution)
        center_y = math.floor(map_y_m / resolution)
        radius_cells = math.ceil(clearance / resolution)
        checked = 0
        maximum_occupancy = -1
        for offset_y in range(-radius_cells, radius_cells + 1):
            for offset_x in range(-radius_cells, radius_cells + 1):
                if (offset_x * resolution) ** 2 + (offset_y * resolution) ** 2 > clearance**2:
                    continue
                cell_x = center_x + offset_x
                cell_y = center_y + offset_y
                if not (0 <= cell_x < grid.info.width and 0 <= cell_y < grid.info.height):
                    raise FleetSmokeError(
                        f"{robot} SLAM clearance disk for RMF ({rmf_x:.2f}, {rmf_y:.2f}) "
                        f"extends outside the map at cell ({cell_x}, {cell_y})"
                    )
                occupancy = int(grid.data[cell_y * grid.info.width + cell_x])
                if occupancy < 0 or occupancy > MAX_SLAM_OCCUPANCY:
                    raise FleetSmokeError(
                        f"{robot} short-route cell is not known clear: RMF "
                        f"({rmf_x:.2f}, {rmf_y:.2f}), map cell ({cell_x}, {cell_y}), "
                        f"occupancy={occupancy}, allowed=0..{MAX_SLAM_OCCUPANCY}"
                    )
                maximum_occupancy = max(maximum_occupancy, occupancy)
                checked += 1
        return checked, maximum_occupancy

    def check_initial_map_routes(self, start_poses: dict[str, list[float]]) -> None:
        """Reject unknown or occupied mission routes before RMF task dispatch."""
        self.initial_map_checks = {}
        for robot in ROBOTS:
            grid = self.maps.get(robot)
            if grid is None:
                raise FleetSmokeError(f"No SLAM occupancy grid received from {robot}")
            self.initial_map_checks[robot] = {}
            previous_x, previous_y = start_poses[robot][:2]
            for name, (goal_x, goal_y) in PREFLIGHT_PATHS[robot]:
                distance = math.hypot(goal_x - previous_x, goal_y - previous_y)
                sample_step = max(float(grid.info.resolution), 0.1)
                sample_distance = min(0.6, distance)
                checked_cells = 0
                max_occupancy = 0
                record = {
                    "checkpoint": name,
                    "rmf_pose": [goal_x, goal_y],
                    "clearance_m": PREFLIGHT_CLEARANCE_M,
                    "checked_cells": 0,
                    "maximum_occupancy": None,
                }
                self.initial_map_checks[robot][name] = record
                while sample_distance < distance:
                    ratio = sample_distance / distance if distance else 1.0
                    x = previous_x + ratio * (goal_x - previous_x)
                    y = previous_y + ratio * (goal_y - previous_y)
                    cells, occupancy = self._known_clear_disk(
                        robot, grid, x, y, PREFLIGHT_CLEARANCE_M
                    )
                    checked_cells += cells
                    max_occupancy = max(max_occupancy, occupancy)
                    sample_distance += sample_step
                cells, occupancy = self._known_clear_disk(
                    robot, grid, goal_x, goal_y, PREFLIGHT_CLEARANCE_M
                )
                checked_cells += cells
                max_occupancy = max(max_occupancy, occupancy)
                record["checked_cells"] = checked_cells
                record["maximum_occupancy"] = max_occupancy
                record["status"] = "known_free"
                previous_x, previous_y = goal_x, goal_y

    def dispatch_goal_tasks(self) -> dict[str, str]:
        request_ids: dict[str, str] = {}
        assigned_robots: set[str] = set()
        for goal in GOAL_NAMES:
            now: RosTime = self.get_clock().now().to_msg()
            start_ms = int(now.sec) * 1000 + round(int(now.nanosec) / 1_000_000)
            request_id = "fleet_" + str(uuid.uuid4())
            request_ids[goal] = request_id
            self.request_ids[goal] = request_id
            task = {
                "category": "compose",
                "description": {
                    "category": "go_to_place",
                    "phases": [
                        {
                            "activity": {
                                "category": "go_to_place",
                                "description": {"one_of": [{"waypoint": goal}]},
                            }
                        }
                    ],
                },
                "unix_millis_request_time": start_ms,
                "unix_millis_earliest_start_time": start_ms,
                "requester": "warehouse_a300_fleet_smoke",
                "fleet_name": FLEET,
            }
            payload = {"type": "dispatch_task_request", "request": task}
            message = ApiRequest()
            message.request_id = request_id
            message.json_msg = json.dumps(payload)
            self.api_pub.publish(message)
            self.get_logger().info(
                f"Submitted fleet-dispatch goal: {goal} for {FLEET} (request_id={request_id})"
            )
            self.spin_until(
                f"RMF task API response for {goal}",
                lambda: request_id in self.responses
                and self.responses[request_id]["type"] == ApiResponse.TYPE_RESPONDING,
                20.0,
            )
            response = self.responses[request_id]["payload"]
            if response.get("success") is False:
                raise FleetSmokeError(f"RMF rejected fleet-dispatch request for {goal}: {response}")
            self.spin_until(
                f"RMF dispatcher to assign {goal} to an idle A300",
                lambda: (state := self.dispatch_state_for_request(request_id)) is not None
                and state["status"] == DispatchState.STATUS_DISPATCHED
                and state["is_assigned"]
                and state["expected_robot_name"] in ROBOTS
                and state["expected_robot_name"] not in assigned_robots,
                90.0,
            )
            assignment = self.dispatch_state_for_request(request_id)
            assigned_robot = str(assignment["expected_robot_name"])
            assigned_robots.add(assigned_robot)
            self.get_logger().info(f"RMF dispatcher assigned {goal} to {assigned_robot}")
            if len(assigned_robots) == 1:
                first_robot = assigned_robot
                start_x, start_y = STARTS[first_robot]

                def first_robot_has_active_task_and_progressed() -> bool:
                    state = self.fleet_states.get(first_robot)
                    if state is None:
                        return False
                    distance = math.hypot(
                        float(state.location.x) - start_x,
                        float(state.location.y) - start_y,
                    )
                    return bool(str(state.task_id).strip()) and distance >= 0.15

                self.spin_until(
                    f"{first_robot} with an active RMF task and at least 0.15 m of measured pose progress",
                    first_robot_has_active_task_and_progressed,
                    120.0,
                )
                state = self.fleet_states[first_robot]
                distance = math.hypot(
                    float(state.location.x) - start_x,
                    float(state.location.y) - start_y,
                )
                self.second_dispatch_precondition = {
                    "first_goal": goal,
                    "first_robot": first_robot,
                    "first_task_request_id": request_id,
                    "first_robot_task_id": str(state.task_id),
                    "submitted_while_first_task_active_after_pose_progress": True,
                    "first_robot_mode": int(state.mode.mode),
                    "first_robot_pose_rmf": [
                        float(state.location.x),
                        float(state.location.y),
                        float(state.location.yaw),
                    ],
                    "first_robot_distance_from_start_m": distance,
                }
                self.get_logger().info(
                    f"First A300 has active task {state.task_id!s} and moved {distance:.2f} m; "
                    "submitting the second task while its task remains active"
                )
        return request_ids

    def snapshot(self, request_ids: dict[str, str], start_poses: dict[str, list[float]]) -> dict[str, Any]:
        goals = {}
        for goal in GOAL_NAMES:
            request_id = request_ids[goal]
            response = self.responses.get(request_id)
            task_payload = response.get("payload", {}) if response else {}
            task_id = _find_task_id(task_payload) or request_id
            dispatch_state = self.dispatch_state_for_request(request_id)
            robot = dispatch_state.get("expected_robot_name") if dispatch_state else None
            if robot not in ROBOTS or robot not in self.fleet_states:
                raise FleetSmokeError(f"No assigned fleet state is available for goal {goal}: {dispatch_state}")
            state = self.fleet_states[robot]
            x, y, yaw = float(state.location.x), float(state.location.y), float(state.location.yaw)
            gx, gy = GOAL_POSES[goal]
            goal_yaw = GOAL_YAWS[robot]
            goals[robot] = {
                "task_goal": goal,
                "goal_pose_rmf": {"x": gx, "y": gy, "yaw": goal_yaw},
                "task_request_id": request_id,
                "rmf_task_id": task_id,
                "dispatch_response": response,
                "dispatch_state": dispatch_state,
                "start_pose_rmf": start_poses[robot],
                "start_yaw_error_rad": _angle_error(start_poses[robot][2], START_YAWS[robot]),
                "final_pose_rmf": {"x": x, "y": y, "yaw": yaw},
                "goal_xy_error_m": math.hypot(x - gx, y - gy),
                "goal_yaw_error_rad": _angle_error(yaw, goal_yaw),
                "mode": int(state.mode.mode),
                "fleet_state_task_id": str(state.task_id),
            }

        robot_records: dict[str, Any] = {}
        participant_by_robot: dict[str, int] = {}
        for participant_id, record in self.itineraries.items():
            name = self.participant_names.get(participant_id, "")
            for robot in ROBOTS:
                if name == robot or name.endswith("/" + robot) or name.endswith("_" + robot):
                    participant_by_robot[robot] = participant_id
                    robot_records[robot] = record
        route_samples = {
            robot: _trajectory_samples(robot_records[robot]["routes"])
            for robot in robot_records
        }
        min_distance = None
        if all(robot in route_samples for robot in ROBOTS):
            min_distance = _minimum_simultaneous_distance(
                route_samples[ROBOTS[0]], route_samples[ROBOTS[1]]
            )

        mutex_history = []
        mutex_owners_seen = []
        for record in self.mutex_owner_history:
            if record["group"] != MUTEX_GROUP:
                continue
            claimant = int(record["claimant"])
            participant_name = (
                "unclaimed"
                if claimant == UNCLAIMED_PARTICIPANT
                else self.participant_names.get(claimant, "")
            )
            robot_name = next(
                (
                    robot
                    for robot in ROBOTS
                    if participant_name == robot
                    or participant_name.endswith("/" + robot)
                    or participant_name.endswith("_" + robot)
                ),
                None,
            )
            item = {**record, "participant_name": participant_name, "robot_name": robot_name}
            mutex_history.append(item)
            if robot_name is not None and robot_name not in mutex_owners_seen:
                mutex_owners_seen.append(robot_name)

        if set(ROBOTS).issubset(mutex_owners_seen):
            conflict_result = (
                f"RMF mutex-group supervisor reported ownership of {MUTEX_GROUP} "
                f"by {', '.join(mutex_owners_seen)}; both fleet-dispatched goals completed without non-floor contacts"
            )
        elif self.negotiation_notices:
            conflict_result = "RMF negotiation notices observed; both fleet-dispatched goals completed without non-floor Gazebo contacts"
        elif min_distance is not None and min_distance >= FOOTPRINT_DIAMETER_M - 0.02:
            conflict_result = "captured RMF trajectories are time-separated by at least the modeled A300 footprint diameter"
        else:
            conflict_result = "both tasks completed and native itineraries were reported, but no negotiation event or comparable timed samples proved deconfliction"

        return {
            "status": "passed",
            "simulation": "Gazebo Harmonic headless; Clearpath Husky A300 warehouse world",
            "fleet_status_freshness": {
                "clock": "local monotonic receipt time; FleetState has no per-robot source timestamp",
                "maximum_age_seconds": self.status_stale_after,
                "last_update_age_seconds": {
                    robot: round(time.monotonic() - self.fleet_state_wall_times[robot], 3)
                    for robot in ROBOTS
                },
            },
            "navigation_completion_criteria": {
                "nav2_goal_checker_plugin": "nav2_controller::SimpleGoalChecker",
                "nav2_xy_goal_tolerance_m": NAV2_GOAL_XY_TOLERANCE_M,
                "nav2_yaw_goal_tolerance_rad": NAV2_GOAL_YAW_TOLERANCE_RAD,
                "smoke_xy_tolerance_m": GOAL_XY_TOLERANCE_M,
                "smoke_yaw_tolerance_rad": GOAL_YAW_TOLERANCE_RAD,
                "rmf_task_id_must_be_empty": True,
                "rmf_mode_must_be_idle": True,
            },
            "slam_map_preflight": self.initial_map_checks,
            "dispatch": {
                "fleet": FLEET,
                "request_type": "dispatch_task_request",
                "goals": goals,
                "assigned_robot_names": sorted(goals),
                "second_task_overlap_evidence": self.second_dispatch_precondition,
            },
            "rmf_schedule": {
                "itinerary_set_events": self.itinerary_events,
                "itinerary_participant_ids": {
                    robot: participant_by_robot.get(robot) for robot in ROBOTS
                },
                "itinerary_route_counts": {
                    robot: len(robot_records[robot]["routes"]) if robot in robot_records else 0
                    for robot in ROBOTS
                },
                "negotiation_notice_count": self.negotiation_notices,
                "mutex_group": MUTEX_GROUP,
                "mutex_state_events": self.mutex_state_events,
                "mutex_owner_history": mutex_history,
                "mutex_robots_observed_as_owner": mutex_owners_seen,
                "minimum_sampled_center_distance_m": min_distance,
                "modeled_footprint_diameter_m": FOOTPRINT_DIAMETER_M,
                "conflict_handling": conflict_result,
            },
            "gazebo_contacts": {
                "instrumented_collision_geometries_per_robot": {
                    robot: len(self.contact_counts[robot]) for robot in ROBOTS
                },
                "contact_messages_received": self.contact_messages,
                "per_sensor_messages": self.contact_counts,
                "non_floor_contacts": self.contact_hits,
                "result": "no non-floor Gazebo contacts observed" if not self.contact_hits else "unexpected contact observed",
            },
        }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contact-count", type=int, default=17)
    parser.add_argument("--nav2-xy-goal-tolerance-m", type=float, default=NAV2_GOAL_XY_TOLERANCE_M)
    parser.add_argument("--nav2-yaw-goal-tolerance-rad", type=float, default=NAV2_GOAL_YAW_TOLERANCE_RAD)
    parser.add_argument("--timeout", type=float, default=300.0)
    parser.add_argument("--status-stale-after", type=float, default=15.0)
    parser.add_argument("--status-fault-timeout", type=float, default=35.0)
    parser.add_argument(
        "--fault-injection", choices=("none", "stale-status"), default="none"
    )
    parser.add_argument("--result-file", type=Path, required=True)
    args = parser.parse_args()
    if args.contact_count < 1:
        parser.error("--contact-count must be positive")
    if args.status_stale_after <= 0.0 or args.status_fault_timeout <= args.status_stale_after:
        parser.error("status fault timeout must exceed a positive status stale threshold")
    if (
        args.nav2_xy_goal_tolerance_m <= 0.0
        or args.nav2_yaw_goal_tolerance_rad <= 0.0
        or GOAL_XY_TOLERANCE_M <= args.nav2_xy_goal_tolerance_m
        or GOAL_YAW_TOLERANCE_RAD <= args.nav2_yaw_goal_tolerance_rad
    ):
        parser.error("smoke goal tolerances must be positive and wider than the Nav2 goal-checker tolerances")
    if (
        abs(args.nav2_xy_goal_tolerance_m - NAV2_GOAL_XY_TOLERANCE_M) > 1.0e-9
        or abs(args.nav2_yaw_goal_tolerance_rad - NAV2_GOAL_YAW_TOLERANCE_RAD) > 1.0e-9
    ):
        parser.error("runtime Nav2 goal-checker parameters do not match the smoke configuration")

    rclpy.init(args=sys.argv)
    node = FleetSmokeNode(args.contact_count, args.status_stale_after)
    try:
        node.spin_until("non-zero Gazebo clock", lambda: node.get_clock().now().nanoseconds > 0, 60.0)
        node.verify_contact_publishers(60.0)
        node.spin_until(
            "Open-RMF mutex-group supervisor publisher",
            lambda: node.count_publishers("/mutex_group_states") > 0,
            60.0,
        )
        node.spin_until(
            "both RMF fleet states",
            lambda: set(ROBOTS).issubset(node.fleet_states),
            120.0,
        )
        node.spin_until(
            "Open-RMF task dispatcher state publisher",
            lambda: node.count_publishers("/dispatch_states") > 0,
            60.0,
        )
        node.spin_until(
            "fresh SLAM occupancy grids from both A300s",
            lambda: set(ROBOTS).issubset(node.maps)
            and all(time.monotonic() - node.map_wall_times[robot] < 5.0 for robot in ROBOTS),
            60.0,
            require_fresh_status=True,
        )
        start_poses = {}
        for robot in ROBOTS:
            state = node.fleet_states[robot]
            pose = [float(state.location.x), float(state.location.y), float(state.location.yaw)]
            start_poses[robot] = pose
            expected = STARTS[robot]
            error = math.hypot(pose[0] - expected[0], pose[1] - expected[1])
            yaw_error = _angle_error(pose[2], START_YAWS[robot])
            if state.location.level_name != "L1" or error > 0.65 or yaw_error > 0.35:
                raise FleetSmokeError(
                    f"{robot} RMF start pose {pose} does not align with its graph start {expected} "
                    f"(level={state.location.level_name}, xy_error={error:.3f}m, "
                    f"yaw_error={yaw_error:.3f}rad; expected_yaw={START_YAWS[robot]:.3f}rad)"
                )
        if args.fault_injection == "stale-status":
            injected_robot = ROBOTS[0]
            node.stale_injection_robot = injected_robot
            print(
                f"Injecting observer-side status loss for {injected_robot}; "
                "new FleetState entries for that robot will be discarded.",
                flush=True,
            )
            node.spin_until(
                f"stale status detection for {injected_robot}",
                lambda: injected_robot in node.stale_fleet_statuses(),
                args.status_fault_timeout,
            )
            stale_age = node.stale_fleet_statuses()[injected_robot]
            if node.stale_injection_drops[injected_robot] < 1:
                raise FleetSmokeError("Stale status injection did not suppress any FleetState updates")
            fault_result = {
                "status": "passed",
                "simulation": "Gazebo Harmonic headless; Clearpath Husky A300 warehouse world",
                "fault_injection": {
                    "scenario": "stale-status",
                    "injection_boundary": "RMF smoke-test consumer discards updates for one robot after its initial status",
                    "robot": injected_robot,
                    "stale_threshold_seconds": args.status_stale_after,
                    "detected_status_age_seconds": round(stale_age, 3),
                    "discarded_updates": node.stale_injection_drops[injected_robot],
                    "result": "stale robot status detected before any task was dispatched",
                    "limitation": "This deterministically tests consumer-side stale detection; it does not simulate a DDS/network outage or claim an RMF adapter watchdog.",
                },
            }
            args.result_file.parent.mkdir(parents=True, exist_ok=True)
            args.result_file.write_text(json.dumps(fault_result, indent=2) + "\n", encoding="utf-8")
            print(json.dumps(fault_result, indent=2))
            print(f"PASS: stale FleetState was detected before dispatch. Evidence: {args.result_file}")
            return 0
        node.check_initial_map_routes(start_poses)
        if node.contact_hits:
            raise FleetSmokeError("Non-floor contact observed before task dispatch: " + json.dumps(node.contact_hits[:4]))

        request_ids = node.dispatch_goal_tasks()
        node.spin_until(
            "final task API responses",
            lambda: all(
                request_id in node.responses
                and node.responses[request_id]["type"] == ApiResponse.TYPE_RESPONDING
                for request_id in request_ids.values()
            ),
            20.0,
            require_fresh_status=True,
        )
        failed_responses = {
            goal: node.responses[request_id]
            for goal, request_id in request_ids.items()
            if node.responses[request_id]["type"] != ApiResponse.TYPE_RESPONDING
            or node.responses[request_id]["payload"].get("success") is False
        }
        if failed_responses:
            raise FleetSmokeError("RMF rejected task API requests: " + json.dumps(failed_responses))

        def all_goals_reached_by_distinct_robots() -> bool:
            assigned_robots = set()
            for goal in GOAL_NAMES:
                dispatch_state = node.dispatch_state_for_request(request_ids[goal])
                if (
                    dispatch_state is None
                    or dispatch_state["status"] != DispatchState.STATUS_DISPATCHED
                    or not dispatch_state["is_assigned"]
                    or dispatch_state["fleet_name"] != FLEET
                ):
                    return False
                robot = dispatch_state["expected_robot_name"]
                if robot not in ROBOTS or robot in assigned_robots or robot not in node.fleet_states:
                    return False
                assigned_robots.add(robot)
                state = node.fleet_states[robot]
                gx, gy = GOAL_POSES[goal]
                xy_error = math.hypot(
                    float(state.location.x) - gx, float(state.location.y) - gy
                )
                yaw_error = _angle_error(float(state.location.yaw), GOAL_YAWS[robot])
                if (
                    str(state.task_id).strip()
                    or int(state.mode.mode) != 0
                    or xy_error > GOAL_XY_TOLERANCE_M
                    or yaw_error > GOAL_YAW_TOLERANCE_RAD
                ):
                    return False
            return True

        node.spin_until(
            "the two fleet-dispatched goals to finish on distinct A300s",
            all_goals_reached_by_distinct_robots,
            args.timeout,
            require_fresh_status=True,
        )
        time.sleep(1.0)
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.contact_hits:
            raise FleetSmokeError("Non-floor Gazebo contact observed: " + json.dumps(node.contact_hits[:6]))
        result = node.snapshot(request_ids, start_poses)
        if result["rmf_schedule"]["itinerary_set_events"] == 0:
            raise FleetSmokeError("No native RMF itinerary_set reports were observed")
        if not all(result["rmf_schedule"]["itinerary_route_counts"][robot] > 0 for robot in ROBOTS):
            raise FleetSmokeError("Could not associate itinerary reports with both A300 schedule participants")
        if not set(ROBOTS).issubset(result["rmf_schedule"]["mutex_robots_observed_as_owner"]):
            raise FleetSmokeError(
                f"The shared mutex group was not observed under both robot owners: "
                f"{result['rmf_schedule']['mutex_owner_history']}"
            )
        overlap = result["dispatch"]["second_task_overlap_evidence"]
        if (
            not overlap
            or not overlap["submitted_while_first_task_active_after_pose_progress"]
            or not overlap["first_robot_task_id"]
            or overlap["first_robot_distance_from_start_m"] < 0.15
        ):
            raise FleetSmokeError(
                f"The second fleet request was not submitted after measurable progress under an active first task: {overlap}"
            )
        assigned_robots = set()
        for robot, record in result["dispatch"]["goals"].items():
            state = record["dispatch_state"] or {}
            if state.get("status") == DispatchState.STATUS_FAILED_TO_ASSIGN or state.get("errors"):
                raise FleetSmokeError(f"RMF dispatch for {robot} failed: {state}")
            if (
                not state.get("is_assigned")
                or state.get("fleet_name") != FLEET
                or state.get("expected_robot_name") != robot
            ):
                raise FleetSmokeError(f"RMF task assignment does not match {robot}: {state}")
            assigned_robots.add(robot)
            if record["fleet_state_task_id"]:
                raise FleetSmokeError(
                    f"{robot} still has active RMF task "
                    f"{record['fleet_state_task_id']!r} after the completion wait: {record}"
                )
            if record["mode"] != 0:
                raise FleetSmokeError(f"{robot} is not idle after its RMF task: {record}")
            if record["goal_xy_error_m"] > GOAL_XY_TOLERANCE_M:
                raise FleetSmokeError(
                    f"{robot} final XY error {record['goal_xy_error_m']:.3f} m exceeds "
                    f"{GOAL_XY_TOLERANCE_M:.3f} m: {record}"
                )
            if record["goal_yaw_error_rad"] > GOAL_YAW_TOLERANCE_RAD:
                raise FleetSmokeError(
                    f"{robot} final yaw error {record['goal_yaw_error_rad']:.3f} rad exceeds "
                    f"{GOAL_YAW_TOLERANCE_RAD:.3f} rad: {record}"
                )
        if assigned_robots != set(ROBOTS):
            raise FleetSmokeError(f"RMF did not assign one distinct goal to each A300: {sorted(assigned_robots)}")
        if node.contact_hits:
            raise FleetSmokeError("Non-floor Gazebo contact observed during fleet motion")

        args.result_file.parent.mkdir(parents=True, exist_ok=True)
        args.result_file.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(result, indent=2))
        print(f"PASS: RMF fleet-dispatched both goals to distinct A300s with overlapping task demand, observed shared-mutex ownership and itineraries, and found no non-floor contacts. Evidence: {args.result_file}")
        return 0
    except Exception as error:
        observed_robots = {}
        for robot in ROBOTS:
            state = node.fleet_states.get(robot)
            if state is not None:
                observed_robots[robot] = {
                    "x": float(state.location.x),
                    "y": float(state.location.y),
                    "yaw": float(state.location.yaw),
                    "level": str(state.location.level_name),
                    "mode": int(state.mode.mode),
                    "task_id": str(state.task_id),
                }
        failure = {
            "status": "failed",
            "reason": str(error),
            "fleet_status_freshness": {
                "maximum_age_seconds": node.status_stale_after,
                "stale_age_seconds": {
                    robot: round(age, 3)
                    for robot, age in node.stale_fleet_statuses().items()
                },
                "injected_discarded_updates": node.stale_injection_drops,
            },
            "task_request_ids": node.request_ids,
            "observed_robots": observed_robots,
            "dispatch_states": node.dispatches,
            "task_api_responses": node.responses,
            "itinerary_set_events": node.itinerary_events,
            "itineraries": node.itineraries,
            "negotiation_notice_count": node.negotiation_notices,
            "mutex_state_events": node.mutex_state_events,
            "mutex_assignments": node.mutex_assignments,
            "mutex_owner_history": node.mutex_owner_history,
            "slam_map_preflight": node.initial_map_checks,
            "contact_messages_received": node.contact_messages,
            "non_floor_contacts": node.contact_hits[:20],
        }
        args.result_file.parent.mkdir(parents=True, exist_ok=True)
        args.result_file.write_text(json.dumps(failure, indent=2) + "\n", encoding="utf-8")
        print(f"FAIL: {error}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
