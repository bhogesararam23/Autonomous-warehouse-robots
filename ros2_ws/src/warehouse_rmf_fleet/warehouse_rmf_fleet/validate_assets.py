"""Validate the generated RMF graph and fleet configuration against the RMF parser."""

from __future__ import annotations

import argparse
import json
import math
from collections import deque
from pathlib import Path
from typing import Any

import rmf_adapter.easy_full_control as rmf_easy
import yaml

EXPECTED_ROBOTS = {"a300_00000", "a300_00001"}
EXPECTED_GOALS = {"goal_west", "goal_east"}
EXPECTED_STARTS = {"a300_00000_start", "a300_00001_start"}
MUTEX_GROUP = "a300_shared_aisle"
EXPECTED_MUTEX_WAYPOINTS: set[str] = set()
MIN_LAYOUT_CLEARANCE_M = 1.35  # Simulation layout threshold; not a safety rating.
EXPECTED_GRAPH_POSES = {
    "a300_00000_start": (0.0, 0.0),
    "goal_west": (1.0, 0.0),
    "goal_east": (2.5, 0.0),
    "a300_00001_start": (3.5, 0.0),
}


def _load_yaml(path: Path) -> dict[str, Any]:
    loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(loaded, dict):
        raise ValueError(f"Expected a YAML mapping at {path}")
    return loaded


def _reachable(adjacency: dict[int, set[int]], start: int, end: int) -> bool:
    seen = {start}
    pending = deque([start])
    while pending:
        current = pending.popleft()
        if current == end:
            return True
        for neighbor in adjacency.get(current, set()):
            if neighbor not in seen:
                seen.add(neighbor)
                pending.append(neighbor)
    return False


def validate_assets(config_path: Path, graph_path: Path) -> dict[str, Any]:
    config = _load_yaml(config_path)
    graph_data = _load_yaml(graph_path)
    fleet = config.get("rmf_fleet", {})
    interfaces = config.get("robot_interfaces", {})
    if set(fleet.get("robots", {})) != EXPECTED_ROBOTS:
        raise ValueError("rmf_fleet.robots must define exactly a300_00000 and a300_00001")
    if set(interfaces) != EXPECTED_ROBOTS:
        raise ValueError("robot_interfaces must define exactly both A300 robots")

    levels = graph_data.get("levels", {})
    if "L1" not in levels:
        raise ValueError("The generated navigation graph must contain level L1")
    level = levels["L1"]
    vertices = level.get("vertices", [])
    lanes = level.get("lanes", [])
    names: dict[str, int] = {}
    mutex_waypoints: set[str] = set()
    for index, vertex in enumerate(vertices):
        if len(vertex) != 3:
            raise ValueError(f"Vertex {index} must contain x, y, and properties")
        properties = vertex[2] or {}
        name = properties.get("name")
        if name and properties.get("mutex") == MUTEX_GROUP:
            mutex_waypoints.add(str(name))
            if not properties.get("is_holding_point"):
                raise ValueError(f"Mutex waypoint {name!r} must be a holding point for short Nav2 legs")
        if name:
            if name in names:
                raise ValueError(f"Duplicate waypoint name {name!r}")
            names[str(name)] = index
        if name in EXPECTED_STARTS | EXPECTED_GOALS and not properties.get("is_holding_point"):
            raise ValueError(f"Dispatch/start waypoint {name!r} must be a holding point")
        if name in EXPECTED_STARTS | EXPECTED_GOALS and properties.get("mutex"):
            raise ValueError(f"Dispatch/start waypoint {name!r} must remain outside the aisle mutex")
    if not EXPECTED_STARTS | EXPECTED_GOALS <= set(names):
        missing = (EXPECTED_STARTS | EXPECTED_GOALS) - set(names)
        raise ValueError(f"Missing named start/goals: {sorted(missing)}")
    if mutex_waypoints != EXPECTED_MUTEX_WAYPOINTS:
        raise ValueError(
            f"The shared aisle must protect exactly {sorted(EXPECTED_MUTEX_WAYPOINTS)}; "
            f"found {sorted(mutex_waypoints)}"
        )
    for waypoint_name, (expected_x, expected_y) in EXPECTED_GRAPH_POSES.items():
        waypoint = vertices[names[waypoint_name]]
        if abs(float(waypoint[0]) - expected_x) > 0.01 or abs(float(waypoint[1]) - expected_y) > 0.01:
            raise ValueError(
                f"{waypoint_name} must be at ({expected_x}, {expected_y}) in the current fleet layout"
            )
    if set(fleet["robots"]) != set(interfaces):
        raise ValueError("RMF robot names and ROS interface entries must match")
    for robot_name, robot_config in fleet["robots"].items():
        charger = robot_config.get("charger")
        if not charger or charger not in names:
            raise ValueError(
                f"{robot_name}: charger waypoint {charger!r} is absent from the navigation graph"
            )
        charger_properties = vertices[names[charger]][2] or {}
        if not charger_properties.get("is_charger"):
            raise ValueError(
                f"{robot_name}: waypoint {charger!r} must be marked is_charger in the navigation graph"
            )

    adjacency: dict[int, set[int]] = {}
    directed_lanes: set[tuple[int, int]] = set()
    mutex_lanes: list[tuple[int, int]] = []
    for lane in lanes:
        if len(lane) != 3:
            raise ValueError(f"Invalid lane entry: {lane!r}")
        source, target, properties = int(lane[0]), int(lane[1]), lane[2] or {}
        if not (0 <= source < len(vertices) and 0 <= target < len(vertices)):
            raise ValueError(f"Lane references nonexistent vertex: {lane!r}")
        adjacency.setdefault(source, set()).add(target)
        directed_lanes.add((source, target))
        if properties.get("mutex") == MUTEX_GROUP:
            mutex_lanes.append((source, target))
    expected_mutex_pairs = {
        (names["a300_00000_start"], names["goal_west"]),
        (names["goal_west"], names["goal_east"]),
        (names["goal_east"], names["a300_00001_start"]),
    }
    expected_mutex_directions = {
        direction
        for source, target in expected_mutex_pairs
        for direction in ((source, target), (target, source))
    }
    if set(mutex_lanes) != expected_mutex_directions:
        raise ValueError(
            "The shared aisle mutex must cover both directions of all three linear corridor lanes; "
            f"found {sorted(mutex_lanes)}"
        )
    if directed_lanes != expected_mutex_directions:
        raise ValueError("The validated fleet graph must contain only the three protected bidirectional aisle lanes")
    start_poses = {
        name: (float(vertices[names[name]][0]), float(vertices[names[name]][1]))
        for name in EXPECTED_STARTS
    }
    goal_poses = {
        name: (float(vertices[names[name]][0]), float(vertices[names[name]][1]))
        for name in EXPECTED_GOALS
    }
    goal_separation = math.hypot(
        goal_poses["goal_west"][0] - goal_poses["goal_east"][0],
        goal_poses["goal_west"][1] - goal_poses["goal_east"][1],
    )
    if goal_separation < MIN_LAYOUT_CLEARANCE_M:
        raise ValueError(
            f"The two dispatch goals are only {goal_separation:.3f} m apart; "
            f"layout threshold is {MIN_LAYOUT_CLEARANCE_M:.2f} m"
        )
    opposite_start_clearance = {
        "goal_west": math.hypot(
            goal_poses["goal_west"][0] - start_poses["a300_00001_start"][0],
            goal_poses["goal_west"][1] - start_poses["a300_00001_start"][1],
        ),
        "goal_east": math.hypot(
            goal_poses["goal_east"][0] - start_poses["a300_00000_start"][0],
            goal_poses["goal_east"][1] - start_poses["a300_00000_start"][1],
        ),
    }
    if min(opposite_start_clearance.values()) < MIN_LAYOUT_CLEARANCE_M:
        raise ValueError(
            "A dispatch goal is too close to the other parked start: "
            f"{opposite_start_clearance} (threshold {MIN_LAYOUT_CLEARANCE_M:.2f} m)"
        )
    for start_name, goal_name in (
        ("a300_00000_start", "goal_west"),
        ("a300_00001_start", "goal_east"),
    ):
        start_pose = start_poses[start_name]
        goal_pose = goal_poses[goal_name]
        distance = math.hypot(goal_pose[0] - start_pose[0], goal_pose[1] - start_pose[1])
        if distance < 0.8:
            raise ValueError(f"{goal_name} is too close to its assigned start to test navigation ({distance:.3f} m)")
    for start_name in EXPECTED_STARTS:
        for goal_name in EXPECTED_GOALS:
            if not _reachable(adjacency, names[start_name], names[goal_name]):
                raise ValueError(f"No directed route from {start_name} to {goal_name}")

    for robot_name, interface in interfaces.items():
        origin = interface.get("map_origin_in_rmf")
        spawn = interface.get("spawn_pose_in_rmf")
        if not isinstance(origin, list) or len(origin) != 3:
            raise ValueError(f"{robot_name}: map_origin_in_rmf must be [x,y,yaw]")
        if not isinstance(spawn, list) or len(spawn) != 3:
            raise ValueError(f"{robot_name}: spawn_pose_in_rmf must be [x,y,yaw]")
        start_name = f"{robot_name}_start"
        vertex = vertices[names[start_name]]
        if abs(float(vertex[0]) - float(spawn[0])) > 0.01 or abs(float(vertex[1]) - float(spawn[1])) > 0.01:
            raise ValueError(f"{robot_name}: configured spawn does not match its RMF start waypoint")
        if any(abs(float(origin[i]) - float(spawn[i])) > 0.01 for i in range(3)):
            raise ValueError(f"{robot_name}: map-origin transform must match the local SLAM spawn pose")
        if not str(interface.get("namespace", "")).strip("/"):
            raise ValueError(f"{robot_name}: ROS namespace is empty")

    parsed = rmf_easy.FleetConfiguration.from_config_files(
        str(config_path), str(graph_path)
    )
    if parsed is None:
        raise ValueError("Open-RMF Easy Full Control rejected the fleet configuration/graph")
    parsed_names = {str(name) for name in parsed.known_robots}
    if parsed_names != EXPECTED_ROBOTS:
        raise ValueError(f"RMF parser produced unexpected robots: {sorted(parsed_names)}")

    return {
        "status": "valid",
        "fleet": str(fleet["name"]),
        "robots": sorted(parsed_names),
        "level": "L1",
        "waypoints": {name: [float(vertices[index][0]), float(vertices[index][1])] for name, index in names.items()},
        "dispatch_goals": sorted(EXPECTED_GOALS),
        "dispatch_goal_poses": {
            name: [float(vertices[names[name]][0]), float(vertices[names[name]][1])]
            for name in sorted(EXPECTED_GOALS)
        },
        "shared_mutex_waypoints": sorted(mutex_waypoints),
        "simulation_layout_clearance_threshold_m": MIN_LAYOUT_CLEARANCE_M,
        "charger_waypoints": {
            robot: str(fleet["robots"][robot]["charger"])
            for robot in sorted(fleet["robots"])
        },
        "shared_mutex_directed_lanes": len(mutex_lanes),
        "holding_points": sorted(name for name, index in names.items() if vertices[index][2].get("is_holding_point")),
        "rmf_parser": "accepted",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--nav-graph", type=Path, required=True)
    args = parser.parse_args()
    try:
        result = validate_assets(args.config, args.nav_graph)
    except Exception as error:
        parser.exit(1, f"Fleet asset validation failed: {error}\n")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
