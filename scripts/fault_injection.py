"""Deterministic helpers for simulation-only fault-injection scenarios."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from collections.abc import Iterable, Sequence

BLOCKED_AISLE_MODEL = "smoke_blocked_aisle_obstacle"
BLOCKED_AISLE_SIZE = (0.55, 1.20, 1.50)
BLOCKED_AISLE_POSE_Z = BLOCKED_AISLE_SIZE[2] / 2.0
BLOCKED_AISLE_MIN_DETOUR_M = 0.45
DEFAULT_BLOCKED_AISLE_CENTER = (1.0, 0.0)
# The A300 Nav2 footprint extends 0.495 m forward/back; preserve another 0.13 m
# beyond the obstacle's 0.275 m half-length on both ends of the selected route.
A300_HALF_LENGTH_M = 0.495
BLOCKED_AISLE_ENDPOINT_BODY_GAP_M = 0.13
BLOCKED_AISLE_MIN_ROUTE_DISTANCE_M = round(
    2.0
    * (
        A300_HALF_LENGTH_M
        + BLOCKED_AISLE_SIZE[0] / 2.0
        + BLOCKED_AISLE_ENDPOINT_BODY_GAP_M
    ),
    2,
)


def blocked_aisle_spawn_pose_args(
    center_xy: tuple[float, float], yaw_rad: float
) -> list[str]:
    """Return explicit Gazebo create pose flags matching the validated model SDF."""
    x, y, yaw = float(center_xy[0]), float(center_xy[1]), float(yaw_rad)
    if not all(math.isfinite(value) for value in (x, y, yaw)):
        raise ValueError("Blocked-aisle spawn pose must contain finite values")
    return [
        "-x",
        f"{x:.6f}",
        "-y",
        f"{y:.6f}",
        "-z",
        f"{BLOCKED_AISLE_POSE_Z:.3f}",
        "-Y",
        f"{yaw:.9f}",
    ]


def find_scan_return(
    ranges: Sequence[float],
    *,
    angle_min_rad: float,
    angle_increment_rad: float,
    target_bearing_rad: float,
    max_angle_error_rad: float,
    min_range_m: float,
    max_range_m: float,
) -> dict[str, float | int] | None:
    """Find the closest angularly aligned finite scan return within a range window."""
    parameters = (
        angle_min_rad,
        angle_increment_rad,
        target_bearing_rad,
        max_angle_error_rad,
        min_range_m,
        max_range_m,
    )
    if (
        not all(math.isfinite(value) for value in parameters)
        or max_angle_error_rad < 0
        or min_range_m < 0
        or max_range_m < min_range_m
    ):
        return None

    candidates: list[tuple[float, float, int, float]] = []
    for index, raw_range in enumerate(ranges):
        measured_range = float(raw_range)
        if not math.isfinite(measured_range) or not (
            min_range_m <= measured_range <= max_range_m
        ):
            continue
        beam_angle = angle_min_rad + index * angle_increment_rad
        angle_error = abs(
            math.atan2(
                math.sin(beam_angle - target_bearing_rad),
                math.cos(beam_angle - target_bearing_rad),
            )
        )
        if angle_error <= max_angle_error_rad:
            candidates.append((angle_error, measured_range, index, beam_angle))

    if not candidates:
        return None
    angle_error, measured_range, index, beam_angle = min(candidates)
    return {
        "index": index,
        "beam_angle_rad": beam_angle,
        "range_m": measured_range,
        "angular_error_rad": angle_error,
    }


def blocked_aisle_model_sdf(
    center_xy: tuple[float, float] = DEFAULT_BLOCKED_AISLE_CENTER,
    yaw_rad: float = 0.0,
) -> str:
    """Return a standalone static box SDF, oriented across a chosen aisle route."""
    if not all(math.isfinite(value) for value in (*center_xy, yaw_rad)):
        raise ValueError("Blocked-aisle model pose must contain finite values")
    size_text = " ".join(f"{value:.3f}" for value in BLOCKED_AISLE_SIZE)
    x, y = center_xy
    return f"""<sdf version='1.9'>
  <model name='{BLOCKED_AISLE_MODEL}'>
    <static>true</static>
    <pose>{x:.6f} {y:.6f} {BLOCKED_AISLE_POSE_Z:.3f} 0 0 {yaw_rad:.9f}</pose>
    <link name='obstacle_link'>
      <collision name='obstacle_collision'>
        <geometry><box><size>{size_text}</size></box></geometry>
      </collision>
      <visual name='obstacle_visual'>
        <geometry><box><size>{size_text}</size></box></geometry>
        <material><ambient>0.8 0.1 0.1 1</ambient><diffuse>0.8 0.1 0.1 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
"""


def blocked_aisle_metadata(
    center_xy: tuple[float, float], yaw_rad: float
) -> dict[str, object]:
    return {
        "scenario": "blocked-aisle",
        "model": BLOCKED_AISLE_MODEL,
        "center_xy_m": [float(center_xy[0]), float(center_xy[1])],
        "orientation_yaw_rad": float(yaw_rad),
        "size_xyz_m": list(BLOCKED_AISLE_SIZE),
        "static": True,
        "minimum_route_distance_m": BLOCKED_AISLE_MIN_ROUTE_DISTANCE_M,
        "minimum_planner_cross_track_deviation_m": BLOCKED_AISLE_MIN_DETOUR_M,
    }


def _parse_vector(text: str | None, *, size: int, label: str) -> tuple[float, ...]:
    try:
        values = tuple(float(value) for value in (text or "").split())
    except ValueError as error:
        raise ValueError(f"Blocked-aisle model has an invalid {label}") from error
    if len(values) != size or not all(math.isfinite(value) for value in values):
        raise ValueError(f"Blocked-aisle model has an invalid {label}")
    return values


def validate_blocked_aisle_model(
    model_sdf: str,
    *,
    expected_center_xy: tuple[float, float] | None = None,
    expected_yaw_rad: float | None = None,
) -> dict[str, object]:
    """Require the SDF to match the expected static box dimensions and pose."""
    try:
        root = ET.fromstring(model_sdf)
    except ET.ParseError as error:
        raise ValueError(f"Blocked-aisle model SDF is malformed: {error}") from error
    model = root.find("model")
    if root.tag != "sdf" or model is None or model.get("name") != BLOCKED_AISLE_MODEL:
        raise ValueError(f"SDF does not contain model {BLOCKED_AISLE_MODEL!r}")
    if model.findtext("static") != "true":
        raise ValueError("Blocked-aisle obstacle must be static")

    x, y, z, roll, pitch, yaw = _parse_vector(
        model.findtext("pose"), size=6, label="pose"
    )
    if not math.isclose(z, BLOCKED_AISLE_POSE_Z, rel_tol=0.0, abs_tol=1e-6):
        raise ValueError("Blocked-aisle obstacle height does not match the expected model")
    if not math.isclose(roll, 0.0, rel_tol=0.0, abs_tol=1e-6) or not math.isclose(
        pitch, 0.0, rel_tol=0.0, abs_tol=1e-6
    ):
        raise ValueError("Blocked-aisle obstacle roll and pitch must be zero")
    if expected_center_xy is not None and any(
        not math.isclose(a, b, rel_tol=0.0, abs_tol=1e-5)
        for a, b in zip((x, y), expected_center_xy)
    ):
        raise ValueError("Blocked-aisle obstacle center does not match the selected route")
    if expected_yaw_rad is not None and not math.isclose(
        math.atan2(math.sin(yaw - expected_yaw_rad), math.cos(yaw - expected_yaw_rad)),
        0.0,
        rel_tol=0.0,
        abs_tol=1e-5,
    ):
        raise ValueError("Blocked-aisle obstacle orientation does not cross the selected route")

    collision_size = _parse_vector(
        model.findtext("link/collision/geometry/box/size"),
        size=3,
        label="collision size",
    )
    visual_size = _parse_vector(
        model.findtext("link/visual/geometry/box/size"),
        size=3,
        label="visual size",
    )
    if collision_size != BLOCKED_AISLE_SIZE or visual_size != BLOCKED_AISLE_SIZE:
        raise ValueError("Blocked-aisle box dimensions do not match the expected collision/visual geometry")
    return blocked_aisle_metadata((x, y), yaw)


def inject_blocked_aisle(
    source_sdf: str,
    center_xy: tuple[float, float] = DEFAULT_BLOCKED_AISLE_CENTER,
    yaw_rad: float = 0.0,
) -> tuple[str, dict[str, object]]:
    """Add the test box to a disposable SDF world (used by offline fixture tests)."""
    if BLOCKED_AISLE_MODEL in source_sdf:
        raise ValueError(f"Source world already contains {BLOCKED_AISLE_MODEL}")
    if source_sdf.count("</world>") != 1:
        raise ValueError("Expected exactly one closing </world> tag in the Gazebo SDF")
    try:
        source_root = ET.fromstring(source_sdf)
    except ET.ParseError as error:
        raise ValueError(f"Source Gazebo SDF is malformed: {error}") from error
    source_worlds = source_root.findall("world")
    if len(source_worlds) != 1 or source_worlds[0].get("name") != "warehouse":
        raise ValueError("Expected exactly one Gazebo world named 'warehouse'")

    model_sdf = blocked_aisle_model_sdf(center_xy, yaw_rad)
    model_root = ET.fromstring(model_sdf)
    model_xml = ET.tostring(model_root.find("model"), encoding="unicode")
    instrumented = source_sdf.replace("</world>", f"\n    {model_xml}\n</world>", 1)
    ET.fromstring(instrumented)
    return instrumented, validate_blocked_aisle_model(
        model_sdf, expected_center_xy=center_xy, expected_yaw_rad=yaw_rad
    )


def world_has_blocked_aisle(source_sdf: str) -> bool:
    """Report whether a prepared world already contains the obstacle model."""
    try:
        root = ET.fromstring(source_sdf)
    except ET.ParseError as error:
        raise ValueError(f"Prepared Gazebo world is malformed: {error}") from error
    worlds = root.findall("world")
    if len(worlds) != 1 or worlds[0].get("name") != "warehouse":
        raise ValueError("Expected exactly one Gazebo world named 'warehouse'")
    return worlds[0].find(f"model[@name='{BLOCKED_AISLE_MODEL}']") is not None


def max_cross_track_deviation(
    points: Iterable[tuple[float, float]],
    start: tuple[float, float],
    goal: tuple[float, float],
) -> float:
    """Return maximum perpendicular deviation for route points on the goal segment."""
    dx = goal[0] - start[0]
    dy = goal[1] - start[1]
    length_squared = dx * dx + dy * dy
    length = math.sqrt(length_squared)
    if length == 0.0:
        raise ValueError("start and goal must be different points")
    deviations = []
    for point_x, point_y in points:
        projection = ((point_x - start[0]) * dx + (point_y - start[1]) * dy) / length_squared
        if 0.0 <= projection <= 1.0:
            deviations.append(abs(dx * (point_y - start[1]) - dy * (point_x - start[0])) / length)
    return max(deviations, default=0.0)
