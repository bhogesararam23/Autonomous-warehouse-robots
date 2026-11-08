"""Small, ROS-independent parameter profile for the blocked-aisle smoke run."""

from __future__ import annotations

import math
from typing import Any


FAULT_PROGRESS_MOVEMENT_ALLOWANCE_S = 30.0


def _positive_number(settings: dict[str, Any], name: str, label: str) -> float:
    try:
        value = float(settings[name])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"Nav2 {label} is missing or is not numeric") from error
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"Nav2 {label} must be a finite positive number")
    return value


def apply_fault_test_profile(params: dict[str, Any]) -> dict[str, Any]:
    """Preserve vendor goal tolerances and allow bounded slow progress in the fault run.

    This profile deliberately changes only the progress checker's movement time
    allowance. The A300's installed goal-checker tolerances remain authoritative;
    the smoke client independently enforces its own final-pose tolerances.
    """
    controller = params.get("controller_server", {}).get("ros__parameters")
    if not isinstance(controller, dict):
        raise ValueError("Nav2 controller_server parameters are missing")

    goal_plugins = controller.get("goal_checker_plugins")
    goal_checker = controller.get("general_goal_checker")
    if (
        not isinstance(goal_plugins, (list, tuple))
        or "general_goal_checker" not in goal_plugins
        or not isinstance(goal_checker, dict)
    ):
        raise ValueError("Clearpath A300 Nav2 general_goal_checker schema changed")
    goal_xy = _positive_number(
        goal_checker, "xy_goal_tolerance", "general_goal_checker.xy_goal_tolerance"
    )
    goal_yaw = _positive_number(
        goal_checker, "yaw_goal_tolerance", "general_goal_checker.yaw_goal_tolerance"
    )

    progress_plugins = controller.get("progress_checker_plugins")
    progress_checker = controller.get("progress_checker")
    if (
        not isinstance(progress_plugins, (list, tuple))
        or "progress_checker" not in progress_plugins
        or not isinstance(progress_checker, dict)
    ):
        raise ValueError("Clearpath A300 Nav2 progress_checker schema changed")
    required_radius = _positive_number(
        progress_checker,
        "required_movement_radius",
        "progress_checker.required_movement_radius",
    )
    default_allowance = _positive_number(
        progress_checker,
        "movement_time_allowance",
        "progress_checker.movement_time_allowance",
    )

    progress_checker["movement_time_allowance"] = FAULT_PROGRESS_MOVEMENT_ALLOWANCE_S
    return {
        "profile": "blocked-aisle-simulation",
        "goal_checker_xy_tolerance_m": goal_xy,
        "goal_checker_yaw_tolerance_rad": goal_yaw,
        "progress_checker_required_movement_radius_m": required_radius,
        "progress_checker_default_movement_time_allowance_s": default_allowance,
        "progress_checker_movement_time_allowance_s": FAULT_PROGRESS_MOVEMENT_ALLOWANCE_S,
    }
