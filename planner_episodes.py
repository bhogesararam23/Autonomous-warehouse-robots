"""Deterministic seed-to-endpoint planner episodes and replay traces."""

from __future__ import annotations

from typing import Any

from warehouse_astar_sim import (
    DEFAULT_SCENARIO_ID,
    GRID_HEIGHT,
    GRID_WIDTH,
    InvalidPlannerInputError,
    PlannerSuccess,
    build_obstacles,
    get_joint_period,
    get_scenario,
    is_safe_after_arrival,
    moving_obstacle_positions,
    plan_warehouse,
)


_UINT32_MASK = 0xFFFFFFFF
_INITIAL_XORSHIFT32_STATE = 0x6D2B79F5


class XorShift32:
    """Small deterministic unsigned xorshift32 generator."""

    def __init__(self, seed: int) -> None:
        _validate_seed(seed)
        self.state = _INITIAL_XORSHIFT32_STATE if seed == 0 else seed

    def next_u32(self) -> int:
        x = self.state
        x ^= (x << 13) & _UINT32_MASK
        x &= _UINT32_MASK
        x ^= x >> 17
        x &= _UINT32_MASK
        x ^= (x << 5) & _UINT32_MASK
        x &= _UINT32_MASK
        self.state = x
        return x


def _validate_seed(seed: int) -> None:
    if type(seed) is not int or not 0 <= seed <= _UINT32_MASK:
        raise InvalidPlannerInputError(
            "Seed must be an integer in the range 0 through 0xffffffff."
        )


def _point_event(point: tuple[int, int], time_step: int) -> dict[str, int | str]:
    return {
        "type": "robot-position",
        "timeStep": time_step,
        "x": point[0],
        "y": point[1],
    }


def generate_planner_episode(
    seed: int, scenario_id: str = DEFAULT_SCENARIO_ID
) -> list[dict[str, Any]]:
    """Generate one deterministic planner episode and its replayable events."""
    _validate_seed(seed)
    scenario = get_scenario(scenario_id)
    static_obstacles = build_obstacles()
    moving_obstacles = scenario.moving_obstacles
    occupied_at_zero = set(moving_obstacle_positions(moving_obstacles, 0))

    start_candidates = [
        (x, y)
        for y in range(GRID_HEIGHT)
        for x in range(GRID_WIDTH)
        if (x, y) not in static_obstacles and (x, y) not in occupied_at_zero
    ]
    if not start_candidates:
        raise InvalidPlannerInputError("No valid start candidates are available.")

    random = XorShift32(seed)
    start = start_candidates[random.next_u32() % len(start_candidates)]
    joint_period = get_joint_period(moving_obstacles)
    goal_candidates = [
        (x, y)
        for y in range(GRID_HEIGHT)
        for x in range(GRID_WIDTH)
        if (x, y) not in static_obstacles
        and (x, y) != start
        and is_safe_after_arrival((x, y), 0, moving_obstacles, joint_period)
    ]
    if not goal_candidates:
        raise InvalidPlannerInputError("No valid goal candidates are available.")

    goal = goal_candidates[random.next_u32() % len(goal_candidates)]
    events: list[dict[str, Any]] = [
        {
            "type": "episode-started",
            "seed": seed,
            "scenarioId": scenario.id,
            "start": {"x": start[0], "y": start[1]},
            "goal": {"x": goal[0], "y": goal[1]},
        }
    ]

    outcome = plan_warehouse(start, goal, static_obstacles, moving_obstacles)
    if isinstance(outcome, PlannerSuccess):
        events.extend(
            _point_event(point, time_step)
            for point, time_step in outcome.path
        )
        events.append(
            {
                "type": "episode-finished",
                "status": "success",
                "pathLength": len(outcome.path) - 1,
                "expandedNodes": outcome.expanded_nodes,
            }
        )
    else:
        events.append(
            {
                "type": "episode-finished",
                "status": "failure",
                "code": outcome.code,
            }
        )
    return events


def replay_planner_episode(events: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Validate and replay a canonical episode trace from its first event."""
    if not isinstance(events, list) or not events or not isinstance(events[0], dict):
        raise ValueError("Episode trace must start with an episode-started event.")
    first = events[0]
    if first.get("type") != "episode-started":
        raise ValueError("Episode trace must start with an episode-started event.")
    if "seed" not in first or "scenarioId" not in first:
        raise ValueError("Episode-started event must include seed and scenarioId.")
    try:
        canonical = generate_planner_episode(first["seed"], first["scenarioId"])
    except (InvalidPlannerInputError, TypeError, ValueError) as error:
        raise ValueError("Episode-started event has invalid seed or scenarioId.") from error
    if events != canonical:
        raise ValueError("Episode trace differs from the deterministic canonical trace.")
    return canonical


__all__ = [
    "XorShift32",
    "generate_planner_episode",
    "replay_planner_episode",
]
