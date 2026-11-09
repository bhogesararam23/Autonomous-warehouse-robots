"""Validation for the versioned grid-planner scenario contract."""

from __future__ import annotations

import math
import re
from typing import Any

SCENARIO_SCHEMA_VERSION = 1
MAX_SAFE_INTEGER = (1 << 53) - 1
MAX_GRID_DIMENSION = 512
MAX_GRID_CELLS = 65_536
MAX_PLANNING_TIME = 10_000
MAX_TIME_EXPANDED_STATES = 1_000_000
MAX_SCENARIOS = 64
MAX_SHELVES = 1_024
MAX_MOVERS_PER_SCENARIO = 16
MAX_ROUTE_LENGTH = 512
MAX_JOINT_ROUTE_PERIOD = 100_000
MAX_SCENARIO_ID_LENGTH = 64
MAX_NAME_LENGTH = 256
MAX_DESCRIPTION_LENGTH = 1_024
_COORDINATE_SYSTEM = {
    "unit": "cell",
    "origin": "top-left",
    "xAxisDirection": "right",
    "yAxisDirection": "down",
    "timeStepSemantics": "one-move-or-wait",
}
_SCENARIO_ID = re.compile(r"^[a-z0-9][a-z0-9-]*$")


class ScenarioContractError(ValueError):
    """Raised when a scenario catalog is malformed or internally inconsistent."""


def _object(value: Any, path: str, keys: set[str]) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ScenarioContractError(f"{path} must be an object")
    actual = set(value)
    if actual != keys:
        missing = sorted(keys - actual)
        extra = sorted(actual - keys)
        details = []
        if missing:
            details.append(f"missing {', '.join(missing)}")
        if extra:
            details.append(f"unexpected {', '.join(extra)}")
        raise ScenarioContractError(f"{path} has invalid fields: {'; '.join(details)}")
    return value


def _integer(
    value: Any,
    path: str,
    *,
    minimum: int | None = None,
    maximum: int | None = None,
) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ScenarioContractError(f"{path} must be an integer")
    if isinstance(value, float) and (not math.isfinite(value) or not value.is_integer()):
        raise ScenarioContractError(f"{path} must be an integer")
    if abs(value) > MAX_SAFE_INTEGER:
        raise ScenarioContractError(f"{path} must be a safe integer")
    value = int(value)
    if minimum is not None and value < minimum:
        raise ScenarioContractError(f"{path} must be at least {minimum}")
    if maximum is not None and value > maximum:
        raise ScenarioContractError(f"{path} must be at most {maximum}")
    return value


def _nonempty_string(value: Any, path: str, *, maximum_length: int) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ScenarioContractError(f"{path} must be a non-empty string")
    if len(value) > maximum_length:
        raise ScenarioContractError(f"{path} must be at most {maximum_length} characters")
    return value


def _point(value: Any, path: str) -> tuple[int, int]:
    point = _object(value, path, {"x", "y"})
    return (
        _integer(point["x"], f"{path}.x"),
        _integer(point["y"], f"{path}.y"),
    )


def _inside(point: tuple[int, int], width: int, height: int) -> bool:
    return 0 <= point[0] < width and 0 <= point[1] < height


def _check_pairwise_mover_conflicts(
    movers: list[tuple[str, tuple[tuple[int, int], ...]]], scenario_id: str
) -> None:
    """Reject periodic mover routes that collide in cells or by edge swap."""
    for index, (first_name, first_route) in enumerate(movers):
        for second_name, second_route in movers[index + 1 :]:
            shared_period = math.gcd(len(first_route), len(second_route))
            first_states = {
                (point, time_step % shared_period)
                for time_step, point in enumerate(first_route)
            }
            second_states = {
                (point, time_step % shared_period)
                for time_step, point in enumerate(second_route)
            }
            if first_states & second_states:
                raise ScenarioContractError(
                    f"scenario '{scenario_id}' has moving obstacles "
                    f"'{first_name}' and '{second_name}' occupying the same cell"
                )
            first_edges = {
                (point, first_route[(time_step + 1) % len(first_route)], time_step % shared_period)
                for time_step, point in enumerate(first_route)
            }
            second_reverse_edges = {
                (second_route[(time_step + 1) % len(second_route)], point, time_step % shared_period)
                for time_step, point in enumerate(second_route)
            }
            if first_edges & second_reverse_edges:
                raise ScenarioContractError(
                    f"scenario '{scenario_id}' has moving obstacles "
                    f"'{first_name}' and '{second_name}' colliding by head-on edge swap"
                )


def validate_scenario_catalog(catalog: Any) -> None:
    """Validate all planner inputs before scenarios are exposed to callers."""
    root = _object(
        catalog,
        "catalog",
        {
            "schemaVersion",
            "coordinateSystem",
            "warehouse",
            "defaultScenarioId",
            "scenarios",
        },
    )
    version = _integer(root["schemaVersion"], "schemaVersion")
    if version != SCENARIO_SCHEMA_VERSION:
        raise ScenarioContractError(
            f"unsupported scenario schemaVersion {version}; "
            f"this planner supports {SCENARIO_SCHEMA_VERSION}"
        )

    coordinate_system = _object(
        root["coordinateSystem"], "coordinateSystem", set(_COORDINATE_SYSTEM)
    )
    if coordinate_system != _COORDINATE_SYSTEM:
        raise ScenarioContractError(
            "coordinateSystem must use top-left-origin grid cells and one-move-or-wait steps"
        )

    warehouse = _object(
        root["warehouse"],
        "warehouse",
        {"width", "height", "start", "goal", "maxPlanningTime", "shelves"},
    )
    width = _integer(
        warehouse["width"],
        "warehouse.width",
        minimum=1,
        maximum=MAX_GRID_DIMENSION,
    )
    height = _integer(
        warehouse["height"],
        "warehouse.height",
        minimum=1,
        maximum=MAX_GRID_DIMENSION,
    )
    grid_cells = width * height
    if grid_cells > MAX_GRID_CELLS:
        raise ScenarioContractError(
            f"warehouse grid must contain at most {MAX_GRID_CELLS} cells"
        )
    max_planning_time = _integer(
        warehouse["maxPlanningTime"],
        "warehouse.maxPlanningTime",
        minimum=0,
        maximum=MAX_PLANNING_TIME,
    )
    if grid_cells * (max_planning_time + 1) > MAX_TIME_EXPANDED_STATES:
        raise ScenarioContractError(
            f"warehouse grid and planning horizon exceed {MAX_TIME_EXPANDED_STATES} time-expanded states"
        )
    start = _point(warehouse["start"], "warehouse.start")
    goal = _point(warehouse["goal"], "warehouse.goal")
    if not _inside(start, width, height):
        raise ScenarioContractError("warehouse.start must be inside the grid")
    if not _inside(goal, width, height):
        raise ScenarioContractError("warehouse.goal must be inside the grid")

    shelves = warehouse["shelves"]
    if not isinstance(shelves, list):
        raise ScenarioContractError("warehouse.shelves must be an array")
    if len(shelves) > MAX_SHELVES:
        raise ScenarioContractError(f"warehouse.shelves must contain at most {MAX_SHELVES} entries")
    shelf_names: set[str] = set()
    static_cells: set[tuple[int, int]] = set()
    for index, raw_shelf in enumerate(shelves):
        path = f"warehouse.shelves[{index}]"
        shelf = _object(raw_shelf, path, {"name", "x", "y", "width", "height"})
        name = _nonempty_string(
            shelf["name"], f"{path}.name", maximum_length=MAX_NAME_LENGTH
        )
        if name in shelf_names:
            raise ScenarioContractError(f"duplicate shelf name '{name}'")
        shelf_names.add(name)
        x = _integer(shelf["x"], f"{path}.x", minimum=0, maximum=MAX_GRID_DIMENSION - 1)
        y = _integer(shelf["y"], f"{path}.y", minimum=0, maximum=MAX_GRID_DIMENSION - 1)
        shelf_width = _integer(
            shelf["width"], f"{path}.width", minimum=1, maximum=MAX_GRID_DIMENSION
        )
        shelf_height = _integer(
            shelf["height"], f"{path}.height", minimum=1, maximum=MAX_GRID_DIMENSION
        )
        if x + shelf_width > width or y + shelf_height > height:
            raise ScenarioContractError(f"{path} must fit entirely inside the grid")
        cells = {
            (cell_x, cell_y)
            for cell_x in range(x, x + shelf_width)
            for cell_y in range(y, y + shelf_height)
        }
        if static_cells & cells:
            raise ScenarioContractError(f"{path} overlaps another shelf")
        static_cells.update(cells)

    if start in static_cells:
        raise ScenarioContractError("warehouse.start is blocked by a shelf")
    if goal in static_cells:
        raise ScenarioContractError("warehouse.goal is blocked by a shelf")

    default_id = _nonempty_string(
        root["defaultScenarioId"],
        "defaultScenarioId",
        maximum_length=MAX_SCENARIO_ID_LENGTH,
    )
    scenarios = root["scenarios"]
    if not isinstance(scenarios, list) or not scenarios:
        raise ScenarioContractError("scenarios must be a non-empty array")
    if len(scenarios) > MAX_SCENARIOS:
        raise ScenarioContractError(f"scenarios must contain at most {MAX_SCENARIOS} entries")
    scenario_ids: set[str] = set()
    for scenario_index, raw_scenario in enumerate(scenarios):
        scenario_path = f"scenarios[{scenario_index}]"
        scenario = _object(
            raw_scenario,
            scenario_path,
            {"id", "name", "description", "movingObstacles"},
        )
        scenario_id = _nonempty_string(
            scenario["id"],
            f"{scenario_path}.id",
            maximum_length=MAX_SCENARIO_ID_LENGTH,
        )
        if not _SCENARIO_ID.fullmatch(scenario_id):
            raise ScenarioContractError(
                f"{scenario_path}.id must contain lowercase letters, digits, and hyphens"
            )
        if scenario_id in scenario_ids:
            raise ScenarioContractError(f"duplicate scenario id '{scenario_id}'")
        scenario_ids.add(scenario_id)
        _nonempty_string(
            scenario["name"], f"{scenario_path}.name", maximum_length=MAX_NAME_LENGTH
        )
        _nonempty_string(
            scenario["description"],
            f"{scenario_path}.description",
            maximum_length=MAX_DESCRIPTION_LENGTH,
        )
        raw_movers = scenario["movingObstacles"]
        if not isinstance(raw_movers, list):
            raise ScenarioContractError(f"{scenario_path}.movingObstacles must be an array")
        if len(raw_movers) > MAX_MOVERS_PER_SCENARIO:
            raise ScenarioContractError(
                f"{scenario_path}.movingObstacles must contain at most {MAX_MOVERS_PER_SCENARIO} entries"
            )
        mover_names: set[str] = set()
        movers: list[tuple[str, tuple[tuple[int, int], ...]]] = []
        joint_period = 1
        for mover_index, raw_mover in enumerate(raw_movers):
            mover_path = f"{scenario_path}.movingObstacles[{mover_index}]"
            mover = _object(raw_mover, mover_path, {"name", "route"})
            mover_name = _nonempty_string(
                mover["name"], f"{mover_path}.name", maximum_length=MAX_NAME_LENGTH
            )
            if mover_name in mover_names:
                raise ScenarioContractError(
                    f"scenario '{scenario_id}' has duplicate mover name '{mover_name}'"
                )
            mover_names.add(mover_name)
            raw_route = mover["route"]
            if not isinstance(raw_route, list) or not raw_route:
                raise ScenarioContractError(f"{mover_path}.route must be a non-empty array")
            if len(raw_route) > MAX_ROUTE_LENGTH:
                raise ScenarioContractError(
                    f"{mover_path}.route must contain at most {MAX_ROUTE_LENGTH} points"
                )
            route = tuple(
                _point(point, f"{mover_path}.route[{point_index}]")
                for point_index, point in enumerate(raw_route)
            )
            for point in route:
                if not _inside(point, width, height):
                    raise ScenarioContractError(
                        f"{mover_path}.route contains a point outside the grid"
                    )
                if point in static_cells:
                    raise ScenarioContractError(
                        f"{mover_path}.route intersects a shelf at {point}"
                    )
            for index, point in enumerate(route):
                following = route[(index + 1) % len(route)]
                if abs(point[0] - following[0]) + abs(point[1] - following[1]) > 1:
                    raise ScenarioContractError(
                        f"{mover_path}.route must move at most one cell per step"
                    )
            if route[0] == start:
                raise ScenarioContractError(
                    f"scenario '{scenario_id}' starts with mover '{mover_name}' on the robot"
                )
            movers.append((mover_name, route))
            joint_period = math.lcm(joint_period, len(route))
            if joint_period > MAX_JOINT_ROUTE_PERIOD:
                raise ScenarioContractError(
                    f"scenario '{scenario_id}' joint mover period exceeds {MAX_JOINT_ROUTE_PERIOD} steps"
                )
        _check_pairwise_mover_conflicts(movers, scenario_id)

    if default_id not in scenario_ids:
        raise ScenarioContractError(
            f"defaultScenarioId '{default_id}' does not name a scenario"
        )
