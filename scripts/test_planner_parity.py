#!/usr/bin/env python3
"""Compare the Python and TypeScript planners on shared deterministic fixtures."""

from __future__ import annotations

import json
import math
from pathlib import Path
import subprocess
import sys
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
FIXTURE_PATH = ROOT / "tests" / "planner_parity_cases.json"
TS_RUNNER = ROOT / "ui" / "run_planner_parity.ts"
TSX = ROOT / "ui" / "node_modules" / ".bin" / "tsx"
sys.path.insert(0, str(ROOT))

from warehouse_astar_sim import MovingObstacle, astar  # noqa: E402


def run_python_cases(cases: list[dict[str, Any]]) -> list[dict[str, Any]]:
    results = []
    for case in cases:
        static_obstacles = {
            (point["x"], point["y"]) for point in case["staticObstacles"]
        }
        moving_obstacles = [
            MovingObstacle(
                mover["name"],
                tuple((point["x"], point["y"]) for point in mover["route"]),
            )
            for mover in case["movingObstacles"]
        ]
        try:
            path, expanded_nodes = astar(
                (case["start"]["x"], case["start"]["y"]),
                (case["goal"]["x"], case["goal"]["y"]),
                static_obstacles,
                moving_obstacles,
                grid_width=case["gridWidth"],
                grid_height=case["gridHeight"],
                max_planning_time=case["maxPlanningTime"],
            )
            results.append(
                {
                    "id": case["id"],
                    "status": "success",
                    "pathLength": len(path) - 1,
                    "expandedNodes": expanded_nodes,
                    "path": [
                        {"x": point[0], "y": point[1], "timeStep": time_step}
                        for point, time_step in path
                    ],
                }
            )
        except RuntimeError as error:
            if "No safe path found" not in str(error):
                raise
            results.append({"id": case["id"], "status": "no-safe-path"})
    return results


def run_typescript_cases() -> list[dict[str, Any]]:
    if not TSX.is_file():
        raise RuntimeError(
            f"TypeScript test runner is missing: {TSX}; install UI dependencies first"
        )
    completed = subprocess.run(
        [str(TSX), str(TS_RUNNER), str(FIXTURE_PATH)],
        cwd=ROOT / "ui",
        check=True,
        capture_output=True,
        text=True,
    )
    try:
        return json.loads(completed.stdout)
    except json.JSONDecodeError as error:
        raise RuntimeError(
            f"TypeScript planner did not return JSON: {completed.stdout!r}"
        ) from error


def validate_success_path(case: dict[str, Any], result: dict[str, Any], engine: str) -> None:
    width = case["gridWidth"]
    height = case["gridHeight"]
    start = case["start"]
    goal = case["goal"]
    blocked = {(point["x"], point["y"]) for point in case["staticObstacles"]}
    movers = [mover["route"] for mover in case["movingObstacles"]]
    path = result.get("path")
    if not isinstance(path, list) or not path:
        raise AssertionError(f"{engine} returned an empty path for {case['id']}")
    if (path[0]["x"], path[0]["y"], path[0]["timeStep"]) != (
        start["x"],
        start["y"],
        0,
    ):
        raise AssertionError(f"{engine} path for {case['id']} has the wrong start")
    if (path[-1]["x"], path[-1]["y"]) != (goal["x"], goal["y"]):
        raise AssertionError(f"{engine} path for {case['id']} does not reach its goal")
    if result.get("pathLength") != len(path) - 1:
        raise AssertionError(f"{engine} path length is inconsistent for {case['id']}")

    for index, state in enumerate(path):
        point = (state["x"], state["y"])
        if state["timeStep"] != index:
            raise AssertionError(f"{engine} path for {case['id']} has a time-step gap")
        if not (0 <= point[0] < width and 0 <= point[1] < height):
            raise AssertionError(f"{engine} path for {case['id']} leaves the grid")
        if point in blocked:
            raise AssertionError(f"{engine} path for {case['id']} crosses a static obstacle")
        for route in movers:
            if point == (route[index % len(route)]["x"], route[index % len(route)]["y"]):
                raise AssertionError(f"{engine} path for {case['id']} hits a moving obstacle")
        if index == 0:
            continue
        previous = path[index - 1]
        previous_point = (previous["x"], previous["y"])
        if abs(point[0] - previous_point[0]) + abs(point[1] - previous_point[1]) > 1:
            raise AssertionError(f"{engine} path for {case['id']} has a non-adjacent move")
        for route in movers:
            obstacle_before = route[(index - 1) % len(route)]
            obstacle_after = route[index % len(route)]
            before = (obstacle_before["x"], obstacle_before["y"])
            after = (obstacle_after["x"], obstacle_after["y"])
            if previous_point == after and point == before:
                raise AssertionError(f"{engine} path for {case['id']} swaps with a mover")

    joint_period = math.lcm(*(len(route) for route in movers)) if movers else 1
    arrival = path[-1]["timeStep"]
    for time_step in range(arrival, arrival + joint_period):
        for route in movers:
            obstacle = route[time_step % len(route)]
            if (obstacle["x"], obstacle["y"]) == (goal["x"], goal["y"]):
                raise AssertionError(
                    f"{engine} path for {case['id']} ends at a periodically unsafe goal"
                )


def main() -> int:
    cases = json.loads(FIXTURE_PATH.read_text(encoding="utf-8"))["cases"]
    python_first = run_python_cases(cases)
    python_second = run_python_cases(cases)
    if python_first != python_second:
        raise AssertionError("Python planner output changed across identical runs")

    ts_first = run_typescript_cases()
    ts_second = run_typescript_cases()
    if ts_first != ts_second:
        raise AssertionError("TypeScript planner output changed across identical runs")

    by_id = {case["id"]: case for case in cases}
    python_by_id = {result["id"]: result for result in python_first}
    ts_by_id = {result["id"]: result for result in ts_first}
    if set(python_by_id) != set(by_id) or set(ts_by_id) != set(by_id):
        raise AssertionError("Planner outputs do not cover every shared fixture exactly once")

    for case in cases:
        case_id = case["id"]
        expected = case["expected"]
        python_result = python_by_id[case_id]
        ts_result = ts_by_id[case_id]
        if python_result["status"] != ts_result["status"]:
            raise AssertionError(
                f"Planner failure-class mismatch for {case_id}: "
                f"Python={python_result['status']}, TypeScript={ts_result['status']}"
            )
        if python_result["status"] != expected["status"]:
            raise AssertionError(
                f"Unexpected result for {case_id}: {python_result['status']} "
                f"(expected {expected['status']})"
            )
        if expected["status"] == "success":
            expected_length = expected["pathLength"]
            for engine, result in (("Python", python_result), ("TypeScript", ts_result)):
                if result["pathLength"] != expected_length:
                    raise AssertionError(
                        f"{engine} returned {result['pathLength']} steps for {case_id}; "
                        f"expected optimal length {expected_length}"
                    )
                validate_success_path(case, result, engine)
            print(
                f"PASS {case_id}: both planners found safe {expected_length}-step paths "
                "and repeated runs were deterministic"
            )
        else:
            print(f"PASS {case_id}: both planners returned {expected['status']}")

    print(f"PASS: {len(cases)} shared differential fixtures")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
