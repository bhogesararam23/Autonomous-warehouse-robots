#!/usr/bin/env python3
"""Verify seeded planning episodes replay identically across Python and TypeScript."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
FIXTURE_PATH = ROOT / "tests" / "seeded_episode_cases.json"
TS_RUNNER = ROOT / "ui" / "run_episode_replay.ts"
TSX = ROOT / "ui" / "node_modules" / ".bin" / "tsx"
sys.path.insert(0, str(ROOT))

from planner_episodes import (  # noqa: E402
    generate_planner_episode,
    replay_planner_episode,
)
from warehouse_astar_sim import (  # noqa: E402
    GRID_HEIGHT,
    GRID_WIDTH,
    build_obstacles,
    get_joint_period,
    get_scenario,
    has_dynamic_collision,
    is_safe_after_arrival,
)


def canonical(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"))


def validate_success_episode(case: dict[str, Any], events: list[dict[str, Any]]) -> None:
    terminal = events[-1]
    if terminal.get("status") != "success":
        return

    started = events[0]
    positions = events[1:-1]
    scenario = get_scenario(case["scenarioId"])
    obstacles = build_obstacles()
    movers = scenario.moving_obstacles
    if not positions or terminal.get("pathLength") != len(positions) - 1:
        raise AssertionError(f"Successful episode {case['id']} has an invalid path length")

    for index, event in enumerate(positions):
        point = (event.get("x"), event.get("y"))
        if event.get("type") != "robot-position" or event.get("timeStep") != index:
            raise AssertionError(f"Episode {case['id']} has a time-step gap or non-position event")
        if not (0 <= point[0] < GRID_WIDTH and 0 <= point[1] < GRID_HEIGHT):
            raise AssertionError(f"Episode {case['id']} leaves the warehouse grid")
        if point in obstacles:
            raise AssertionError(f"Episode {case['id']} crosses a fixed shelf")
        if any(mover.position_at(index) == point for mover in movers):
            raise AssertionError(f"Episode {case['id']} intersects a moving obstacle")

        if index == 0:
            if point != (started["start"]["x"], started["start"]["y"]):
                raise AssertionError(f"Episode {case['id']} starts at the wrong point")
            continue

        previous_event = positions[index - 1]
        previous = (previous_event["x"], previous_event["y"])
        if abs(point[0] - previous[0]) + abs(point[1] - previous[1]) > 1:
            raise AssertionError(f"Episode {case['id']} contains a non-adjacent move")
        if has_dynamic_collision(previous, point, index, movers):
            raise AssertionError(f"Episode {case['id']} collides or edge-swaps with a mover")

    goal = (started["goal"]["x"], started["goal"]["y"])
    if (positions[-1]["x"], positions[-1]["y"]) != goal:
        raise AssertionError(f"Episode {case['id']} does not finish at its goal")
    if not is_safe_after_arrival(goal, len(positions) - 1, movers, get_joint_period(movers)):
        raise AssertionError(f"Episode {case['id']} ends at a periodically unsafe goal")


def run_python_cases(cases: list[dict[str, Any]]) -> list[dict[str, Any]]:
    results = []
    for case in cases:
        first = generate_planner_episode(case["seed"], case["scenarioId"])
        second = generate_planner_episode(case["seed"], case["scenarioId"])
        replayed = replay_planner_episode(first)
        if canonical(first) != canonical(second):
            raise AssertionError(f"Python episode {case['id']} changed for the same seed")
        if canonical(first) != canonical(replayed):
            raise AssertionError(f"Python replay changed the event stream for {case['id']}")
        if (
            not first
            or first[0].get("type") != "episode-started"
            or first[-1].get("type") != "episode-finished"
        ):
            raise AssertionError(f"Python episode {case['id']} has an invalid event envelope")
        expected = case["expected"]
        if first[0].get("start") != expected["start"]:
            raise AssertionError(f"Python start endpoint changed for seed case {case['id']}")
        if first[0].get("goal") != expected["goal"]:
            raise AssertionError(f"Python goal endpoint changed for seed case {case['id']}")
        terminal = first[-1]
        if terminal.get("status") != expected["status"]:
            raise AssertionError(f"Python outcome changed for seed case {case['id']}")
        if expected["status"] == "success" and terminal.get("pathLength") != expected[
            "pathLength"
        ]:
            raise AssertionError(f"Python path length changed for seed case {case['id']}")
        validate_success_episode(case, first)
        results.append({"id": case["id"], "events": first})
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
            f"TypeScript episode runner did not return JSON: {completed.stdout!r}"
        ) from error


def main() -> int:
    fixture_file = json.loads(FIXTURE_PATH.read_text(encoding="utf-8"))
    cases = fixture_file["cases"]
    python_results = run_python_cases(cases)
    ts_first = run_typescript_cases()
    ts_second = run_typescript_cases()
    if canonical(ts_first) != canonical(ts_second):
        raise AssertionError("TypeScript event streams changed across identical runs")

    python_by_id = {result["id"]: result for result in python_results}
    ts_by_id = {result["id"]: result for result in ts_first}
    expected_ids = {case["id"] for case in cases}
    if set(python_by_id) != expected_ids or set(ts_by_id) != expected_ids:
        raise AssertionError("Episode outputs do not cover every shared seed exactly once")

    success_count = 0
    for case in cases:
        case_id = case["id"]
        python_events = python_by_id[case_id]["events"]
        ts_events = ts_by_id[case_id]["events"]
        if canonical(python_events) != canonical(ts_events):
            raise AssertionError(f"Python/TypeScript event sequence differs for {case_id}")
        terminal = python_events[-1]
        if terminal.get("status") == "success":
            success_count += 1
        print(
            f"PASS {case_id}: seed {case['seed']} replayed {len(python_events)} "
            f"identical events ({terminal.get('status')})"
        )

    if success_count == 0:
        raise AssertionError("Shared seed cases did not produce any successful episode")
    print(f"PASS: {len(cases)} seeded episode replay fixtures")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
