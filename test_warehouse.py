import json
from collections import deque
from pathlib import Path
import subprocess
import sys
import unittest
from contextlib import redirect_stdout
from io import StringIO
from unittest.mock import patch

from warehouse_astar_sim import (
    DEFAULT_SCENARIO_ID,
    GRID_HEIGHT,
    GRID_WIDTH,
    MOVING_OBSTACLES,
    SCENARIO_CONFIG_PATH,
    SCENARIOS,
    MovingObstacle,
    Point,
    State,
    astar,
    build_obstacles,
    get_joint_period,
    get_scenario,
    main,
    parse_args,
    simulate_path,
)


def shortest_path_length(start, goal, obstacles, grid_width, grid_height):
    frontier = deque([(start, 0)])
    visited = {start}
    while frontier:
        (x, y), distance = frontier.popleft()
        for neighbor in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            nx, ny = neighbor
            if not (0 <= nx < grid_width and 0 <= ny < grid_height):
                continue
            if neighbor in obstacles or neighbor in visited:
                continue
            if neighbor == goal:
                return distance + 1
            visited.add(neighbor)
            frontier.append((neighbor, distance + 1))
    return None


class TestWarehousePlanner(unittest.TestCase):
    def setUp(self):
        self.obstacles = build_obstacles()
        self.moving_obstacles = MOVING_OBSTACLES

    def test_scenarios_match_shared_catalog(self):
        with SCENARIO_CONFIG_PATH.open(encoding="utf-8") as scenario_file:
            catalog = json.load(scenario_file)

        self.assertEqual(DEFAULT_SCENARIO_ID, catalog["defaultScenarioId"])
        self.assertEqual(
            list(SCENARIOS),
            [scenario_data["id"] for scenario_data in catalog["scenarios"]],
        )

        for scenario_data in catalog["scenarios"]:
            scenario = get_scenario(scenario_data["id"])
            self.assertEqual(scenario.name, scenario_data["name"])
            self.assertEqual(scenario.description, scenario_data["description"])
            self.assertEqual(
                [
                    {
                        "name": obstacle.name,
                        "route": [{"x": x, "y": y} for x, y in obstacle.route],
                    }
                    for obstacle in scenario.moving_obstacles
                ],
                scenario_data["movingObstacles"],
            )

    def test_default_scenario_remains_standard(self):
        self.assertEqual(parse_args([]).scenario, DEFAULT_SCENARIO_ID)
        standard = get_scenario(DEFAULT_SCENARIO_ID)
        self.assertEqual(standard.name, "Standard warehouse")
        self.assertIs(standard.moving_obstacles, MOVING_OBSTACLES)

    def test_cli_skips_svg_generation_when_requested(self):
        with patch("warehouse_astar_sim.save_svg") as save_svg:
            with redirect_stdout(StringIO()) as output:
                exit_code = main(["--no-svg"])

        self.assertEqual(exit_code, 0)
        save_svg.assert_not_called()
        self.assertIn("Path length: 42 time steps", output.getvalue())
        self.assertNotIn("SVG visualization:", output.getvalue())

    def test_cli_generates_svg_by_default(self):
        with patch("warehouse_astar_sim.save_svg") as save_svg:
            with redirect_stdout(StringIO()) as output:
                exit_code = main([])

        self.assertEqual(exit_code, 0)
        save_svg.assert_called_once()
        self.assertIn("SVG visualization:", output.getvalue())

    def test_cli_planning_failure_returns_nonzero(self):
        with patch("warehouse_astar_sim.astar", side_effect=RuntimeError("no path")):
            with redirect_stdout(StringIO()) as output:
                exit_code = main(["--no-svg"])

        self.assertEqual(exit_code, 1)
        self.assertIn("Failure reason: no path", output.getvalue())

    def test_cli_replay_failure_returns_nonzero(self):
        invalid_path = [((1, 1), 0), ((23, 13), 1)]
        with patch("warehouse_astar_sim.astar", return_value=(invalid_path, 1)):
            with redirect_stdout(StringIO()):
                exit_code = main(["--no-svg"])

        self.assertEqual(exit_code, 1)

    def test_collision_demos_run_and_report_expected_collision(self):
        repo_root = Path(__file__).resolve().parent
        demos = (
            ("collision_demo.py", "Collisions detected: 1"),
            ("collision_demo_after_arrival.py", "Collisions after arrival: 1"),
        )

        for script, expected_output in demos:
            with self.subTest(script=script):
                result = subprocess.run(
                    [sys.executable, str(repo_root / script)],
                    cwd=repo_root,
                    capture_output=True,
                    text=True,
                    check=False,
                )
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn(expected_output, result.stdout)

    def test_simulate_path_counts_spatial_and_static_failures(self):
        invalid_paths = (
            ([((0, 0), 0), ((2, 0), 1)], set(), 3, 1),
            ([((0, 0), 0), ((3, 0), 1)], set(), 3, 1),
            ([((0, 0), 0), ((1, 0), 1)], {(1, 0)}, 3, 1),
            ([((0, 0), 0), ((1, 0), 2)], set(), 3, 1),
        )

        for path, obstacles, grid_width, grid_height in invalid_paths:
            with self.subTest(path=path, obstacles=obstacles):
                self.assertEqual(
                    simulate_path(
                        path,
                        [],
                        obstacles=obstacles,
                        grid_width=grid_width,
                        grid_height=grid_height,
                    ),
                    (0, 1),
                )

        self.assertEqual(simulate_path([], [], obstacles=set()), (0, 1))

    def test_peak_traffic_scenario_is_collision_safe(self):
        scenario = get_scenario("peak-traffic")
        moving_obstacles = scenario.moving_obstacles
        self.assertEqual(scenario.name, "Peak cross-aisle traffic")
        self.assertEqual(len(moving_obstacles), len(MOVING_OBSTACLES) + 1)

        for obstacle in moving_obstacles:
            for x, y in obstacle.route:
                self.assertTrue(0 <= x < GRID_WIDTH and 0 <= y < GRID_HEIGHT)
                self.assertNotIn((x, y), self.obstacles)

        path, _ = astar((1, 1), (22, 12), self.obstacles, moving_obstacles)
        self.validate_path(
            path,
            self.obstacles,
            moving_obstacles,
            get_joint_period(moving_obstacles),
        )
        collisions, failures = simulate_path(path, moving_obstacles)
        self.assertEqual(collisions, 0)
        self.assertEqual(failures, 0)

    def test_larger_warehouse_path_preserves_safety_invariants(self):
        grid_width = 64
        grid_height = 40
        start = (1, 1)
        goal = (62, 38)
        gates = {
            12: {6, 7, 8},
            24: {14, 15, 16},
            36: {22, 23, 24},
            48: {30, 31, 32},
            60: {34, 35, 36},
        }
        obstacles = {
            (x, y)
            for x in gates
            for y in range(grid_height)
            if y not in gates[x]
        }
        moving_obstacles = [
            MovingObstacle("Gate patrol 1", ((12, 6), (12, 7), (12, 8), (12, 7))),
            MovingObstacle(
                "Gate patrol 2",
                ((24, 14), (24, 15), (24, 16), (24, 15), (24, 14)),
            ),
            MovingObstacle(
                "Cross-aisle patrol",
                ((40, 27), (41, 27), (42, 27), (41, 27)),
            ),
        ]

        path, expanded_nodes = astar(
            start,
            goal,
            obstacles,
            moving_obstacles,
            grid_width=grid_width,
            grid_height=grid_height,
            max_planning_time=180,
        )

        self.assertGreater(expanded_nodes, 0)
        self.assertEqual(path[0], (start, 0))
        self.assertEqual(path[-1][0], goal)
        self.validate_path(
            path,
            obstacles,
            moving_obstacles,
            get_joint_period(moving_obstacles),
            grid_width=grid_width,
            grid_height=grid_height,
        )
        collisions, failures = simulate_path(
            path,
            moving_obstacles,
            obstacles=obstacles,
            grid_width=grid_width,
            grid_height=grid_height,
        )
        self.assertEqual(collisions, 0)
        self.assertEqual(failures, 0)

    def test_start_occupied_at_time_zero_is_rejected(self):
        obstacle = MovingObstacle("Starting obstacle", ((1, 1), (2, 1)))

        with self.assertRaisesRegex(RuntimeError, "No safe path found"):
            astar((1, 1), (3, 1), set(), [obstacle])

    def test_out_of_bounds_or_shelf_occupied_start_and_goal_are_rejected(self):
        invalid_routes = [
            ((-1, 1), (1, 1)),
            ((1, 1), (GRID_WIDTH, 12)),
            ((5, 0), (22, 12)),
            ((1, 1), (5, 0)),
        ]

        for start, goal in invalid_routes:
            with self.subTest(start=start, goal=goal):
                with self.assertRaises(ValueError):
                    astar(start, goal, self.obstacles, [])

    def test_malformed_moving_obstacle_routes_are_rejected(self):
        invalid_routes = [
            ("empty route", ()),
            ("fractional x", ((1.5, 1),)),
            ("fractional y", ((1, 1.5),)),
            ("out-of-bounds x", ((3, 1),)),
            ("out-of-bounds y", ((1, 3),)),
        ]

        for name, route in invalid_routes:
            with self.subTest(name=name, route=route):
                obstacle = MovingObstacle(name, route)
                with self.assertRaisesRegex(ValueError, "Moving obstacle"):
                    astar(
                        (0, 0),
                        (0, 0),
                        set(),
                        [obstacle],
                        grid_width=3,
                        grid_height=3,
                        max_planning_time=0,
                    )

    def test_start_equal_to_goal_returns_zero_step_path(self):
        path, expanded_nodes = astar(
            (0, 0),
            (0, 0),
            set(),
            [],
            grid_width=1,
            grid_height=1,
            max_planning_time=0,
        )

        self.assertEqual(path, [((0, 0), 0)])
        self.assertEqual(expanded_nodes, 1)

    def test_path_length_matches_independent_shortest_path_baseline(self):
        grid_width = 6
        grid_height = 5
        start = (0, 2)
        goal = (5, 2)
        obstacles = {(2, y) for y in range(grid_height - 1)}
        baseline_length = shortest_path_length(
            start, goal, obstacles, grid_width, grid_height
        )
        path, _ = astar(
            start,
            goal,
            obstacles,
            [],
            grid_width=grid_width,
            grid_height=grid_height,
            max_planning_time=20,
        )

        self.assertEqual(baseline_length, 9)
        self.assertEqual(len(path) - 1, baseline_length)
        self.validate_path(
            path,
            obstacles,
            [],
            get_joint_period([]),
            grid_width=grid_width,
            grid_height=grid_height,
        )

    def test_unreachable_goal_raises_no_safe_path(self):
        with self.assertRaisesRegex(RuntimeError, "No safe path found"):
            astar(
                (0, 0),
                (2, 0),
                {(1, 0)},
                [],
                grid_width=3,
                grid_height=1,
                max_planning_time=3,
            )

    def test_route_longer_than_planning_horizon_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "No safe path found"):
            astar(
                (0, 0),
                (2, 0),
                set(),
                [],
                grid_width=3,
                grid_height=1,
                max_planning_time=1,
            )

    def test_start_equal_to_goal_is_rejected_if_cycle_returns_to_it(self):
        obstacle = MovingObstacle("Returning obstacle", ((1, 1), (0, 0)))

        with self.assertRaisesRegex(RuntimeError, "No safe path found"):
            astar(
                (0, 0),
                (0, 0),
                set(),
                [obstacle],
                grid_width=2,
                grid_height=2,
                max_planning_time=0,
            )

    def test_non_integer_dimensions_horizon_and_coordinates_are_rejected(self):
        invalid_calls = [
            {
                "start": (0, 0),
                "goal": (0, 0),
                "options": {"grid_width": 1.5, "grid_height": 1},
            },
            {
                "start": (0, 0),
                "goal": (0, 0),
                "options": {"grid_width": True, "grid_height": 1},
            },
            {
                "start": (0, 0),
                "goal": (0, 0),
                "options": {"grid_width": 1, "grid_height": 1.5},
            },
            {
                "start": (0, 0),
                "goal": (0, 0),
                "options": {
                    "grid_width": 1,
                    "grid_height": 1,
                    "max_planning_time": 0.5,
                },
            },
            {
                "start": (0, 0),
                "goal": (0, 0),
                "options": {
                    "grid_width": 1,
                    "grid_height": 1,
                    "max_planning_time": True,
                },
            },
            {
                "start": (0, 0),
                "goal": (0, 0),
                "options": {
                    "grid_width": 1,
                    "grid_height": 1,
                    "max_planning_time": float("nan"),
                },
            },
            {
                "start": (0, 0),
                "goal": (0, 0),
                "options": {
                    "grid_width": 1,
                    "grid_height": 1,
                    "max_planning_time": float("inf"),
                },
            },
            {
                "start": (0.5, 0),
                "goal": (0.5, 0),
                "options": {"grid_width": 1, "grid_height": 1},
            },
            {
                "start": (0, 0),
                "goal": (0.5, 0),
                "options": {"grid_width": 1, "grid_height": 1},
            },
        ]

        for call in invalid_calls:
            with self.subTest(call=call):
                with self.assertRaises(ValueError):
                    astar(call["start"], call["goal"], set(), [], **call["options"])

    def validate_path(
        self,
        path: list[State],
        obstacles: set[Point],
        moving_obstacles: list[MovingObstacle],
        joint_period: int,
        *,
        grid_width: int = GRID_WIDTH,
        grid_height: int = GRID_HEIGHT,
    ):
        self.assertTrue(len(path) > 0, "Path should not be empty")

        for index, (point, time_step) in enumerate(path):
            self.assertEqual(time_step, index, "Time steps must be consecutive from zero")
            self.assertTrue(0 <= point[0] < grid_width, "Out of bounds X")
            self.assertTrue(0 <= point[1] < grid_height, "Out of bounds Y")
            self.assertNotIn(point, obstacles, "Path collided with a static obstacle")
            for obstacle in moving_obstacles:
                self.assertNotEqual(
                    point,
                    obstacle.position_at(time_step),
                    f"Same cell collision at {point} time {time_step}",
                )

            if index == 0:
                continue

            prev, prev_t = path[index - 1]
            curr, curr_t = point, time_step

            # Time monotonically increases by 1
            self.assertEqual(curr_t, prev_t + 1, "Time step must increase by 1")

            # Valid transition (Manhattan distance <= 1)
            dist = abs(curr[0] - prev[0]) + abs(curr[1] - prev[1])
            self.assertTrue(dist <= 1, f"Invalid transition from {prev} to {curr}")

            # Dynamic collisions
            for obs in moving_obstacles:
                obs_prev = obs.position_at(prev_t)
                obs_curr = obs.position_at(curr_t)

                # Edge swap
                self.assertFalse(
                    curr == obs_prev and prev == obs_curr,
                    f"Edge swap at {prev}->{curr} time {curr_t}",
                )

        # Post arrival policy
        final_pos, final_t = path[-1]
        for t in range(final_t, final_t + joint_period):
            for obs in moving_obstacles:
                self.assertNotEqual(final_pos, obs.position_at(t), f"Goal collision at time {t} after arrival")

    def test_safe_goal(self):
        start = (1, 1)
        goal = (22, 12)
        path, nodes = astar(start, goal, self.obstacles, self.moving_obstacles)
        self.validate_path(path, self.obstacles, self.moving_obstacles, get_joint_period(self.moving_obstacles))

    def test_unsafe_goal_20_9(self):
        start = (1, 1)
        goal = (20, 9)
        with self.assertRaises(RuntimeError):
            astar(start, goal, self.obstacles, self.moving_obstacles)

    def test_wait_logic(self):
        # A scenario where it has to wait to let an obstacle pass
        # Start (0, 0), goal (2, 0). Obstacle moves from (1, 0) to (1, 1) and back
        obs = [MovingObstacle("O1", ((1, 1), (1, 0), (1, 1)))]
        obstacles = set()
        path, nodes = astar((0, 0), (2, 0), obstacles, obs)
        self.validate_path(path, obstacles, obs, get_joint_period(obs))
        # Need at least 2 steps (dist) + wait time
        self.assertTrue(len(path) > 3)

    def test_obstacle_period_edge_case(self):
        obs1 = MovingObstacle("O1", ((10, 10), (10, 11))) # len 2
        obs2 = MovingObstacle("O2", ((5, 5), (5, 6), (5, 7))) # len 3
        # joint period should be 6
        self.assertEqual(get_joint_period([obs1, obs2]), 6)
        self.assertEqual(get_joint_period([]), 1)

if __name__ == '__main__':
    unittest.main()
