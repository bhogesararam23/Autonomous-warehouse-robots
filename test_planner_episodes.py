import unittest

from planner_episodes import (
    XorShift32,
    generate_planner_episode,
    replay_planner_episode,
)
from warehouse_astar_sim import InvalidPlannerInputError


class TestPlannerEpisodes(unittest.TestCase):
    def test_fixed_seed_repeatability_and_event_shapes(self):
        first = generate_planner_episode(0)
        second = generate_planner_episode(0)
        self.assertEqual(first, second)
        self.assertEqual(first[0]["type"], "episode-started")
        self.assertEqual(first[0]["seed"], 0)
        self.assertEqual(first[0]["scenarioId"], "standard")
        self.assertEqual(first[-1]["type"], "episode-finished")
        self.assertEqual(first[-1]["status"], "success")
        position_events = [event for event in first if event["type"] == "robot-position"]
        self.assertEqual(first[-1]["pathLength"], len(position_events) - 1)

    def test_replay_returns_canonical_trace(self):
        events = generate_planner_episode(17, "peak-traffic")
        self.assertEqual(replay_planner_episode(events), events)

    def test_tampered_trace_is_rejected(self):
        events = generate_planner_episode(1)
        tampered = [dict(event) for event in events]
        tampered[0]["seed"] = 2
        with self.assertRaises(ValueError):
            replay_planner_episode(tampered)

        tampered = [dict(event) for event in events]
        tampered[1]["x"] += 1
        with self.assertRaises(ValueError):
            replay_planner_episode(tampered)

    def test_invalid_seeds_are_rejected(self):
        for seed in (True, False, -1, 0x1_0000_0000, 1.0, "1", None):
            with self.subTest(seed=seed):
                with self.assertRaises(InvalidPlannerInputError):
                    generate_planner_episode(seed)
                with self.assertRaises(InvalidPlannerInputError):
                    XorShift32(seed)

    def test_successful_episode_has_continuous_time_ordered_route(self):
        events = generate_planner_episode(4)
        start = events[0]
        positions = events[1:-1]
        self.assertTrue(positions)
        self.assertEqual(positions[0]["timeStep"], 0)
        self.assertEqual(
            (positions[0]["x"], positions[0]["y"]),
            (start["start"]["x"], start["start"]["y"]),
        )
        for previous, current in zip(positions, positions[1:]):
            self.assertEqual(current["timeStep"], previous["timeStep"] + 1)
            self.assertLessEqual(
                abs(current["x"] - previous["x"])
                + abs(current["y"] - previous["y"]),
                1,
            )
        self.assertEqual(
            (positions[-1]["x"], positions[-1]["y"]),
            (start["goal"]["x"], start["goal"]["y"]),
        )


if __name__ == "__main__":
    unittest.main()
