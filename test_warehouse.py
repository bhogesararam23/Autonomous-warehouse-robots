import unittest
from warehouse_astar_sim import (
    astar, build_obstacles, MOVING_OBSTACLES, MovingObstacle,
    GRID_WIDTH, GRID_HEIGHT, Point, State, get_joint_period
)

class TestWarehousePlanner(unittest.TestCase):
    def setUp(self):
        self.obstacles = build_obstacles()
        self.moving_obstacles = MOVING_OBSTACLES

    def validate_path(self, path: list[State], obstacles: set[Point], moving_obstacles: list[MovingObstacle], joint_period: int):
        self.assertTrue(len(path) > 0, "Path should not be empty")

        # Check transitions and collisions
        for i in range(1, len(path)):
            prev, prev_t = path[i-1]
            curr, curr_t = path[i]

            # Time monotonically increases by 1
            self.assertEqual(curr_t, prev_t + 1, "Time step must increase by 1")

            # Boundaries
            self.assertTrue(0 <= curr[0] < GRID_WIDTH, "Out of bounds X")
            self.assertTrue(0 <= curr[1] < GRID_HEIGHT, "Out of bounds Y")

            # Static obstacles
            self.assertNotIn(curr, obstacles, "Collided with static obstacle")

            # Valid transition (Manhattan distance <= 1)
            dist = abs(curr[0] - prev[0]) + abs(curr[1] - prev[1])
            self.assertTrue(dist <= 1, f"Invalid transition from {prev} to {curr}")

            # Dynamic collisions
            for obs in moving_obstacles:
                obs_prev = obs.position_at(prev_t)
                obs_curr = obs.position_at(curr_t)

                # Same cell
                self.assertNotEqual(curr, obs_curr, f"Same cell collision at {curr} time {curr_t}")

                # Edge swap
                self.assertFalse(curr == obs_prev and prev == obs_curr, f"Edge swap at {prev}->{curr} time {curr_t}")

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
