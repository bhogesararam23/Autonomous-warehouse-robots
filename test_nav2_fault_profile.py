import sys
import unittest
from pathlib import Path


LAUNCH_DIR = (
    Path(__file__).resolve().parent
    / "ros2_ws"
    / "src"
    / "warehouse_astar_planner"
    / "launch"
)
sys.path.insert(0, str(LAUNCH_DIR))

from nav2_fault_profile import apply_fault_test_profile


class Nav2FaultProfileTest(unittest.TestCase):
    @staticmethod
    def sample_parameters():
        return {
            "controller_server": {
                "ros__parameters": {
                    "goal_checker_plugins": ["general_goal_checker"],
                    "general_goal_checker": {
                        "xy_goal_tolerance": 0.25,
                        "yaw_goal_tolerance": 0.25,
                    },
                    "progress_checker_plugins": ["progress_checker"],
                    "progress_checker": {
                        "required_movement_radius": 0.5,
                        "movement_time_allowance": 10.0,
                    },
                }
            }
        }

    def test_fault_profile_preserves_goal_and_radius_and_extends_allowance(self):
        params = self.sample_parameters()
        metadata = apply_fault_test_profile(params)
        controller = params["controller_server"]["ros__parameters"]

        self.assertEqual(controller["general_goal_checker"]["xy_goal_tolerance"], 0.25)
        self.assertEqual(controller["general_goal_checker"]["yaw_goal_tolerance"], 0.25)
        self.assertEqual(
            controller["progress_checker"]["required_movement_radius"], 0.5
        )
        self.assertEqual(
            controller["progress_checker"]["movement_time_allowance"], 30.0
        )
        self.assertEqual(metadata["profile"], "blocked-aisle-simulation")
        self.assertEqual(metadata["goal_checker_xy_tolerance_m"], 0.25)
        self.assertEqual(metadata["goal_checker_yaw_tolerance_rad"], 0.25)

    def test_fault_profile_rejects_missing_goal_checker_plugin(self):
        params = self.sample_parameters()
        params["controller_server"]["ros__parameters"]["goal_checker_plugins"] = []
        with self.assertRaisesRegex(ValueError, "general_goal_checker schema changed"):
            apply_fault_test_profile(params)

    def test_fault_profile_rejects_invalid_progress_checker_defaults(self):
        params = self.sample_parameters()
        params["controller_server"]["ros__parameters"]["progress_checker"][
            "movement_time_allowance"
        ] = 0.0
        with self.assertRaisesRegex(ValueError, "finite positive number"):
            apply_fault_test_profile(params)


if __name__ == "__main__":
    unittest.main()
