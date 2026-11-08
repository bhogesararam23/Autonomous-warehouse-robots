import subprocess
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / "scripts"))

from fault_injection import (  # noqa: E402
    BLOCKED_AISLE_MODEL,
    blocked_aisle_model_sdf,
    blocked_aisle_spawn_pose_args,
    find_scan_return,
    inject_blocked_aisle,
    max_cross_track_deviation,
    validate_blocked_aisle_model,
    world_has_blocked_aisle,
)


class FaultInjectionTest(unittest.TestCase):
    def test_blocked_aisle_model_is_validated_against_expected_geometry(self):
        model_sdf = blocked_aisle_model_sdf()

        metadata = validate_blocked_aisle_model(model_sdf)

        model = ET.fromstring(model_sdf).find(f"model[@name='{BLOCKED_AISLE_MODEL}']")
        self.assertIsNotNone(model)
        self.assertEqual(metadata["scenario"], "blocked-aisle")
        self.assertEqual(metadata["center_xy_m"], [1.0, 0.0])
        self.assertEqual(metadata["minimum_route_distance_m"], 1.8)
        self.assertEqual(model.findtext("static"), "true")
        self.assertEqual(
            model.findtext("link/collision/geometry/box/size"), "0.550 1.200 1.500"
        )

    def test_spawn_pose_arguments_explicitly_set_world_position_and_yaw(self):
        arguments = blocked_aisle_spawn_pose_args((2.732796, -0.0005), -0.002403)

        self.assertEqual(
            arguments,
            [
                "-x",
                "2.732796",
                "-y",
                "-0.000500",
                "-z",
                "0.750",
                "-Y",
                "-0.002403000",
            ],
        )
        with self.assertRaises(ValueError):
            blocked_aisle_spawn_pose_args((float("nan"), 0.0), 0.0)

    def test_malformed_or_changed_obstacle_model_is_rejected(self):
        with self.assertRaises(ValueError):
            validate_blocked_aisle_model("<sdf><model")
        changed_size = blocked_aisle_model_sdf().replace(
            "0.550 1.200 1.500", "0.650 1.200 1.500"
        )
        with self.assertRaises(ValueError):
            validate_blocked_aisle_model(changed_size)

    def test_scan_return_requires_a_finite_hit_in_the_expected_bearing_and_range(self):
        evidence = find_scan_return(
            [float("inf"), 1.15, float("nan")],
            angle_min_rad=-0.1,
            angle_increment_rad=0.1,
            target_bearing_rad=0.0,
            max_angle_error_rad=0.02,
            min_range_m=0.1,
            max_range_m=1.5,
        )

        self.assertIsNotNone(evidence)
        self.assertEqual(evidence["index"], 1)
        self.assertAlmostEqual(evidence["range_m"], 1.15)

    def test_scan_return_rejects_off_axis_and_out_of_range_hits(self):
        evidence = find_scan_return(
            [1.0, 1.6],
            angle_min_rad=0.2,
            angle_increment_rad=0.1,
            target_bearing_rad=0.0,
            max_angle_error_rad=0.05,
            min_range_m=0.1,
            max_range_m=1.5,
        )

        self.assertIsNone(evidence)

    def test_scan_return_handles_bearing_wraparound(self):
        evidence = find_scan_return(
            [0.8],
            angle_min_rad=-3.141592653589793,
            angle_increment_rad=0.1,
            target_bearing_rad=3.141592653589793,
            max_angle_error_rad=0.01,
            min_range_m=0.1,
            max_range_m=1.0,
        )

        self.assertIsNotNone(evidence)
        self.assertAlmostEqual(evidence["angular_error_rad"], 0.0)

    def test_disposable_world_injection_is_detectable(self):
        source = "<sdf version='1.9'><world name='warehouse'><physics/></world></sdf>"

        result, metadata = inject_blocked_aisle(source)

        root = ET.fromstring(result)
        model = root.find(f"world/model[@name='{BLOCKED_AISLE_MODEL}']")
        self.assertIsNotNone(model)
        self.assertTrue(world_has_blocked_aisle(result))
        self.assertEqual(metadata["scenario"], "blocked-aisle")
        self.assertFalse(world_has_blocked_aisle(source))

    def test_duplicate_or_malformed_world_is_rejected(self):
        with self.assertRaises(ValueError):
            inject_blocked_aisle("<sdf><world name='warehouse'></sdf>")
        with self.assertRaises(ValueError):
            inject_blocked_aisle("<sdf><world name='warehouse'></world><world></world></sdf>")

    def test_cross_track_deviation_distinguishes_detour_from_straight_path(self):
        start = (0.0, 0.0)
        goal = (2.5, 0.0)
        self.assertEqual(
            max_cross_track_deviation([(0.0, 0.0), (1.0, 0.0), goal], start, goal),
            0.0,
        )
        self.assertAlmostEqual(
            max_cross_track_deviation([(0.0, 0.0), (1.0, 0.8), goal], start, goal),
            0.8,
        )

    def test_points_outside_the_requested_route_segment_do_not_count_as_detours(self):
        start = (0.0, 0.0)
        goal = (2.5, 0.0)
        self.assertAlmostEqual(
            max_cross_track_deviation(
                [(-1.0, 3.0), (1.0, 0.2), (3.0, 4.0)], start, goal
            ),
            0.2,
        )

    def test_preparation_rejects_faults_when_world_instrumentation_is_skipped(self):
        helper = Path(__file__).parent / "scripts" / "prepare_husky_nav2_smoke.py"
        result = subprocess.run(
            [
                sys.executable,
                str(helper),
                "--setup-path",
                "/unused/setup",
                "--source-world",
                "/unused/world.sdf",
                "--output-world",
                "/unused/output.sdf",
                "--manifest",
                "/unused/manifest.json",
                "--fault-injection",
                "blocked-aisle",
                "--skip-world-instrumentation",
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(result.returncode, 1)
        self.assertIn("cannot be combined", result.stderr)

    def test_identical_start_and_goal_cannot_define_a_detour_line(self):
        with self.assertRaises(ValueError):
            max_cross_track_deviation([(0.0, 0.0)], (0.0, 0.0), (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
