import math
import unittest

from warehouse_rmf_fleet.coordinate_transform import (
    local_to_rmf,
    normalize_angle,
    rmf_to_local,
)


class CoordinateTransformTest(unittest.TestCase):
    def test_translation_round_trip(self):
        origin = (5.0, -2.0, 0.0)
        local = (1.25, 0.5, -0.3)
        rmf_pose = local_to_rmf(local, origin)
        for actual, expected in zip(rmf_pose, (6.25, -1.5, -0.3)):
            self.assertAlmostEqual(actual, expected, places=12)
        recovered = rmf_to_local(rmf_pose, origin)
        for actual, expected in zip(recovered, local):
            self.assertAlmostEqual(actual, expected, places=12)

    def test_rotated_round_trip(self):
        origin = (-3.0, 4.0, math.pi / 2.0)
        local = (2.0, -1.0, -math.pi / 4.0)
        rmf_pose = local_to_rmf(local, origin)
        self.assertAlmostEqual(rmf_pose[0], -2.0, places=12)
        self.assertAlmostEqual(rmf_pose[1], 6.0, places=12)
        self.assertAlmostEqual(rmf_pose[2], math.pi / 4.0, places=12)
        recovered = rmf_to_local(rmf_pose, origin)
        for actual, expected in zip(recovered, local):
            self.assertAlmostEqual(actual, expected, places=12)

    def test_angle_normalization_handles_wraparound(self):
        self.assertAlmostEqual(normalize_angle(3.0 * math.pi), -math.pi)
        self.assertAlmostEqual(normalize_angle(-3.0 * math.pi), -math.pi)

    def test_wrong_pose_arity_is_rejected(self):
        with self.assertRaises(ValueError):
            local_to_rmf((1.0, 2.0), (0.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
