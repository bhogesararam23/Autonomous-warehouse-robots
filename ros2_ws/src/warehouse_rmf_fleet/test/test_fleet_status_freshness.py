import math
import unittest

from warehouse_rmf_fleet.fleet_status_freshness import stale_status_ages


class FleetStatusFreshnessTest(unittest.TestCase):
    def test_status_at_threshold_is_fresh_but_older_status_is_stale(self):
        received = {"a300_00000": 95.0, "a300_00001": 94.999}

        stale = stale_status_ages(received, now=100.0, max_age_seconds=5.0)

        self.assertEqual(set(stale), {"a300_00001"})
        self.assertAlmostEqual(stale["a300_00001"], 5.001)

    def test_future_receipt_timestamp_is_invalid(self):
        stale = stale_status_ages(
            {"a300_00000": 101.0}, now=100.0, max_age_seconds=5.0
        )

        self.assertEqual(stale, {"a300_00000": -1.0})

    def test_nonfinite_receipt_timestamp_is_invalid(self):
        stale = stale_status_ages(
            {"a300_00000": math.nan}, now=100.0, max_age_seconds=5.0
        )

        self.assertTrue(math.isinf(stale["a300_00000"]))

    def test_invalid_clock_and_threshold_are_rejected(self):
        with self.assertRaises(ValueError):
            stale_status_ages({}, now=math.inf, max_age_seconds=5.0)
        with self.assertRaises(ValueError):
            stale_status_ages({}, now=100.0, max_age_seconds=0.0)


if __name__ == "__main__":
    unittest.main()
