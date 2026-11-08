import copy
import math
import unittest

from warehouse_rmf_fleet.robot_availability import (
    RMFRobotAvailabilityGate,
    RMFUpdateHandleNotReady,
    TransformFreshnessMonitor,
)

NANOSECONDS_PER_SECOND = 1_000_000_000


class FakeCommission:
    def __init__(self, dispatched=True, direct=True, idle=True):
        self.accept_dispatched_tasks = dispatched
        self.accept_direct_tasks = direct
        self.perform_idle_behavior = idle


class FakeRobotUpdateHandle:
    def __init__(self, commission):
        self._commission = copy.copy(commission)
        self.set_calls = 0

    def commission(self):
        return copy.copy(self._commission)

    def set_commission(self, commission):
        self._commission = copy.copy(commission)
        self.set_calls += 1


class FakeEasyUpdateHandle:
    def __init__(self, commission):
        self.robot_update_handle = FakeRobotUpdateHandle(commission)

    def more(self):
        return self.robot_update_handle


class PendingEasyUpdateHandle:
    def more(self):
        return None


class TransformFreshnessMonitorTest(unittest.TestCase):
    def setUp(self):
        self.monitor = TransformFreshnessMonitor(max_age_seconds=1.0)

    def test_recent_advancing_transform_is_fresh(self):
        self.assertTrue(
            self.monitor.observe(
                10 * NANOSECONDS_PER_SECOND,
                10 * NANOSECONDS_PER_SECOND,
                monotonic_now=20.0,
            )
        )
        self.assertTrue(
            self.monitor.observe(
                10 * NANOSECONDS_PER_SECOND + 400_000_000,
                10 * NANOSECONDS_PER_SECOND + 400_000_000,
                monotonic_now=20.4,
            )
        )

    def test_old_and_future_transform_timestamps_are_stale(self):
        self.assertFalse(
            self.monitor.observe(
                10 * NANOSECONDS_PER_SECOND,
                11 * NANOSECONDS_PER_SECOND + 1,
                monotonic_now=20.0,
            )
        )
        future_monitor = TransformFreshnessMonitor(max_age_seconds=1.0)
        self.assertFalse(
            future_monitor.observe(
                12 * NANOSECONDS_PER_SECOND,
                11 * NANOSECONDS_PER_SECOND,
                monotonic_now=20.0,
            )
        )

    def test_paused_ros_clock_does_not_keep_a_frozen_transform_fresh(self):
        stamp = 10 * NANOSECONDS_PER_SECOND
        self.assertTrue(self.monitor.observe(stamp, stamp, monotonic_now=20.0))
        self.assertTrue(self.monitor.observe(stamp, stamp, monotonic_now=20.9))
        self.assertFalse(self.monitor.observe(stamp, stamp, monotonic_now=21.01))

        recovered_stamp = stamp + 100_000_000
        self.assertTrue(
            self.monitor.observe(
                recovered_stamp,
                recovered_stamp,
                monotonic_now=21.1,
            )
        )

    def test_backwards_transform_time_fails_closed_until_time_advances(self):
        self.assertTrue(
            self.monitor.observe(
                20 * NANOSECONDS_PER_SECOND,
                20 * NANOSECONDS_PER_SECOND,
                monotonic_now=30.0,
            )
        )
        self.assertFalse(
            self.monitor.observe(
                5 * NANOSECONDS_PER_SECOND,
                5 * NANOSECONDS_PER_SECOND,
                monotonic_now=31.0,
            )
        )
        self.assertTrue(
            self.monitor.observe(
                5 * NANOSECONDS_PER_SECOND + 100_000_000,
                5 * NANOSECONDS_PER_SECOND + 100_000_000,
                monotonic_now=31.1,
            )
        )

    def test_zero_clock_values_and_invalid_configuration_fail_closed(self):
        self.assertFalse(self.monitor.observe(0, 1, monotonic_now=1.0))
        self.assertFalse(self.monitor.observe(1, 0, monotonic_now=1.0))
        for value in (0.0, -1.0, math.inf, math.nan):
            with self.subTest(value=value), self.assertRaises(ValueError):
                TransformFreshnessMonitor(value)


class RMFRobotAvailabilityGateTest(unittest.TestCase):
    def test_uninitialized_rmf_handle_is_reported_as_pending(self):
        with self.assertRaises(RMFUpdateHandleNotReady):
            RMFRobotAvailabilityGate(PendingEasyUpdateHandle())

    def test_stale_transition_blocks_new_tasks_and_recovery_restores_policy(self):
        easy_handle = FakeEasyUpdateHandle(FakeCommission(True, False, True))
        gate = RMFRobotAvailabilityGate(easy_handle)
        robot_handle = easy_handle.robot_update_handle

        self.assertTrue(gate.mark_unavailable())
        self.assertFalse(gate.available)
        self.assertFalse(robot_handle.commission().accept_dispatched_tasks)
        self.assertFalse(robot_handle.commission().accept_direct_tasks)
        self.assertTrue(robot_handle.commission().perform_idle_behavior)
        self.assertFalse(gate.mark_unavailable())
        self.assertEqual(robot_handle.set_calls, 1)

        self.assertTrue(gate.mark_available())
        self.assertTrue(gate.available)
        restored = robot_handle.commission()
        self.assertTrue(restored.accept_dispatched_tasks)
        self.assertFalse(restored.accept_direct_tasks)
        self.assertTrue(restored.perform_idle_behavior)
        self.assertFalse(gate.mark_available())
        self.assertEqual(robot_handle.set_calls, 2)

    def test_healthy_operator_policy_changes_are_preserved_on_recovery(self):
        easy_handle = FakeEasyUpdateHandle(FakeCommission(True, True, True))
        gate = RMFRobotAvailabilityGate(easy_handle)
        robot_handle = easy_handle.robot_update_handle

        changed = robot_handle.commission()
        changed.accept_dispatched_tasks = False
        changed.accept_direct_tasks = True
        robot_handle.set_commission(changed)
        gate.refresh_baseline()
        gate.mark_unavailable()
        gate.mark_available()

        restored = robot_handle.commission()
        self.assertFalse(restored.accept_dispatched_tasks)
        self.assertTrue(restored.accept_direct_tasks)


if __name__ == "__main__":
    unittest.main()
