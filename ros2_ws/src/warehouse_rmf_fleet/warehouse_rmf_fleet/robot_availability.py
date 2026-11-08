"""Simulation bridge guards for stale robot telemetry and task intake."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

NANOSECONDS_PER_SECOND = 1_000_000_000
DEFAULT_TRANSFORM_MAX_AGE_SECONDS = 1.0


class TransformFreshnessMonitor:
    """Check TF source time and source-time progress using a steady local clock.

    ROS time catches delayed or future-stamped transforms. The steady-time
    progress check also catches a paused simulation clock or a transform source
    that repeats the same old timestamp.
    """

    def __init__(self, max_age_seconds: float = DEFAULT_TRANSFORM_MAX_AGE_SECONDS):
        if not math.isfinite(max_age_seconds) or max_age_seconds <= 0.0:
            raise ValueError("max_age_seconds must be finite and positive")
        self._max_age_ns = int(round(max_age_seconds * NANOSECONDS_PER_SECOND))
        self._max_age_seconds = float(max_age_seconds)
        if self._max_age_ns <= 0:
            raise ValueError("max_age_seconds must be at least one nanosecond")
        self._last_stamp_ns: int | None = None
        self._last_progress_monotonic: float | None = None

    def observe(
        self,
        transform_stamp_ns: int,
        ros_now_ns: int,
        *,
        monotonic_now: float | None = None,
    ) -> bool:
        """Return whether a transform is fresh in ROS time and still advancing.

        A backwards transform timestamp is treated as a time reset and fails
        closed for that sample. A later advancing timestamp can recover.
        """
        if isinstance(transform_stamp_ns, bool) or not isinstance(transform_stamp_ns, int):
            raise ValueError("transform_stamp_ns must be an integer")
        if isinstance(ros_now_ns, bool) or not isinstance(ros_now_ns, int):
            raise ValueError("ros_now_ns must be an integer")
        wall_now = time.monotonic() if monotonic_now is None else monotonic_now
        if not math.isfinite(wall_now):
            raise ValueError("monotonic_now must be finite")

        if transform_stamp_ns <= 0 or ros_now_ns <= 0:
            return False

        if self._last_stamp_ns is None:
            self._last_stamp_ns = transform_stamp_ns
            self._last_progress_monotonic = wall_now
        elif transform_stamp_ns < self._last_stamp_ns:
            self._last_stamp_ns = transform_stamp_ns
            self._last_progress_monotonic = wall_now
            return False
        elif transform_stamp_ns > self._last_stamp_ns:
            self._last_stamp_ns = transform_stamp_ns
            self._last_progress_monotonic = wall_now

        assert self._last_progress_monotonic is not None
        ros_age_ns = ros_now_ns - transform_stamp_ns
        wall_age = wall_now - self._last_progress_monotonic
        return (
            0 <= ros_age_ns <= self._max_age_ns
            and 0.0 <= wall_age <= self._max_age_seconds
        )


class RMFUpdateHandleNotReady(RuntimeError):
    """The EasyFullControl worker has not attached its RobotUpdateHandle yet."""


@dataclass(frozen=True)
class TaskAcceptancePolicy:
    """RMF's existing task-intake policy before a telemetry fault."""

    accept_dispatched_tasks: bool
    accept_direct_tasks: bool


class RMFRobotAvailabilityGate:
    """Temporarily block new RMF tasks while a robot's pose is stale.

    The gate preserves the robot's last observed intake policy, changes only the
    dispatched/direct task acceptance flags, and restores that policy on fresh
    telemetry. It does not cancel or stop an already-running robot action.
    """

    def __init__(self, easy_update_handle: object) -> None:
        self._robot_update_handle = easy_update_handle.more()
        if self._robot_update_handle is None:
            raise RMFUpdateHandleNotReady(
                "RMF has not initialized the underlying RobotUpdateHandle yet"
            )
        self._baseline = self._read_policy()
        self._available = True

    @property
    def available(self) -> bool:
        return self._available

    def refresh_baseline(self) -> None:
        """Track policy changes made while telemetry is healthy."""
        if self._available:
            self._baseline = self._read_policy()

    def mark_unavailable(self) -> bool:
        """Stop accepting new direct/dispatched tasks; return on transitions."""
        if not self._available:
            return False
        self._write_policy(TaskAcceptancePolicy(False, False))
        self._available = False
        return True

    def mark_available(self) -> bool:
        """Restore the last healthy task-intake policy; return on transitions."""
        if self._available:
            return False
        self._write_policy(self._baseline)
        self._available = True
        return True

    def _read_policy(self) -> TaskAcceptancePolicy:
        commission = self._robot_update_handle.commission()
        return TaskAcceptancePolicy(
            accept_dispatched_tasks=bool(commission.accept_dispatched_tasks),
            accept_direct_tasks=bool(commission.accept_direct_tasks),
        )

    def _write_policy(self, policy: TaskAcceptancePolicy) -> None:
        commission = self._robot_update_handle.commission()
        commission.accept_dispatched_tasks = policy.accept_dispatched_tasks
        commission.accept_direct_tasks = policy.accept_direct_tasks
        self._robot_update_handle.set_commission(commission)
