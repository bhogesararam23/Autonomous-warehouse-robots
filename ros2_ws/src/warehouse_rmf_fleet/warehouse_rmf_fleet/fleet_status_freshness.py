"""Wall-clock freshness checks for received RMF robot status updates."""

from __future__ import annotations

import math
from collections.abc import Mapping


def stale_status_ages(
    last_received_at: Mapping[str, float], *, now: float, max_age_seconds: float
) -> dict[str, float]:
    """Return robot names whose local receipt timestamps are stale or invalid.

    RMF ``FleetState`` does not expose a per-robot source timestamp, so callers
    must supply local monotonic receipt times. A future timestamp is treated as
    invalid rather than fresh.
    """
    if not math.isfinite(now):
        raise ValueError("now must be a finite monotonic timestamp")
    if not math.isfinite(max_age_seconds) or max_age_seconds <= 0.0:
        raise ValueError("max_age_seconds must be finite and positive")

    stale: dict[str, float] = {}
    for robot, received_at in last_received_at.items():
        if not math.isfinite(received_at):
            stale[robot] = math.inf
            continue
        age = now - received_at
        if age < 0.0 or age > max_age_seconds:
            stale[robot] = age
    return stale
