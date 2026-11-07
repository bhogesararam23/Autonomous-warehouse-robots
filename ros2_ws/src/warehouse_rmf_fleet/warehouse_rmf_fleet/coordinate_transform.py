"""Planar coordinate transforms used at the RMF-to-Nav2 fleet boundary."""

from __future__ import annotations

import math
from typing import Sequence

Pose2D = tuple[float, float, float]


def normalize_angle(angle: float) -> float:
    """Normalize an angle into [-pi, pi), including exact odd multiples of pi."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def local_to_rmf(local_pose: Sequence[float], origin_in_rmf: Sequence[float]) -> Pose2D:
    """Transform a robot-local map pose into the shared RMF facility frame.

    ``origin_in_rmf`` is the facility-frame pose of the local map origin. The
    transform supports translation and yaw rotation; scale is intentionally not
    supported because both maps use metres.
    """
    if len(local_pose) != 3 or len(origin_in_rmf) != 3:
        raise ValueError("local_pose and origin_in_rmf must each contain x, y, yaw")
    x, y, yaw = (float(value) for value in local_pose)
    ox, oy, oyaw = (float(value) for value in origin_in_rmf)
    c = math.cos(oyaw)
    s = math.sin(oyaw)
    return (
        ox + c * x - s * y,
        oy + s * x + c * y,
        normalize_angle(oyaw + yaw),
    )


def rmf_to_local(rmf_pose: Sequence[float], origin_in_rmf: Sequence[float]) -> Pose2D:
    """Transform a shared RMF facility pose into one robot's local map frame."""
    if len(rmf_pose) != 3 or len(origin_in_rmf) != 3:
        raise ValueError("rmf_pose and origin_in_rmf must each contain x, y, yaw")
    x, y, yaw = (float(value) for value in rmf_pose)
    ox, oy, oyaw = (float(value) for value in origin_in_rmf)
    dx = x - ox
    dy = y - oy
    c = math.cos(oyaw)
    s = math.sin(oyaw)
    return (
        c * dx + s * dy,
        -s * dx + c * dy,
        normalize_angle(yaw - oyaw),
    )
