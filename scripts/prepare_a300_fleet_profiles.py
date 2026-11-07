#!/usr/bin/env python3
"""Create two disposable Clearpath A300 profiles with distinct ROS namespaces."""

from __future__ import annotations

import argparse
from pathlib import Path

import yaml

ROBOTS = ("a300_00000", "a300_00001")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--template", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    template = yaml.safe_load(args.template.read_text(encoding="utf-8"))
    if not isinstance(template, dict):
        parser.error("The Clearpath profile template must contain a YAML mapping")

    for name in ROBOTS:
        profile = yaml.safe_load(yaml.safe_dump(template))
        # Gazebo Harmonic does not simulate the A300 Phidgets Spatial sensor;
        # the fleet smoke uses the diff-drive controller's simulated wheel odometry.
        profile.setdefault("platform", {})["enable_ekf"] = False
        index = int(name[-1])
        profile["serial_number"] = f"a300-{index:05d}"
        profile["system"]["hosts"][0]["hostname"] = f"cpr-a300-{index:05d}"
        profile["system"]["ros2"]["namespace"] = name
        output = args.output_dir / name
        output.mkdir(parents=True, exist_ok=True)
        (output / "robot.yaml").write_text(
            yaml.safe_dump(profile, sort_keys=False), encoding="utf-8"
        )
        print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
