#!/usr/bin/env python3
"""Prepare a disposable Clearpath profile and Gazebo contact-instrumented world."""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from xml.sax.saxutils import escape


def run(command: list[str], *, capture: bool = False) -> str:
    result = subprocess.run(
        command,
        check=True,
        text=True,
        stdout=subprocess.PIPE if capture else None,
        stderr=subprocess.PIPE if capture else None,
    )
    return result.stdout if capture else ""


def converted_model(urdf: Path) -> ET.Element:
    sdf_text = run(["gz", "sdf", "-p", str(urdf)], capture=True)
    root = ET.fromstring(sdf_text)
    model = root.find("model")
    if model is None:
        raise RuntimeError("Gazebo did not convert the A300 URDF into an SDF model")
    return model


def instrument_robot(
    setup_path: Path,
    manifest_path: Path,
    robot_namespace: str = "a300_00000",
) -> list[dict[str, str]]:
    xacro_path = setup_path / "robot.urdf.xacro"
    if not xacro_path.is_file():
        raise FileNotFoundError(f"Clearpath did not generate {xacro_path}")

    plain_urdf = setup_path / "robot.smoke_base.urdf"
    run(["xacro", str(xacro_path), "-o", str(plain_urdf)])
    model = converted_model(plain_urdf)

    geometries: list[tuple[str, str]] = []
    for link in model.findall("link"):
        link_name = link.get("name")
        for collision in link.findall("collision"):
            collision_name = collision.get("name")
            if link_name and collision_name:
                geometries.append((link_name, collision_name))
    if not geometries:
        raise RuntimeError("No collision geometries were found in the converted A300 model")

    original = xacro_path.read_text(encoding="utf-8")
    if original.count("</robot>") != 1:
        raise RuntimeError("Unexpected Clearpath robot.urdf.xacro structure")

    sensors: list[dict[str, str]] = []
    blocks: list[str] = []
    for index, (link_name, collision_name) in enumerate(geometries):
        topic = f"/{robot_namespace.strip('/')}/smoke/contact/sensor_{index:02d}"
        sensor_name = f"warehouse_smoke_contact_{index:02d}"
        blocks.append(
            "  <gazebo reference=\"{link}\">\n"
            "    <sensor name=\"{sensor}\" type=\"contact\">\n"
            "      <always_on>true</always_on>\n"
            "      <update_rate>25</update_rate>\n"
            "      <contact>\n"
            "        <collision>{collision}</collision>\n"
            "        <topic>{topic}</topic>\n"
            "      </contact>\n"
            "    </sensor>\n"
            "  </gazebo>\n".format(
                link=escape(link_name),
                sensor=escape(sensor_name),
                collision=escape(collision_name),
                topic=escape(topic),
            )
        )
        sensors.append(
            {
                "index": index,
                "link": link_name,
                "collision": collision_name,
                "sensor": sensor_name,
                "topic": topic,
            }
        )

    instrumented = original.replace("</robot>", "".join(blocks) + "</robot>")
    xacro_path.write_text(instrumented, encoding="utf-8")

    instrumented_urdf = setup_path / "robot.smoke_instrumented.urdf"
    run(["xacro", str(xacro_path), "-o", str(instrumented_urdf)])
    instrumented_model = converted_model(instrumented_urdf)
    found: dict[str, tuple[str | None, str | None]] = {}
    for link in instrumented_model.findall("link"):
        for sensor in link.findall("sensor"):
            collision = sensor.findtext("contact/collision")
            topic = sensor.findtext("contact/topic")
            found[sensor.get("name", "")] = (collision, topic)

    for sensor in sensors:
        if found.get(sensor["sensor"]) != (sensor["collision"], sensor["topic"]):
            raise RuntimeError(
                "Gazebo conversion did not preserve contact sensor "
                f"{sensor['sensor']} on {sensor['collision']}"
            )

    manifest_path.write_text(
        json.dumps(
            {
                "robot": "Clearpath Husky A300",
                "namespace": robot_namespace.strip("/"),
                "contact_sensor_count": len(sensors),
                "collision_geometries": sensors,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    return sensors


def instrument_world(source_world: Path, output_world: Path) -> None:
    source = source_world.read_text(encoding="utf-8")
    if "gz-sim-contact-system" in source:
        raise RuntimeError("The source world unexpectedly already contains contact instrumentation")
    physics_pattern = re.compile(
        r"<plugin\b(?=[^>]*(?:name=['\"]gz::sim::systems::Physics['\"]|"
        r"filename=['\"]libgz-sim-physics-system\.so['\"]))"
        r"[^>]*(?:/>|>.*?</plugin>)",
        re.DOTALL,
    )
    match = physics_pattern.search(source)
    if match is None:
        raise RuntimeError(f"Could not find Gazebo's Physics system plugin in {source_world}")
    contact_plugin = (
        '\n    <plugin filename="gz-sim-contact-system" '
        'name="gz::sim::systems::Contact"/>'
    )
    instrumented = source[: match.end()] + contact_plugin + source[match.end() :]
    ET.fromstring(instrumented)
    output_world.write_text(instrumented, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--setup-path", required=True, type=Path)
    parser.add_argument("--source-world", required=True, type=Path)
    parser.add_argument("--output-world", required=True, type=Path)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--robot-namespace", default="a300_00000")
    parser.add_argument("--skip-world-instrumentation", action="store_true")
    args = parser.parse_args()

    try:
        sensors = instrument_robot(
            args.setup_path, args.manifest, args.robot_namespace
        )
        if not args.skip_world_instrumentation:
            instrument_world(args.source_world, args.output_world)
    except (OSError, subprocess.CalledProcessError, ET.ParseError, RuntimeError) as error:
        print(f"Smoke-test instrumentation failed: {error}", file=sys.stderr)
        return 1

    print(
        f"Prepared disposable headless test assets: {len(sensors)} contact sensors; "
        f"namespace={args.robot_namespace}; world={args.output_world}; "
        f"manifest={args.manifest}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
