#!/usr/bin/env python3
"""Launch Clearpath's A300 Nav2 configuration with this package's A* plugin."""

import atexit
import os
import sys
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from clearpath_config.clearpath_config import ClearpathConfig
from clearpath_config.common.utils.yaml import read_yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace

sys.path.insert(0, str(Path(__file__).resolve().parent))
from nav2_fault_profile import apply_fault_test_profile


SCAN_TOPIC_PATHS = (
    ("local_costmap", "local_costmap", "ros__parameters", "voxel_layer", "scan"),
    ("global_costmap", "global_costmap", "ros__parameters", "obstacle_layer", "scan"),
    ("collision_monitor", "ros__parameters", "scan"),
)


def set_nested(mapping, keys, value):
    """Set an existing parameter path and fail loudly if a vendor schema changes."""
    target = mapping
    for key in keys[:-1]:
        target = target[key]
    target[keys[-1]] = value


def write_merged_params(params):
    """Keep the generated Nav2 YAML available for the lifetime of the launch process."""
    descriptor, path = tempfile.mkstemp(prefix="warehouse_a300_nav2_", suffix=".yaml")
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as output:
            yaml.safe_dump(params, output, sort_keys=False)
    except Exception:
        os.unlink(path)
        raise

    atexit.register(lambda: os.path.exists(path) and os.unlink(path))
    return path


def launch_setup(context, *args, **kwargs):
    del args, kwargs
    setup_path = LaunchConfiguration("setup_path").perform(context)
    fault_test_profile_enabled = (
        LaunchConfiguration("fault_test_profile").perform(context).lower() == "true"
    )
    robot_yaml = Path(setup_path) / "robot.yaml"
    if not robot_yaml.is_file():
        raise RuntimeError(
            f"Clearpath robot configuration not found: {robot_yaml}. "
            "Copy this package's config/robot.yaml to the setup path first."
        )

    clearpath_config = ClearpathConfig(read_yaml(str(robot_yaml)))
    namespace = clearpath_config.system.namespace
    platform_model = clearpath_config.platform.get_platform_model()
    if platform_model != "a300":
        raise RuntimeError(
            "This launch profile is specifically for the Clearpath Husky A300; "
            f"robot.yaml resolved to platform '{platform_model}'."
        )

    package_share = get_package_share_directory("warehouse_astar_planner")
    nav2_demos_share = get_package_share_directory("clearpath_nav2_demos")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")
    base_params_path = os.path.join(
        nav2_demos_share, "config", platform_model, "nav2.yaml"
    )
    planner_overlay_path = os.path.join(
        package_share, "config", "planner_params.yaml"
    )
    nav2_launch_path = os.path.join(
        nav2_bringup_share, "launch", "navigation_launch.py"
    )

    params = read_yaml(base_params_path)
    planner_overlay = read_yaml(planner_overlay_path)
    planner_server = params["planner_server"]["ros__parameters"]
    plugin_ids = planner_server.get("planner_plugins", [])
    if "GridBased" not in plugin_ids:
        raise RuntimeError(
            "Clearpath A300 Nav2 defaults no longer define the expected GridBased planner ID"
        )
    planner_server.setdefault("GridBased", {}).update(
        planner_overlay["planner_server"]["ros__parameters"]["GridBased"]
    )

    goal_checker_overrides = (
        planner_overlay.get("controller_server", {})
        .get("ros__parameters", {})
        .get("general_goal_checker", {})
    )
    if fault_test_profile_enabled:
        profile = apply_fault_test_profile(params)
        print(f"Applied Nav2 fault-test profile: {profile}", flush=True)
    elif goal_checker_overrides:
        controller_parameters = params.get("controller_server", {}).get(
            "ros__parameters", {}
        )
        goal_checker_plugins = controller_parameters.get("goal_checker_plugins", [])
        goal_checker = controller_parameters.get("general_goal_checker")
        if "general_goal_checker" not in goal_checker_plugins or not isinstance(
            goal_checker, dict
        ):
            raise RuntimeError(
                "Clearpath A300 Nav2 defaults no longer expose the expected "
                "general_goal_checker plugin"
            )
        unsupported = set(goal_checker_overrides) - set(goal_checker)
        if unsupported:
            raise RuntimeError(
                "Clearpath A300 goal-checker parameter schema changed; "
                f"unsupported overrides: {sorted(unsupported)}"
            )
        goal_checker.update(goal_checker_overrides)

    scan_topic = LaunchConfiguration("scan_topic").perform(context)
    if not scan_topic:
        scan_topic = f"/{namespace}/sensors/lidar2d_0/scan"
    for path in SCAN_TOPIC_PATHS:
        set_nested(params, path + ("topic",), scan_topic)

    merged_params_path = write_merged_params(params)
    nav2_launch = PythonLaunchDescriptionSource(nav2_launch_path)

    # Match Clearpath's Jazzy launch convention: Nav2's composition expression
    # expects the Python literals "True" or "False", not lowercase launch strings.
    composition_literal = str(
        LaunchConfiguration("use_composition").perform(context).lower() == "true"
    )
    namespace_group = GroupAction(
        [
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                nav2_launch,
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "params_file": merged_params_path,
                    "autostart": LaunchConfiguration("autostart"),
                    "use_composition": composition_literal,
                    "use_respawn": LaunchConfiguration("use_respawn"),
                    "log_level": LaunchConfiguration("log_level"),
                    "namespace": namespace,
                }.items(),
            ),
        ]
    )
    return [namespace_group]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "setup_path",
                default_value=os.path.expanduser("~/clearpath"),
                description="Directory containing the generated Clearpath robot.yaml setup",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                choices=["true", "false"],
                description="Use the Gazebo simulation clock",
            ),
            DeclareLaunchArgument(
                "scan_topic",
                default_value="",
                description="Override the A300 2D lidar topic",
            ),
            DeclareLaunchArgument(
                "fault_test_profile",
                default_value="false",
                choices=["true", "false"],
                description=(
                    "Use bounded controller progress timing for the blocked-aisle "
                    "simulation regression only."
                ),
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                choices=["true", "false"],
                description="Autostart Nav2 lifecycle nodes",
            ),
            DeclareLaunchArgument(
                "use_composition",
                default_value="false",
                choices=["false"],
                description="Composition is disabled for this fixed A300 launch profile.",
            ),
            DeclareLaunchArgument(
                "use_respawn",
                default_value="false",
                choices=["true", "false"],
                description="Respawn Nav2 processes if they exit unexpectedly",
            ),
            DeclareLaunchArgument("log_level", default_value="info"),
            OpaqueFunction(function=launch_setup),
        ]
    )
