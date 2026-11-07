#!/usr/bin/env python3
"""Launch a headless Clearpath A300 world with temporary contact bridges."""

from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    del args, kwargs
    setup_path = Path(LaunchConfiguration("setup_path").perform(context)).resolve()
    world_path = Path(LaunchConfiguration("world_path").perform(context)).resolve()
    contact_count = int(LaunchConfiguration("contact_count").perform(context))
    if not (setup_path / "robot.yaml").is_file():
        raise RuntimeError(f"Clearpath robot.yaml not found in {setup_path}")
    if not world_path.is_file():
        raise RuntimeError(f"Instrumented Gazebo world not found: {world_path}")
    if contact_count < 1:
        raise RuntimeError("At least one robot contact geometry must be instrumented")

    clearpath_gz_share = Path(get_package_share_directory("clearpath_gz"))
    ros_gz_sim_share = Path(get_package_share_directory("ros_gz_sim"))
    ros_gz_bridge_share = Path(get_package_share_directory("ros_gz_bridge"))
    prefix_shares = [
        Path(prefix) / "share"
        for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)
        if prefix
    ]
    resource_paths = [
        world_path.parent,
        clearpath_gz_share / "worlds",
        clearpath_gz_share / "meshes",
        *prefix_shares,
    ]
    inherited_resources = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_strings = list(dict.fromkeys(str(path) for path in resource_paths))
    if inherited_resources:
        resource_strings.append(inherited_resources)

    gz_sim_launch = ros_gz_sim_share / "launch" / "gz_sim.launch.py"
    spawn_launch = clearpath_gz_share / "launch" / "robot_spawn.launch.py"
    if not gz_sim_launch.is_file() or not spawn_launch.is_file():
        raise RuntimeError("Installed Clearpath / ros_gz simulator launch files are missing")

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gz_sim_launch)),
        launch_arguments={
            "gz_args": f"{world_path} -s -r --headless-rendering -v 2",
            "on_exit_shutdown": "true",
        }.items(),
    )
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(spawn_launch)),
        launch_arguments={
            "setup_path": str(setup_path),
            "world": "warehouse",
            "generate": "false",
            "rviz": "false",
            "use_sim_time": "true",
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "z": LaunchConfiguration("z"),
            "yaw": LaunchConfiguration("yaw"),
        }.items(),
    )
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="warehouse_smoke_clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )
    contact_topics = [
        f"/a300_00000/smoke/contact/sensor_{index:02d}"
        for index in range(contact_count)
    ]
    contact_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="warehouse_smoke_contact_bridge",
        output="screen",
        arguments=[
            f"{topic}@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts"
            for topic in contact_topics
        ],
    )

    return [
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=os.pathsep.join(resource_strings),
        ),
        gz_server,
        clock_bridge,
        robot,
        contact_bridge,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("setup_path", description="Disposable Clearpath setup directory"),
            DeclareLaunchArgument("world_path", description="Temporary contact-enabled warehouse SDF"),
            DeclareLaunchArgument("contact_count", description="Number of A300 collision sensors"),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.3"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            OpaqueFunction(function=launch_setup),
        ]
    )
