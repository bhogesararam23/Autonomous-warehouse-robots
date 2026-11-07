"""Launch two namespaced A300 simulations and their Open-RMF fleet services."""

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
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    del args, kwargs
    setup_paths = [
        Path(LaunchConfiguration("setup_path_0").perform(context)).resolve(),
        Path(LaunchConfiguration("setup_path_1").perform(context)).resolve(),
    ]
    for path in setup_paths:
        if not (path / "robot.yaml").is_file():
            raise RuntimeError(f"Clearpath profile not found: {path / 'robot.yaml'}")
        if not (path / "robot.urdf.xacro").is_file():
            raise RuntimeError(f"Clearpath description not generated in {path}")

    package_share = Path(get_package_share_directory("warehouse_rmf_fleet"))
    clearpath_share = Path(get_package_share_directory("clearpath_gz"))
    ros_gz_sim_share = Path(get_package_share_directory("ros_gz_sim"))
    clearpath_spawn = clearpath_share / "launch" / "robot_spawn.launch.py"
    clearpath_slam = Path(get_package_share_directory("clearpath_nav2_demos")) / "launch" / "slam.launch.py"
    planner_launch = Path(get_package_share_directory("warehouse_astar_planner")) / "launch" / "nav2.launch.py"
    world_path = Path(LaunchConfiguration("world_path").perform(context)).resolve()
    contact_count = int(LaunchConfiguration("contact_count").perform(context))
    if not world_path.is_file():
        raise RuntimeError(f"Gazebo world not found: {world_path}")
    if contact_count < 1:
        raise RuntimeError("The smoke run expects instrumented contact sensors on both A300s")
    for path in (clearpath_spawn, clearpath_slam, planner_launch):
        if not path.is_file():
            raise RuntimeError(f"Required launch file is missing: {path}")

    prefix_shares = [
        Path(prefix) / "share"
        for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)
        if prefix
    ]
    resource_paths = [world_path.parent, clearpath_share / "worlds", clearpath_share / "meshes", *prefix_shares]
    inherited_resources = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_strings = list(dict.fromkeys(str(path) for path in resource_paths))
    if inherited_resources:
        resource_strings.append(inherited_resources)

    use_sim_time = LaunchConfiguration("use_sim_time")
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ros_gz_sim_share / "launch" / "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": f"{world_path} -s -r --headless-rendering -v 2",
            "on_exit_shutdown": "true",
        }.items(),
    )
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="warehouse_rmf_clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )
    namespaces = ("a300_00000", "a300_00001")
    x_positions = (
        LaunchConfiguration("robot_0_x"),
        LaunchConfiguration("robot_1_x"),
    )
    yaw_positions = (
        "0.0",
        LaunchConfiguration("robot_1_yaw"),
    )
    spawns = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(clearpath_spawn)),
            launch_arguments={
                "setup_path": str(setup_paths[index]),
                "world": "warehouse",
                "generate": "false",
                "rviz": "false",
                "use_sim_time": "true",
                "x": x_positions[index],
                "y": "0.0",
                "z": "0.3",
                "yaw": yaw_positions[index],
            }.items(),
        )
        for index in range(2)
    ]
    contact_topics = [
        f"/{namespace}/smoke/contact/sensor_{index:02d}"
        for namespace in namespaces
        for index in range(contact_count)
    ]
    contact_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="warehouse_rmf_contact_bridge",
        output="screen",
        arguments=[
            f"{topic}@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts"
            for topic in contact_topics
        ],
    )

    slam_launches = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(clearpath_slam)),
            launch_arguments={
                "setup_path": str(setup_paths[index]),
                "use_sim_time": use_sim_time,
                "sync": "true",
                "autostart": "true",
            }.items(),
        )
        for index in range(2)
    ]
    nav2_launches = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(planner_launch)),
            launch_arguments={
                "setup_path": str(setup_paths[index]),
                "use_sim_time": use_sim_time,
            }.items(),
        )
        for index in range(2)
    ]

    building_map = package_share / "config" / "warehouse_a300_fleet.building.yaml"
    fleet_config = package_share / "config" / "rmf_fleet.yaml"
    nav_graph = package_share / "config" / "nav_graphs" / "0.yaml"
    rmf_core = [
        Node(
            package="rmf_traffic_ros2",
            executable="rmf_traffic_schedule",
            name="rmf_traffic_schedule_primary",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            cwd=str(world_path.parent),
        ),
        Node(
            package="rmf_traffic_ros2",
            executable="rmf_traffic_blockade",
            name="rmf_traffic_blockade",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            cwd=str(world_path.parent),
        ),
        Node(
            package="rmf_fleet_adapter",
            executable="mutex_group_supervisor",
            name="mutex_group_supervisor",
            output="screen",
            cwd=str(world_path.parent),
        ),
        Node(
            package="rmf_building_map_tools",
            executable="building_map_server",
            name="warehouse_a300_building_map_server",
            arguments=[str(building_map)],
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="rmf_task_ros2",
            executable="rmf_task_dispatcher",
            name="rmf_task_dispatcher",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "bidding_time_window": 2.0,
                    "use_unique_hex_string_with_task_id": True,
                    "server_uri": "",
                }
            ],
        ),
    ]
    adapter = Node(
        package="warehouse_rmf_fleet",
        executable="fleet_adapter",
        name="warehouse_a300_fleet_adapter",
        arguments=[
            "--config",
            str(fleet_config),
            "--nav-graph",
            str(nav_graph),
            "--use-sim-time",
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return [
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH", value=os.pathsep.join(resource_strings)
        ),
        gz_server,
        clock_bridge,
        contact_bridge,
        spawns[0],
        TimerAction(period=18.0, actions=[spawns[1]]),
        TimerAction(period=40.0, actions=[slam_launches[0]]),
        TimerAction(period=55.0, actions=[slam_launches[1]]),
        TimerAction(period=60.0, actions=rmf_core),
        # Nav2 lifecycle activation and costmap setup are CPU-heavy; keep the
        # two A300 bring-ups separate and let both action servers settle before
        # the adapter starts registering their fleet states.
        TimerAction(period=70.0, actions=[nav2_launches[0]]),
        TimerAction(period=120.0, actions=[nav2_launches[1]]),
        TimerAction(period=170.0, actions=[adapter]),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("setup_path_0", description="Disposable A300 profile directory for a300_00000"),
            DeclareLaunchArgument("setup_path_1", description="Disposable A300 profile directory for a300_00001"),
            DeclareLaunchArgument("world_path", description="Gazebo Harmonic warehouse SDF"),
            DeclareLaunchArgument("contact_count", default_value="17"),
            DeclareLaunchArgument("robot_0_x", default_value="0.0"),
            DeclareLaunchArgument("robot_1_x", default_value="3.5"),
            DeclareLaunchArgument(
                "robot_1_yaw",
                default_value="3.141592653589793",
                description="Facility heading of a300_00001 in radians (π faces west)",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            OpaqueFunction(function=launch_setup),
        ]
    )
