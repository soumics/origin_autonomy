# Copyright 2026 Soumic Sarkar
# SPDX-License-Identifier: Apache-2.0
"""Navigation for the Origin One with a selectable backend.

    ros2 launch origin_frontier_explore navigation.launch.py backend:=nav2    # Nav2
    ros2 launch origin_frontier_explore navigation.launch.py backend:=custom  # A* + DWA

Both serve the nav2_msgs/action/NavigateToPose action on /navigate_to_pose and accept RViz
"2D Goal Pose" (/goal_pose). Both need /map and TF map -> base_link from
origin_lidar_localization (mapping or localization mode). The lidar self-filter
(/robot/lidar/points -> /robot/lidar/points_filtered) is started for either backend.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

NAV2_NODES = [
    "controller_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "velocity_smoother",
    "collision_monitor",
]


def launch_setup(context):
    pkg = get_package_share_directory("origin_frontier_explore")
    custom_params = os.path.join(pkg, "config", "custom_nav.yaml")
    nav2_params = os.path.join(pkg, "config", "nav2_params.yaml")
    backend = LaunchConfiguration("backend").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() in ("true", "1")
    common = {"use_sim_time": use_sim_time}
    lidar = LaunchConfiguration("lidar_topic").perform(context)
    filtered = lidar + "_filtered"
    odom = LaunchConfiguration("odom_topic").perform(context)
    cmd_vel = LaunchConfiguration("cmd_vel_topic").perform(context)
    # The Nav2 file names the simulation topics; rewrite them for the robot at hand.
    nav2_params = RewrittenYaml(
        source_file=nav2_params, root_key="", convert_types=True,
        param_rewrites={"topic": filtered, "odom_topic": odom, "cmd_vel_out_topic": cmd_vel,
                        "default_nav_to_pose_bt_xml": os.path.join(
                            pkg, "behavior_trees", "navigate_to_pose_no_backup.xml")})

    nodes = [Node(
        package="origin_frontier_explore", executable="cloud_self_filter",
        name="cloud_self_filter", output="screen",
        parameters=[custom_params, common],
        remappings=[("input", lidar), ("output", filtered)],
    )]

    if backend == "custom":
        nodes.append(Node(
            package="origin_frontier_explore", executable="custom_navigator",
            name="custom_navigator", output="screen",
            parameters=[custom_params, common],
            remappings=[
                ("cmd_vel", cmd_vel),
                ("map", "/map"),
                ("cloud", filtered),
                ("odom", odom),
                ("goal_pose", "/goal_pose"),
            ],
        ))
    elif backend == "nav2":
        cmd_nav = [("cmd_vel", "cmd_vel_nav")]
        nodes += [
            Node(package="nav2_controller", executable="controller_server", output="screen",
                 parameters=[nav2_params, common], remappings=cmd_nav),
            Node(package="nav2_planner", executable="planner_server", name="planner_server",
                 output="screen", parameters=[nav2_params, common]),
            Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server",
                 output="screen", parameters=[nav2_params, common], remappings=cmd_nav),
            Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator",
                 output="screen", parameters=[nav2_params, common]),
            Node(package="nav2_velocity_smoother", executable="velocity_smoother",
                 name="velocity_smoother", output="screen",
                 parameters=[nav2_params, common], remappings=cmd_nav),
            Node(package="nav2_collision_monitor", executable="collision_monitor",
                 name="collision_monitor", output="screen", parameters=[nav2_params, common]),
        ]
        # Activate Nav2 only once RTAB-Map publishes map -> base_link; the costmaps fail to
        # activate otherwise (in localization mode the database must load first).
        wait_tf = Node(
            package="origin_frontier_explore", executable="wait_for_transform.py",
            name="wait_for_transform", output="screen",
            arguments=["map", "base_link", "120"], parameters=[common],
        )
        lifecycle_manager = Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_navigation", output="screen",
            parameters=[common, {"autostart": True, "node_names": NAV2_NODES}],
        )
        nodes += [
            wait_tf,
            RegisterEventHandler(OnProcessExit(target_action=wait_tf, on_exit=[lifecycle_manager])),
        ]
    else:
        raise RuntimeError(f"backend must be 'nav2' or 'custom', got '{backend}'")
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("backend", default_value="nav2", description="nav2 | custom"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("lidar_topic", default_value="/robot/lidar/points"),
        DeclareLaunchArgument("odom_topic", default_value="/robot/odom"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/robot/cmd_vel",
                              description="On the real Origin One: /robot/cmd_vel_user"),
        OpaqueFunction(function=launch_setup),
    ])
