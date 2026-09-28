# Copyright 2026 Soumic Sarkar
# SPDX-License-Identifier: Apache-2.0
"""Autonomous exploration: RTAB-Map builds the map while the frontier explorer drives.

    ros2 launch origin_frontier_explore explore.launch.py use_sim_time:=true backend:=nav2
    ros2 launch origin_frontier_explore explore.launch.py use_sim_time:=true backend:=custom

Starts with no map. RTAB-Map (mapping mode, new database at database_path) builds the map
incrementally and closes loops. The frontier explorer sends NavigateToPose goals to the
selected backend until no frontiers are left, then returns to the start. The map database is
written when RTAB-Map shuts down (Ctrl-C).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("origin_frontier_explore")
    slam_launch = os.path.join(
        get_package_share_directory("origin_lidar_localization"), "launch", "slam.launch.py")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("backend", default_value="nav2", description="nav2 | custom"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("slam", default_value="true",
                              description="Start RTAB-Map in mapping mode"),
        DeclareLaunchArgument("database_path", default_value="~/.ros/origin_rtabmap.db"),
        DeclareLaunchArgument("rviz", default_value="false"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            condition=IfCondition(LaunchConfiguration("slam")),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "localization": "false",
                "database_path": LaunchConfiguration("database_path"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "navigation.launch.py")),
            launch_arguments={
                "backend": LaunchConfiguration("backend"),
                "use_sim_time": use_sim_time,
            }.items(),
        ),
        Node(
            package="origin_frontier_explore", executable="frontier_explorer",
            name="frontier_explorer", output="screen",
            parameters=[os.path.join(pkg, "config", "explorer.yaml"),
                        {"use_sim_time": use_sim_time}],
            remappings=[("map", "/map"), ("cmd_vel", "/robot/cmd_vel")],
        ),
        Node(
            package="rviz2", executable="rviz2",
            arguments=["-d", os.path.join(pkg, "rviz", "explore.rviz")],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
