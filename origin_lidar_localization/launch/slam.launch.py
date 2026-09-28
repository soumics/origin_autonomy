# Copyright 2026 Soumic Sarkar
# SPDX-License-Identifier: Apache-2.0
"""3D lidar SLAM / localization for the Origin One with RTAB-Map.

Mapping (default):   ros2 launch origin_lidar_localization slam.launch.py
Localization:        ros2 launch origin_lidar_localization slam.launch.py localization:=true

Mapping starts a fresh database at `database_path` (the old one is deleted). Localization
loads that database read-only. Publishes /map (2D occupancy grid for Nav2),
/rtabmap/cloud_map (3D point cloud map), /rtabmap/octomap_* and TF map -> icp_odom, plus
<lidar_topic>_viz: the Ouster cloud for RViz, released once its TF to map is available.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _is_true(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ("true", "1")


def launch_setup(context):
    pkg = get_package_share_directory("origin_lidar_localization")
    params_file = os.path.join(pkg, "config", "rtabmap_lidar3d.yaml")

    localization = _is_true(context, "localization")
    deskewing = _is_true(context, "deskewing")
    database_path = os.path.expanduser(LaunchConfiguration("database_path").perform(context))
    lidar_topic = LaunchConfiguration("lidar_topic").perform(context)

    common = {"use_sim_time": _is_true(context, "use_sim_time"), "database_path": database_path}

    # Real Ouster clouds carry per-point timestamps; deskew them against the wheel odometry frame.
    # Gazebo's gpu_lidar has no per-point time field, so this stays off in simulation.
    scan_topic = lidar_topic + "/deskewed" if deskewing else lidar_topic

    rtabmap_params = {}
    arguments = []
    if localization:
        rtabmap_params.update({
            "Mem/IncrementalMemory": "false",
            "Mem/InitWMWithAllNodes": "true",
            "RGBD/StartAtOrigin": LaunchConfiguration("start_at_origin").perform(context),
        })
    else:
        arguments.append("-d")  # delete the previous database and start a new map

    nodes = []
    if deskewing:
        nodes.append(Node(
            package="rtabmap_util", executable="lidar_deskewing", namespace="rtabmap",
            output="screen",
            parameters=[{"use_sim_time": common["use_sim_time"], "fixed_frame_id": "odom",
                         "wait_for_transform": 0.2, "slerp": True}],
            remappings=[("input_cloud", lidar_topic)],
        ))

    nodes += [
        Node(
            package="rtabmap_odom", executable="icp_odometry", namespace="rtabmap",
            output="screen",
            parameters=[params_file, common],
            remappings=[("scan_cloud", scan_topic), ("odom", "icp_odom")],
        ),
        Node(
            package="rtabmap_slam", executable="rtabmap", namespace="rtabmap",
            output="screen",
            parameters=[params_file, common, rtabmap_params],
            remappings=[
                ("scan_cloud", scan_topic),
                ("odom", "icp_odom"),
                ("map", "/map"),
                ("initialpose", "/initialpose"),
                ("goal", "/goal_pose_rtabmap"),  # keep RViz 2D Goal Pose for Nav2
            ],
            arguments=arguments,
        ),
    ]
    # Ouster cloud for RViz, released once map -> lidar TF exists (see tf_synced_relay.py).
    nodes.append(Node(
        package="origin_lidar_localization", executable="tf_synced_relay.py",
        name="lidar_viz_relay", output="screen",
        parameters=[{"use_sim_time": common["use_sim_time"], "fixed_frame": "map"}],
        remappings=[("input", lidar_topic), ("output", lidar_topic + "_viz")],
    ))
    return nodes


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory("origin_lidar_localization"), "rviz", "slam.rviz")
    return LaunchDescription([
        DeclareLaunchArgument("localization", default_value="false",
                              description="false: build a new map; true: localize in the saved map"),
        DeclareLaunchArgument("database_path", default_value="~/.ros/origin_rtabmap.db",
                              description="RTAB-Map database (map) file"),
        DeclareLaunchArgument("lidar_topic", default_value="/robot/lidar/points",
                              description="Ouster PointCloud2 topic"),
        DeclareLaunchArgument("deskewing", default_value="false",
                              description="Deskew lidar scans (real Ouster only)"),
        DeclareLaunchArgument("start_at_origin", default_value="true",
                              description="Localization: assume the robot starts where the map started. "
                                          "Otherwise set the pose with RViz 2D Pose Estimate"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        OpaqueFunction(function=launch_setup),
        Node(
            package="rviz2", executable="rviz2", arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
