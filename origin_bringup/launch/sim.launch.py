# Copyright 2026 Soumic Sarkar
# SPDX-License-Identifier: Apache-2.0
"""Gazebo Harmonic simulation of the Origin One in an origin_bringup test world.

    ros2 launch origin_bringup sim.launch.py                  # origin_office world, GUI
    ros2 launch origin_bringup sim.launch.py headless:=true

Reuses origin_one_gazebo's origin_sim_common.launch.py (robot spawn, ros_gz bridge,
robot_state_publisher); only the world file and spawn pose come from this package.
With the GUI, lidar_ray_markers.py draws the Ouster beams in Gazebo (lidar_rays:=false to hide).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory("origin_bringup")
    sim_common = os.path.join(
        get_package_share_directory("origin_one_gazebo"), "launch", "origin_sim_common.launch.py")

    args = [
        DeclareLaunchArgument("world", default_value="origin_office.sdf",
                              description="World file in origin_bringup/worlds"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("lidar_rays", default_value="true",
                              description="Draw the Ouster beams in the Gazebo GUI"),
        DeclareLaunchArgument("rviz", default_value="false",
                              description="Open origin_one_gazebo's sensor RViz config"),
        DeclareLaunchArgument("drive_configuration", default_value="skid_steer_drive"),
        DeclareLaunchArgument("robot_pose_x", default_value="0.0"),
        DeclareLaunchArgument("robot_pose_y", default_value="0.0"),
        DeclareLaunchArgument("robot_pose_yaw", default_value="0.0"),
    ]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_common),
        launch_arguments={
            # origin_sim_common joins this onto its own worlds/ dir; an absolute path overrides that
            "world": PathJoinSubstitution([bringup, "worlds", LaunchConfiguration("world")]),
            "headless": LaunchConfiguration("headless"),
            "rviz": LaunchConfiguration("rviz"),
            "drive_configuration": LaunchConfiguration("drive_configuration"),
            "robot_pose_x": LaunchConfiguration("robot_pose_x"),
            "robot_pose_y": LaunchConfiguration("robot_pose_y"),
            "robot_pose_yaw": LaunchConfiguration("robot_pose_yaw"),
        }.items(),
    )

    rays = Node(
        package="origin_bringup", executable="lidar_ray_markers.py", name="lidar_ray_markers",
        output="screen",
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("lidar_rays"), "'.lower() == 'true' and '",
            LaunchConfiguration("headless"), "'.lower() != 'true'"])),
    )

    return LaunchDescription(args + [sim, rays])
