# Copyright 2026 Soumic Sarkar
# SPDX-License-Identifier: Apache-2.0
"""Single entry point for the Origin One autonomy stack.

    # Gazebo, no map: explore autonomously, build the 3D map, detect/track/count objects
    ros2 launch origin_bringup bringup.launch.py

    # Gazebo, saved map: localize and navigate to RViz "2D Goal Pose" goals
    ros2 launch origin_bringup bringup.launch.py mode:=navigate

    # Real robot (no Gazebo, wall clock)
    ros2 launch origin_bringup bringup.launch.py sim:=false

Started in order:
  1. Gazebo + robot (sim:=true)
  2. RTAB-Map: mapping (mode:=explore) or localization in database_path (mode:=navigate)
  3. Navigation (backend:=nav2 | custom), plus the frontier explorer in explore mode
  4. YOLO perception (perception:=false to skip)
  5. RViz (rviz:=false to skip): robot, TF, Ouster, map, costmaps, paths, frontiers,
     YOLO image and 3D object markers

Main arguments: sim, mode, backend, world, database_path, perception, rviz, lidar_rays,
headless, yolo_model.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _share(pkg, *path):
    return os.path.join(get_package_share_directory(pkg), *path)


def _include(pkg, launch_file, **args):
    return GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_share(pkg, "launch", launch_file)),
        launch_arguments={k: str(v) for k, v in args.items()}.items())], scoped=True)


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def flag(name):
        return arg(name).lower() in ("true", "1", "yes")

    sim = flag("sim")
    mode = arg("mode")
    if mode not in ("explore", "navigate"):
        raise RuntimeError(f"mode must be 'explore' or 'navigate', got '{mode}'")
    backend = arg("backend")
    use_sim_time = "true" if sim else "false"
    delay = 8.0 if sim else 0.0  # let Gazebo start and spawn the robot first

    actions = []
    if sim:
        actions.append(_include(
            "origin_bringup", "sim.launch.py", world=arg("world"), headless=arg("headless"),
            lidar_rays=arg("lidar_rays")))

    if mode == "explore":
        stack = [_include(
            "origin_frontier_explore", "explore.launch.py", use_sim_time=use_sim_time,
            backend=backend, database_path=arg("database_path"), rviz="false")]
    else:
        stack = [
            _include("origin_lidar_localization", "slam.launch.py", use_sim_time=use_sim_time,
                     localization="true", start_at_origin=arg("start_at_origin"),
                     database_path=arg("database_path")),
            _include("origin_frontier_explore", "navigation.launch.py",
                     use_sim_time=use_sim_time, backend=backend),
        ]
    actions.append(TimerAction(period=delay, actions=stack))

    if flag("perception"):
        perception = {"use_sim_time": use_sim_time, "sim": "true" if sim else "false"}
        if arg("yolo_model"):
            perception["model"] = arg("yolo_model")
        actions.append(TimerAction(period=delay, actions=[
            _include("origin_yolo_perception", "perception.launch.py", **perception)]))

    if flag("rviz"):
        actions.append(TimerAction(period=delay / 2, actions=[Node(
            package="rviz2", executable="rviz2", name="rviz2", output="log",
            arguments=["-d", _share("origin_bringup", "rviz", "bringup.rviz")],
            parameters=[{"use_sim_time": sim}])]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="true",
                              description="true: Gazebo + sim time; false: real robot"),
        DeclareLaunchArgument("mode", default_value="explore",
                              description="explore: map from scratch autonomously; "
                                          "navigate: localize in database_path, goals from RViz"),
        DeclareLaunchArgument("backend", default_value="nav2", description="nav2 | custom"),
        DeclareLaunchArgument("world", default_value="origin_office_people.sdf",
                              description="Gazebo world in origin_bringup/worlds"),
        DeclareLaunchArgument("database_path", default_value="~/.ros/origin_rtabmap.db",
                              description="RTAB-Map map database (written in explore mode)"),
        DeclareLaunchArgument("start_at_origin", default_value="false",
                              description="navigate mode: robot starts where mapping started"),
        DeclareLaunchArgument("perception", default_value="true"),
        DeclareLaunchArgument("yolo_model", default_value="",
                              description="Override YOLO weights, e.g. yolo26s.pt"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("lidar_rays", default_value="true",
                              description="Draw the Ouster beams in the Gazebo GUI"),
        DeclareLaunchArgument("headless", default_value="false",
                              description="Run Gazebo without its GUI"),
        OpaqueFunction(function=launch_setup),
    ])
