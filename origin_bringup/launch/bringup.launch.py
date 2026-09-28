# Copyright 2026 Soumic Sarkar
# SPDX-License-Identifier: Apache-2.0
"""Single entry point for the Origin One autonomy stack.

    ros2 launch origin_bringup bringup.launch.py            # everything automatic

sim:=auto (default) first looks for a running Origin One on the network (detect_origin_one.py,
~8 s). If found, the stack runs on the robot with the detected lidar / camera / odom / cmd_vel
topics and network-wide ROS discovery. If not, it says so (the robot is not started or not on
this PC's network) and runs the Gazebo simulation. sim:=true forces the simulation;
sim:=false waits for the robot.

mode:=auto (default) localizes in the saved map (database_path) if it exists, otherwise it
explores from scratch; when exploration finishes, mission_manager saves the map and switches
RTAB-Map to localization automatically. mode:=explore / mode:=navigate force one of them
(explore keeps a backup of an existing map).

Started in order:
  1. Gazebo + robot (simulation only)
  2. RTAB-Map: mapping (explore) or localization (navigate)
  3. Navigation (backend:=nav2 | custom), plus the frontier explorer and mission manager
     in explore mode
  4. YOLO perception (perception:=false to skip)
  5. RViz (rviz:=false to skip)

Topic arguments (lidar_topic, image_topic, depth_topic, camera_info_topic, odom_topic,
cmd_vel_topic) override the detected / default topics when set.
"""

import datetime
import json
import os
import shutil
import subprocess
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _share(pkg, *path):
    return os.path.join(get_package_share_directory(pkg), *path)


def _include(pkg, launch_file, **args):
    return GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_share(pkg, "launch", launch_file)),
        launch_arguments={k: str(v) for k, v in args.items()}.items())], scoped=True)


SIM_TOPICS = {
    "lidar": "/robot/lidar/points", "image": "/robot/camera/color/image_raw",
    "depth": "/robot/camera/depth/image_raw", "camera_info": "/robot/camera/color/camera_info",
    "odom": "/robot/odom", "cmd_vel": "/robot/cmd_vel",
}
BANNER = "=" * 78


def _say(*lines):
    print("\n".join([BANNER, *("  " + line for line in lines), BANNER]), flush=True)


def _detect(timeout=8.0):
    """Run detect_origin_one.py with network-wide discovery; returns its JSON result."""
    exe = os.path.join(get_package_prefix("origin_bringup"), "lib", "origin_bringup",
                       "detect_origin_one.py")
    env = dict(os.environ, ROS_AUTOMATIC_DISCOVERY_RANGE="SUBNET")
    try:
        r = subprocess.run([exe, "--timeout", str(timeout)], env=env, capture_output=True,
                           text=True, timeout=timeout + 30)
        return json.loads(r.stdout.strip().splitlines()[-1])
    except Exception as e:  # noqa: BLE001 - report any probe failure as "not found"
        return {"found": False, "reason": f"detection failed: {e}"}


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def flag(name):
        return arg(name).lower() in ("true", "1", "yes")

    # ---------------------------------------------------------------- robot or simulation
    sim_arg = arg("sim").lower()
    if sim_arg not in ("auto", "true", "false"):
        raise RuntimeError(f"sim must be auto, true or false, got '{sim_arg}'")
    topics = dict(SIM_TOPICS)
    if sim_arg == "true":
        sim = True
    else:
        _say("Looking for the Origin One on the network (about 8 s) ...")
        while True:
            found = _detect()
            if found.get("found"):
                break
            reason = found.get("reason", "")
            if sim_arg == "auto":
                _say("Origin One NOT detected: " + reason,
                     "The robot has not been started, or it is not on the same network as",
                     "this PC / laptop (check Wi-Fi, ROS_DOMAIN_ID, and that the robot's ROS",
                     "stack is running). Starting the Gazebo SIMULATION instead.",
                     "Use sim:=false to wait for the robot, sim:=true to skip this check.")
                break
            _say("Origin One NOT detected: " + reason,
                 "The robot has not been started, or it is not on the same network as",
                 "this PC / laptop. Retrying in 5 s (Ctrl-C to stop) ...")
            time.sleep(5.0)
        sim = not found.get("found")
        if not sim:
            for k in topics:
                if found.get(k):
                    topics[k] = found[k]
    for k in topics:  # explicit topic arguments win
        if arg(k + "_topic"):
            topics[k] = arg(k + "_topic")

    # ---------------------------------------------------------------- explore or navigate
    database = os.path.expanduser(arg("database_path"))
    mode = arg("mode")
    if mode not in ("auto", "explore", "navigate"):
        raise RuntimeError(f"mode must be auto, explore or navigate, got '{mode}'")
    have_map = os.path.isfile(database)
    if mode == "auto":
        mode = "navigate" if have_map else "explore"
    if mode == "navigate" and not have_map:
        raise RuntimeError(f"mode:=navigate needs a saved map, {database} does not exist")
    if mode == "explore" and have_map:
        backup = database + datetime.datetime.now().strftime(".%Y%m%d-%H%M%S.bak")
        shutil.move(database, backup)
        print(f"Existing map kept as {backup}", flush=True)

    backend = arg("backend")
    use_sim_time = "true" if sim else "false"
    delay = 8.0 if sim else 0.0  # let Gazebo start and spawn the robot first
    _say(f"Origin One autonomy: {'SIMULATION (Gazebo)' if sim else 'REAL ROBOT'}, "
         f"mode {mode.upper()}, backend {backend}",
         f"map: {database} ({'exists' if have_map else 'new'})",
         *(f"{k:12s} {v}" for k, v in topics.items()),
         "explore: when no frontiers are left the map is saved and RTAB-Map switches to"
         " localization" if mode == "explore" else
         "navigate: send goals with RViz '2D Goal Pose' (set the start with '2D Pose Estimate')")

    actions = []
    if not sim:
        # Talk to the robot over the network (the image defaults to this machine only).
        actions.append(SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "SUBNET"))
    else:
        actions.append(_include(
            "origin_bringup", "sim.launch.py", world=arg("world"), headless=arg("headless"),
            lidar_rays=arg("lidar_rays"),
            rviz="false"))  # our RViz below; without this, rviz:=true leaks into the fork's RViz

    nav_topics = {"lidar_topic": topics["lidar"], "odom_topic": topics["odom"],
                  "cmd_vel_topic": topics["cmd_vel"]}
    if mode == "explore":
        stack = [
            _include("origin_frontier_explore", "explore.launch.py", use_sim_time=use_sim_time,
                     backend=backend, database_path=database, rviz="false", **nav_topics),
            Node(package="origin_bringup", executable="mission_manager.py",
                 name="mission_manager", output="screen",
                 parameters=[{"use_sim_time": sim, "database_path": database}]),
        ]
    else:
        stack = [
            _include("origin_lidar_localization", "slam.launch.py", use_sim_time=use_sim_time,
                     localization="true", start_at_origin=arg("start_at_origin"),
                     database_path=database, lidar_topic=topics["lidar"]),
            _include("origin_frontier_explore", "navigation.launch.py",
                     use_sim_time=use_sim_time, backend=backend, **nav_topics),
        ]
    actions.append(TimerAction(period=delay, actions=stack))

    if flag("perception"):
        perception = {"use_sim_time": use_sim_time, "sim": "true" if sim else "false",
                      "image_topic": topics["image"], "depth_topic": topics["depth"],
                      "camera_info_topic": topics["camera_info"], "cloud_topic": topics["lidar"]}
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
        DeclareLaunchArgument("sim", default_value="auto",
                              description="auto: real robot if found on the network, else "
                                          "Gazebo; true: Gazebo; false: wait for the robot"),
        DeclareLaunchArgument("mode", default_value="auto",
                              description="auto: navigate if a saved map exists, else explore; "
                                          "explore: map from scratch; navigate: saved map"),
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
        *[DeclareLaunchArgument(f"{k}_topic", default_value="",
                                description=f"Override the {k} topic (default: detected / sim)")
          for k in ("lidar", "image", "depth", "camera_info", "odom", "cmd_vel")],
        OpaqueFunction(function=launch_setup),
    ])
