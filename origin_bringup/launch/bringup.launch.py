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
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable,
                            TimerAction)
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


ROBOT_ADDRESSES = ("192.168.192.1", "192.168.100.11")  # Origin One Wi-Fi AP, Ethernet port
ZENOH_PORT = 7447


def _gateway():
    """Default gateway of this machine (the robot, when connected to its own Wi-Fi)."""
    try:
        with open("/proc/net/route") as f:
            for line in f.readlines()[1:]:
                fields = line.split()
                if fields[1] == "00000000" and int(fields[3], 16) & 2:
                    g = int(fields[2], 16)
                    return ".".join(str((g >> s) & 0xFF) for s in (0, 8, 16, 24))
    except OSError:
        pass
    return ""


def _bridge_cmd(robot_ip, ros_distro):
    """zenoh-bridge-ros2dds client towards the robot's Zenoh server (Avular's documented way to
    reach the Origin One from another computer). Several endpoints: the first one that answers
    is used."""
    ips = [robot_ip] if robot_ip else list(ROBOT_ADDRESSES)
    if not robot_ip and _gateway() and _gateway() not in ips:
        ips.append(_gateway())
    # Only the robot's /robot/... interface is routed: keeps the robot's own /tf (uptime-stamped,
    # with its map -> odom) out of our stack, and our /tf, /map, ... out of the robot's software.
    cmd = ["zenoh-bridge-ros2dds", "client"]
    config = os.environ.get("ORIGIN_ZENOH_CONFIG",
                            "/opt/origin_ws/src/origin_autonomy/config/zenoh_bridge.json5")
    if os.path.isfile(config):
        cmd += ["-c", config]
    for ip in ips:
        cmd += ["-e", f"tcp/{ip}:{ZENOH_PORT}"]
    env = dict(os.environ, ROS_DISTRO=ros_distro)
    return cmd, env, ips


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


def _check_interface():
    """Print which of the Origin One's documented topics/services are visible here."""
    exe = os.path.join(get_package_prefix("origin_bringup"), "lib", "origin_bringup",
                       "check_origin_one.py")
    env = dict(os.environ, ROS_AUTOMATIC_DISCOVERY_RANGE="SUBNET")
    try:
        r = subprocess.run([exe], env=env, capture_output=True, text=True, timeout=60)
        print(r.stdout, flush=True)
    except Exception as e:  # noqa: BLE001 - the check is informative only
        print(f"Interface check failed: {e}", flush=True)


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
        use_bridge = flag("zenoh_bridge")
        while True:
            bridge = None
            if use_bridge:
                cmd, env, ips = _bridge_cmd(arg("robot_ip"), arg("bridge_ros_distro"))
                _say("Looking for the Origin One (about 12 s): Zenoh bridge to "
                     + ", ".join(f"{ip}:{ZENOH_PORT}" for ip in ips) + " ...")
                bridge = subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL,
                                          stderr=subprocess.DEVNULL)
                time.sleep(4.0)  # connect to the robot and declare its topics locally
            else:
                _say("Looking for the Origin One on the network (about 8 s) ...")
            try:
                found = _detect()
                if found.get("found"):
                    _check_interface()  # while the probe bridge is still connected
            finally:
                if bridge is not None:
                    bridge.terminate()
                    try:
                        bridge.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        bridge.kill()
            if found.get("found"):
                break
            reason = found.get("reason", "")
            if sim_arg == "auto":
                _say("Origin One NOT detected: " + reason,
                     "The robot has not been started, or this PC / laptop is not connected to",
                     "its network (the robot's Wi-Fi, or its Ethernet port). Starting the",
                     "Gazebo SIMULATION instead.",
                     "Use sim:=false to wait for the robot, sim:=true to skip this check.")
                break
            _say("Origin One NOT detected: " + reason,
                 "The robot has not been started, or this PC / laptop is not connected to",
                 "its network (the robot's Wi-Fi, or its Ethernet port).",
                 "Retrying in 5 s (Ctrl-C to stop) ...")
            time.sleep(5.0)
        sim = not found.get("found")
        if not sim:
            for k in topics:
                if found.get(k):
                    topics[k] = found[k]
            if found.get("lidar_note"):
                _say(f"Lidar over Wi-Fi: {found['lidar_note']} -> using {found['lidar']}")
            # Never drive the robot's internal /robot/cmd_vel: Avular's input for custom
            # software is /robot/cmd_vel_user (executed in USER control mode).
            topics["cmd_vel"] = "/robot/cmd_vel_user"
            if not found.get("cmd_vel_user_listened"):
                _say("WARNING: the robot does not (yet) listen on /robot/cmd_vel_user.",
                     "Driving will not work until it does (check with origin-check-robot).")
    for k in topics:  # explicit topic arguments win
        if arg(k + "_topic"):
            topics[k] = arg(k + "_topic")

    # ---------------------------------------------------------------- explore or navigate
    database = os.path.expanduser(arg("database_path") or (
        "~/.ros/origin_rtabmap_sim.db" if sim else "~/.ros/origin_rtabmap_robot.db"))
    legacy = os.path.expanduser("~/.ros/origin_rtabmap.db")  # before sim/robot were separate
    if sim and not arg("database_path") and os.path.isfile(legacy) \
            and not os.path.exists(database):
        shutil.move(legacy, database)
    mode = arg("mode")
    if mode not in ("auto", "explore", "navigate"):
        raise RuntimeError(f"mode must be auto, explore or navigate, got '{mode}'")
    have_map = os.path.isfile(database)
    # mission_manager writes <database>.yaml/.pgm when exploration finishes. A database without
    # it is an interrupted exploration (e.g. a stopped container), not a map to navigate in.
    finished_map = os.path.splitext(database)[0] + ".yaml"
    have_finished_map = have_map and os.path.isfile(finished_map)
    if mode == "auto":
        mode = "navigate" if have_finished_map else "explore"
        if have_map and not have_finished_map:
            print(f"{database} is an unfinished exploration: exploring again", flush=True)
    if mode == "navigate" and not have_map:
        raise RuntimeError(f"mode:=navigate needs a saved map, {database} does not exist")
    if mode == "explore" and have_map:
        stamp = datetime.datetime.now().strftime(".%Y%m%d-%H%M%S.bak")
        shutil.move(database, database + stamp)
        for ext in (".yaml", ".pgm"):
            old = os.path.splitext(database)[0] + ext
            if os.path.isfile(old):
                shutil.move(old, old + stamp)
        print(f"Existing map kept as {database + stamp}", flush=True)
        have_map = False

    backend = arg("backend")
    use_sim_time = "true" if sim else "false"
    # Gazebo: let it start and spawn the robot; robot: let the bridge connect first.
    delay = 8.0 if sim else 6.0
    drive_hint = [] if sim else [
        "REAL ROBOT: nothing drives until you enable it (keep a hand on the e-stop):",
        "   docker exec origin_run origin-enable-driving    # USER control mode + start",
        "   docker exec origin_run origin-stop-driving      # stop, control mode back to NONE"]
    _say(f"Origin One autonomy: {'SIMULATION (Gazebo)' if sim else 'REAL ROBOT'}, "
         f"mode {mode.upper()}, backend {backend}",
         f"map: {database} ({'exists' if have_map else 'new'})",
         *(f"{k:12s} {v}" for k, v in topics.items()),
         "explore: when no frontiers are left the map is saved and RTAB-Map switches to"
         " localization" if mode == "explore" else
         "navigate: send goals with RViz '2D Goal Pose' (set the start with '2D Pose Estimate')",
         *drive_hint)

    actions = []
    if not sim:
        # Network-wide discovery: the Zenoh bridge's DDS side (CycloneDDS) is not found by
        # localhost-only participants. Our ROS_DOMAIN_ID differs from the robot's, so the
        # robot's own DDS traffic stays separate; everything from the robot comes via Zenoh.
        actions.append(SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "SUBNET"))
        if flag("zenoh_bridge"):
            cmd, env, _ = _bridge_cmd(arg("robot_ip"), arg("bridge_ros_distro"))
            actions.append(ExecuteProcess(cmd=cmd, additional_env={
                "ROS_DISTRO": env["ROS_DISTRO"]}, output="log", name="zenoh_bridge"))
        # The robot's sensor clocks are not this PC's clock: consume re-stamped copies.
        # Camera over Wi-Fi: the compressed stream (JPEG, ~20x smaller than raw RGB) leaves the
        # link to the lidar (seen: lidar 0.8 Hz with raw images).
        restamped = {"lidar": "/origin/lidar/points",
                     "image": "/origin/camera/color/image_raw/compressed",
                     "camera_info": "/origin/camera/color/camera_info"}
        src = {k: (topics[k] if topics[k].startswith("/robot/") else SIM_TOPICS[k])
               for k in ("lidar", "image", "camera_info")}
        src["image"] = src["image"].rstrip("/") + "/compressed"
        pairs = [f"{src[k]}:{v}:{t}" for k, v, t in (
            ("lidar", restamped["lidar"], "sensor_msgs/msg/PointCloud2"),
            ("image", restamped["image"], "sensor_msgs/msg/CompressedImage"),
            ("camera_info", restamped["camera_info"], "sensor_msgs/msg/CameraInfo"))]
        topics.update(restamped)
        actions.append(_include("origin_bringup", "robot_model.launch.py",
                                odom_topic=topics["odom"], restamp_pairs=",".join(pairs)))
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
                     backend=backend, database_path=database, rviz="false",
                     explorer_autostart="true" if sim else "false", **nav_topics),
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
                      "compressed": "false" if sim else "true",
                      "image_topic": topics["image"],
                      # On the robot, 3D positions come from the lidar; not subscribing to depth
                      # keeps it off the Wi-Fi link.
                      "depth_topic": topics["depth"] if sim else "/perception/depth_unused",
                      "camera_info_topic": topics["camera_info"], "cloud_topic": topics["lidar"]}
        if arg("yolo_model"):
            perception["model"] = arg("yolo_model")
        actions.append(TimerAction(period=delay, actions=[
            _include("origin_yolo_perception", "perception.launch.py", **perception)]))

    if flag("rviz"):
        actions.append(TimerAction(period=delay / 2, actions=[Node(
            package="rviz2", executable="rviz2", name="rviz2", output="log",
            respawn=True, respawn_delay=3.0,  # crashed once on the robot (std::system_error)
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
        DeclareLaunchArgument("database_path", default_value="",
                              description="RTAB-Map map database; default "
                                          "~/.ros/origin_rtabmap_sim.db / _robot.db"),
        DeclareLaunchArgument("robot_ip", default_value="",
                              description="Origin One address; default: try 192.168.192.1 "
                                          "(its Wi-Fi), 192.168.100.11 (Ethernet), gateway"),
        DeclareLaunchArgument("zenoh_bridge", default_value="true",
                              description="Reach the robot through zenoh-bridge-ros2dds "
                                          "(Avular's documented way); false: direct DDS"),
        DeclareLaunchArgument("bridge_ros_distro", default_value="jazzy",
                              description="ROS_DISTRO for the local bridge (this PC runs Jazzy)"),
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
