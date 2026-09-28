#!/usr/bin/env python3
# Copyright 2026 Soumic Sarkar
# SPDX-License-Identifier: Apache-2.0
"""Look for a running Avular Origin One on the network and work out its sensor topics.

    detect_origin_one.py [--timeout 8]   ->   prints one JSON object on stdout

Discovery runs with ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET (set by the caller) for `timeout`
seconds; then the ROS graph is classified:

  * A simulation (a /clock publisher, or sensors published by ros_gz_bridge) is not a robot.
  * lidar:  a sensor_msgs/PointCloud2 topic, preferring names with ouster/lidar/points
  * camera: a sensor_msgs/Image colour topic (+ its CameraInfo), and a depth Image if present
  * odom:   a nav_msgs/Odometry topic
  * cmd_vel: the Origin One's cmd_vel_controller input for custom software (/robot/cmd_vel_user)
             if it exists, else a geometry_msgs/Twist cmd_vel topic

Output: {"found": bool, "reason": str, "lidar": str, "image": str, "depth": str,
         "camera_info": str, "odom": str, "cmd_vel": str, "topics": int}
"""

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node

IGNORE = ("_filtered", "_viz", "/rtabmap/", "/perception/", "/local_costmap", "/global_costmap")
BUILTIN = ("/rosout", "/parameter_events")  # every participant has these


def pick(candidates, prefer, avoid=()):
    cands = [c for c in candidates if not any(a in c for a in avoid)]
    for key in prefer:
        for c in cands:
            if key in c:
                return c
    return cands[0] if cands else ""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--timeout", type=float, default=8.0)
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = Node("origin_one_detector")
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    topics = {n: t for n, t in node.get_topic_names_and_types()
              if not any(i in n for i in IGNORE) and n not in BUILTIN}

    def of_type(type_name):
        return sorted(n for n, t in topics.items() if type_name in t and
                      node.count_publishers(n) > 0)

    clouds = of_type("sensor_msgs/msg/PointCloud2")
    images = of_type("sensor_msgs/msg/Image")
    infos = of_type("sensor_msgs/msg/CameraInfo")
    odoms = of_type("nav_msgs/msg/Odometry")
    twists = sorted(n for n, t in topics.items() if "geometry_msgs/msg/Twist" in t)

    lidar = pick(clouds, ("ouster", "lidar", "points"), avoid=("depth", "camera"))
    sim_publishers = set()
    if lidar:
        sim_publishers = {i.node_name for i in node.get_publishers_info_by_topic(lidar)}
    is_sim = "/clock" in topics or "ros_gz_bridge" in sim_publishers

    color = pick([i for i in images if "depth" not in i], ("color/image_raw", "image_raw", "image"))
    prefix = color.rsplit("/", 2)[0] if color.count("/") >= 2 else ""
    depth = pick([i for i in images if "depth" in i],
                 ("aligned_depth_to_color/image_raw", "depth/image_rect_raw", "depth/image_raw"))
    info = pick([i for i in infos if "depth" not in i],
                (color.rsplit("/", 1)[0] + "/camera_info" if color else "zzz", prefix, "color"))
    odom = pick(odoms, ("/robot/odom", "odom"), avoid=("icp_odom",))
    cmd = ("/robot/cmd_vel_user" if node.count_subscribers("/robot/cmd_vel_user") > 0
           else pick(twists, ("/robot/cmd_vel", "cmd_vel")))

    found = bool(lidar) and not is_sim
    if found:
        reason = "Origin One sensors found on the network"
    elif lidar and is_sim:
        reason = "only a Gazebo simulation is running on the network (it has /clock)"
    elif topics:
        reason = "ROS 2 is running on the network, but no lidar point cloud is published"
    else:
        reason = "no ROS 2 participants found on the network"
    print(json.dumps({
        "found": found, "reason": reason, "lidar": lidar, "image": color, "depth": depth,
        "camera_info": info, "odom": odom, "cmd_vel": cmd, "topics": len(topics),
    }))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
