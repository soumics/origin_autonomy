#!/usr/bin/env python3
# Copyright 2026 Soumic Sarkar
# SPDX-License-Identifier: Apache-2.0
"""Block until a TF transform is available, then exit (used to order launch startup).

    wait_for_transform.py <target_frame> <source_frame> [timeout_s] --ros-args -p use_sim_time:=true

RTAB-Map publishes map -> icp_odom only after it has processed its first scans (and, in
localization mode, loaded the database), so Nav2's lifecycle manager is started after this exits.
"""

import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def main():
    rclpy.init()
    args = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    target, source = args[0], args[1]
    timeout = float(args[2]) if len(args) > 2 else 120.0

    node = Node("wait_for_transform")
    buffer = Buffer()
    TransformListener(buffer, node)
    start = time.monotonic()
    last_log = 0.0
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
        if buffer.can_transform(target, source, Time(), timeout=Duration(seconds=0.0)):
            node.get_logger().info(f"{target} -> {source} available")
            break
        elapsed = time.monotonic() - start
        if elapsed > timeout:
            node.get_logger().error(f"{target} -> {source} not available after {timeout:.0f} s")
            break
        if elapsed - last_log > 5.0:
            node.get_logger().info(f"Waiting for {target} -> {source} ...")
            last_log = elapsed
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
