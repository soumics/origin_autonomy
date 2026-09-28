#!/usr/bin/env python3
# Copyright 2026 Soumic Sarkar
# SPDX-License-Identifier: Apache-2.0
"""Draw the simulated Ouster beams in the Gazebo GUI (simulation only).

Subscribes to the Gazebo lidar scan (gz.msgs.LaserScan on /robot/lidar, which carries the
sensor's world pose) and sends a LINE_LIST marker (sensor -> hit point for a subsample of the
beams) to the GUI's marker service. Pure gz-transport: no ROS or TF needed.

    lidar_ray_markers.py [--topic /robot/lidar] [--rate 5] [--h-step 16] [--v-step 4]
"""

import argparse
import math
import threading
import time

from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.laserscan_pb2 import LaserScan
from gz.msgs10.marker_pb2 import Marker
from gz.transport13 import Node


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/robot/lidar")
    ap.add_argument("--rate", type=float, default=5.0, help="marker updates per second")
    ap.add_argument("--h-step", type=int, default=16, help="draw every n-th horizontal beam")
    ap.add_argument("--v-step", type=int, default=4, help="draw every n-th vertical ring")
    ap.add_argument("--misses", action="store_true", help="also draw beams without a hit")
    args, _ = ap.parse_known_args()  # ignore --ros-args when started from a launch file

    node = Node()
    latest = {}
    lock = threading.Lock()

    def on_scan(msg: LaserScan):
        with lock:
            latest["scan"] = msg

    if not node.subscribe(LaserScan, args.topic, on_scan):
        raise SystemExit(f"cannot subscribe to {args.topic}")

    period = 1.0 / args.rate
    warned = False
    while True:
        time.sleep(period)
        with lock:
            scan = latest.pop("scan", None)
        if scan is None:
            continue

        m = Marker()
        m.ns = "ouster_rays"
        m.id = 1
        m.action = Marker.ADD_MODIFY
        m.type = Marker.LINE_LIST
        m.visibility = Marker.GUI
        m.pose.CopyFrom(scan.world_pose)
        m.lifetime.nsec = int(2.5 * period * 1e9)
        for c in (m.material.ambient, m.material.diffuse, m.material.emissive):
            c.r, c.g, c.b, c.a = 1.0, 0.15, 0.1, 0.9

        h_count = max(scan.count, 1)
        v_count = max(scan.vertical_count, 1)
        for v in range(0, v_count, args.v_step):
            va = scan.vertical_angle_min + v * scan.vertical_angle_step
            for h in range(0, h_count, args.h_step):
                r = scan.ranges[v * h_count + h]
                if not math.isfinite(r) or r >= scan.range_max:
                    if not args.misses:
                        continue
                    r = scan.range_max
                if r < scan.range_min:
                    continue
                ha = scan.angle_min + h * scan.angle_step
                p0 = m.point.add()
                p0.x = p0.y = p0.z = 0.0
                p1 = m.point.add()
                p1.x = r * math.cos(va) * math.cos(ha)
                p1.y = r * math.cos(va) * math.sin(ha)
                p1.z = r * math.sin(va)

        # The GUI's /marker service is one-way (gz.msgs.Empty reply, never sent), so this call
        # always "times out"; the marker is delivered regardless. Keep the timeout short.
        if not node.service_info("/marker") and not warned:
            print("lidar_ray_markers: waiting for the Gazebo GUI (/marker service)", flush=True)
            warned = True
        node.request("/marker", m, Marker, Empty, 5)


if __name__ == "__main__":
    main()
