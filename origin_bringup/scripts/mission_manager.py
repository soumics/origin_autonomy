#!/usr/bin/env python3
# Copyright 2026 Soumic Sarkar
# SPDX-License-Identifier: Apache-2.0
"""Hands over from exploration to localization automatically.

When the frontier explorer reports "done" (no frontiers left, back at the start), this node
  1. saves the RTAB-Map database (/rtabmap/rtabmap/backup),
  2. writes a 2D copy of the map (<database>.pgm/.yaml, map_server format) from /map,
  3. switches RTAB-Map to localization (/rtabmap/rtabmap/set_mode_localization),
and publishes the mission state on /origin/mission_status: exploring | saving | localization.
Navigation keeps running, so the robot can be sent goals (RViz "2D Goal Pose") right away.
"""

import os
import threading
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from std_srvs.srv import Empty

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class MissionManager(Node):

    def __init__(self):
        super().__init__("mission_manager")
        self.rtabmap = self.declare_parameter("rtabmap_node", "/rtabmap/rtabmap").value
        self.database = os.path.expanduser(
            self.declare_parameter("database_path", "~/.ros/origin_rtabmap.db").value)
        self.save_2d = self.declare_parameter("save_2d_map", True).value
        self.pub = self.create_publisher(String, "/origin/mission_status", LATCHED)
        self.clients_ = {n: self.create_client(Empty, f"{self.rtabmap}/{n}")
                         for n in ("backup", "set_mode_localization")}
        self.create_subscription(String, "/frontier_explorer/status", self.on_status, LATCHED)
        self.map = None
        self.create_subscription(OccupancyGrid, "/map", lambda m: setattr(self, "map", m), LATCHED)
        self.state = None
        self.set_state("exploring")

    def set_state(self, s):
        self.state = s
        self.pub.publish(String(data=s))

    def on_status(self, msg: String):
        if msg.data == "done" and self.state == "exploring":
            self.set_state("saving")
            # Service calls wait on futures completed by the main spin thread.
            threading.Thread(target=self.hand_over, daemon=True).start()

    def call(self, name, timeout=60.0):
        cli = self.clients_[name]
        if not cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(f"{self.rtabmap}/{name} not available")
            return False
        fut = cli.call_async(Empty.Request())
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < timeout:
            time.sleep(0.05)
        return fut.done()

    def write_2d(self, base):
        """map_server-compatible trinary .pgm + .yaml of the latest /map."""
        m = self.map
        if m is None:
            self.get_logger().warn("2D map not saved: no /map received")
            return
        w, h = m.info.width, m.info.height
        rows = []
        for y in range(h - 1, -1, -1):  # PGM rows go top-down
            row = m.data[y * w:(y + 1) * w]
            rows.append(bytes(205 if v < 0 else (0 if v >= 65 else 254) for v in row))
        with open(base + ".pgm", "wb") as f:
            f.write(f"P5\n{w} {h}\n255\n".encode())
            f.write(b"".join(rows))
        o = m.info.origin.position
        with open(base + ".yaml", "w") as f:
            f.write(f"image: {os.path.basename(base)}.pgm\nmode: trinary\n"
                    f"resolution: {m.info.resolution:.3f}\norigin: [{o.x:.3f}, {o.y:.3f}, 0]\n"
                    "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")

    def hand_over(self):
        self.get_logger().info("Exploration finished: saving the map ...")
        saved = self.call("backup")
        if self.save_2d:
            self.write_2d(os.path.splitext(self.database)[0])
        localized = self.call("set_mode_localization")
        self.set_state("localization" if localized else "error")
        banner = "=" * 72
        self.get_logger().info(
            f"\n{banner}\n  MAP {'SAVED' if saved else 'NOT SAVED'}: {self.database}\n"
            f"  RTAB-Map switched to LOCALIZATION{'' if localized else ' FAILED'}.\n"
            f"  Send navigation goals with RViz '2D Goal Pose'. Next start localizes here.\n"
            f"{banner}")


def main():
    rclpy.init()
    node = MissionManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
