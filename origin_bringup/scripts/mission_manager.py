#!/usr/bin/env python3
# Copyright 2026 Soumic Sarkar
# SPDX-License-Identifier: Apache-2.0
"""Hands over from exploration to localization automatically.

When the frontier explorer reports "done" (no frontiers left, back at the start), this node
  1. saves the RTAB-Map database (/rtabmap/rtabmap/backup),
  2. writes a 2D copy of the map (<database>.pgm/.yaml, nav2 map_saver) for other tools,
  3. switches RTAB-Map to localization (/rtabmap/rtabmap/set_mode_localization),
and publishes the mission state on /origin/mission_status: exploring | saving | localization.
Navigation keeps running, so the robot can be sent goals (RViz "2D Goal Pose") right away.
"""

import os
import subprocess
import threading
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
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

    def hand_over(self):
        self.get_logger().info("Exploration finished: saving the map ...")
        saved = self.call("backup")
        if self.save_2d:
            base = os.path.splitext(self.database)[0]
            use_sim_time = self.get_parameter("use_sim_time").value
            r = subprocess.run(
                ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", base,
                 "--ros-args", "-p", f"use_sim_time:={str(use_sim_time).lower()}",
                 "-p", "save_map_timeout:=20.0"],
                capture_output=True, text=True, timeout=60)
            if r.returncode != 0:
                self.get_logger().warn(f"2D map not saved: {r.stderr.strip()[-200:]}")
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
