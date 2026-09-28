#!/usr/bin/env python3
# Copyright 2026 Soumic Sarkar
# SPDX-License-Identifier: Apache-2.0
"""Robot TF for the PC side of the Zenoh bridge.

- Relays the Origin One's /robot/tf and /robot/tf_static to /tf and /tf_static, dropping the
  robot's own map -> odom (our RTAB-Map provides map -> icp_odom -> odom; a frame may only have
  one parent).
- Fallback: if no odom -> base_link arrives through /robot/tf (seen on a real Origin One: the
  topic was not available through the bridge), odom -> base_link is published from /robot/odom
  (nav_msgs/Odometry, frames from its header), which the robot does provide.
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from tf2_msgs.msg import TFMessage


class TfRelay(Node):

    def __init__(self):
        super().__init__("tf_relay")
        drop = self.declare_parameter("drop", ["map:odom"]).value  # "parent:child" pairs
        self.drop = {tuple(p.split(":", 1)) for p in drop if ":" in p}
        self.fallback_after = self.declare_parameter("odom_fallback_after", 2.0).value
        # The robot's clock is not synchronised with this PC: use this PC's time (see
        # sensor_restamp.py, which does the same for the lidar and camera).
        self.restamp = self.declare_parameter("restamp", True).value
        static = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(TFMessage, "/tf", 100)
        self.pub_static = self.create_publisher(TFMessage, "/tf_static", static)
        self.create_subscription(TFMessage, "/robot/tf",
                                 lambda m: self.relay(m, self.pub, dynamic=True), 100)
        self.create_subscription(TFMessage, "/robot/tf_static",
                                 lambda m: self.relay(m, self.pub_static), static)
        self.create_subscription(Odometry, "odom", self.on_odom, qos_profile_sensor_data)
        self.start = self.get_clock().now()
        self.robot_tf_seen = False
        self.fallback_announced = False

    def relay(self, msg, pub, dynamic=False):
        keep = [t for t in msg.transforms
                if (t.header.frame_id.lstrip("/"), t.child_frame_id.lstrip("/")) not in self.drop]
        if dynamic and any(t.child_frame_id.lstrip("/") == "base_link" for t in keep):
            self.robot_tf_seen = True
        if dynamic and self.restamp:
            now = self.get_clock().now().to_msg()
            for t in keep:
                t.header.stamp = now
        if keep:
            pub.publish(TFMessage(transforms=keep))

    def on_odom(self, msg: Odometry):
        if self.robot_tf_seen:
            return
        if (self.get_clock().now() - self.start).nanoseconds * 1e-9 < self.fallback_after:
            return
        if not self.fallback_announced:
            self.get_logger().warn(
                "No odom -> base_link on /robot/tf: publishing it from the odometry topic "
                f"({msg.header.frame_id} -> {msg.child_frame_id})")
            self.fallback_announced = True
        t = TransformStamped()
        t.header = msg.header
        if self.restamp:
            t.header.stamp = self.get_clock().now().to_msg()
        t.child_frame_id = msg.child_frame_id or "base_link"
        if not t.header.frame_id:
            t.header.frame_id = "odom"
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.x, p.y, p.z
        t.transform.rotation = q
        self.pub.publish(TFMessage(transforms=[t]))


def main():
    rclpy.init()
    rclpy.spin(TfRelay())


if __name__ == "__main__":
    main()
