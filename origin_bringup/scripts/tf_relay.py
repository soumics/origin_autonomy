#!/usr/bin/env python3
# Copyright 2026 Soumic Sarkar
# SPDX-License-Identifier: Apache-2.0
"""Relay the Origin One's TF (/robot/tf, /robot/tf_static) to /tf and /tf_static.

The robot publishes its transforms under /robot/tf, including its own map -> odom (from the
Avular autopilot). Our RTAB-Map provides map -> icp_odom -> odom, so the robot's map -> odom is
dropped (a frame may only have one parent); everything else, e.g. odom -> base_link, is passed.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage


class TfRelay(Node):

    def __init__(self):
        super().__init__("tf_relay")
        drop = self.declare_parameter("drop", ["map:odom"]).value  # "parent:child" pairs
        self.drop = {tuple(p.split(":", 1)) for p in drop if ":" in p}
        static = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(TFMessage, "/tf", 100)
        self.pub_static = self.create_publisher(TFMessage, "/tf_static", static)
        self.create_subscription(TFMessage, "/robot/tf",
                                 lambda m: self.relay(m, self.pub), 100)
        self.create_subscription(TFMessage, "/robot/tf_static",
                                 lambda m: self.relay(m, self.pub_static), static)

    def relay(self, msg, pub):
        keep = [t for t in msg.transforms
                if (t.header.frame_id.lstrip("/"), t.child_frame_id.lstrip("/")) not in self.drop]
        if keep:
            pub.publish(TFMessage(transforms=keep))


def main():
    rclpy.init()
    rclpy.spin(TfRelay())


if __name__ == "__main__":
    main()
