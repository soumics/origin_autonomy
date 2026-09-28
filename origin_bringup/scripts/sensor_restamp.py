#!/usr/bin/env python3
# Copyright 2026 Soumic Sarkar
# SPDX-License-Identifier: Apache-2.0
"""Re-stamp the Origin One's sensor messages with this computer's clock.

On a real Origin One the Ouster cloud arrived stamped with the sensor's internal clock (about
7000 s, i.e. its uptime; the driver's TIME_FROM_INTERNAL_OSC mode), and the robot's clock is not
synchronised with this PC. SLAM, TF and Nav2 need one time base, so every message is
republished with its arrival time (Wi-Fi latency, tens of ms, is the remaining error).

    ~pairs: ["<in>:<out>:<pkg/msg/Type>", ...]
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

DEFAULT = [
    "/robot/lidar/points:/origin/lidar/points:sensor_msgs/msg/PointCloud2",
    "/robot/camera/color/image_raw:/origin/camera/color/image_raw:sensor_msgs/msg/Image",
    "/robot/camera/color/camera_info:/origin/camera/color/camera_info:sensor_msgs/msg/CameraInfo",
]


class SensorRestamp(Node):

    def __init__(self):
        super().__init__("sensor_restamp")
        pairs = self.declare_parameter("pairs", DEFAULT).value
        self.warned = set()
        self._keep = []
        for p in pairs:
            src, dst, typ = p.split(":")
            if src == dst:
                self.get_logger().error(f"Refusing to re-stamp {src} onto itself")
                continue
            msg_type = get_message(typ)
            pub = self.create_publisher(msg_type, dst, qos_profile_sensor_data)
            self._keep.append(self.create_subscription(
                msg_type, src, lambda m, pub=pub, src=src: self.relay(m, pub, src),
                qos_profile_sensor_data))
            self.get_logger().info(f"{src} -> {dst} (re-stamped with this PC's clock)")

    def relay(self, msg, pub, src):
        now = self.get_clock().now()
        if src not in self.warned:
            offset = now.nanoseconds * 1e-9 - (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            self.get_logger().info(f"{src}: robot stamp differs from this PC by {offset:.1f} s")
            self.warned.add(src)
        msg.header.stamp = now.to_msg()
        pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(SensorRestamp())


if __name__ == "__main__":
    main()
